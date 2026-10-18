/*
 * wrap_model.c
 *
 * Diameter estimation and layer planning for wrap().  No hardware access --
 * see wrap_model.h for the overview of how it works.
 */

#include <math.h>
#include <string.h>
#include "wrap_model.h"

#define PI_F 3.14159265f

// Dwelling at an end with fewer encoder counts per turn than this fraction
// of the circumference means fiber is not paying out.
#define FIBER_MIN_FRACTION    0.5f

// Wind a cell in the next layer only if it still needs at least this fraction of a layer.
#define WIND_IF_NEED_FRACTION 0.5f

// How fast the learned per-layer growth follows the measurements (0..1).
#define GROWTH_LEARN_GAIN     0.3f

// End cells get hoop wraps on top of the helical layer, so their growth is much
// less predictable: give them this many layers' worth of process noise.
#define END_CELL_NOISE_LAYERS 2.0f


// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void wrapDefaultConfig(WrapConfig *c) {
    c->rightEndIn      = 1.000f;
    c->lengthIn        = 12.000f;
    c->largeDiaIn      = 4.000f;
    c->smallDiaIn      = 3.250f;
    c->mandrelDiaIn    = 3.000f;
    c->fullFinish      = 1;

    c->towWidthIn      = 0.250f;
    c->towThicknessIn  = 0.012f;
    c->overlap         = 0.05f;

    c->circuits        = 5;
    c->dwellRevs       = 0.75f;
    c->tieRevs         = 2.0f;

    c->barrelRps       = 0.5f;
    c->maxCarriageIps  = 1.5f;

    c->cellIn          = 0.25f;
    c->initialSigmaIn  = 0.100f;
    c->measSigmaIn     = 0.020f;
    c->processSigmaIn  = 0.005f;
    c->gateSigmas      = 4.0f;
    c->maxLayers       = 80;
}

const char *wrapCheckConfig(const WrapConfig *c) {
    if (c->lengthIn < 1.0f)                                   return "Length too short";
    if (c->rightEndIn < 0.0f)                                 return "Right end < home";
    if (c->lengthIn / c->cellIn > WRAP_MAX_CELLS)             return "Too many cells";
    if (c->smallDiaIn > c->largeDiaIn)                        return "Small dia > large";
    if (c->mandrelDiaIn >= c->smallDiaIn)                     return "Mandrel >= small dia";
    if (c->towWidthIn <= 0.0f || c->towThicknessIn <= 0.0f)   return "Bad tow size";
    if (c->circuits < 1 || c->circuits > WRAP_MAX_CIRCUITS)   return "Passes 1-12 only";
    if (c->dwellRevs < 0.5f)                                  return "Dwell < 0.5 rev";
    if (c->barrelRps <= 0.0f)                                 return "Barrel speed 0";

    // n tow widths must fit around the smallest circumference with room to spare
    float nw = c->circuits * c->towWidthIn * (1.0f - c->overlap);
    if (nw >= 0.9f * PI_F * c->mandrelDiaIn)                  return "Too many passes";
    return 0;
}


// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

// Carriage advance per barrel revolution (inches) so that `circuits` passes in
// one direction lay side by side with no gap.
//   Adjacent passes in one direction are lead/circuits apart along the axis.
//   Measured square to the fiber that spacing is (lead/circuits) * sin(angle),
//   and it must equal the tow width.  With tan(angle) = C/lead this gives:
//       lead = nw * C / sqrt(C^2 - nw^2)        (nw = circuits * tow width)
static float leadPerRev(float circumference, float nw) {
    return nw * circumference / sqrtf(circumference * circumference - nw * nw);
}

float diameterFromReading(float cellIn, float dy, float de) {
    if (dy <= 0.0f) return -1.0f;
    float fiberIn = de / COUNT_PER_INCH;                  // fiber laid while crossing the cell
    float around2 = fiberIn * fiberIn - cellIn * cellIn;  // fiber^2 = travel^2 + around^2
    if (around2 <= 0.0f) return -1.0f;
    float circumference = sqrtf(around2) * STEPS_PER_REVOLUTION / dy;
    return circumference / PI_F;
}

void modelHoopReading(WrapModel *m, int end, int32_t dy, int32_t de) {
    if (dy < STEPS_PER_REVOLUTION / 4) return;            // too short to trust
    float fiberIn = (float)de / COUNT_PER_INCH;
    m->hoopDia[end]   = fiberIn * STEPS_PER_REVOLUTION / dy / PI_F;
    m->hoopRevs[end] += (float)dy / STEPS_PER_REVOLUTION;
}


// ---------------------------------------------------------------------------
// Model life cycle
// ---------------------------------------------------------------------------

int modelInit(WrapModel *m, const WrapConfig *c) {
    int n = (int)(c->lengthIn / c->cellIn + 0.5f);
    if (n < 2 || n > WRAP_MAX_CELLS) return 0;

    m->nCells            = n;
    m->cellIn            = c->lengthIn / n;              // cells fit the length exactly
    m->nominalLayerAddIn = 4.0f * c->towThicknessIn;     // two plies, both sides of the diameter
    m->layerAddIn        = m->nominalLayerAddIn;
    m->layersDone        = 0;
    m->measureDone       = 0;
    m->fullLayersDone    = 0;
    memset(m->flags, 0, sizeof(m->flags));

    for (int i = 0; i < n; i++) {
        float f = (i + 0.5f) / n;                        // 0 at the right end, 1 at the left
        m->target[i] = c->largeDiaIn + (c->smallDiaIn - c->largeDiaIn) * f;
        m->est[i]    = c->mandrelDiaIn;
        m->var[i]    = c->initialSigmaIn * c->initialSigmaIn;
    }
    modelClearSamples(m);
    return 1;
}

void modelClearSamples(WrapModel *m) {
    memset(m->sumDy, 0, sizeof(m->sumDy));
    memset(m->sumDe, 0, sizeof(m->sumDe));
    memset(m->nSamp, 0, sizeof(m->nSamp));
    m->hoopDia[END_RIGHT]  = m->hoopDia[END_LEFT]  = -1.0f;
    m->hoopRevs[END_RIGHT] = m->hoopRevs[END_LEFT] = 0.0f;
}


// ---------------------------------------------------------------------------
// Planning the next layer
// ---------------------------------------------------------------------------

// After the measurement: mark the cells with the least room.  A single pass is
// noisy cell by cell, so look for the REF_WINDOW-cell stretch with the least
// average room (ridge cells left out).  Those cells size the full-length finish
// for the rest of the wrap.  A genuine low spot that wide still counts.
#define REF_WINDOW 5
static void markReferenceCells(WrapModel *m) {
    const int half = REF_WINDOW / 2;
    float best = 1e9f;
    int   bestAt = m->nCells - 1;
    for (int i = 0; i < m->nCells; i++) {
        float sum = 0.0f;
        int   n = 0;
        for (int j = i - half; j <= i + half; j++) {
            if (j < 0 || j >= m->nCells || (m->flags[j] & CELL_RIDGE)) continue;
            sum += m->target[j] - m->est[j];
            n++;
        }
        if (n >= 3 && sum / n < best) { best = sum / n; bestAt = i; }
    }
    for (int j = bestAt - half; j <= bestAt + half; j++)
        if (j >= 0 && j < m->nCells) m->flags[j] |= CELL_REFERENCE;
}

// K: how many full-length layers still fit, from the average room left on the
// reference cells (ridges left out if possible), rounded to the nearest layer.
static int layersLeft(const WrapModel *m) {
    float sum = 0.0f, sumAll = 0.0f;
    int   n = 0, nAll = 0;
    for (int i = 0; i < m->nCells; i++) {
        if (!(m->flags[i] & CELL_REFERENCE)) continue;
        float need = m->target[i] - m->est[i];
        sumAll += need; nAll++;
        if (!(m->flags[i] & CELL_RIDGE)) { sum += need; n++; }
    }
    if (n == 0) { sum = sumAll; n = nAll; }
    if (n == 0) return 0;
    float need = sum / n;
    if (need <= 0.0f) return 0;
    return (int)(need / m->layerAddIn + 0.5f);
}

// The first run of cells (from the right), at least MIN_RUN_IN long, that is
// at least `fraction` of a layer short of "target - reserve".  Shorter runs are
// skipped: each fill layer adds hoop wraps at both of its turnarounds, so a
// short one does more harm than good -- and it must not hide a real gap further on.
#define MIN_RUN_IN 1.0f
static void layerExtent(const WrapModel *m, float reserve, float fraction, int *first, int *end) {
    float threshold = fraction * m->layerAddIn;
    int   minRun    = (int)(MIN_RUN_IN / m->cellIn + 0.5f);
    if (minRun < 2) minRun = 2;
    int i = 0;
    *first = *end = 0;
    while (i < m->nCells) {
        while (i < m->nCells && (m->target[i] - reserve - m->est[i]) <  threshold) i++;
        int start = i;
        while (i < m->nCells && (m->target[i] - reserve - m->est[i]) >= threshold) i++;
        if (i - start >= minRun) { *first = start; *end = i; return; }
    }
}

// Decide what the next layer is: measurement, fill (and where), full, or done.
static int chooseLayer(const WrapModel *m, const WrapConfig *c, LayerPlan *p) {
    p->layersLeft = layersLeft(m);

    if (!m->measureDone) {
        p->kind = LAYER_MEASURE;
        p->firstCell = 0;
        p->endCell   = m->nCells;
        return 1;
    }
    if (m->layersDone >= c->maxLayers) return 0;

    int   k       = c->fullFinish ? p->layersLeft : 0;
    float reserve = k * m->layerAddIn;
    // Before the finish: fill anything half a layer behind.  Once full-length
    // layers have started: only touch up stretches a whole layer behind.
    float fraction = m->fullLayersDone ? 1.0f : WIND_IF_NEED_FRACTION;
    int   first, end;
    layerExtent(m, reserve, fraction, &first, &end);
    if (end > first) {
        p->kind = LAYER_FILL;
        p->firstCell = first;
        p->endCell   = end;
        return 1;
    }
    if (k >= 1) {
        p->kind = LAYER_FULL;
        p->firstCell = 0;
        p->endCell   = m->nCells;
        return 1;
    }
    return 0;
}

int modelPlanLayer(const WrapModel *m, const WrapConfig *c, LayerPlan *p) {
    if (!chooseLayer(m, c, p)) return PLAN_COMPLETE;
    const int first = p->firstCell, end = p->endCell;

    // Fiber sits, on average, half a layer above the current surface.
    float midLayer = 0.5f * m->layerAddIn;
    float nw       = c->circuits * c->towWidthIn * (1.0f - c->overlap);

    // 1. Lead per revolution for each cell (stored temporarily in carriageSpeed[]).
    float maxLead = 0.0f;
    for (int i = first; i < end; i++) {
        float circ = PI_F * (m->est[i] + midLayer);
        if (nw >= 0.95f * circ) return PLAN_IMPOSSIBLE;
        float lead = leadPerRev(circ, nw);
        p->carriageSpeed[i] = lead;
        if (lead > maxLead) maxLead = lead;
    }

    // 2. Barrel speed: as configured, unless that makes the carriage too fast.
    //    The carriage must also be able to stop within half a cell, because it
    //    changes speed at every cell boundary (see runPass in wrap.c).
    float cellSteps   = m->cellIn * STEPS_PER_INCH;
    float maxCarriage = c->maxCarriageIps * STEPS_PER_INCH;               // steps/s
    float stopLimit   = sqrtf(STEPPER_ACCELERATION * cellSteps);          // v^2/2a = half a cell
    if (stopLimit < maxCarriage) maxCarriage = stopLimit;

    float rps = c->barrelRps;
    if (maxLead * rps * STEPS_PER_INCH > maxCarriage)
        rps = maxCarriage / (maxLead * STEPS_PER_INCH);
    p->barrelSpeed = rps * STEPS_PER_REVOLUTION;

    // 3. Carriage speed, and how much fiber should come off the spool, per cell.
    for (int i = first; i < end; i++) {
        float lead  = p->carriageSpeed[i];
        float circ  = PI_F * (m->est[i] + midLayer);
        float ratio = circ / lead;                                   // around per inch of travel
        float fiber = m->cellIn * sqrtf(1.0f + ratio * ratio);       // fiber per cell, inches
        p->carriageSpeed[i] = lead * rps * STEPS_PER_INCH;
        p->expectDe[i]      = (int32_t)(fiber * COUNT_PER_INCH);
    }

    float circRight = PI_F * (m->est[first]   + midLayer);
    float circLeft  = PI_F * (m->est[end - 1] + midLayer);
    // The measurement pass only knows the mandrel guess: just catch "no fiber at all".
    float minFraction = (p->kind == LAYER_MEASURE) ? 0.125f : FIBER_MIN_FRACTION;
    p->fiberShift = (p->kind == LAYER_MEASURE) ? 3 : 1;
    p->minDePerRev[END_RIGHT] = (int32_t)(minFraction * circRight * COUNT_PER_INCH);
    p->minDePerRev[END_LEFT]  = (int32_t)(minFraction * circLeft  * COUNT_PER_INCH);

    // The measurement is a single out-and-back at the normal helix angle.
    p->circuits  = (p->kind == LAYER_MEASURE) ? 1 : c->circuits;
    p->gateShift = (p->kind == LAYER_MEASURE) ? 2 : 3;   // 25% vs the mandrel guess, then 12.5%
    return end - first;
}


// ---------------------------------------------------------------------------
// End-of-layer update (Kalman filter, one independent filter per cell)
// ---------------------------------------------------------------------------

// The reading for one cell, as "surface diameter after this layer", or -1.
// `add` is how much this layer is expected to add to the diameter.
static float cellReading(const WrapModel *m, const WrapConfig *c, const LayerPlan *p, int i,
                         float add, float *weight) {
    *weight = 0.0f;
    // End cells: the dwell (hoop) reading.  Centreline + half a ply ~ surface.
    if (i == p->firstCell && m->hoopDia[END_RIGHT] > 0.0f) {
        *weight = 1.0f;
        return m->hoopDia[END_RIGHT] + c->towThicknessIn;
    }
    if (i == p->endCell - 1 && m->hoopDia[END_LEFT] > 0.0f) {
        *weight = 1.0f;
        return m->hoopDia[END_LEFT] + c->towThicknessIn;
    }
    // Body cells: averaged helical readings (the diameter at the fiber's centre).
    if (m->nSamp[i] == 0) return -1.0f;
    float n = m->nSamp[i];
    float d = diameterFromReading(m->cellIn, m->sumDy[i] / n, m->sumDe[i] / n);
    if (d <= 0.0f) return -1.0f;
    *weight = n;
    if (p->kind == LAYER_MEASURE)
        return d - c->towThicknessIn + add;   // laid on the bare surface: centre = surface + 1 ply
    return d + 0.5f * add;                    // laid, on average, half a layer below the final surface
}

// Readings for every wound cell (z <= 0: none).  Then drop body readings that
// disagree with their neighbours: the surface is smooth over an inch, so a lone
// jump is a slip or a bad sample, not the barrel.  Returns how many were dropped.
#define NEIGHBOURS  2                     // cells each side
static int gatherReadings(const WrapModel *m, const WrapConfig *c, const LayerPlan *p,
                          float add, float *z, float *w) {
    static uint8_t drop[WRAP_MAX_CELLS];
    const int first = p->firstCell, end = p->endCell;
    const float limit = 4.0f * c->measSigmaIn;
    int dropped = 0;

    for (int i = first; i < end; i++) {
        z[i] = cellReading(m, c, p, i, add, &w[i]);
        drop[i] = 0;
    }
    for (int i = first + 1; i < end - 1; i++) {           // body cells only
        if (z[i] <= 0.0f) continue;
        float nb[2 * NEIGHBOURS];
        int   n = 0;
        for (int j = i - NEIGHBOURS; j <= i + NEIGHBOURS; j++) {
            if (j == i || j <= first || j >= end - 1 || z[j] <= 0.0f) continue;
            nb[n++] = z[j];
        }
        if (n < 2) continue;
        for (int a = 1; a < n; a++)                         // tiny insertion sort
            for (int b = a; b > 0 && nb[b - 1] > nb[b]; b--) { float t = nb[b]; nb[b] = nb[b - 1]; nb[b - 1] = t; }
        float median = (n & 1) ? nb[n / 2] : 0.5f * (nb[n / 2 - 1] + nb[n / 2]);
        if (fabsf(z[i] - median) > limit) { drop[i] = 1; dropped++; }
    }
    for (int i = first; i < end; i++)
        if (drop[i]) z[i] = -1.0f;
    return dropped;
}

void modelUpdate(WrapModel *m, const WrapConfig *c, const LayerPlan *p, ModelStats *st) {
    static float   correction[WRAP_MAX_CELLS];   // static: keep it off the small stack
    static uint8_t measured[WRAP_MAX_CELLS];
    static float   reading[WRAP_MAX_CELLS], weightOf[WRAP_MAX_CELLS];

    const int   first = p->firstCell, end = p->endCell;
    // One out-and-back covers 1/circuits of the surface: that much of a layer.
    const float add   = (p->kind == LAYER_MEASURE) ? m->layerAddIn / c->circuits : m->layerAddIn;
    const float qBody = c->processSigmaIn * c->processSigmaIn;
    const float qEnd  = (END_CELL_NOISE_LAYERS * add) * (END_CELL_NOISE_LAYERS * add);
    const float r1    = c->measSigmaIn * c->measSigmaIn;
    const float gate2 = c->gateSigmas * c->gateSigmas;

    float growthSum = 0.0f;
    int   growthN = 0, accepted = 0;
    int   rejected = gatherReadings(m, c, p, add, reading, weightOf);

    // 1. Predict every wound cell, then correct the ones with a reading.
    for (int i = first; i < end; i++) {
        int   isEnd  = (i == first || i == end - 1);
        float before = m->est[i];
        float pred   = before + add;
        float var    = m->var[i] + (isEnd ? qEnd : qBody);
        float weight = weightOf[i];
        float z      = reading[i];

        correction[i] = 0.0f;
        measured[i]   = 0;
        if (z > 0.0f) {
            float innov = z - pred;
            float s     = var + r1 / weight;
            if (innov * innov <= gate2 * s) {
                float k = var / s;
                correction[i] = k * innov;
                var = (1.0f - k) * var;
                measured[i] = isEnd ? 2 : 1;
                accepted++;
                if (!isEnd) { growthSum += z - before; growthN++; }
            } else {
                // Rejected.  If the estimate (not the reading) is what's wrong, the
                // next reading will disagree again -- widen the uncertainty so a
                // repeat is believed instead of being rejected forever.
                float wide = 0.25f * innov * innov;      // innovation = 2 sigma
                if (var < wide) var = wide;
                rejected++;
            }
        }
        m->est[i] = pred;
        m->var[i] = var;
    }

    // 2. Apply corrections.  Cells without a reading borrow the correction of the
    //    nearest measured BODY cell (end cells grow differently: hoop wraps).
    for (int i = first; i < end; i++) {
        if (measured[i]) { m->est[i] += correction[i]; continue; }
        for (int d = 1; d < end - first; d++) {
            if (i - d >= first && measured[i - d] == 1) { m->est[i] += correction[i - d]; break; }
            if (i + d <  end   && measured[i + d] == 1) { m->est[i] += correction[i + d]; break; }
        }
    }

    // 3. The tow straddles each turnaround, so the cell just outside the layer
    //    gets half of every dwell band.  It is not measured -- predict it.
    //    (Half a band of 2 plies per turn, averaged over the cell = 1 ply per turn.)
    float ply = c->towThicknessIn * (m->layerAddIn / m->nominalLayerAddIn);
    if (first - 1 >= 0) {
        float g = ply * m->hoopRevs[END_RIGHT];
        m->est[first - 1] += g;
        m->var[first - 1] += 0.25f * g * g;
    }
    if (end < m->nCells) {
        float g = ply * m->hoopRevs[END_LEFT];
        m->est[end] += g;
        m->var[end] += 0.25f * g * g;
    }

    // 4. If (nearly) everything was rejected the starting guess was probably far off:
    //    widen the uncertainty so the next layer's readings are believed.
    if (rejected >= 3 && accepted < rejected) {
        for (int i = first; i < end; i++)
            m->var[i] = 4.0f * c->initialSigmaIn * c->initialSigmaIn;
    }

    // 5. Learn how much a layer really adds (tow compaction, tension, resin ...).
    //    Body cells only, and not on the first layer ("before" is still a guess then).
    if (growthN >= 3 && m->layersDone >= 1 && p->kind != LAYER_MEASURE) {
        float observed = growthSum / growthN;
        float learned  = m->layerAddIn + GROWTH_LEARN_GAIN * (observed - m->layerAddIn);
        float lo = 0.5f * m->nominalLayerAddIn, hi = 2.0f * m->nominalLayerAddIn;
        m->layerAddIn = learned < lo ? lo : (learned > hi ? hi : learned);
    }

    // 6. Remember which cells now carry turnaround hoop wraps.
    m->flags[first] |= CELL_RIDGE;
    m->flags[end - 1] |= CELL_RIDGE;
    if (first - 1 >= 0)  m->flags[first - 1] |= CELL_RIDGE;
    if (end < m->nCells) m->flags[end]       |= CELL_RIDGE;

    if (p->kind == LAYER_MEASURE) { m->measureDone = 1; markReferenceCells(m); }
    else                          m->layersDone++;
    if (p->kind == LAYER_FULL)    m->fullLayersDone++;
    modelClearSamples(m);

    if (st) {
        st->measured   = accepted;
        st->rejected   = rejected;
        st->layerAddIn = m->layerAddIn;
    }
}
