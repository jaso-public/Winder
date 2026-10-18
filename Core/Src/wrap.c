/*
 * wrap.c
 *
 * wrap(): wind carbon tow onto the barrel, layer by layer, until the outside
 * is a cone from largeDiaIn (right end) to smallDiaIn (left end).
 *
 * Call tree
 * ---------
 *   wrap()
 *     prepare()                 homed?  settings  first plan  move to the right end  operator starts
 *     windAllLayers()           barrel runs continuously from here until the end
 *       windLayer()             one layer = `circuits` out-and-back passes on a fixed rhythm
 *         dwellUntil()          wait at an end (>= dwellRevs turns), measure the hoop wraps
 *         runPass()             traverse cell by cell: speed per cell, sample, check fiber
 *       modelUpdate()           (wrap_model.c) diameter estimate, once per layer
 *       modelPlanLayer()        (wrap_model.c) where and how fast the next layer goes
 *     stopAll() / uiShowResult()
 *
 * Timing rules
 * ------------
 *  - While the carriage is moving (runPass) only integer bookkeeping is done:
 *    no printf, no LCD, no float maths beyond one compare per cell.
 *  - LCD and serial output happen while dwelling at an end.  If they make us
 *    late for a departure we wait one more whole turn, which keeps the barrel
 *    angle -- and therefore the coverage pattern -- exact.
 *  - All the estimation and planning happens once per layer, at the right end.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "stepper.h"
#include "winder.h"
#include "wrap.h"
#include "wrap_model.h"
#include "wrap_ui.h"

extern Stepper barrelStepper;
extern Stepper carriageStepper;

// Which way is "left" in carriage steps?  Homing puts position 0 at the optical
// sensor and every position after that is positive; the Move Carriage screen
// labels the positive direction "Left".  So the left (small) end is further from
// home than the right end.  Change to (-1) if that is backwards on the machine.
#define LEFT_DIR            (+1)

#define AT_SPEED_FRACTION   0.9f                         // use a cell's reading only if at >= 90% speed
#define FIBER_OUT_CELLS     2                            // this many low-fiber cells in a row = fault
#define PARK_SPEED          (STEPS_PER_INCH / 2)         // carriage moves when not winding
#define LATE_MARGIN         (STEPS_PER_REVOLUTION / 100) // never schedule a departure closer than this
#define DWELL_SLACK         (STEPS_PER_REVOLUTION / 50)  // a little extra dwell to absorb jitter

volatile uint8_t wrapLimitTripped = 0;

static WrapConfig cfg;            // kept between wraps so edits are remembered until power-off
static int        cfgLoaded = 0;
static WrapModel  model;
static LayerPlan  plan;
static char       faultDetail[21];

// A moment during the wrap: where the barrel was and how much fiber had gone out.
typedef struct {
    int32_t barrel;
    int32_t enc;
} Mark;


// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------

// Signed distance from a to b, correct even if the counters wrap around.
static inline int32_t stepsBetween(int32_t a, int32_t b) { return (int32_t)((uint32_t)b - (uint32_t)a); }
static inline int32_t stepsAdd(int32_t a, int32_t d)     { return (int32_t)((uint32_t)a + (uint32_t)d); }

static Mark markNow(void) {
    Mark m = { getPosition(&barrelStepper), readEncoderValue() };
    return m;
}

// Carriage position (steps) of cell boundary b.  Boundary 0 = right end of the wrap.
static int32_t boundaryPos(int b) {
    float rightIn = cfg.rightEndIn;
    float fromRight = b * model.cellIn;
    return (int32_t)(rightIn * STEPS_PER_INCH + 0.5f) + LEFT_DIR * (int32_t)(fromRight * STEPS_PER_INCH + 0.5f);
}

// Checked continuously in every wait loop.  Cheap: a flag, two pins, a tick compare.
static WrapStatus pollSafety(void) {
    if (wrapLimitTripped)      return WRAP_LIMIT_SWITCH;
    if (isLeft() && isRight()) return WRAP_ABORTED;
    lightsService();
    return WRAP_OK;
}

static void stopAll(void) {
    stop(&carriageStepper);
    stop(&barrelStepper);
    waitUntilStopped(&carriageStepper);
    waitUntilStopped(&barrelStepper);
}

static WrapStatus waitCarriageStopped(void) {
    while (carriageStepper.mode != STOPPED) {
        WrapStatus s = pollSafety();
        if (s) return s;
    }
    return WRAP_OK;
}

// Move the carriage without winding considerations (positioning).
static WrapStatus moveCarriageTo(int32_t pos, float speed) {
    if (getPosition(&carriageStepper) == pos) return WRAP_OK;
    moveToPosition(&carriageStepper, speed, pos);
    return waitCarriageStopped();
}

// Wait until the barrel reaches an absolute position.
static WrapStatus waitForBarrel(int32_t target) {
    while (stepsBetween(getPosition(&barrelStepper), target) > 0) {
        WrapStatus s = pollSafety();
        if (s) return s;
    }
    return WRAP_OK;
}

static int gcd(int a, int b) {
    while (b) { int t = a % b; a = b; b = t; }
    return a;
}


// ---------------------------------------------------------------------------
// Turnarounds
// ---------------------------------------------------------------------------

// Make a planned departure safe: at least dwellRevs after arriving, and not in
// the past.  Only ever moves it by whole turns, so the barrel angle is kept.
static int32_t safeDeparture(int32_t planned, Mark arrived) {
    const int32_t minDwell = (int32_t)(cfg.dwellRevs * STEPS_PER_REVOLUTION);
    while (stepsBetween(arrived.barrel, planned) < minDwell ||
           stepsBetween(getPosition(&barrelStepper), planned) < LATE_MARGIN)
        planned = stepsAdd(planned, STEPS_PER_REVOLUTION);
    return planned;
}

// Dwell at one end until the barrel reaches `leaveAt`.  The fiber laid while
// dwelling went straight around (hoop), so it measures the end diameter, and it
// also proves fiber is still coming off the spool.
static WrapStatus dwellUntil(int end, Mark arrived, int32_t leaveAt) {
    WrapStatus s = waitForBarrel(leaveAt);
    if (s) return s;

    Mark now   = markNow();
    int32_t dy = stepsBetween(arrived.barrel, now.barrel);
    int32_t de = abs(stepsBetween(arrived.enc, now.enc));
    float revs = (float)dy / STEPS_PER_REVOLUTION;

    if (de < plan.minDePerRev[end] * revs) {
        long need = (long)(plan.minDePerRev[end] * revs);
        snprintf(faultDetail, sizeof(faultDetail), "%c dwell %ld<%ld", end == END_RIGHT ? 'R' : 'L', (long)de, need);
        printf("[wrap] FIBER: dwell at %s end, %ld barrel steps, encoder %ld counts, needed %ld\r\n",
               end == END_RIGHT ? "right" : "left", (long)dy, (long)de, need);
        return WRAP_FIBER_OUT;
    }
    modelHoopReading(&model, end, dy, de);
    return WRAP_OK;
}


// ---------------------------------------------------------------------------
// One pass
// ---------------------------------------------------------------------------

// Traverse from boundary `from` to boundary `to`.  At every cell boundary:
// read the barrel and encoder, check fiber is paying out, keep the reading if
// the carriage was at full speed across the cell, and set the next cell's speed.
static WrapStatus runPass(int from, int to, Mark *arrived) {
    const int step   = (to > from) ? 1 : -1;            // +1 = outbound, toward the left end
    const int posDir = step * LEFT_DIR;                 // carriage step direction
    int  cell        = (step > 0) ? from : from - 1;
    int  lowFiber    = 0;
    int  wasAtSpeed  = 0;                               // starting from rest
    Mark last        = markNow();

    moveToPosition(&carriageStepper, plan.carriageSpeed[cell], boundaryPos(to));

    for (int b = from + step; ; b += step) {
        const int32_t edge = boundaryPos(b);
        while ((getPosition(&carriageStepper) - edge) * posDir < 0) {
            WrapStatus s = pollSafety();
            if (s) return s;
        }

        // ---- at a cell boundary: integer bookkeeping only ----
        Mark    now     = markNow();
        int     atSpeed = carriageStepper.currentSpeed >= AT_SPEED_FRACTION * plan.carriageSpeed[cell];
        int32_t dy      = stepsBetween(last.barrel, now.barrel);
        int32_t de      = abs(stepsBetween(last.enc, now.enc));

        if (fiberLow(&plan, cell, de)) {
            if (++lowFiber >= FIBER_OUT_CELLS) {
                long need = (long)(plan.expectDe[cell] >> plan.fiberShift);
                snprintf(faultDetail, sizeof(faultDetail), "cell %d %ld<%ld", cell, (long)de, need);
                printf("[wrap] FIBER: pass, cell %d, %ld barrel steps, encoder %ld counts, needed %ld (expected %ld)\r\n",
                       cell, (long)dy, (long)de, need, (long)plan.expectDe[cell]);
                return WRAP_FIBER_OUT;
            }
        } else {
            lowFiber = 0;
        }
        if (atSpeed && wasAtSpeed) modelOfferSample(&model, &plan, cell, dy, de);

        last       = now;
        wasAtSpeed = atSpeed;
        if (b == to) break;

        cell += step;
        changeSpeed(&carriageStepper, plan.carriageSpeed[cell]);
    }

    WrapStatus s = waitCarriageStopped();               // it stops exactly on the end boundary
    *arrived = markNow();
    return s;
}


// ---------------------------------------------------------------------------
// One layer
// ---------------------------------------------------------------------------

// A layer is plan.circuits out-and-back passes.  Every circuit takes exactly the
// same number of barrel steps, `period` = q/circuits of a turn, with q sharing no
// factor with circuits.  So circuit k starts k*q/circuits of a turn after circuit
// 0 and the starts land on each of the `circuits` evenly-spaced angles exactly
// once: the passes lie side by side with no gaps.  The left-end departure is a
// fixed time after each start, so the return passes interleave the same way.
// Circuit 0 is used to measure how long each pass takes (in barrel steps); q is
// then the smallest that still gives >= dwellRevs at both ends.
static WrapStatus windLayer(int layer, Mark *at, int32_t firstDwell) {
    const int     n        = plan.circuits;
    const int32_t minDwell = (int32_t)(cfg.dwellRevs * STEPS_PER_REVOLUTION) + DWELL_SLACK;
    int32_t start0 = 0, outSteps = 0;
    int     q = 0;
    Mark    arrived = *at;
    WrapStatus s;

    for (int k = 0; k < n; k++) {
        uiShowCircuit(layer, k, n);                     // slow LCD write: do it before scheduling

        // ---- right end: wait for this circuit's start angle ----
        int32_t start;
        if (k == 0) {
            start = stepsAdd(arrived.barrel, firstDwell);
            if (stepsBetween(getPosition(&barrelStepper), start) < LATE_MARGIN)
                start = stepsAdd(getPosition(&barrelStepper), LATE_MARGIN);
            start0 = start;
        } else {
            int32_t offset = (int32_t)(((int64_t)k * q * STEPS_PER_REVOLUTION) / n);
            start = safeDeparture(stepsAdd(start0, offset), arrived);
        }
        if ((s = dwellUntil(END_RIGHT, arrived, start))) return s;

        // ---- outbound: right end -> left end ----
        if ((s = runPass(plan.firstCell, plan.endCell, &arrived))) return s;
        if (k == 0) outSteps = stepsBetween(start, arrived.barrel);

        // ---- left end: leave a fixed time after this circuit started ----
        int32_t back = safeDeparture(stepsAdd(start, outSteps + minDwell), arrived);
        if ((s = dwellUntil(END_LEFT, arrived, back))) return s;

        // ---- return: left end -> right end ----
        if ((s = runPass(plan.endCell, plan.firstCell, &arrived))) return s;

        if (k == 0 && n > 1) {
            int32_t backSteps = stepsBetween(back, arrived.barrel);
            int64_t needed    = (int64_t)outSteps + minDwell + backSteps + minDwell;
            q = (int)((needed * n + STEPS_PER_REVOLUTION - 1) / STEPS_PER_REVOLUTION);
            while (gcd(q, n) != 1) q++;
            printf("[wrap] layer %d: out %ld back %ld steps, circuit = %d/%d turn\r\n",
                   layer, (long)outSteps, (long)backSteps, q, n);
        }
    }
    *at = arrived;
    return WRAP_OK;
}


// ---------------------------------------------------------------------------
// All layers
// ---------------------------------------------------------------------------

static void logLayer(int layer, const ModelStats *st) {
    static const char *kind[] = { "measure", "fill", "full" };
    char add[10], r[10], l[10], rps[10];
    fmtFixed(add, sizeof(add), st->layerAddIn, 4);
    fmtFixed(r,   sizeof(r),   model.est[0], 3);
    fmtFixed(l,   sizeof(l),   model.est[model.nCells - 1], 3);
    fmtFixed(rps, sizeof(rps), plan.barrelSpeed / STEPS_PER_REVOLUTION, 2);
    printf("[wrap] layer %d (%s) done: cells %d-%d rps %s  measured %d rejected %d  growth/layer %s  dia R %s L %s  full layers left %d\r\n",
           layer, kind[plan.kind], plan.firstCell, plan.endCell - 1, rps, st->measured, st->rejected, add, r, l,
           plan.layersLeft);
}

static WrapStatus windAllLayers(int *layerOut) {
    const int32_t tieSteps = (int32_t)(cfg.tieRevs  * STEPS_PER_REVOLUTION);
    const int32_t minDwell = (int32_t)(cfg.dwellRevs * STEPS_PER_REVOLUTION) + DWELL_SLACK;
    WrapStatus s;

    lightsSet(LIGHT_WINDING);
    stepperStart(&barrelStepper, plan.barrelSpeed, CW);
    Mark at = markNow();                                 // anchor wraps count as the first dwell

    for (int layer = 1; ; layer++) {
        *layerOut = layer;
        uiShowLayer(layer, &model, &plan);
        if ((s = windLayer(layer, &at, layer == 1 ? tieSteps : minDwell))) return s;

        // ---- end of layer, at the right end: estimate, then plan the next ----
        ModelStats st;
        modelUpdate(&model, &cfg, &plan, &st);
        logLayer(layer, &st);
        int r = modelPlanLayer(&model, &cfg, &plan);
        if (r == PLAN_IMPOSSIBLE) {
            snprintf(faultDetail, sizeof(faultDetail), "Tow too wide/passes");
            return WRAP_CONFIG_ERROR;
        }
        if (r == PLAN_COMPLETE) break;

        changeSpeed(&barrelStepper, plan.barrelSpeed);

        // The next layer may turn around further in: move there (laying fiber as we go).
        int32_t newRight = boundaryPos(plan.firstCell);
        if (getPosition(&carriageStepper) != newRight) {
            if ((s = moveCarriageTo(newRight, plan.carriageSpeed[plan.firstCell]))) return s;
            at = markNow();
        }
    }

    // Finished: tie-off hoop wraps where we stand, then stop the barrel.
    uiMessage("Tying off", NULL, NULL, NULL);
    if ((s = waitForBarrel(stepsAdd(at.barrel, tieSteps)))) return s;
    stop(&barrelStepper);
    waitUntilStopped(&barrelStepper);
    return WRAP_COMPLETE;
}


// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

static const char *checkSettings(void) {
    const char *problem = wrapCheckConfig(&cfg);
    if (problem) return problem;

    int32_t maxPos = (int32_t)(CARRIAGE_MAX_TRAVEL_IN * STEPS_PER_INCH);
    int32_t right  = (int32_t)(cfg.rightEndIn * STEPS_PER_INCH);
    int32_t left   = right + LEFT_DIR * (int32_t)(cfg.lengthIn * STEPS_PER_INCH);
    if (right < 0 || left < 0 || right > maxPos || left > maxPos) return "Past carriage travel";
    return 0;
}

static WrapStatus prepare(void) {
    lightsSet(LIGHT_SETUP);

    if (!carriageLocationKnown) {
        lightsSet(LIGHT_WAITING);
        uiMessage("Wrap", "Carriage not homed", "C = home now", "L+R = cancel");
        if (!uiWaitForCenter()) return WRAP_ABORTED;
        lightsSet(LIGHT_SETUP);
        homeCarriage();
        if (!carriageLocationKnown) return WRAP_NOT_HOMED;
    }

    if (!uiEditConfig(&cfg)) return WRAP_ABORTED;

    const char *problem = checkSettings();
    if (problem) {
        snprintf(faultDetail, sizeof(faultDetail), "%s", problem);
        return WRAP_CONFIG_ERROR;
    }

    modelInit(&model, &cfg);
    int r = modelPlanLayer(&model, &cfg, &plan);
    if (r <= 0) {
        snprintf(faultDetail, sizeof(faultDetail), r == PLAN_COMPLETE ? "Nothing to wind" : "Tow too wide");
        return WRAP_CONFIG_ERROR;
    }

    lightsSet(LIGHT_SETUP);
    uiMessage("Wrap", "Moving to right end", "Stand clear", NULL);
    WrapStatus s = moveCarriageTo(boundaryPos(plan.firstCell), PARK_SPEED);
    if (s) return s;

    lightsSet(LIGHT_WAITING);
    uiMessage("Attach fiber at the", "right end of barrel", "C = start winding", "L+R = cancel");
    if (!uiWaitForCenter()) return WRAP_ABORTED;
    return WRAP_OK;
}


// ---------------------------------------------------------------------------
// wrap()
// ---------------------------------------------------------------------------

WrapStatus wrap(void) {
    if (!cfgLoaded) {
        wrapDefaultConfig(&cfg);
        cfgLoaded = 1;
    }
    faultDetail[0]   = 0;
    wrapLimitTripped = 0;

    // The barrel must be at rest before stepperStart() (it halts otherwise).
    stop(&barrelStepper);
    waitUntilStopped(&barrelStepper);

    int layer = 0;
    WrapStatus s = prepare();
    if (s == WRAP_OK) s = windAllLayers(&layer);

    if (s != WRAP_COMPLETE) stopAll();
    if (s == WRAP_LIMIT_SWITCH) carriageLocationKnown = 0;   // drives were cut: steps were lost

    if (s == WRAP_ABORTED && layer == 0) return s;            // cancelled during setup: back to menu
    uiShowResult(s, layer, faultDetail);
    return s;
}
