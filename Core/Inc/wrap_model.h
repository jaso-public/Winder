/*
 * wrap_model.h
 *
 * The "brains" of wrap(): pure math, no hardware access, so it can be read
 * (and tested on a PC) on its own.
 *
 * Coordinates
 * -----------
 * The wrap length is divided into cells (default 0.25").  Cell 0 is at the
 * RIGHT (large) end, cell nCells-1 is at the LEFT (small) end.  Boundary b is
 * the edge between cell b-1 and cell b, so boundary 0 is the right end of the
 * wrap and boundary nCells is the left end.  A layer covers cells
 * firstCell .. endCell-1, turning around at boundaries firstCell and endCell.
 *
 * How a layer is laid
 * -------------------
 *  - A layer is `circuits` out-and-back passes.  Passes in the same direction
 *    are identical helices rotated 1/circuits of a turn apart, so together they
 *    lay one gap-free ply in each direction (two plies per layer).
 *  - For that to be gap-free the carriage must advance a specific lead per
 *    barrel revolution, which depends on the local circumference, so the
 *    carriage speed is set per cell from that cell's estimated diameter.
 *  - At each end the barrel keeps turning at least `dwellRevs` before the
 *    carriage reverses.  Those hoop wraps build the end bands up faster than
 *    the helical body, so the model tracks the end cells separately (it
 *    measures them during the dwell) and simply stops turning around on a
 *    cell once it has reached its diameter.
 *
 * Building the cone: measure, fill, finish
 * ----------------------------------------
 *  Each cell has a target diameter on a straight line from largeDiaIn (right)
 *  to smallDiaIn (left).  The wrap goes through three phases:
 *
 *  1. MEASURE  one full-length out-and-back pass.  Its readings replace the
 *              mandrel guess with the real starting profile.
 *  2. FILL     partial layers that build the taper early.  K = how many
 *              full-length layers still fit at the place with the least room
 *              (normally the small end).  Every cell is filled up to
 *              "target - K layers", so after filling the surface is already
 *              the cone shape, just K layers undersize.  A fill layer covers
 *              the first run of cells (from the right) still at least half a
 *              layer short of that.
 *  3. FULL     full-length layers, K of them, that bring the whole cone up to
 *              size together, so the outside layers are continuous end to end.
 *              If the shape drifts (a stretch of at least 1" falls a whole
 *              layer behind), a fill layer is slipped in before the next one.
 *
 *  K is judged on the "reference" cells: the ones with the least room after the
 *  measurement (within one layer of the minimum -- normally the small end).
 *  Averaging over them, and leaving out cells under a turnaround (hoop-wrap
 *  ridges), keeps one noisy or proud cell from cutting the finish short.
 *
 * Diameter estimate (once per layer, a small Kalman filter per cell)
 * -----------------------------------------------------------------
 *  - Prediction: every wound cell grew by `layerAddIn` (starts at 4 x tow
 *    thickness and is re-learned from the measurements).
 *  - Body cells: at every cell boundary wrap.c records how far the barrel
 *    turned (dy) and how much fiber went out (de).  Fiber, carriage travel and
 *    the distance around the barrel form a right triangle -> circumference.
 *  - End cells: during the dwell the fiber goes straight around, so
 *    fiber-per-turn is the circumference.
 *  - Readings far from the prediction are rejected; cells with no reading
 *    borrow the correction of their nearest measured neighbour.
 */

#ifndef INC_WRAP_MODEL_H_
#define INC_WRAP_MODEL_H_

#include <stdint.h>
#include "machine.h"

#define WRAP_MAX_CELLS     96   // 96 x 0.25" = 24", more than the carriage travel
#define WRAP_MAX_CIRCUITS  12

#define END_RIGHT 0
#define END_LEFT  1

#define CELL_RIDGE      0x01   // has had turnaround (hoop) wraps
#define CELL_REFERENCE  0x02   // least room after the measurement: sizes the full-length finish

// Everything the operator (or code) can configure for a wrap.  Lengths in inches.
typedef struct {
    // -- shape --
    float rightEndIn;        // carriage distance from home to the right (large) end
    float lengthIn;          // wrap length, right end to left end
    float largeDiaIn;        // finished diameter at the right end
    float smallDiaIn;        // finished diameter at the left end
    float mandrelDiaIn;      // bare barrel diameter: starting guess, refined by measurement
    int   fullFinish;        // 1 = fill the taper first, finish with full-length layers

    // -- fiber --
    float towWidthIn;        // flat width of the tow as it lands
    float towThicknessIn;    // thickness of one ply
    float overlap;           // fraction of tow width to overlap (0.05 = 5%) so errors leave no gaps

    // -- pattern --
    int   circuits;          // out-and-back passes per layer (4-5 typical)
    float dwellRevs;         // minimum barrel turns at each end before reversing (0.75)
    float tieRevs;           // hoop wraps at the start and at the end to anchor the fiber

    // -- speed --
    float barrelRps;         // barrel speed; lowered automatically if the carriage would be too fast
    float maxCarriageIps;    // carriage speed limit, inches/second

    // -- estimation --
    float cellIn;            // cell size along the barrel
    float initialSigmaIn;    // how uncertain mandrelDiaIn is (1 sigma)
    float measSigmaIn;       // noise of one diameter reading (1 sigma)
    float processSigmaIn;    // how unpredictable one layer's growth is (1 sigma)
    float gateSigmas;        // readings further than this many sigmas from the prediction are ignored
    int   maxLayers;         // safety stop
} WrapConfig;

// Per-cell state: the diameter estimate plus this layer's raw readings.
typedef struct {
    int   nCells;
    float cellIn;                         // actual cell size (length / nCells)
    float est[WRAP_MAX_CELLS];            // estimated outside diameter
    float var[WRAP_MAX_CELLS];            // variance of that estimate
    float target[WRAP_MAX_CELLS];         // finished diameter (the cone)
    float layerAddIn;                     // learned diameter growth per full layer
    float nominalLayerAddIn;              // 4 x tow thickness
    int   layersDone;                     // fill + full layers (not the measurement)
    int   measureDone;
    int   fullLayersDone;
    uint8_t flags[WRAP_MAX_CELLS];        // CELL_RIDGE / CELL_REFERENCE

    // raw readings accumulated during the current layer (integer adds only)
    int32_t sumDy[WRAP_MAX_CELLS];        // barrel steps
    int32_t sumDe[WRAP_MAX_CELLS];        // encoder counts
    uint8_t nSamp[WRAP_MAX_CELLS];
    float   hoopDia[2];                   // latest dwell reading at each end (<= 0: none)
    float   hoopRevs[2];                  // total dwell turns at each end this layer
} WrapModel;

#define LAYER_MEASURE 0   // first pass: read the starting profile
#define LAYER_FILL    1   // partial layer: build the taper
#define LAYER_FULL    2   // full-length layer: bring the cone up to size

// What wrap.c needs to run one layer.  Built by modelPlanLayer().
typedef struct {
    int     kind;                            // LAYER_MEASURE / LAYER_FILL / LAYER_FULL
    int     firstCell;                       // wind cells firstCell .. endCell-1
    int     endCell;
    int     layersLeft;                      // K: full-length layers still to go
    int     circuits;                        // out-and-back passes in this layer
    int     gateShift;                       // keep readings within expect/2^gateShift (0 = keep all)
    float   barrelSpeed;                     // steps/s
    float   carriageSpeed[WRAP_MAX_CELLS];   // steps/s for each cell
    int32_t expectDe[WRAP_MAX_CELLS];        // encoder counts expected while crossing each cell
    int32_t minDePerRev[2];                  // fewer counts per turn while dwelling = fiber problem
    int     fiberShift;                      // fiber problem below expect/2^fiberShift in a cell
} LayerPlan;

// Summary of one modelUpdate(), for the display and the serial log.
typedef struct {
    int   measured;      // cells with an accepted reading
    int   rejected;      // cells whose reading was thrown out
    float layerAddIn;    // growth per layer after learning
} ModelStats;

// Result codes for modelPlanLayer()
#define PLAN_COMPLETE    0     // nothing left to wind: the cone is built
#define PLAN_IMPOSSIBLE (-1)   // tow too wide for the diameter

void        wrapDefaultConfig(WrapConfig *cfg);
const char *wrapCheckConfig(const WrapConfig *cfg);     // NULL if OK, else a short (<=20 char) reason

int  modelInit(WrapModel *m, const WrapConfig *cfg);
int  modelPlanLayer(const WrapModel *m, const WrapConfig *cfg, LayerPlan *plan);  // >0 = cells to wind
void modelUpdate(WrapModel *m, const WrapConfig *cfg, const LayerPlan *plan, ModelStats *stats);
void modelClearSamples(WrapModel *m);

// Fiber-centreline diameter implied by a helical cell reading, or -1 if unusable.
float diameterFromReading(float cellIn, float dy, float de);

// Record a dwell (pure hoop) reading at one end: dy barrel steps, de encoder counts.
void modelHoopReading(WrapModel *m, int end, int32_t dy, int32_t de);


// ---- Called at every cell boundary while the carriage is moving: integer work only ----

// Too little fiber came off the spool: broken or run-out tow.  Normally under
// half of what is expected; on the measurement pass (diameter still a guess)
// only under 1/8.
static inline int fiberLow(const LayerPlan *p, int cell, int32_t de) {
    return de < (p->expectDe[cell] >> p->fiberShift);
}

// Keep a reading only if it is close to what the plan expects (within 1/8 =
// 12.5%), so a slip or a jerk is dropped instead of spoiling the cell's
// average.  gateShift 0 (the measurement pass, whose expectation is only the
// mandrel guess) keeps every reading.
static inline void modelOfferSample(WrapModel *m, const LayerPlan *p, int cell, int32_t dy, int32_t de) {
    if (p->gateShift) {
        int32_t expect = p->expectDe[cell];
        int32_t diff   = de - expect;
        if (diff < 0) diff = -diff;
        if (diff > (expect >> p->gateShift)) return;
    }
    m->sumDy[cell] += dy;
    m->sumDe[cell] += de;
    m->nSamp[cell]++;
}

#endif /* INC_WRAP_MODEL_H_ */
