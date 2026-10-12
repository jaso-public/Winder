/*
 * wrap_ui.c
 *
 * Operator interface for wrap(): light tower, LCD screens, buttons and the
 * settings editor.  See wrap_ui.h for what the lights mean.
 */

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "lcd.h"
#include "winder.h"
#include "wrap_ui.h"

extern ButtonState bs;

#define LCD_COLS        20
#define FLASH_PERIOD_MS 250


// ---------------------------------------------------------------------------
// Light tower
// ---------------------------------------------------------------------------

static LightMode lightMode = LIGHT_SETUP;
static uint32_t  lightToggledAt;
static int       lightOn;

static int lightColor(LightMode m) {
    switch (m) {
    case LIGHT_SETUP:
    case LIGHT_WAITING: return YELLOW;
    case LIGHT_WINDING:
    case LIGHT_DONE:    return GREEN;
    case LIGHT_FAULT:   return RED;
    }
    return RED;
}

static int lightFlashes(LightMode m) {
    return m == LIGHT_WAITING || m == LIGHT_DONE || m == LIGHT_FAULT;
}

static void lightApply(void) {
    setLight(lightOn ? lightColor(lightMode) : 0);     // 0 = all off
}

void lightsSet(LightMode mode) {
    lightMode      = mode;
    lightOn        = 1;
    lightToggledAt = HAL_GetTick();
    lightApply();
}

void lightsService(void) {
    if (!lightFlashes(lightMode)) return;
    uint32_t now = HAL_GetTick();
    if (now - lightToggledAt < FLASH_PERIOD_MS) return;
    lightToggledAt = now;
    lightOn = !lightOn;
    lightApply();
}


// ---------------------------------------------------------------------------
// LCD helpers
// ---------------------------------------------------------------------------

void fmtFixed(char *buf, int size, float v, int decimals) {
    int32_t scale = 1;
    for (int i = 0; i < decimals; i++) scale *= 10;
    int neg = v < 0.0f;
    if (neg) v = -v;
    int32_t x = (int32_t)(v * scale + 0.5f);
    if (decimals == 0)
        snprintf(buf, size, "%s%ld", neg ? "-" : "", (long)x);
    else
        snprintf(buf, size, "%s%ld.%0*ld", neg ? "-" : "", (long)(x / scale), decimals, (long)(x % scale));
}

void uiLine(int row, const char *text) {
    char line[LCD_COLS + 1];
    snprintf(line, sizeof(line), "%-20.20s", text ? text : "");
    lcd_set_cursor(0, row);
    lcd_write_string(line);
}

void uiMessage(const char *l0, const char *l1, const char *l2, const char *l3) {
    uiLine(0, l0);
    uiLine(1, l1);
    uiLine(2, l2);
    uiLine(3, l3);
    printf("[wrap] %s | %s | %s | %s\r\n", l0 ? l0 : "", l1 ? l1 : "", l2 ? l2 : "", l3 ? l3 : "");
}

int uiWaitForCenter(void) {
    updateButtonState(&bs);
    (void)newPress(&bs.center);                 // ignore a press made before we got here
    while (1) {
        updateButtonState(&bs);
        lightsService();
        if (isLeft() && isRight()) {
            while (isLeft() || isRight()) lightsService();
            return 0;
        }
        if (newPress(&bs.center)) return 1;
    }
}


// ---------------------------------------------------------------------------
// Settings editor:  Top/Bottom = choose,  Left/Right = -/+ (hold to repeat),
//                   C = start,  Left+Right = cancel
// ---------------------------------------------------------------------------

typedef struct {
    const char *label;
    float      *f;          // either a float ...
    int        *i;          // ... or an int setting
    float       step, lo, hi;
    int         decimals;
} EditItem;

static void itemValue(const EditItem *it, char *buf, int size) {
    if (it->f) fmtFixed(buf, size, *it->f, it->decimals);
    else       snprintf(buf, size, "%d", *it->i);
}

static void itemAdjust(const EditItem *it, int dir) {
    if (it->f) {
        float v = *it->f + dir * it->step;
        *it->f = v < it->lo ? it->lo : (v > it->hi ? it->hi : v);
    } else {
        int v = *it->i + dir * (int)it->step;
        *it->i = v < (int)it->lo ? (int)it->lo : (v > (int)it->hi ? (int)it->hi : v);
    }
}

static void editorDraw(const EditItem *items, int count, int current, int top) {
    uiLine(0, "Setup C=go  L+R=quit");
    for (int row = 0; row < 3; row++) {
        int k = top + row;
        char line[48], val[12];          // uiLine() trims to 20
        if (k >= count) { uiLine(row + 1, ""); continue; }
        itemValue(&items[k], val, sizeof(val));
        snprintf(line, sizeof(line), "%c%-11.11s%8s", k == current ? '>' : ' ', items[k].label, val);
        uiLine(row + 1, line);
    }
}

int uiEditConfig(WrapConfig *cfg) {
    WrapConfig w = *cfg;                         // edit a copy; keep it only on C
    const EditItem items[] = {
        { "Length in",   &w.lengthIn,       0, 0.125f, 1.0f,  CARRIAGE_MAX_TRAVEL_IN, 3 },
        { "Right end",   &w.rightEndIn,     0, 0.125f, 0.0f,  CARRIAGE_MAX_TRAVEL_IN, 3 },
        { "Large dia",   &w.largeDiaIn,     0, 0.010f, 0.5f,  12.0f, 3 },
        { "Small dia",   &w.smallDiaIn,     0, 0.010f, 0.5f,  12.0f, 3 },
        { "Mandrel dia", &w.mandrelDiaIn,   0, 0.010f, 0.5f,  12.0f, 3 },
        { "Tow width",   &w.towWidthIn,     0, 0.005f, 0.02f, 1.0f,  3 },
        { "Tow thick",   &w.towThicknessIn, 0, 0.001f, 0.001f,0.1f,  3 },
        { "Passes/lyr",  0, &w.circuits,       1,      1,     WRAP_MAX_CIRCUITS, 0 },
        { "Barrel rps",  &w.barrelRps,      0, 0.05f,  0.05f, 3.0f,  2 },
        { "Full finish", 0, &w.fullFinish,     1,      0,     1,     0 },
    };
    const int count = sizeof(items) / sizeof(items[0]);
    int current = 0, top = 0;
    uint32_t repeatAt = 0;

    updateButtonState(&bs);
    (void)newPress(&bs.top); (void)newPress(&bs.bottom); (void)newPress(&bs.center);
    (void)newPress(&bs.left); (void)newPress(&bs.right);
    editorDraw(items, count, current, top);

    while (1) {
        updateButtonState(&bs);
        lightsService();

        if (isLeft() && isRight()) {             // cancel
            while (isLeft() || isRight()) {}
            return 0;
        }
        if (newPress(&bs.center)) {
            *cfg = w;
            return 1;
        }

        int moved = 0;
        if (newPress(&bs.top)    && current > 0)         { current--; moved = 1; }
        if (newPress(&bs.bottom) && current < count - 1) { current++; moved = 1; }
        if (current < top)      top = current;
        if (current > top + 2)  top = current - 2;

        // Left/Right: one step per press, then auto-repeat while held.
        uint32_t now = HAL_GetTick();
        int dir = 0;
        if (newPress(&bs.right))      { dir = +1; repeatAt = now + 400; }
        else if (newPress(&bs.left))  { dir = -1; repeatAt = now + 400; }
        else if ((isRight() != isLeft()) && (int32_t)(now - repeatAt) >= 0) {
            dir = isRight() ? +1 : -1;
            repeatAt = now + 80;
        }
        if (dir) { itemAdjust(&items[current], dir); moved = 1; }

        if (moved) editorDraw(items, count, current, top);
    }
}


// ---------------------------------------------------------------------------
// Progress and result screens
// ---------------------------------------------------------------------------

void uiShowCircuit(int layer, int circuit, int circuits) {
    char line[48];
    snprintf(line, sizeof(line), "Layer %d  pass %d/%d", layer, circuit + 1, circuits);
    uiLine(0, line);
}

void uiShowLayer(int layer, const WrapModel *m, const LayerPlan *p) {
    char line[48], a[12], b[12];     // uiLine() trims to 20

    uiShowCircuit(layer, 0, p->circuits);

    if (p->kind == LAYER_MEASURE) {
        uiLine(1, "Measuring profile");
    } else if (p->kind == LAYER_FULL) {
        snprintf(line, sizeof(line), "Full length, %d left", p->layersLeft);
        uiLine(1, line);
    } else {
        fmtFixed(a, sizeof(a), p->firstCell * m->cellIn, 2);
        fmtFixed(b, sizeof(b), p->endCell * m->cellIn, 2);
        snprintf(line, sizeof(line), "Fill %s-%s in", a, b);
        uiLine(1, line);
    }

    fmtFixed(a, sizeof(a), m->est[0], 3);
    fmtFixed(b, sizeof(b), m->target[0], 3);
    snprintf(line, sizeof(line), "Right %s>%s", a, b);
    uiLine(2, line);

    fmtFixed(a, sizeof(a), m->est[m->nCells - 1], 3);
    fmtFixed(b, sizeof(b), m->target[m->nCells - 1], 3);
    snprintf(line, sizeof(line), "Left  %s>%s", a, b);
    uiLine(3, line);
}

void uiShowResult(WrapStatus s, int layer, const char *detail) {
    char lay[48];                                // uiLine() trims to 20
    snprintf(lay, sizeof(lay), "at layer %d", layer);

    switch (s) {
    case WRAP_COMPLETE:
        lightsSet(LIGHT_DONE);
        snprintf(lay, sizeof(lay), "%d layers wound", layer);
        uiMessage("WRAP COMPLETE", lay, "Cut fiber, remove", "C = menu");
        break;
    case WRAP_ABORTED:
        lightsSet(LIGHT_WAITING);
        uiMessage("Wrap stopped", "by operator", lay, "C = menu");
        break;
    case WRAP_FIBER_OUT:
        lightsSet(LIGHT_FAULT);
        uiMessage("FIBER OUT / BROKEN", lay, detail, "C = menu");
        break;
    case WRAP_LIMIT_SWITCH:
        lightsSet(LIGHT_FAULT);
        uiMessage("LIMIT SWITCH TRIPPED", lay, "Re-home carriage", "C = menu");
        break;
    case WRAP_NOT_HOMED:
        lightsSet(LIGHT_FAULT);
        uiMessage("Carriage not homed", "Home it first", NULL, "C = menu");
        break;
    case WRAP_CONFIG_ERROR:
        lightsSet(LIGHT_FAULT);
        uiMessage("Settings problem:", detail, NULL, "C = menu");
        break;
    default:
        return;
    }
    uiWaitForCenter();
}
