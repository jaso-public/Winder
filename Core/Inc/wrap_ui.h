/*
 * wrap_ui.h
 *
 * Everything the operator sees or presses during wrap(): the light tower,
 * the 20x4 LCD and the buttons.
 *
 * Light tower
 * -----------
 *   Yellow steady    setting up / carriage moving into position -- stand clear
 *   Yellow flashing  waiting for the operator (attach fiber, press C)
 *   Green  steady    winding
 *   Green  flashing  wrap finished -- cut the fiber
 *   Red    flashing  stopped on a fault -- read the LCD
 */

#ifndef INC_WRAP_UI_H_
#define INC_WRAP_UI_H_

#include "wrap.h"
#include "wrap_model.h"

typedef enum {
    LIGHT_SETUP,      // yellow steady
    LIGHT_WAITING,    // yellow flashing
    LIGHT_WINDING,    // green steady
    LIGHT_DONE,       // green flashing
    LIGHT_FAULT,      // red flashing
} LightMode;

void lightsSet(LightMode mode);
void lightsService(void);            // call often from wait loops; does the flashing

// LCD helpers.  NULL lines are left blank.  Each LCD character takes ~2 ms.
void uiMessage(const char *l0, const char *l1, const char *l2, const char *l3);
void uiLine(int row, const char *text);  // overwrite one row, padded to 20 characters

// Wait for the operator: 1 = Center pressed, 0 = Left+Right held (cancel).
int  uiWaitForCenter(void);

// Let the operator review/edit the main settings.  1 = start, 0 = cancelled.
int  uiEditConfig(WrapConfig *cfg);

// Progress screens
void uiShowLayer(int layer, const WrapModel *m, const LayerPlan *p);
void uiShowCircuit(int layer, int circuit, int circuits);

// Fault / result screen.  Waits for C.
void uiShowResult(WrapStatus status, int layer, const char *detail);

// Fixed-point formatting (the build has no float printf).
void fmtFixed(char *buf, int size, float value, int decimals);

#endif /* INC_WRAP_UI_H_ */
