/*
 * wrap.h
 *
 * wrap(): wind carbon tow onto the barrel, layer by layer, until the outside
 * matches a cone from the large diameter (right end) to the small diameter
 * (left end).
 *
 * The code is split into three parts:
 *   wrap.c        the machine: passes, dwells/turnarounds, sampling, faults
 *   wrap_model.c  the math:    diameter estimate and the plan for each layer
 *   wrap_ui.c     the operator: light tower, LCD, buttons, settings editor
 */

#ifndef INC_WRAP_H_
#define INC_WRAP_H_

#include <stdint.h>

typedef enum {
    WRAP_OK = 0,
    WRAP_COMPLETE,       // finished: the cone is built
    WRAP_ABORTED,        // operator held Left+Right
    WRAP_FIBER_OUT,      // the encoder says fiber stopped paying out
    WRAP_LIMIT_SWITCH,   // a carriage limit switch tripped (drives are cut off)
    WRAP_NOT_HOMED,      // carriage position unknown
    WRAP_CONFIG_ERROR,   // settings are impossible
} WrapStatus;

WrapStatus wrap(void);

// Set by the limit-switch interrupt (HAL_GPIO_EXTI_Callback in main.c).
extern volatile uint8_t wrapLimitTripped;

#endif /* INC_WRAP_H_ */
