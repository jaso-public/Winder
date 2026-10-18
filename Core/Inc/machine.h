/*
 * machine.h
 *
 * Mechanical calibration of the winder.  Shared by winder.c and the wrap module
 * so every piece of code uses the same numbers.
 */

#ifndef INC_MACHINE_H_
#define INC_MACHINE_H_

#define STEPS_PER_INCH        25000   // carriage stepper steps per inch of travel
#define STEPS_PER_REVOLUTION  10880   // barrel stepper steps per barrel revolution
#define COUNT_PER_INCH        610     // fiber encoder counts per inch of fiber paid out

// Usable carriage travel measured from the home (optical) sensor.
// Calibrate Carriage drives 24" from home, so the machine has at least that.
#define CARRIAGE_MAX_TRAVEL_IN 24.0f

// The measurement never looks further than this for the left shoulder.
#define WRAP_MAX_LENGTH_IN     20.0f

// Stepper acceleration (steps/s^2) -- must match the value passed to stepperInit().
#define STEPPER_ACCELERATION   300000.0f

#endif /* INC_MACHINE_H_ */
