/*
 * stepper.h
 *
 *  Created on: Aug 10, 2025
 *      Author: jaso
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "stm32f3xx_hal.h"



// ----- timing/constants -----
#define TIMER_CLOCK_HZ   36000000.0   // TIM2 @ 36 MHz
#define MIN_BOOT_SPEED   1.0f         // steps/s
#define VELOCITY_EPS     1e-6f

// ----- math helpers -----
#define SGN(x)             (((x) > 0) - ((x) < 0))        // -1,0,+1
#define ABSF(x)            (((x) < 0.0f) ? -(x) : (x))
#define COPY_SIGN_MAG(m,s) (((s) >= 0.0) ? fabs(m) : -fabs(m))

// provided by you
uint64_t getTicks(void);
void halt(char* message);

// direction of rotation
#define CW 12
#define CCW 13

// state of stepper
#define STOPPED 1
#define VELOCITY 2
#define POSITION 3


typedef struct {
	// mostly-constant configuration

	const char *stepperName;      // human-readable name

	TIM_HandleTypeDef *timerHandle;

	volatile uint32_t *compareRegister;   // &TIMx->CCRy
	uint32_t compareFlag;                 // TIM_FLAG_CCx
	uint32_t compareInterruptSource;      // TIM_IT_CCx

	GPIO_TypeDef *pulsePort;
	uint16_t pulsePin;

	GPIO_TypeDef *directionPort;
	uint16_t directionPin;

	IRQn_Type computeIrqNumber;

	uint32_t pulseWidthTicks;             // e.g., 36 ticks (~2 µs @36 MHz)
} StepperConfiguration;



typedef struct {
	StepperConfiguration *config;

	volatile uint32_t lastPulseTick;               // last scheduled rising tick
    float acceleration2x;                 // steps/s^2

    volatile float currentSpeed;          // steps/s
    volatile float speedSquared;
    volatile float desiredSpeed;          // steps/s (cap)

    volatile int stopping;
    volatile int mode;                             // one of STOPPED, POSITION, VELOCITY
    volatile int32_t currentPosition;
    volatile int32_t desiredPosition;
    volatile int32_t stepIncrement;                // +1 or -1

} Stepper;

// ----- api -----
void stepperInit(Stepper *s, StepperConfiguration *cfg, float acceleration);
void moveToPosition(Stepper *s, float initialSpeed, int32_t desiredPosition);
void stepperStart(Stepper *s, float desiredSpeed, int direction);

void stop(Stepper *s);
void waitUntilStopped(Stepper *s);
void changeSpeed(Stepper *s, float newSpeed);

void printStepperInfo(Stepper *s);

void computeNextStepperEvent(Stepper *s);
void stepperHandleIrq(Stepper *s);


#endif // STEPPER_H
