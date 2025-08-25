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


// stepper.h
#ifndef STEPPER_DEBUG
#define STEPPER_DEBUG 0
#endif

#if STEPPER_DEBUG
  #include <stdio.h>
  #define STEPPER_NAME(s) ((s)->config->stepperName ? (s)->config->stepperName : "?")
  #define DBG(s, fmt, ...)  printf("[STEP:%s] " fmt "\r\n", STEPPER_NAME(s), ##__VA_ARGS__)
#else
  #define DBG(s, fmt, ...)  do{}while(0)
#endif

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

	uint32_t pulseWidthTicks;             // e.g., 72 ticks (~2 µs @36 MHz)
} StepperConfiguration;



typedef struct {
	StepperConfiguration *config;

    uint32_t lastPulseTick;               // last scheduled rising tick (TIM2 domain)

    volatile float currentSpeed;                   // steps/s
    volatile float speedSquared;
    volatile float desiredSpeed;                   // steps/s (cap)
    float acceleration2x;                   // steps/s^2

    int32_t currentSteps;              // steps
    int32_t desiredSteps;              // steps

} Stepper;

// ----- api -----
void stepperInit(Stepper *s, StepperConfiguration *cfg, float acceleration);
void stepperInitTimebase(Stepper *s);
void stepperStart(Stepper *s, float desiredSpeed, int32_t desiredPosition);
void computeNextStepperEvent(Stepper *s);
void stepperHandleIrq(Stepper *s);

#endif // STEPPER_H
