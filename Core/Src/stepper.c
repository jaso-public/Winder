/*
 * stepper.c
 *
 *  Created on: Aug 10, 2025
 *      Author: jaso
 */


#include <stdio.h>

#include "stepper.h"

void stepperInit(Stepper *s, StepperConfiguration *cfg, float acceleration)
{

    s->config                      = cfg;
    s->currentSteps                = 0;
    s->desiredSteps                = 0;
    s->currentSpeed                = 0.0f;
    s->speedSquared                = 0.0f;
    s->desiredSpeed                = 0;
    s->acceleration2x              = acceleration * 2.0f;
    s->lastPulseTick               = 0;

	DBG(s, "[%s] stepperInit acceleration:%.2f desiredSpeedInitial:%.2f\r\n",
            		cfg->stepperName, acceleration, desiredSpeedInitial);

    HAL_GPIO_WritePin(s->config->pulsePort,     s->config->pulsePin,     GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s->config->directionPort, s->config->directionPin, GPIO_PIN_RESET);

    // Hard-disable this channel’s compare interrupt and clear any stale flags
	TIM_HandleTypeDef *htim = s->config->timerHandle;
	__HAL_TIM_DISABLE_IT(htim, s->config->compareInterruptSource);
	__HAL_TIM_CLEAR_IT(htim,   s->config->compareInterruptSource);
	__HAL_TIM_CLEAR_FLAG(htim, s->config->compareFlag);

	// Park CCR slightly in the past so it can’t immediately match
	uint32_t now = htim->Instance->CNT;
	*s->config->compareRegister = now - 2u;   // recent past; won’t trigger
}

void stepperStart(Stepper *s, float desiredSpeed, int32_t desiredSteps) {

	StepperConfiguration *cfg = s->config;

    s->desiredSpeed    = desiredSpeed;
    s->desiredSteps = desiredSteps;

//
//    HAL_GPIO_WritePin(s->config->directionPort, s->config->directionPin,
//                      (dir > 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Raise immediately; schedule same-channel falling edge
    HAL_GPIO_WritePin(s->config->pulsePort, s->config->pulsePin, GPIO_PIN_SET);

    s->lastPulseTick          = cfg->timerHandle->Instance->CNT;
    *cfg->compareRegister = s->lastPulseTick + cfg->pulseWidthTicks;

    __HAL_TIM_ENABLE_IT(cfg->timerHandle, cfg->compareInterruptSource);
}


#define MIN_LEAD_TICKS 36u

void computeNextStepperEvent(Stepper *s) {

    StepperConfiguration *cfg = s->config;

    float ds = s->desiredSpeed;
    float ss = s->currentSpeed;

    if (s->desiredSteps > 0) {
        // we want to get the stepper to a particular Steps with a velocity of zero
        uint32_t stepsToStop = (uint32_t) ceilf(s->speedSquared / s->acceleration2x);
        if (s->desiredSteps - s->currentSteps < stepsToStop) {
            s->desiredSpeed = 0.0f;
        }
    }

    if (s->currentSpeed < ds) {
        // we need to accelerate to reach the desired speed
        s->speedSquared += s->acceleration2x;
        s->currentSpeed = sqrtf(s->speedSquared);

        if (s->currentSpeed > s->desiredSpeed) {
            s->currentSpeed = s->desiredSpeed;
            s->speedSquared = s->currentSpeed * s->currentSpeed;
        }

    } else if (s->currentSpeed > ds) {
        // we need to brake to reach the desired speed
        s->speedSquared -= s->acceleration2x;
        if (s->speedSquared <= s->desiredSpeed) {
            s->currentSpeed = s->desiredSpeed;
            s->speedSquared = s->currentSpeed * s->currentSpeed;
        } else {
            s->currentSpeed = sqrtf(s->speedSquared);
        }
    }

//    if (fabs(s->currentSpeed - s->desiredSpeed) < 1.0f) {
//        s->currentSpeed = s->desiredSpeed;
//        s->speedSquared = s->currentSpeed * s->currentSpeed;
//    }

    if (s->currentSpeed == 0.0) {
        s->speedSquared = 0.0f;
        __HAL_TIM_DISABLE_IT(cfg->timerHandle, cfg->compareInterruptSource);
        printf("sdfsdfsdf\r\n");
        printf("\r\n\ndesiredSpeed: %.1f s->acceleration2x:%.1f s->currentSpeed:%.1f startSpeed:%.1f \r\n", ds, s->acceleration2x, s->currentSpeed, ss);
        return;
    }

    s->lastPulseTick += (uint32_t) (TIMER_CLOCK_HZ / s->currentSpeed);
    *cfg->compareRegister = s->lastPulseTick;
    s->currentSteps += 1;
}



void stepperHandleIrq(Stepper *s) {
    StepperConfiguration *cfg = s->config;
    TIM_HandleTypeDef *htim = cfg->timerHandle;

    if (__HAL_TIM_GET_FLAG(htim, cfg->compareFlag) && __HAL_TIM_GET_IT_SOURCE(htim, cfg->compareInterruptSource)) {

        __HAL_TIM_CLEAR_IT(htim, cfg->compareInterruptSource);

        if (cfg->pulsePort->ODR & cfg->pulsePin) {
            // FALL: drive low an schedule a low priority interrupt to compute next pulse
            HAL_GPIO_WritePin(cfg->pulsePort, cfg->pulsePin, GPIO_PIN_RESET);
            NVIC_SetPendingIRQ(cfg->computeIrqNumber);
        } else {
            // RISE: drive high and schedule the FALL compare
            HAL_GPIO_WritePin(cfg->pulsePort, cfg->pulsePin, GPIO_PIN_SET);
            *cfg->compareRegister = htim->Instance->CNT + cfg->pulseWidthTicks;
        }
    }
}

