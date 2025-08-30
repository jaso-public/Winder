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
    s->currentPosition             = 0;
    s->desiredPosition             = 0;
    s->currentSpeed                = 0.0f;
    s->speedSquared                = 0.0f;
    s->desiredSpeed                = 0;
    s->acceleration2x              = acceleration * 2.0f;
    s->lastPulseTick               = 0;
    s->mode                        = STOPPED;


    HAL_GPIO_WritePin(s->config->pulsePort,     s->config->pulsePin,     GPIO_PIN_RESET);

    // Hard-disable this channel’s compare interrupt and clear any stale flags
	TIM_HandleTypeDef *htim = s->config->timerHandle;
	__HAL_TIM_DISABLE_IT(htim, s->config->compareInterruptSource);
	__HAL_TIM_CLEAR_IT(htim,   s->config->compareInterruptSource);
	__HAL_TIM_CLEAR_FLAG(htim, s->config->compareFlag);

	// Park CCR slightly in the past so it can’t immediately match
	uint32_t now = htim->Instance->CNT;
	*s->config->compareRegister = now - 2u;   // recent past; won’t trigger
}

void moveToPosition(Stepper *s, float initialSpeed, int32_t desiredPosition) {

    StepperConfiguration *cfg = s->config;
    if(s->mode != STOPPED) halt("BUG: moveToPosition() state not STOPPED");

    if(desiredPosition == s->currentPosition) {
        printf("moveToPosition desiredPosition:%ld == currentPosition:%ld -- not moving\r\n", desiredPosition, s->currentPosition);
        return;
    }

	s->mode = POSITION;

    s->desiredSpeed = initialSpeed;
    s->desiredPosition = desiredPosition;

    if(s->desiredPosition > s->currentPosition) {
        HAL_GPIO_WritePin(cfg->directionPort, cfg->directionPin, GPIO_PIN_SET);
        s->stepIncrement = 1;
    } else {
        HAL_GPIO_WritePin(cfg->directionPort, cfg->directionPin, GPIO_PIN_RESET);
        s->stepIncrement = -1;
    }

    // Raise immediately; schedule same-channel falling edge
    HAL_GPIO_WritePin(cfg->pulsePort, cfg->pulsePin, GPIO_PIN_SET);

    s->lastPulseTick = cfg->timerHandle->Instance->CNT;
    *cfg->compareRegister = s->lastPulseTick + cfg->pulseWidthTicks;

    __HAL_TIM_ENABLE_IT(cfg->timerHandle, cfg->compareInterruptSource);
}

void stepperStart(Stepper *s, float desiredSpeed, int direction) {

    StepperConfiguration *cfg = s->config;
    if(s->mode != STOPPED) halt("BUG: stepperStart() state not STOPPED");


    s->mode = VELOCITY;

    s->desiredSpeed = desiredSpeed;

    if(direction == CW) {
        HAL_GPIO_WritePin(s->config->directionPort, s->config->directionPin, GPIO_PIN_SET);
        printf("stating stepper CW\r\n");
        s->stepIncrement = 1;
    } else {
        HAL_GPIO_WritePin(s->config->directionPort, s->config->directionPin, GPIO_PIN_RESET);
        printf("stating stepper CCW\r\n");
        s->stepIncrement = -1;
    }

    // Raise immediately; schedule same-channel falling edge
    HAL_GPIO_WritePin(s->config->pulsePort, s->config->pulsePin, GPIO_PIN_SET);

    s->lastPulseTick = cfg->timerHandle->Instance->CNT;
    *cfg->compareRegister = s->lastPulseTick + cfg->pulseWidthTicks;

    __HAL_TIM_ENABLE_IT(cfg->timerHandle, cfg->compareInterruptSource);
}


void computeNextStepperEvent(Stepper *s) {

    StepperConfiguration *cfg = s->config;

    if (s->mode == POSITION) {
        // we want to get the stepper to a particular Steps with a velocity of zero
        uint32_t stepsToStop = (uint32_t) ceilf(s->speedSquared / s->acceleration2x);
        int32_t stepsRemaining = s->desiredPosition - s->currentPosition;
        if(stepsRemaining < 0) stepsRemaining = -stepsRemaining;
        if (stepsRemaining < stepsToStop) {
            s->desiredSpeed = 0.0f;
        }
    }

    if (s->currentSpeed < s->desiredSpeed) {
        // we need to accelerate to reach the desired speed
        s->speedSquared += s->acceleration2x;
        s->currentSpeed = sqrtf(s->speedSquared);

        if (s->currentSpeed > s->desiredSpeed) {
            s->currentSpeed = s->desiredSpeed;
            s->speedSquared = s->currentSpeed * s->currentSpeed;
        }

    } else if (s->currentSpeed > s->desiredSpeed) {
        // we need to brake to reach the desired speed
        s->speedSquared -= s->acceleration2x;
        if (s->speedSquared <= s->desiredSpeed) {
            s->currentSpeed = s->desiredSpeed;
            s->speedSquared = s->currentSpeed * s->currentSpeed;
        } else {
            s->currentSpeed = sqrtf(s->speedSquared);
        }
    }

    if (s->currentSpeed == 0.0) {
        s->speedSquared = 0.0f;
        __HAL_TIM_DISABLE_IT(cfg->timerHandle, cfg->compareInterruptSource);
        s->mode = STOPPED;
        return;
    }

    s->lastPulseTick += (uint32_t) (TIMER_CLOCK_HZ / s->currentSpeed);
    *cfg->compareRegister = s->lastPulseTick;
}



void stepperHandleIrq(Stepper *s) {
    StepperConfiguration *cfg = s->config;
    TIM_HandleTypeDef *htim = cfg->timerHandle;

    if (__HAL_TIM_GET_FLAG(htim, cfg->compareFlag) && __HAL_TIM_GET_IT_SOURCE(htim, cfg->compareInterruptSource)) {

        __HAL_TIM_CLEAR_IT(htim, cfg->compareInterruptSource);

        if (cfg->pulsePort->ODR & cfg->pulsePin) {
            // FALL: drive low an schedule a low priority interrupt to compute next pulse
            HAL_GPIO_WritePin(cfg->pulsePort, cfg->pulsePin, GPIO_PIN_RESET);
            s->currentPosition += s->stepIncrement;
            NVIC_SetPendingIRQ(cfg->computeIrqNumber);
        } else {
            // RISE: drive high and schedule the FALL compare
            HAL_GPIO_WritePin(cfg->pulsePort, cfg->pulsePin, GPIO_PIN_SET);
            *cfg->compareRegister = htim->Instance->CNT + cfg->pulseWidthTicks;
        }
    }
}

