/*
 * winder.c
 *
 *  Created on: Jul 9, 2025
 *      Author: jaso
 */

#include <lcd.h>
#include <main.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <stm32f303x8.h>
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_gpio.h>
#include <sys/_stdint.h>
#include <winder.h>

#define DEBOUNCE_DELAY 20

int isLeft() {
	return HAL_GPIO_ReadPin(ButtonLeft_GPIO_Port, ButtonLeft_Pin) == 0;
}

int isRight() {
	return HAL_GPIO_ReadPin(ButtonRight_GPIO_Port, ButtonRight_Pin) == 0;
}

int isTop() {
	return HAL_GPIO_ReadPin(ButtonTop_GPIO_Port, ButtonTop_Pin) == 0;
}

int isBottom() {
	return HAL_GPIO_ReadPin(ButtonBottom_GPIO_Port, ButtonBottom_Pin) == 0;
}

int isCenter() {
	return HAL_GPIO_ReadPin(ButtonCenter_GPIO_Port, ButtonCenter_Pin) == 0;
}


void setLight(int color) {
	HAL_GPIO_WritePin(RedLight_GPIO_Port, RedLight_Pin, color==RED);
	HAL_GPIO_WritePin(GreenLight_GPIO_Port, GreenLight_Pin, color==GREEN);
	HAL_GPIO_WritePin(YellowLight_GPIO_Port, YellowLight_Pin, color==YELLOW);
}

void testLights() {
	printf("\r\n\n\nLeft:   Red\r\n");
	printf("Top:    Yellow\r\n");
	printf("Right:  Green\r\n");
	printf("Center: Exit\r\n");

	while(1) {
		HAL_GPIO_WritePin(RedLight_GPIO_Port, RedLight_Pin, isLeft());
		HAL_GPIO_WritePin(GreenLight_GPIO_Port, GreenLight_Pin, isRight());
		HAL_GPIO_WritePin(YellowLight_GPIO_Port, YellowLight_Pin, isTop());

		if(isCenter()) {
			HAL_Delay(DEBOUNCE_DELAY);
			while(isCenter()) HAL_Delay(DEBOUNCE_DELAY);
			return;
		}
	}
}

void randomFlash() {
    printf("\r\n\nCenter: Exit\r\n\n\n");

    // Optional: seed the RNG once if needed
    srand(HAL_GetTick());  // Uses system uptime in ms

    while (1) {
        // Randomly set each light ON or OFF
        HAL_GPIO_WritePin(RedLight_GPIO_Port, RedLight_Pin, rand() % 2);
        HAL_GPIO_WritePin(GreenLight_GPIO_Port, GreenLight_Pin, rand() % 2);
        HAL_GPIO_WritePin(YellowLight_GPIO_Port, YellowLight_Pin, rand() % 2);

        // Delay a little between flashes
        HAL_Delay(200);  // adjust as desired

        // Exit if center button is pressed
        if (isCenter()) {
            HAL_Delay(DEBOUNCE_DELAY);
            while (isCenter()) HAL_Delay(DEBOUNCE_DELAY);
            return;
        }
    }
}

//
//#define STEP_PIN      GPIO_PIN_0
//#define STEP_PORT     GPIOA
//
//#define DIR_PIN       GPIO_PIN_1
//#define DIR_PORT      GPIOA
//
//#define STEPS         16000
//#define ACCEL         100000.0f  // steps/sec^2
//
//// ~1us delay function using DWT (Data Watchpoint & Trace) cycle counter
//static inline void delayMicroseconds(uint32_t us) {
//    uint32_t start = DWT->CYCCNT;
//    uint32_t ticks = us * (SystemCoreClock / 1000000);
//    while ((DWT->CYCCNT - start) < ticks);
//}
//
//void enableDWT() {
//    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//    DWT->CYCCNT = 0;
//    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//}
//
//void pulse_step() {
//    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
//    delayMicroseconds(5);  // minimum 5us
//    HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
//}
//
//void move_stepper(uint32_t steps, float accel) {
//    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);  // Set direction
//    delayMicroseconds(10);  // DIR setup time
//
//    float v;       // current velocity
//    float dt_us;   // delay per step in microseconds
//    float s;       // step position (index)
//    float v_max = sqrt(2 * accel * (steps / 2.0f));  // peak velocity at midpoint
//
//    for (uint32_t i = 0; i < steps; i++) {
//        s = (i < steps / 2) ? i : (steps - i);
//        v = sqrtf(2.0f * accel * s);
//        if (v < 1) v = 1;
//        dt_us = 1e6f / v;
//        pulse_step();
//        delayMicroseconds((uint32_t)dt_us);
//    }
//}



void display_menu(char* prompt, char** options, int count, int current, int start) {

	setLight(YELLOW);

	lcd_clear();
	lcd_write_string(prompt);
	int index = 1;

	printf("\r\n\n%s\r\n", prompt);

	int stop = start+3;
	if(stop > count) stop = count;

	for(int i=start ; i<stop ; i++) {
		lcd_set_cursor(0,index++);
		if(i == current) {
			printf("** ");
			lcd_write_string("** ");
		} else {
			printf("   ");
			lcd_write_string("   ");

		}

		printf("%s\r\n", options[i]);
		lcd_write_string(options[i]);
	}
}

int main_menu() {

	char* prompt = "Select:";
	char* selections[] = {
			"Move Steppers",
			"Barrel Profile 1",
			"Barrel Profile 2",
			"Test Lights",
			"Test Left Limit",
			"Test Right Limit",
			"Random Flashing"
	};

	int start = 0;
	int current = 0;
	int count = sizeof(selections) / sizeof(char*);
	printf("count: %d\r\n", count);

	display_menu(prompt, selections, count, current, start);

	while(1) {

		while(1) {
			if(isTop()) {
				current--;
				if(current < 0) current = 0;
				if(current < start) start--;
				display_menu(prompt, selections, count, current, start);

				HAL_Delay(DEBOUNCE_DELAY);
				while(isTop()) HAL_Delay(DEBOUNCE_DELAY);
				break;
			}

			if(isBottom()) {
				current++;
				if(current>=count) current = count-1;
				if(current >= start+3) start++;
				display_menu(prompt, selections, count, current, start);

				HAL_Delay(DEBOUNCE_DELAY);
				while(isBottom()) HAL_Delay(DEBOUNCE_DELAY);

				break;
			}

			if(isCenter()) {
				printf("\r\n\nselected: %d %s\r\n\n", current, selections[current]);
				HAL_Delay(DEBOUNCE_DELAY);
				while(isCenter()) HAL_Delay(DEBOUNCE_DELAY);

				display_menu(prompt, selections, count, current, start);

				if(current == 3) testLights();
				if(current == 6) randomFlash();
				display_menu(prompt, selections, count, current, start);
				break;
			}
		}
	}




	void stepper_init(stepper_t* s,
	                  GPIO_TypeDef* step_group, uint16_t step_pin,
	                  GPIO_TypeDef* dir_group,  uint16_t dir_pin) {
	    s->stepGroup   = step_group;
	    s->stepPin     = step_pin;
	    s->dirGroup    = dir_group;
	    s->dirPin      = dir_pin;

	    s->desired_velocity = 0.0;
	    s->current_velocity = 0.0;
	    s->acceleration     = ACCELERATION;  // constant, e.g. 300000.0

	    s->state = IDLE;

	    // Set pins to known state
	    HAL_GPIO_WritePin(s->stepGroup, s->stepPin, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(s->dirGroup, s->dirPin, GPIO_PIN_RESET);
	}


	void setDirection(stepper_t* s) {
		if(s->current_velocity == 0.0) {
			if(s->desired_velocity >= 0) {
			    HAL_GPIO_WritePin(s->dirGroup, s->dirPin, GPIO_PIN_SET);
			} else {
			    HAL_GPIO_WritePin(s->dirGroup, s->dirPin, GPIO_PIN_RESET);
			}
		} else {
			if(s->current_velocity >= 0.0) {
			    HAL_GPIO_WritePin(s->dirGroup, s->dirPin, GPIO_PIN_SET);
			} else {
			    HAL_GPIO_WritePin(s->dirGroup, s->dirPin, GPIO_PIN_RESET);
			}
		}
	}


	void fireEvent(stepper_t* s) {
		if(s->state == START_PULSE) {
			setDirection(s);
			HAL_GPIO_WritePin(s->stepGroup, s->stepPin, GPIO_PIN_SET);
			s->state = END_PULSE;
			s->nextEvent = s->nextPulseEnd;
		} else if(s->state == END_PULSE) {
			HAL_GPIO_WritePin(s->stepGroup, s->stepPin, GPIO_PIN_RESET);
			s->state = COMPUTE_NEXT;
			s->nextEvent = UINT64_MAX;
		}
	}



	void computeNext(stepper_t* s) {

		if(s->state == IDLE) {
			if(s->desired_velocity == 0.0) return;

			HAL_GPIO_WritePin(s->stepGroup, s->stepPin, GPIO_PIN_SET);
			s->lastPulseTime = getTicks();
			s->nextPulseEnd = s->lastPulseTime + PULSE_WIDTH_TICKS;
			s->nextEvent = s->nextPulseEnd;
			s->state = END_PULSE;

		    // Initial velocity after the first step
		    s->current_velocity = copysign(sqrt(2.0 * s->acceleration), s->desired_velocity);
		    return;
		}

		if(s->state != COMPUTE_NEXT) return;

	    // Compute current speed
	    double v = fabs(s->current_velocity);
	    if (v < 1.0) v = 1.0;

	    double delay = 1.0 / v;
	    s->lastPulseTime += delay * TICKS_PER_SEC;
		s->nextPulseEnd = s->lastPulseTime + PULSE_WIDTH_TICKS;
		s->nextEvent = s->lastPulseTime;
		s->state = START_PULSE;

		double v0 = fabs(s->current_velocity);
		double a = (fabs(s->desired_velocity) > v0) ? s->acceleration : -s->acceleration;
		a = copysign(a, s->desired_velocity);
		double v1;

		if (a > 0) {
		    v1 = sqrt(v0 * v0 + 2 * s->acceleration);
		    if (v1 > fabs(s->desired_velocity)) v1 = fabs(s->desired_velocity);
		} else if (a < 0) {
		    double tmp = v0 * v0 - 2 * s->acceleration;
		    v1 = (tmp > 0) ? sqrt(tmp) : 0;
		    if (v1 < fabs(s->desired_velocity)) v1 = fabs(s->desired_velocity);
		} else {
		    v1 = v0;
		}

		s->current_velocity = copysign(v1, s->desired_velocity);

	    // Re-enter idle if we're done
	    if (s->desired_velocity == 0.0 && fabs(s->current_velocity) < 1.0) {
	        s->current_velocity = 0.0;
	        s->state = IDLE;
	    }
	}

	uint8_t checkButton(uint8_t* pressed, uint64_t* time, uint8_t status, uint64_t now) {
		if( (now - *time) < DEBOUNCE) return 0;

		if(*pressed) {
			if(!status) {
				*time = now;
				*pressed = 0;
				return 1;
			}
		} else {
			if(status) {
				*time = now;
				*pressed = 1;
				return 1;
			}
		}

		return 0;
	}

	void moveLoop(stepper_t* s) {


		uint8_t exitting = 0;

		uint8_t  L_pressed = 0;
		uint64_t L_time = 0;

		uint8_t  R_pressed = 0;
		uint64_t R_time = 0;

		uint8_t  T_pressed = 0;
		uint64_t T_time = 0;

		uint8_t  B_pressed = 0;
		uint64_t B_time = 0;

		uint8_t  C_pressed = isCenter();
		uint64_t C_time = getTicks();

		while(1) {

			if(exitting && s->state == IDLE) return;

			uint64_t now = getTick();
			uint64_t ticksToEvent = s->nextEvent - now;


			if(ticksToEvent < TOO_CLOSE_TO_EVENT) {
				while(getTick() < s->nextEvent);
				fireEvent(s);
				continue;
			}

			// we have time to checks things before the event will occur
			checkButton(&L_pressed, &L_time, isLeft(), now);
			checkButton(&R_pressed, &R_time, isRight(), now);
			if(checkButton(&T_pressed, &T_time, isTop(), now)) {
				if(T_pressed) s->desired_velocity *= 1.1;
			}

			if(checkButton(&B_pressed, &B_time, isBottom(), now)) {
				if(B_pressed) s->desired_velocity /= 1.1;
			}

			if(checkButton(&C_pressed, &C_time, isCenter(), now)) {
				if(C_pressed) {
					exitting = 1;
					s->desired_velocity = 0;
				}
			}

			if( !exitting) {
				if(L_pressed) {
					s->desired_velocity = fabs(s->desired_velocity);
				} else if(R_pressed) {
					s->desired_velocity = -1.0 * fabs(s->desired_velocity);
				} else {
					s->desired_velocity = 0;
				}
			}

			updateEvent(s);
		}
	}
}
