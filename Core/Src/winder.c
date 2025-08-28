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
#include <inttypes.h>
#include <string.h>

#include <stm32f303x8.h>
#include <stm32f3xx_hal.h>
#include <stm32f3xx_hal_gpio.h>
#include <sys/_stdint.h>

#include "stepper.h"
#include <winder.h>
#include "gitversion.h"

extern Stepper barrelStepper;
extern Stepper carriageStepper;


#define DEBOUNCE_DELAY 20

ButtonState bs;

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

uint8_t checkButton(Button *b, uint8_t status, uint64_t now) {
    if( (now - b->time) < DEBOUNCE) return 0;

    if(b->pressed) {
        if(!status) {
            b->time = now;
            b->pressed = 0;
            return 1;
        }
    } else {
        if(status) {
            b->time = now;
            b->pressed = 1;
            b->count++;
            printf("in count %d\r\n", b->count);
            return 1;
        }
    }

    return 0;
}

void initializeButton(Button *b) {
    b->pressed = 0;
    b->count = 0;
    b->last = 0;
    b->time = 0;
}

void initializeButtonState(ButtonState *bs) {
    initializeButton(&bs->left);
    initializeButton(&bs->right);
    initializeButton(&bs->center);
    initializeButton(&bs->top);
    initializeButton(&bs->bottom);
}

void updateButtonState(ButtonState *bs) {
    uint64_t now = getTicks();
    checkButton(&bs->left, isLeft(), now);
    checkButton(&bs->right, isRight(), now);
    checkButton(&bs->center, isCenter(), now);
    checkButton(&bs->top, isTop(), now);
    checkButton(&bs->bottom, isBottom(), now);
}

int newPress(Button *b) {
    if(b->count == b->last) return 0;
    b->last = b->count;
    return 1;
}


void setLight(int color) {
	HAL_GPIO_WritePin(RedLight_GPIO_Port, RedLight_Pin, color==RED);
	HAL_GPIO_WritePin(GreenLight_GPIO_Port, GreenLight_Pin, color==GREEN);
	HAL_GPIO_WritePin(YellowLight_GPIO_Port, YellowLight_Pin, color==YELLOW);
}

void testLights() {

	lcd_clear();
	lcd_write_string("Light Test");
	lcd_set_cursor(0, 1);
	lcd_write_string("Left & Right to Exit");
	lcd_set_cursor(0, 2);
	lcd_write_string("T=red C=yellow B=green");

	while(1) {

		if( isTop())    {
	        HAL_GPIO_WritePin(RedLight_GPIO_Port, RedLight_Pin, GPIO_PIN_SET);
		} else {
	        HAL_GPIO_WritePin(RedLight_GPIO_Port, RedLight_Pin, GPIO_PIN_RESET);
		}

		if( isCenter())    {
	        HAL_GPIO_WritePin(YellowLight_GPIO_Port, YellowLight_Pin, GPIO_PIN_SET);
		} else {
	        HAL_GPIO_WritePin(YellowLight_GPIO_Port, YellowLight_Pin, GPIO_PIN_RESET);
		}

		if( isBottom())    {
	        HAL_GPIO_WritePin(GreenLight_GPIO_Port, GreenLight_Pin, GPIO_PIN_SET);
		} else {
	        HAL_GPIO_WritePin(GreenLight_GPIO_Port, GreenLight_Pin, GPIO_PIN_RESET);
		}

		if(isLeft() && isRight()) return;
	}
}

void randomFlashing() {

    // Optional: seed the RNG once if needed
    srand(HAL_GetTick());  // Uses system uptime in ms

	lcd_clear();
	lcd_write_string("Random Lights");
	lcd_set_cursor(0, 1);
	lcd_write_string("Left & Right to Exit");

    while (1) {
        // Randomly set each light ON or OFF
        HAL_GPIO_WritePin(RedLight_GPIO_Port, RedLight_Pin, rand() % 2);
        HAL_GPIO_WritePin(GreenLight_GPIO_Port, GreenLight_Pin, rand() % 2);
        HAL_GPIO_WritePin(YellowLight_GPIO_Port, YellowLight_Pin, rand() % 2);

        // Delay a little between flashes
        HAL_Delay(200);

		if(isLeft() && isRight()) return;
    }
}



	void moveLoop() {


//		uint8_t exitting = 0;
//
//		uint8_t  L_pressed = 0;
//		uint64_t L_time = 0;
//
//		uint8_t  R_pressed = 0;
//		uint64_t R_time = 0;
//
//		uint8_t  T_pressed = 0;
//		uint64_t T_time = 0;
//
//		uint8_t  B_pressed = 0;
//		uint64_t B_time = 0;
//
//		uint8_t  C_pressed = isCenter();
//		uint64_t C_time = getTicks();
//

//		// we have time to checks things before the event will occur
//		checkButton(&L_pressed, &L_time, isLeft(), now);
//		checkButton(&R_pressed, &R_time, isRight(), now);
//		if(checkButton(&T_pressed, &T_time, isTop(), now)) {
//			if(T_pressed) s->desired_speed *= 1.1;
//		}
//
//		if(checkButton(&B_pressed, &B_time, isBottom(), now)) {
//			if(B_pressed) s->desired_speed /= 1.1;
//		}
//
//		if(checkButton(&C_pressed, &C_time, isCenter(), now)) {
//			if(C_pressed) {
//				exitting = 1;
//				s->desired_velocity = 0;
//			}
//
//		}

	}

#define STOPPED 1
#define ACCELERATE 2
#define STEADY 3
#define DECCELERATE 4

	void wait(uint16_t stop) {
		uint16_t curr = TIM3->CNT;
		if(curr > stop) {
			uint16_t prev = curr;
			while(1) {
				curr = TIM3->CNT;
				if(curr < prev) break;
				prev = curr;
			}
		}

		while(TIM3->CNT <= stop) {}

//	HAL_Delay(1);
	}

	void runStepper() {
		printf("Entering runStepper\r\n");

		lcd_clear();
		lcd_write_string("Run Stepper Test");
		lcd_set_cursor(0, 1);

		uint16_t last = TIM3->CNT;
		double a = 3000;
		double vmax = 500;
		int state = ACCELERATE;
		double velocity = 0;

		uint32_t pos = 0;

		while(1) {
			HAL_GPIO_WritePin(BarrelPulse_GPIO_Port, BarrelPulse_Pin, GPIO_PIN_SET);
			last += 180;
			wait(last);
			HAL_GPIO_WritePin(BarrelPulse_GPIO_Port, BarrelPulse_Pin, GPIO_PIN_RESET);
			pos++;

			if(pos==2000) {
				printf("decel\r\n");
				state = DECCELERATE;
			}

			if(state == ACCELERATE) {
				velocity = sqrt(velocity*velocity + 2.0*a);
				if(velocity > vmax) {
					velocity = vmax;
					state = STEADY;
				}
			}

			if(state == DECCELERATE) {
				double v2 = velocity * velocity - 2 * a;
				if(v2 < 0) {
					velocity = 0;
					state = STOPPED;
					return;
				}
				velocity = sqrt(v2);
			}

			last += (uint16_t) (TIMER_CLOCK_HZ / velocity);
			wait(last);

		}

	}

	void buttonTest() {

		uint8_t  L_pressed = 0;
		uint8_t  R_pressed = 0;
		uint8_t  T_pressed = 0;
		uint8_t  B_pressed = 0;
		uint8_t  C_pressed = 0;
		uint8_t changed = 1;

		lcd_clear();
		lcd_write_string("Button Test");
		lcd_set_cursor(0, 1);
		lcd_write_string("Left & Right to Exit");
		lcd_set_cursor(0, 2);
		lcd_write_string("Pressed: ");

		while(1) {
			if( L_pressed != isLeft())   {changed = 1; L_pressed = isLeft(); }
			if( R_pressed != isRight())  {changed = 1; R_pressed = isRight(); }
			if( T_pressed != isTop())    {changed = 1; T_pressed = isTop(); }
			if( B_pressed != isBottom()) {changed = 1; B_pressed = isBottom(); }
			if( C_pressed != isCenter()) {changed = 1; C_pressed = isCenter(); }

			if(changed) {
				lcd_set_cursor(0, 2);
				lcd_write_string("Pressed: ");
				if(L_pressed) lcd_write_string("L");
				if(R_pressed) lcd_write_string("R");
				if(T_pressed) lcd_write_string("T");
				if(B_pressed) lcd_write_string("B");
				if(C_pressed) lcd_write_string("C");
				lcd_write_string("     ");
				changed = 0;
			}

			if(isLeft() && isRight()) return;
		}
	}

void showDisplay(Stepper* s, uint8_t left) {
    char buffer[20];
    lcd_clear();
    lcd_write_string(left?"Left ":"Right ");
    sprintf(buffer, "%.1f", s->desiredSpeed);
    lcd_write_string(buffer);
}

void moveStepper(Stepper* s) {
    uint8_t exitting = 0;
    uint8_t running = 0;
    uint8_t left = 0;

    ButtonState bs;
    initializeButtonState(&bs);

    lcd_clear();
    lcd_write_string("Choose a direction");

    while(!exitting) {
        updateButtonState(&bs);

        if(isLeft() && isRight()) break;

        if(s->currentSpeed == 0.0f) {
            running = 0;
        }

        if(newPress(&bs.left) && !running) {
            running = 1;
            HAL_GPIO_WritePin(s->config->directionPort, s->config->directionPin, GPIO_PIN_SET);
            stepperStart(s, 1000, 0);
            printf("left %.1f %.1f\r\n", s->desiredSpeed, s->currentSpeed);
            left = 1;
            showDisplay(s, left);

        }

        if(newPress(&bs.right) && !running) {
            running = 1;
            HAL_GPIO_WritePin(s->config->directionPort, s->config->directionPin, GPIO_PIN_RESET);
            stepperStart(s, 1000, 0);
            printf("right %.1f %.1f\r\n", s->desiredSpeed, s->currentSpeed);
            left = 0;
            showDisplay(s, left);
        }

        if(newPress(&bs.center)) {
            s->desiredSpeed = 0.0f;
            printf("center %.1f %.1f\r\n", s->desiredSpeed, s->currentSpeed);
            showDisplay(s, left);
        }

        if(newPress(&bs.top)) {
             s->desiredSpeed *= 1.1;
             printf("top %.1f %.1f\r\n", s->desiredSpeed, s->currentSpeed);
             showDisplay(s, left);
        }

        if(newPress(&bs.bottom)) {
            s->desiredSpeed /= 1.1;
            printf("bottom %.1f %.1f\r\n", s->desiredSpeed, s->currentSpeed);
            showDisplay(s, left);
        }
	}

    s->desiredSpeed = 0.0f;
    while( s->currentSpeed > 0.0f ) {}

}


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

	void displayInfo(char* date, char* time) {

		lcd_clear();
		lcd_set_cursor(0, 0);
		lcd_write_string("Winder ");
		lcd_write_string(date);

		lcd_set_cursor(7, 1);
		lcd_write_string(__TIME__);

		lcd_set_cursor(7, 2);
		lcd_write_string(GIT_HASH);

		lcd_set_cursor(0, 3);
		lcd_write_string("Center to continue. ");


		ButtonState bs;
	    initializeButtonState(&bs);


		while(1) {
	        updateButtonState(&bs);
	        if(newPress(&bs.center)) return;
		}
	}




int main_menu(char* date, char* time) {

    char* prompt = "Select:";
    char* selections[] = {
            "Move Carriage",
            "Move Barrel",
            "Barrel Profile 1",
            "Barrel Profile 2",
            "Test Lights",
            "Test Left Limit",
            "Test Right Limit",
            "Random Flashing",
            "Button Test",
            "Display Info"
    };

    initializeButtonState(&bs);

    int start = 0;
    int current = 0;
    int count = sizeof(selections) / sizeof(char*);
    printf("number of menu selections: %d\r\n", count);

    display_menu(prompt, selections, count, current, start);

    while(1) {
        updateButtonState(&bs);

        if(newPress(&bs.top)) {
            current--;
            if(current < 0) current = 0;
            if(current < start) start--;
            display_menu(prompt, selections, count, current, start);
        }

        if(newPress(&bs.bottom)) {
            current++;
            if(current>=count) current = count-1;
            if(current >= start+3) start++;
            display_menu(prompt, selections, count, current, start);
        }

        if(newPress(&bs.center)) {
            printf("\r\n\nselected: %d %s\r\n\n", current, selections[current]);

            display_menu(prompt, selections, count, current, start);

            if(strcmp(selections[current], "Move Carriage") == 0) moveStepper(&carriageStepper);
            if(strcmp(selections[current], "Move Barrel") == 0) moveStepper(&barrelStepper);


            if(strcmp(selections[current], "Test Lights") == 0) testLights();
            if(strcmp(selections[current], "Random Flashing") == 0) randomFlashing();
            if(strcmp(selections[current], "Button Test") == 0) buttonTest();
            if(strcmp(selections[current], "Display Info") == 0) displayInfo(date, time);

            display_menu(prompt, selections, count, current, start);
        }
    }

    return 0;
}
