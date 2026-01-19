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
#include "winder.h"
#include "gitversion.h"

extern Stepper barrelStepper;
extern Stepper carriageStepper;


#define STEPS_PER_INCH 25000
#define STEPS_PER_REVOLUTION 10880
#define COUNT_PER_INCH 610
#define CELLS 128

#define DEBOUNCE_DELAY 20

extern ButtonState bs;


// the y component of the motion (this is the amount of curved length around the barrel)
float yDis(float eCnt, float xCnt) {
    float h = eCnt / COUNT_PER_INCH;
    float a = xCnt / STEPS_PER_INCH;
    return sqrt(h*h - a*a);
}

float circumference(float yCnt, float yDis) {
    return yDis * STEPS_PER_REVOLUTION / yCnt;
}

float diameter(float circumference) {
    return circumference / 3.14159268;
}
int isLeft() {
	return HAL_GPIO_ReadPin(ButtonLeft_GPIO_Port, ButtonLeft_Pin) == 0;
}


int x[CELLS];
int y[CELLS];
int e[CELLS];


void waitAndRecord(int cell) {
    if(carriageStepper.stepIncrement > 0) {
        while(carriageStepper.currentPosition < x[cell] || carriageStepper.mode != STOPPED);
    } else {
        while(carriageStepper.currentPosition > x[cell] || carriageStepper.mode != STOPPED);
    }

    y[cell] = barrelStepper.currentPosition;
    e[cell] = readEncoderValue();

    printf("pos:%ld  x:%d y:%d e:%d\r\n", carriageStepper.currentPosition, x[cell], y[cell], e[cell]);
}

void profile1() {
    float start =  0.450;
//  float end =   17.625;
    float end =    7.625;

    float delta = end-start;

    int cells = delta * 4;
    float width = delta / cells;

    for(int i=0 ; i<cells ; i++) {
        x[i] = (start + (i * width)) * STEPS_PER_INCH;
    }
    x[cells] = end * STEPS_PER_INCH;

    // for(int i=0 ; i<=cells ; i++) printf("%d: %d  delta:%d\n", i, x[i], (x[i+1]-x[i]));


    // start the barrel rotating
    barrelStepper.currentPosition = 0;
    stepperStart(&barrelStepper, STEPS_PER_REVOLUTION/4, CW);

    // make the first pass measurements
    moveToPosition(&carriageStepper, STEPS_PER_INCH/2, x[cells]);
    for(int i=0 ; i<=cells ; i++) waitAndRecord(i);

    stop(&barrelStepper);
    waitUntilStopped(&barrelStepper);

    printStepperInfo(&barrelStepper);
    printStepperInfo(&carriageStepper);

    for(int i=0 ; i<=cells ; i++) printf("%d: x:%d(%d) y:%d(%d) e:%d(%d)\r\n", i, x[i], (x[i+1]-x[i]), y[i], (y[i+1]-y[i]), e[i], (e[i+1]-e[i]));
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
    uint16_t now = TIM1->CNT;
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


void readEncoder() {


    lcd_clear();
    lcd_write_string("Read Encoder");


    lcd_set_cursor(0, 3);
    lcd_write_string("Left & Right to Exit");


    while (1) {
        uint32_t count = readEncoderValue();
        char buffer[40];
        sprintf(buffer, "count: %ld   ", count);

        lcd_set_cursor(0, 1);
        lcd_write_string(buffer);

        for(int i=0 ; i<200 ; i++) {
            if(isLeft() && isRight()) return;
            HAL_Delay(1);
        }
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
            stepperStart(s, 10000, CW);
            printf("left %.1f %.1f\r\n", s->desiredSpeed, s->currentSpeed);
            left = 1;
            showDisplay(s, left);
        }

        if(newPress(&bs.right) && !running) {
            running = 1;
            stepperStart(s, 10000, CCW);
            printf("right %.1f %.1f\r\n", s->desiredSpeed, s->currentSpeed);
            left = 0;
            showDisplay(s, left);
        }

        if(newPress(&bs.center)) {
            stop(s);
            printf("center %.1f %.1f\r\n", s->desiredSpeed, s->currentSpeed);
            showDisplay(s, left);
        }

        if(newPress(&bs.top)) {
             changeSpeed(s, s->desiredSpeed * 1.1);
             printf("top %.1f %.1f\r\n", s->desiredSpeed, s->currentSpeed);
             showDisplay(s, left);
        }

        if(newPress(&bs.bottom)) {
            changeSpeed(s, s->desiredSpeed / 1.1);
            printf("bottom %.1f %.1f\r\n", s->desiredSpeed, s->currentSpeed);
            showDisplay(s, left);
        }
	}

    stop(s);
    waitUntilStopped(s);

}


void moveToPositionAndWait(int speed, int position) {
    moveToPosition(&carriageStepper, speed, position);
    waitUntilStopped(&carriageStepper);
}

void waitForOptical(int speed) {
    // move right until the optical sensor trips
    stepperStart(&carriageStepper, speed, CCW);
    while(HAL_GPIO_ReadPin(OpticalSensor_GPIO_Port, OpticalSensor_Pin)) {}
    long oldValue = carriageStepper.currentPosition;
    carriageStepper.currentPosition = 0;
    stop(&carriageStepper);
    waitUntilStopped(&carriageStepper);
    printf("oldValue: %ld   finalPosition: %ld\n", oldValue, carriageStepper.currentPosition);
    moveToPositionAndWait(100, 0);
}

void homeCarriage() {
    lcd_clear();
    lcd_write_string("Home the carriage");

    if(HAL_GPIO_ReadPin(OpticalSensor_GPIO_Port, OpticalSensor_Pin) == GPIO_PIN_RESET ) {
        lcd_set_cursor(0, 1);
        lcd_write_string("Optosensor engaged");
        lcd_set_cursor(0, 2);
        lcd_write_string("Moving to the right");

        carriageStepper.currentPosition = 0;
        moveToPositionAndWait(5000, 20000);
    }

    lcd_clear();
    lcd_write_string("Home the carriage");
    lcd_set_cursor(0, 1);
    lcd_write_string("High spd to sensor");

    // move right until the optical sensor trips
    waitForOptical(15000);


    lcd_clear();
    lcd_write_string("Home the carriage");
    lcd_set_cursor(0, 1);
    lcd_write_string("move away from sensor");

    moveToPositionAndWait(1000, 1000);


    lcd_clear();
    lcd_write_string("Home the carriage");
    lcd_set_cursor(0, 1);
    lcd_write_string("Slow spd to sensor");

    waitForOptical(250);
}

void displayOpto(int opt) {
    lcd_clear();
    lcd_write_string("Opto Sensor Test");
    lcd_set_cursor(0, 1);
    lcd_write_string(opt ? "Sensor is High" : "Sensor is Low");
    lcd_set_cursor(0, 2);
    lcd_write_string("Press C to exit");
}

void optoTest() {

    int state = HAL_GPIO_ReadPin(OpticalSensor_GPIO_Port, OpticalSensor_Pin) == GPIO_PIN_SET;
    displayOpto(state);

    while(1) {
        updateButtonState(&bs);

        int newState = HAL_GPIO_ReadPin(OpticalSensor_GPIO_Port, OpticalSensor_Pin) == GPIO_PIN_SET;
        if(state != newState) {
            state = newState;
            displayOpto(state);
        }
        if(newPress(&bs.center)) break;
    }
}

void calibrateCarriage() {

    char buffer[20];

    lcd_clear();
    lcd_set_cursor(0, 0);
    snprintf(buffer, sizeof(buffer), "Start: %ld", carriageStepper.currentPosition);
    lcd_write_string(buffer);


    uint32_t distance = STEPS_PER_INCH * 24;

    moveToPosition(&carriageStepper, 16000, carriageStepper.currentPosition + distance);

    uint32_t pos = 0;
    do {
        pos = carriageStepper.currentPosition;
        lcd_set_cursor(0, 1);
        snprintf(buffer, sizeof(buffer), "Position: %ld", pos);
        lcd_write_string(buffer);
        HAL_Delay(200);
    } while(carriageStepper.desiredPosition != pos);

    lcd_set_cursor(0, 3);
    lcd_write_string("'C' to exit");

    while(1) {
        updateButtonState(&bs);
        if(newPress(&bs.center)) break;
    }
}


void calibrateBarrel() {

    char buffer[20];

    lcd_clear();
    lcd_set_cursor(0, 0);
    snprintf(buffer, sizeof(buffer), "Start: %ld", barrelStepper.currentPosition);
    lcd_write_string(buffer);


    uint32_t distance = STEPS_PER_REVOLUTION * 10;

    moveToPosition(&barrelStepper, 16000, barrelStepper.currentPosition + distance);

    uint32_t pos = 0;
    do {
        pos = barrelStepper.currentPosition;
        lcd_set_cursor(0, 1);
        snprintf(buffer, sizeof(buffer), "Position: %ld", pos);
        lcd_write_string(buffer);
        HAL_Delay(200);
    } while(barrelStepper.desiredPosition != pos);

    lcd_set_cursor(0, 3);
    lcd_write_string("'C' to exit");

    while(1) {
        updateButtonState(&bs);
        if(newPress(&bs.center)) break;
    }
}

void doBoth() {
    char buffer[20];

    lcd_clear();
    lcd_set_cursor(0, 0);
    snprintf(buffer, sizeof(buffer), "Start: %ld", carriageStepper.currentPosition);
    lcd_write_string(buffer);


    uint32_t distance = STEPS_PER_INCH * 17.5;

    moveToPosition(&carriageStepper, STEPS_PER_INCH/2, carriageStepper.currentPosition + distance);
    stepperStart(&barrelStepper, STEPS_PER_REVOLUTION/4, CW);


    int32_t step = STEPS_PER_INCH/4;
    int32_t next = carriageStepper.currentPosition + step;

    int32_t c=0, b=0, e=0;
    int32_t co=0, bo=0, eo=0;

    do {
        do {
            c = carriageStepper.currentPosition;
            if(isCenter()) break;
        } while( c<next);

        next += step;

        b = barrelStepper.currentPosition;
        e = readEncoderValue();

        lcd_set_cursor(0, 0);
        snprintf(buffer, sizeof(buffer), "Encoder:  %ld   ", e);
        lcd_write_string(buffer);

        lcd_set_cursor(0, 2);
        snprintf(buffer, sizeof(buffer), "Barrel:   %ld   ", b);
        lcd_write_string(buffer);

        lcd_set_cursor(0, 1);
        snprintf(buffer, sizeof(buffer), "Carriage: %ld   ", c);
        lcd_write_string(buffer);

        float yd= yDis((e-eo), (c-co));
        float circum = circumference((b-bo), yd);

        lcd_set_cursor(0, 3);
        snprintf(buffer, sizeof(buffer), "Circum: %.3f   ", circum);
        lcd_write_string(buffer);


        printf("%ld(%ld) %ld(%ld) %ld(%ld) dia:%f\r\n", c,(c-co), b, (b-bo), e, (e-eo), circum);
        co=c; bo=b; eo = e;

        // break out of the loop to stop early
        if(isCenter()) break;

    } while(carriageStepper.desiredPosition != c);

    stop(&barrelStepper);
    stop(&carriageStepper);
    printf("told the steppers to stop\r\n");
    waitUntilStopped(&barrelStepper);
    waitUntilStopped(&carriageStepper);

    printStepperInfo(&barrelStepper);
    printStepperInfo(&carriageStepper);

    // wait to return to main menu
    lcd_set_cursor(0, 3);
    lcd_write_string("'C' to exit");

    while(1) {
        updateButtonState(&bs);
        if(newPress(&bs.center)) break;
    }




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
            "Home Carriage",
            "Profile 1",
            "Profile 2",
            "Move Carriage",
            "Move Barrel",
            "Read Encoder",
            "Calibrate Carriage",
            "Calibrate Barrel",
            "Test Lights",
            "Test Left Limit",
            "Test Right Limit",
            "Random Flashing",
            "Button Test",
            "Opto Test",
            "Display Info"
    };

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

            if(strcmp(selections[current], "Home Carriage") == 0) homeCarriage();
            if(strcmp(selections[current], "Profile 1") == 0) profile1();

            if(strcmp(selections[current], "Move Carriage") == 0) moveStepper(&carriageStepper);
            if(strcmp(selections[current], "Move Barrel") == 0) moveStepper(&barrelStepper);

            if(strcmp(selections[current], "Calibrate Carriage") == 0) calibrateCarriage();
            if(strcmp(selections[current], "Calibrate Barrel") == 0) calibrateBarrel();


            if(strcmp(selections[current], "Profile 2") == 0) doBoth();


            if(strcmp(selections[current], "Read Encoder") == 0) readEncoder();

            if(strcmp(selections[current], "Test Lights") == 0) testLights();
            if(strcmp(selections[current], "Opto Test") == 0) optoTest();
            if(strcmp(selections[current], "Random Flashing") == 0) randomFlashing();
            if(strcmp(selections[current], "Button Test") == 0) buttonTest();
            if(strcmp(selections[current], "Display Info") == 0) displayInfo(date, time);

            display_menu(prompt, selections, count, current, start);
        }
    }

    return 0;
}
