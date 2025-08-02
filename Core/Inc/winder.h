/*
 * winder.h
 *
 *  Created on: Jul 10, 2025
 *      Author: jaso
 */

#ifndef INC_WINDER_H_
#define INC_WINDER_H_

// the clock is running at 36 Mhz
#define TICKS_PER_SEC 36000000

// stepper acceleration steps/second^2
#define ACCELERATION 300000


// how wide is the pulse (20 microseconds)
#define PULSE_WIDTH_TICKS 720

// if an event will happen with 100 microseconds then just wait to fire the event.
#define TOO_CLOSE_TO_EVENT 3600

// button debounce is 50ms or 180000 clock cylces
#define DEBOUNCE  180000

#define IDLE         1
#define START_PULSE  2
#define END_PULSE    3
#define COMPUTE_NEXT 4

#define RED    1
#define YELLOW 2
#define GREEN  3


typedef struct {
	GPIO_TypeDef* stepGroup;
	uint16_t stepPin;

	GPIO_TypeDef* dirGroup;
	uint16_t dirPin;

	double lastPulseTime;
	double desired_speed;
	double desired_velocity;
	double current_velocity;
	double acceleration;

	uint8_t  state;
	uint64_t nextEvent;
	uint64_t nextPulseStart;
	uint64_t nextPulseEnd;
} stepper_t;


int isLeft();
int isRight();
int isTop();
int isBottom();
int isCenter();

int main_menu();

#endif /* INC_WINDER_H_ */
