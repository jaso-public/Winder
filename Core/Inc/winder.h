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

// button debounce is 50ms or 180000 clock cycles
#define DEBOUNCE  60000

#define RED    1
#define YELLOW 2
#define GREEN  3

typedef struct {
    uint16_t time;
    int count;
    int last;
    uint8_t  pressed;
} Button;

typedef struct {
    Button left;
    Button right;
    Button center;
    Button top;
    Button bottom;
} ButtonState;

void initializeButtonState(ButtonState *bs);
void updateButtonState(ButtonState *bs);
int newPress(Button *b);

void setLight(int color);

int isLeft();
int isRight();
int isTop();
int isBottom();
int isCenter();

int main_menu();

#endif /* INC_WINDER_H_ */
