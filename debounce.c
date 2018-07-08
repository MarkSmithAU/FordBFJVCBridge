/*
(c) Mark Smith 2018
GPL v3
Not licensed for commercial use
*/

#include "debounce.h"

enum {DEBOUNCE_INACTIVE, DEBOUNCE_ACTIVE, DEBOUNCE_DEBOUNCE};

void initDebounce(debounceData *data, unsigned long ms, char idleState, char oneShot)
{
	data->debounceTime = 0;
	data->debounceTarget = ms;
	data->inactiveState = idleState;
	data->oneShot = oneShot;
	data->state = DEBOUNCE_INACTIVE;
}

extern volatile unsigned char tick;

char getDebounced(debounceData *data, char value) {
	switch (data->state) {
		case DEBOUNCE_INACTIVE:
			// has the button been pressed?
			if (value != data->inactiveState) {
				data->state = DEBOUNCE_DEBOUNCE;
				data->debounceTime = 0L;
			}
			return data->inactiveState;

		case DEBOUNCE_DEBOUNCE:
			if (value != data->inactiveState) {
				if (data->debounceTime >= data->debounceTarget) {
					data->state = DEBOUNCE_ACTIVE;
					return value;
				}
				if (tick) {
					data->debounceTime++;
				}
			}
			else {
				data->state = DEBOUNCE_INACTIVE;
			}
			return data->inactiveState;

		case DEBOUNCE_ACTIVE:
			if (value != data->inactiveState) {
				if (data->oneShot == 0) {
					return value;
				} 
				else {
					return data->inactiveState;
				}

			}
			else {
				data->state = DEBOUNCE_INACTIVE;
			}
			return data->inactiveState;
	}
	return data->inactiveState;
}
