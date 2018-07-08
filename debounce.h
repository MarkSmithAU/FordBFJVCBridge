/*
(c) Mark Smith 2018
GPL v3
Not licensed for commercial use
*/

// Debounce Data
struct debounce_data {
	char state; // state machine state
	unsigned long debounceTime; // this counts the milliseconds
	unsigned long debounceTarget; // taregt to reach for state transition
	char inactiveState; // the line is not active when it is in this state
	char oneShot; // should we only return positive once, (i.e. on the state transition)
};
typedef struct debounce_data debounceData;

char getDebounced(debounceData *data, char value);
void initDebounce(debounceData *data, unsigned long ms, char idleState, char oneShot);
