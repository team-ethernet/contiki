#include <avr/pgmspace.h>
#include "rss2.h"
#include "pwr.h"

void pwr_1_init() {
	DDRE |= (1<<PWR_1);
	pwr_1_off();
}

void pwr_1_on() {
	PORTE &= ~(1<<PWR_1);
}

void pwr_1_off() {
	PORTE |= (1<<PWR_1);
}

void pwr_1_toggle() {
	PORTE ^= (1<<PWR_1);
}