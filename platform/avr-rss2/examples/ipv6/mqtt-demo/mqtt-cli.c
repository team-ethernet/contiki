#include <stdio.h>
#include <stdarg.h>

#define BUFSIZE 512
static char mqttbuf[BUFSIZE];
static int remaining;

void
handle_serial_input(const char *line);

char
*handle_mqtt_input(const char *line) {
	remaining = BUFSIZE;
	mqttbuf[0] = 0;
	handle_serial_input(line);
	return mqttbuf;
}

static void
xprintf(char *fmt, ...) {
	va_list args;
	int len;
	
	va_start(args, fmt);
	len = vsnprintf(&mqttbuf[BUFSIZE-remaining], remaining, fmt,  args); 
	va_end(args);
	if (len >= remaining) {
		mqttbuf[BUFSIZE-1] = 0;
		remaining = 0;
	}
	else {
		remaining -= len; /* Don't count terminating null */
	}
}

#define printf(...) xprintf(__VA_ARGS__)

#include "apps/cli/cli.c"
