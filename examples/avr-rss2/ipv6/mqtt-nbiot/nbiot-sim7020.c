/*
 * Copyright (c) 2017, Copyright Robert Olsson
 * KTH Royal Institute of Technology NSLAB KISTA STOCHOLM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 *
 * Author  : Robert Olsson roolss@kth.se
 * Created : 2017-05-22
 */

/**
 * \file
 *         Example uses avr-rss2 platform
 *         
 */

#include "contiki.h"
#include "sys/etimer.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "i2c.h"
#include "dev/leds.h"
#include "tcp-socket-gprs.h"
#include "gprs-a6.h"
#include "at_wait.h"

#include "dev/rs232.h"
#include "lib/ringbuf.h"

#define APN GPRS_CONF_APN
#define PDPTYPE "IP"
//#define PDPTYPE "IPV6"

uint32_t baud;

uint8_t uart = RS232_PORT_1;

#define GPRS_MAX_CONNECTION 1
struct gprs_connection gprs_connections[GPRS_MAX_CONNECTION];
struct gprs_context gprs_context;

process_event_t uart_input_event;

static struct gprs_status status;

static void
enqueue_event(process_event_t ev, void *data);

#define min(A, B) ((A) <= (B) ? (A) : (B))
/*---------------------------------------------------------------------------*/
PROCESS(uart_reader, "UART input process");
PROCESS(a6at, "GPRS A6 module");


#define IGNORE_CHAR(c) (c == 0x0d)
#define END 0x0a

#define BUFSIZE 128
static struct ringbuf rxbuf;
static uint8_t rxbuf_data[BUFSIZE];

int
uart_input_byte(unsigned char c)
{
  static uint8_t overflow = 0; /* Buffer overflow: ignore until END */

#if 0
  if(IGNORE_CHAR(c)) {
    return 0;
  }
#endif
  
  //printf("%02x ", c);
  //printf("%c", c);
  
  if(!overflow) {
    /* Add character */
    if(ringbuf_put(&rxbuf, c) == 0) {
      /* Buffer overflow: ignore the rest of the line */
      overflow = 1;
    }
  } else {
    printf("OVERFLOW±n");
    /* Buffer overflowed:
     * Only (try to) add terminator characters, otherwise skip */
    if(c == END && ringbuf_put(&rxbuf, c) != 0) {
      overflow = 0;
    }
  }
  return 1;
}

/*---------------------------------------------------------------------------*/
void
gprs_init() {
  gprs_set_context(&gprs_context, PDPTYPE, APN);
  memset(&gprs_statistics, 0, sizeof(gprs_statistics));
  process_start(&uart_reader, NULL);
  process_start(&a6at, NULL);
}
/*---------------------------------------------------------------------------*/
struct gprs_connection *
alloc_gprs_connection() {
  return &gprs_connections[GPRS_MAX_CONNECTION-1];
}
/*---------------------------------------------------------------------------*/
static void
free_gprs_connection(struct gprs_connection *gprsconn) {
  return;
}
/*---------------------------------------------------------------------------*/
static struct gprs_connection *
find_gprs_connection() {
  return &gprs_connections[GPRS_MAX_CONNECTION-1];
}
/*---------------------------------------------------------------------------*/
static void
call_event(struct tcp_socket_gprs *s, tcp_socket_gprs_event_t event)
{
  if(s != NULL && s->event_callback != NULL) {
    s->event_callback(s, s->ptr, event);
  }
}
/*---------------------------------------------------------------------------*/

static void
gprs_call_event(struct gprs_connection *gprsconn, gprs_conn_event_t event)
{
  if(gprsconn != NULL && gprsconn->event_callback != NULL) {
    gprsconn->event_callback(gprsconn, gprsconn->callback_arg, event);
  }
}
/*---------------------------------------------------------------------------*/
int
gprs_set_context(struct gprs_context *gcontext, char *pdptype, char *apn) {
    
  gcontext->active = 0;
  if (strcmp(pdptype, "IP") == 0 || strcmp(pdptype, "IPV6") == 0) {
    gcontext->pdptype = pdptype;
  }
  else
    return -1;
  strncpy(gcontext->apn, apn, GPRS_MAX_APN_LEN);
  gcontext->apn[GPRS_MAX_APN_LEN] = '\0';
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Do we need this */
int
gprs_register(struct gprs_connection *gconn,
              void *callback_arg,
              void *callback,
              gprs_data_callback_t input_callback,
              gprs_event_callback_t event_callback) {

  gconn->socket = callback_arg;
  gconn->callback = callback;
  gconn->callback_arg = callback_arg;
  gconn->input_callback = input_callback;
  gconn->event_callback = event_callback;  
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Do we need this */
int
gprs_unregister(struct gprs_connection *gconn) {

  gconn->socket = NULL;
  gconn->callback_arg = NULL;
  return 0;
}
/*---------------------------------------------------------------------------*/
struct gprs_connection *
gprs_connection(struct gprs_connection *gprsconn,
                const char *proto, const char *ipaddr, uint16_t port,
                struct tcp_socket_gprs *socket) {

  if (gprsconn == NULL) {
    return NULL;
  }
  gprsconn->context = &gprs_context;
  gprsconn->proto = proto;
  gprsconn->socket = socket;
  gprsconn->ipaddr = ipaddr;
  gprsconn->port = port;
  enqueue_event(a6at_gprs_connection, gprsconn);
  return gprsconn;

#if 0
  if (process_post(&a6at, a6at_gprs_connection, gprsconn) != PROCESS_ERR_OK) {
    free_gprs_connection(gprsconn);
    return NULL;
  }
  else {
    return gprsconn;
  }
#endif
}
/*---------------------------------------------------------------------------*/
void
//gprs_send(struct tcp_socket_gprs *socket) {
gprs_send(struct gprs_connection *gprsconn) {
  //(void) process_post(&a6at, a6at_gprs_send, gprsconn);
  enqueue_event(a6at_gprs_send, gprsconn);
}
/*---------------------------------------------------------------------------*/
void
gprs_close(struct tcp_socket_gprs *socket) {
  enqueue_event(a6at_gprs_close, socket->g_c);
  //(void) process_post(&a6at, a6at_gprs_close, socket->g_c);
}
/*---------------------------------------------------------------------------*/
struct gprs_status *
gprs_status() {
  return &status;
}
/*---------------------------------------------------------------------------*/

static
PT_THREAD(wait_csonmi_callback(struct pt *pt, struct at_wait *at, int c));
static
PT_THREAD(wait_ciprcv_callback(struct pt *pt, struct at_wait *at, int c));
static
PT_THREAD(wait_tcpclosed_callback(struct pt *pt, struct at_wait *at, int c));

static
struct at_wait wait_csonmi = {"+CSONMI: ", wait_csonmi_callback, at_match_byte};
static
struct at_wait wait_ciprcv = {"+CIPRCV:", wait_ciprcv_callback, at_match_byte};
static
struct at_wait wait_ok = {"OK", wait_readline_pt, at_match_byte};
static
struct at_wait wait_error = {"ERROR", NULL, at_match_byte};
static
struct at_wait wait_cmeerror = {"+CME ERROR:", wait_readline_pt, at_match_byte};
static
struct at_wait wait_sendprompt = {">", NULL, at_match_byte};
static
struct at_wait wait_tcpclosed = {"+TCPCLOSED:", wait_tcpclosed_callback, at_match_byte};
static
struct at_wait wait_csoerr = {"+CSOERR:", wait_tcpclosed_callback, at_match_byte};
static
struct at_wait wait_csq = {"+CSQ:", wait_readline_pt, at_match_byte};
static
struct at_wait wait_dataaccept = {"DATA ACCEPT: ", wait_readline_pt, at_match_byte};
static
struct at_wait wait_ati = {"Ai Thinker", wait_readlines_pt, at_match_byte};

/*
 * CIPRCV:len,<data>
 */
static
PT_THREAD(wait_ciprcv_callback(struct pt *pt, struct at_wait *at, int c)) {
  static uint8_t rcvdata[GPRS_MAX_RECV_LEN];
  static int rcvpos;
  static uint16_t nbytes;
  struct gprs_connection *gprsconn;
  
  PT_BEGIN(pt);

  /* Get length as a decimal number followed by a comma ','
   */
  nbytes = 0;
  while (c != ',') {
    if (!isdigit(c)) {
      /* Error: bad len */
      printf("ciprcv_callback: bad len\n");
      restart_at(&wait_ciprcv); /* restart */
      PT_EXIT(pt);
    }

    nbytes = nbytes*10 + (c - '0');
    PT_YIELD(pt);
  }

  /* Consume the comma ',' in input */
  PT_YIELD(pt);

  if (nbytes > GPRS_MAX_RECV_LEN)
    nbytes = GPRS_MAX_RECV_LEN;
  rcvpos = 0;
  while (rcvpos < nbytes) {
    rcvdata[rcvpos++] = (uint8_t) c;
    PT_YIELD(pt);
  }

  restart_at(&wait_ciprcv); /* restart */
  gprsconn = find_gprs_connection();
  if (gprsconn) {
    gprsconn->input_callback(gprsconn, gprsconn->callback_arg, rcvdata, nbytes);
  }
  PT_END(pt);
}

/*
 * CIPRCV:len,<data>
 * +CSONMI: 0,8,20020000 *

 */
static
PT_THREAD(wait_csonmi_callback(struct pt *pt, struct at_wait *at, int c)) {
  static uint8_t rcvdata[GPRS_MAX_RECV_LEN];
  static uint16_t rcvlen;
  static uint16_t nbytes;
  static uint16_t port;
  static int rcvpos;

  struct gprs_connection *gprsconn;
  
  PT_BEGIN(pt);

  port = 0;
  while (isdigit(c)) {
    port = port*10 + (c - '0');
    PT_YIELD(pt);
  }
  
  if (c != ',') {
    printf("Expected ',', got '%c'\n'", c);
    restart_at(&wait_csonmi); /* restart */
    PT_EXIT(pt);
  }
  PT_YIELD(pt);
  
  nbytes = 0;
  while (isdigit(c)) {
    nbytes = nbytes*10 + (c - '0');
    PT_YIELD(pt);
  }

  /* Consume the comma ',' in input */
  if (c != ',') {
    printf("Expected ',', got '%c'\n'", c);
    restart_at(&wait_csonmi); /* restart */
    PT_EXIT(pt);
  }
  PT_YIELD(pt);

  /* Data is encoded as hex string, so
   * data length is half the string length */ 
  rcvlen = nbytes >> 1;
  if (rcvlen > GPRS_MAX_RECV_LEN)
    rcvlen = GPRS_MAX_RECV_LEN;

  rcvpos = 0;
  /* Get one hex byte at a time */
  static char hexstr[3];
  while (1) {
    hexstr[0] = c;
    nbytes--;
    PT_YIELD(pt);
    hexstr[1] = c;
    nbytes--;
    hexstr[2] = 0;
    if (rcvpos < GPRS_MAX_RECV_LEN) {
      rcvdata[rcvpos++] = (uint8_t) strtoul(hexstr, NULL, 16);
    }
    if (nbytes == 0)
      break;
    PT_YIELD(pt);
  }

  restart_at(&wait_csonmi); /* restart */
  gprsconn = find_gprs_connection();
  if (gprsconn) {
    gprsconn->input_callback(gprsconn, gprsconn->callback_arg, rcvdata, rcvlen);
  }
  PT_END(pt);
}

/* 
 * Callback for matching +TCPCLOSED keyword 
 */
static
PT_THREAD(wait_tcpclosed_callback(struct pt *pt, struct at_wait *at, int c)) {
  char ret;
  struct gprs_connection *gprsconn;
  ret = wait_readline_pt(pt, at, c);
  if (ret == PT_ENDED) {
    restart_at(&wait_tcpclosed); /* restart */
    gprsconn = find_gprs_connection();
    if (gprsconn && gprsconn->socket) {
      call_event(gprsconn->socket, TCP_SOCKET_CLOSED);
    }
  }
  return ret;
}

static void
wait_init() {
  /* The following are to detect async events -- permanently active */
  atwait_start_atlist(1, &wait_csonmi, &wait_csoerr, NULL);
}

static void
dumpchar(int c) {
  static char atbol = 1; /* at beginning of line */
  
  if (atbol) {
    printf("    ");
    atbol = 0;
  }
  if (c == '\n') {
    putchar('\n');
    atbol = 1;
  }
  else if (c >= ' ' && c <= '~')
    putchar(c);
  else
    putchar('*');
}

int len;
uint8_t buf[200];
 
PROCESS_THREAD(uart_reader, ev, data)
{
  static struct pt wait_pt;
  int c;
  
  PROCESS_BEGIN();

  ringbuf_init(&rxbuf, rxbuf_data, sizeof(rxbuf_data));
  rs232_set_input(uart, uart_input_byte);
  
  wait_init();
  PT_INIT(&wait_pt);
  
  while(1) {
    PROCESS_PAUSE();
    
    c = ringbuf_get(&rxbuf);
    if (c == -1)
      continue;
    //PROCESS_WAIT_UNTIL((c = ringbuf_get(&rxbuf)) != -1);
    dumpchar(c);
    wait_fsm_pt(&wait_pt, c);
  }
  PROCESS_END();
}

#if 0
static void
sendbuf_fun (unsigned char *buf, size_t len) {
  size_t remain = len;

  while (remain > 0) {
    remain -= sc16is_tx(&buf[len - remain], remain); 
  }
}
#endif

void
rs232_send_fix(uint8_t port, unsigned char *c)
{
  char p = *c;
  rs232_send(port, p);
}

static size_t sendbuf_remain;
#define sendbuf(buf, len) { \
  sendbuf_remain = len; \
  while (sendbuf_remain > 0) { \
    if (atwait_matching) {	    \
      printf("MATCHING\n"); \
      PROCESS_WAIT_UNTIL(atwait_matching == 0); \
      printf("DONE MATCHING\n"); \
    } \
    rs232_send_fix(uart, &(buf)[len - sendbuf_remain]);	\
    sendbuf_remain--; \
    continue; \
  } \
}

#define pt_sendbuf(buf, len) { \
  sendbuf_remain = len; \
  while (sendbuf_remain > 0) { \
    if (atwait_matching) {	    \
      printf("MATCHING\n"); \
      PT_WAIT_UNTIL(pt, atwait_matching == 0);   \
      printf("DONE MATCHING\n"); \
    } \
    rs232_send_fix(uart, &(buf)[len - sendbuf_remain]);	\
    sendbuf_remain--; \
    continue; \
  } \
}

#define GPRS_EVENT(E) (E >= a6at_gprs_init && E <= at_match_event)

//#define ATSTR(str) sendbuf_fun(((unsigned char *) str), strlen(str))
//#define ATSTR(str) sendbuf(((unsigned char *) str), strlen(str))
#define ATSTR(str) {printf("-->%s\n", str); sendbuf(((unsigned char *) str), strlen(str))}
#define PT_ATSTR(str) {printf("-->%s\n", str); pt_sendbuf(((unsigned char *) str), strlen(str))}
#define ATBUF(buf, len) sendbuf(buf, len)

static void
enqueue_event(process_event_t ev, void *data); 

#define CLOCKDELAY(ticks) {\
  printf("%d: start clock etimer %u, clock_time() = %lu\n", __LINE__, ticks, clock_time()); \
  etimer_set(&et, (ticks));                                             \
  while (1) { \
    PROCESS_WAIT_EVENT();                           \
    if(etimer_expired(&et)) { \
      printf("%d: timeout: clock_time() = %lu\n", __LINE__, clock_time()); \
      break; \
    } \
    else { \
      if (GPRS_EVENT(ev)) { \
        /* This is a event for us, but we can't do it now. \
         * Put it on the event queue for later. \
         */ \
          enqueue_event(ev, data); \
      } \
    } \
  } \
  }

#define DELAY(SEC) {\
  printf("%d: start etimer %lu, clock_time() = %lu\n", __LINE__, (clock_time_t) SEC*CLOCK_SECOND, clock_time()); \
  CLOCKDELAY((SEC)*CLOCK_SECOND);                                 \
  }

char *eventstr(process_event_t ev) {
  static char buf[64]; 
    if (ev == a6at_gprs_init) sprintf(buf, " a6at_gprs_init (%d)", ev); 
    else if (ev == a6at_gprs_connection) sprintf(buf, "a6at_gprs_connection (%d)", ev); 
    else if (ev == a6at_gprs_send) sprintf(buf, "a6at_gprs_send (%d)", ev); 
    else if (ev == a6at_gprs_close) sprintf(buf, "a6at_gprs_close (%d)", ev); 
    else if (ev == at_match_event) sprintf(buf, "at_match_event (%d)", ev); 
    else if (ev == uart_input_event) sprintf(buf, "uart_input_event (%d)", ev); 
    else sprintf(buf, "unknown)(%d)", ev); 
  return buf;
}
    
static void
event_init() {
  a6at_gprs_init = process_alloc_event();
  a6at_gprs_connection = process_alloc_event();
  a6at_gprs_send = process_alloc_event();
  a6at_gprs_close = process_alloc_event();  
  
  uart_input_event = process_alloc_event();

  printf("a6at_gprs_init == %s\n", eventstr(a6at_gprs_init));
}

#define GPRS_MAX_NEVENTS 8
struct gprs_event {
  process_event_t ev;
  void *data;
} gprs_event_queue[GPRS_MAX_NEVENTS];
static int gprs_nevents;
static int gprs_firstevent;

static void
event_queue_init() {
  gprs_nevents = 0;
  gprs_firstevent = 0;
}

static void
enqueue_event(process_event_t ev, void *data) {
  int index;
  if (gprs_nevents >= GPRS_MAX_NEVENTS) {
    printf("Fatal error: GPRS event queue full adding ev %d\n", ev);
    return;
  }
  index = (gprs_firstevent+gprs_nevents) % GPRS_MAX_NEVENTS;
  gprs_event_queue[index].ev = ev; gprs_event_queue[index].data = data;
  gprs_nevents++;
  printf("ENqueue event %s\n", eventstr(ev));
}

static struct gprs_event *
dequeue_event() {
  struct gprs_event *gprs_event;
  if (gprs_nevents == 0)
    return NULL;
  gprs_event = &gprs_event_queue[gprs_firstevent];
  gprs_nevents--;
  gprs_firstevent = (gprs_firstevent + 1) % GPRS_MAX_NEVENTS;
  printf("Dequeue event %s\n", eventstr(gprs_event->ev));
  return gprs_event;
}

static struct tcp_socket_gprs *socket;
static struct gprs_connection *gprsconn;

static
PT_THREAD(cell_register(struct pt *pt)) {
  struct at_wait *at;
  static char str[80];

  PT_BEGIN(pt);

#ifdef NB_IOT_TELIA	
  PT_ATSTR("AT+COPS=1,2,\"24001\"\r");
  PT_ATWAIT2(10, &wait_ok);

  PT_ATSTR("AT+CFUN=0\r");
  PT_ATWAIT2(10, &wait_ok);

  sprintf(str, "*MCGDEFCONT=\"%s\",%s\r", PDPTYPE, APN); /* Set PDP (Packet Data Protocol) context */
  PT_ATSTR(str);
  PT_ATWAIT2(10, &wait_ok);

  PT_ATSTR("AT+CFUN=1\r");
  PT_ATWAIT2(10, &wait_ok);

  PT_ATSTR("AT+CSQ\r");
  PT_ATWAIT2(10, &wait_ok);
  printf("Did IOT telia register\n");
#endif /* NB_IOT_TELIA */
  PT_END(pt);
}

PROCESS_THREAD(a6at, ev, data) {
  //unsigned char *res;
  struct at_wait *at;
  static struct pt pt;                          \
  static uint8_t minor_tries, major_tries;
  char str[80];

  PROCESS_BEGIN();
  leds_init();
  event_init();
  event_queue_init();

#undef NB_IOT_TELIA	
#ifdef NB_IOT_TELIA	
  ATSTR("AT+COPS=1,2,\"24001\"\r");
  ATWAIT2(10, &wait_ok);

  ATSTR("AT+CFUN=0\r");
  ATWAIT2(10, &wait_ok);

  sprintf(str, "*MCGDEFCONT=\"%s\",%s\r", PDPTYPE, APN); /* Set PDP (Packet Data Protocol) context */
  ATSTR(str);
  ATWAIT2(10, &wait_ok);

  ATSTR("AT+CFUN=1\r");
  ATWAIT2(10, &wait_ok);

  ATSTR("AT+CSQ\r");
  ATWAIT2(10, &wait_ok);

#endif
  
  PT_INIT(&pt);
  while (cell_register(&pt) < PT_EXITED) {
    PROCESS_PAUSE();
  } 

  /*  PROCESS_PT_SPAWN(&pt, cell_register(&pt));*/
 again:

  {
    status.module = GPRS_MODULE_UNNKOWN;
    ATSTR("ATI\r");
    ATWAIT2(10, &wait_ati);
    if (at == NULL) {
      printf("No module version found\n");
    }
    else {
      int i;
      const char *delim = " \t\r,";
      char *p = strtok((char *)&at_line[0], (const char *) delim);
      for(i=0; i <  sizeof(at_line); i++) {
	p = strtok(NULL, delim);
	if(!strcmp(p, "A6")) {
	  status.module = GPRS_MODULE_A6;
	  break;
	}
	if(!strcmp(p, "A7")) {
	  status.module = GPRS_MODULE_A7;
	  break;
	}
      }
    }

#ifdef GPRS_CONF_FORCE_A6
    status.module = GPRS_MODULE_A6; /* To avoid GPS with A7 */
#endif
    printf("Module version %d\n", status.module);

  }

  /* Wait for registration status to become 1 (local registration)
   * or 5 (roaming) or 10 (roaming, non-preferred)
   */
  major_tries = 0;

  while (major_tries++ < 10) {
    static uint8_t creg;
    char *cregstr;
    char *p;

    ATSTR("AT+CREG?\r");
    atwait_record_on();
    ATWAIT2(10, &wait_ok);
    atwait_record_off();
    if (at == NULL)
      continue;
    cregstr = strstr(at_line, "+CREG: ") + strlen("+CREG: ");
    cregstr = (char *)memchr(cregstr, ',', strlen(cregstr));
    creg = atoi((char *) cregstr+1);
    if (creg == 1 || creg == 5 || creg == 10) {/* Wait for registration */
      status.state = GPRS_STATE_REGISTERED;
      break; 
    }
  }

  if (major_tries >= 10) {
#ifdef GPRS_DEBUG
    printf("GPRS registration timeout\n");
#endif /* GPRS_DEBUG */
    gprs_statistics.resets += 1;
    goto again;
  }
  
  /* Then activate context */
  major_tries = 0;

#if 0
  while (major_tries++ < 10 && status.state != GPRS_STATE_ACTIVE)
  {
    static struct gprs_context *gcontext;

    gcontext = &gprs_context;

    /* Deactivate PDP context */
    sprintf(str, "AT+CSTT=\"%s\", \"\", \"\"\r", gcontext->apn); /* Start task and set APN */
    ATSTR(str);   
    ATWAIT2(20, &wait_ok);
      
    ATSTR("AT+CGATT=1\r");
    ATWAIT2(20, &wait_ok);

    ATSTR("AT+CIPMUX=0\r");
    ATWAIT2(5, &wait_ok);

    minor_tries = 0;
    while (minor_tries++ < 10) {

      sprintf(str, "AT+CGDCONT=1,\"%s\",%s\r", gcontext->pdptype, gcontext->apn); /* Set PDP (Packet Data Protocol) context */
      ATSTR(str);
      ATWAIT2(5, &wait_ok);
      ATSTR("AT+CGACT=1,1\r");       /* Sometimes fails with +CME ERROR:148 -- seen when brought up initially, then it seems to work */
      ATWAIT2(20, &wait_ok,  &wait_cmeerror);
      if (at == &wait_ok) {
        break;
      }
      if (at == &wait_cmeerror) {
#ifdef GPRS_DEBUG
        printf("CGACT failed with CME ERROR:%s\n", at_line);
#endif /* GPRS_DEBUG */
        gprs_statistics.at_errors += 1;
      }
      else {
        gprs_statistics.at_timeouts += 1;
      }
      DELAY(5);
    }
    if (minor_tries++ >= 10)
      continue;
    minor_tries = 0;
    while (minor_tries++ < 10) {
      ATSTR("AT+CIPSTATUS?\r"); 
      atwait_record_on();
      ATWAIT2(60, &wait_ok);
      atwait_record_off();
      if (at == &wait_ok) {
        char *gprsact;
        gprsact = strstr(at_line, "+CIPSTATUS:") + strlen("+CIPSTATUS:");
        if (strncmp(gprsact, "0,IP GPRSACT", strlen("0,IP GPRSACT")) == 0) {
          gcontext->active = 1;
          status.state = GPRS_STATE_ACTIVE;
          break;
        }
        else {
          DELAY(5);
        }
      }
    }
    if (minor_tries++ >= 10) {
      /* Failed to activate */
      ATSTR("AT+CIPSHUT\r");
      ATWAIT2(10, &wait_ok);
      continue;
    }
  } /* Context activated */
  if (major_tries >= 10) {
    gprs_statistics.resets += 1;
    goto again;
  }
  
  /* Get IP address */
  ATSTR("AT+CIFSR\r");
  atwait_record_on();
  ATWAIT2(2, &wait_ok);
  atwait_record_off();
  if (at == &wait_ok) {
    printf("Recorded quad: --- '%s' ---\n", at_line);
#if 0
    const char *delim = " \t\r,";
    char *s = strtok(at_line, delim);
    printf("First string '%s'\n", s);
    s = strtok(NULL, delim);
    printf("Second string '%s'\n", s);
    s++; /* Skip newline at pos 0 */
#if NETSTACK_CONF_WITH_IPV6
    snprintf(status.ipaddr, sizeof(status.ipaddr), "::ffff:%s", s);
#else
    snprintf(status.ipaddr, sizeof(status.ipaddr), "%s", s);
#endif 
#endif
  }
  else {
    gprs_statistics.at_timeouts += 1;
  }
  
#ifdef GPRS_DEBUG
  printf("GPRS initialised\n");  
#endif /* GPRS_DEBUG */
  process_post(PROCESS_BROADCAST, a6at_gprs_init, NULL);

#endif  /* #if 0 */
  
  /* Get IP address */
    ATSTR("AT+CGCONTRDP\r");
    atwait_record_on();
    ATWAIT2(2, &wait_ok);
    atwait_record_off();
    if (at == &wait_ok) {
      char *dotquad;
      printf("### Got PDP context params '%s'\n", at_line);
    }

    if (at == NULL)  {
      gprs_statistics.at_timeouts += 1;
    }
    status.state = GPRS_STATE_ACTIVE;
    process_post(PROCESS_BROADCAST, a6at_gprs_init, NULL);
	  
  /* Main loop. Wait for GPRS command and execute them */
  while(1) {
    static struct gprs_event *gprs_event;
  nextcommand:
    gprs_event = dequeue_event();
    if (gprs_event == NULL) {
      PROCESS_PAUSE();
      goto nextcommand;
    }
    else
    if (gprs_event->ev == a6at_gprs_connection) {
#ifdef GPRS_DEBUG
      printf("A6AT GPRS Connection\n");
#endif /* GPRS_DEBUG */

      gprsconn = (struct gprs_connection *) gprs_event->data;
      minor_tries = 0;
      while (minor_tries++ < 10) {
	
	//printf("Here is connection %s %s:%d\n", gprsconn->proto, gprsconn->ipaddr, uip_htons(gprsconn->port));
        //sprintf(str, "AT+CIPSTART= \"%s\", %s, %d\r", gprsconn->proto, gprsconn->ipaddr, uip_ntohs(gprsconn->port));
	//sprintf(str, "AT+CIPSTART= \"%s\", %s, %d\r", gprsconn->proto, gprsconn->ipaddr, uip_ntohs(gprsconn->port));

	printf("Here is connection %s %s:%d\n", gprsconn->proto, gprsconn->ipaddr, uip_htons(gprsconn->port));
	ATSTR("AT+CSOC=1,1,1\r");
	ATWAIT2(10, &wait_ok);

	sprintf(str, "AT+CSOCON=0,%d,\"%s\"\r", uip_ntohs(gprsconn->port), gprsconn->ipaddr);
        ATSTR(str);
        ATWAIT2(60, &wait_ok, &wait_error);        
        if (at == &wait_ok) {
          gprs_statistics.connections += 1;
          call_event(gprsconn->socket, TCP_SOCKET_CONNECTED);
          goto nextcommand;
        }
        /* If we ended up here, we failed to set up connection */
        if (at == NULL) {
          /* Timeout */
          gprs_statistics.connfailed += 1;
          ATSTR("AT+CIPCLOSE\r");
          ATWAIT2(5, &wait_ok);
          ATSTR("AT+CIPSHUT\r");
          ATWAIT2(5, &wait_ok);
          call_event(gprsconn->socket, TCP_SOCKET_TIMEDOUT);
          goto nextcommand;
        }        
        else if (at == &wait_error) {
          /* COMMAND NO RESPONSE! timeout. Sometimes it take longer though and have seen COMMAND NON RESPONSE! followed by CONNECT OK */ 
          /* Seen +CME ERROR:53 */
          printf("CIPSTART failed\n");
          ATSTR("AT+CREG?\r");
          ATWAIT2(2, &wait_ok);
          gprs_statistics.at_errors += 1;
          ATSTR("AT+CIPCLOSE\r");
          ATWAIT2(15, &wait_ok);//ATWAIT2(5, &wait_ok, &wait_cmeerror);
          ATSTR("AT+CIPSHUT\r");
          ATWAIT2(15, &wait_ok);//ATWAIT2(5, &wait_ok,  &wait_cmeerror);

	  /* Test to cure deadlock when closing/shutting down  --ro */
	  if (minor_tries++ > 10) {
	    gprs_statistics.connfailed += 1;
	    goto again;
	  }
	  else {
	    continue;
	  }
        }
      } /* minor_tries */
      if (minor_tries >= 10) {
        gprs_statistics.connfailed += 1;
        call_event(gprsconn->socket, TCP_SOCKET_TIMEDOUT);
        goto nextcommand;
      }
    } /* ev == a6at_gprs_connection */
    else if (gprs_event->ev == a6at_gprs_send) {
      static uint16_t remain;
      static uint16_t len;
      static uint8_t *ptr;
#ifdef GPRS_DEBUG
      printf("A6AT GPRS Send\n");
#endif /* GPRS_DEBUG */
      gprsconn = (struct gprs_connection *) gprs_event->data;
      ptr = gprsconn->output_data_ptr;
      //socket = gprsconn->socket;
      //remain = socket->output_data_len;
      remain = gprsconn->output_data_len;
      ATSTR("ATE0\r\n");
      ATWAIT2(5, &wait_ok);
      if (at == NULL) {
        gprs_statistics.at_timeouts += 1;
        goto failed;
      }
      while (remain > 0) {
        len = (remain <= GPRS_MAX_SEND_LEN ? remain : GPRS_MAX_SEND_LEN);
        printf("Send %d bytes @0x%x\n", len, (unsigned) &ptr[gprsconn->output_data_len-remain]);
        sprintf((char *) buf, "AT+CSODSEND=0,%d\r", len);
        ATSTR((char *) buf); /* sometimes CME ERROR:516 */
        ATWAIT2(5, &wait_ok, &wait_sendprompt, &wait_cmeerror);
        if (at == NULL) {
          gprs_statistics.at_timeouts += 1;
          ATSTR("ATE1\r\n");
          ATWAIT2(5, &wait_ok);
          goto failed;
        }
        else if (at == &wait_cmeerror) {
          gprs_statistics.at_errors += 1;
          ATSTR("ATE1\r\n");
          ATWAIT2(5, &wait_ok);
          goto failed;
        }
        ATBUF(&ptr[gprsconn->output_data_len-remain], len);
        ATWAIT2(30, &wait_ok, &wait_error, &wait_dataaccept);
#if 0
        if (at == NULL || at == &wait_commandnoresponse || at == &wait_error) {
#endif
        if (at == NULL || at == &wait_error) {
          gprs_statistics.at_timeouts += 1;
          goto failed;
        }        
#if 0
        if (remain > len) {
          memcpy(&socket->output_data_ptr[0],
                 &socket->output_data_ptr[len],
                 remain - len);
        }
        socket->output_data_len -= len;
#endif
        remain -= len;
      }

      //call_event(socket, TCP_SOCKET_DATA_SENT);
      gprs_call_event(gprsconn, GPRS_CONN_DATA_SENT);      
      ATSTR("ATE1\r\n");
      ATWAIT2(5, &wait_ok);
      if (at == NULL) {
        gprs_statistics.at_timeouts += 1;
        goto failed;
      }

    } /* ev == a6at_gprs_send */
    else if (gprs_event->ev == a6at_gprs_close) {

#ifdef GPRS_DEBUG
      printf("A6AT GPRS Close\n");
#endif /* GPRS_DEBUG */
      gprsconn = (struct gprs_connection *) gprs_event->data;
      socket = gprsconn->socket;

      ATSTR("AT+CIPCLOSE\r");
      ATWAIT2(15, &wait_ok, &wait_cmeerror);
      if (at == &wait_ok) {
        call_event(socket, TCP_SOCKET_CLOSED); 
      }
      else {
        printf("Call socket_closed\n");
        gprs_statistics.at_timeouts += 1;
        call_event(socket, TCP_SOCKET_CLOSED);
        printf("Called socket_closed\n");
      }
    } /* ev == a6at_gprs_close */
#ifdef GPRS_DEBUG
    else {
      printf("A6AT GPRS Unknown event %d\n", gprs_event->ev);
    }
#endif /* GPRS_DEBUG */

    ATSTR("AT+CSQ\r");
    atwait_record_on();
    ATWAIT2(5, &wait_csq);
    atwait_record_off();

    if (at == NULL)
      continue;
    else {
      char *csq;
      csq = strstr(at_line, "+CSQ:") + strlen("+CSQ:");    
      status.rssi = atoi((char *) csq /*foundbuf*/);
      ATWAIT2(5, &wait_ok);
    }
    continue;
  failed:
    /* Timeout */
    ATSTR("AT+CIPCLOSE\r");
    ATWAIT2(5, &wait_ok, &wait_cmeerror);
    ATSTR("AT+CIPSHUT\r");
    ATWAIT2(5, &wait_ok, &wait_cmeerror);
    call_event(gprsconn->socket, TCP_SOCKET_TIMEDOUT);
  }
  PROCESS_END();
}
