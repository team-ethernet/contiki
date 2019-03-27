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
 *         A simple application showing sc16is I2C UART & GPIO
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
#include "dev/sc16is/sc16is.h"
#include "sc16is-common.h"
#include "tcp-socket-gprs.h"
#include "gprs-a6.h"
#include "at_wait.h"

#define APN GPRS_CONF_APN
#define PDPTYPE "IP"
//#define PDPTYPE "IPV6"

uint32_t baud;
uint8_t at[] = {'A', 'T', 0xd };

#define GPRS_CONNID 0

#define GPRS_MAX_CONNECTION 1
struct gprs_connection gprs_connections[GPRS_MAX_CONNECTION];
struct gprs_context gprs_context;

process_event_t sc16is_input_event;

static struct gprs_status status;

static void
enqueue_event(process_event_t ev, void *data);

#define min(A, B) ((A) <= (B) ? (A) : (B))
/*---------------------------------------------------------------------------*/
PROCESS(sc16is_reader, "I2C UART input process");
PROCESS(a6at, "GPRS A6 module");


/* LED debugging process */
PROCESS(yled, "Yellow LED");
static clock_time_t yled_interval = CLOCK_SECOND/2;

/*---------------------------------------------------------------------------*/
void
gprs_init() {
  int i;
  for (i = 0; i < GPRS_MAX_CONNECTION; i++) { 
    GPRS_CONNECTION_RELEASE(&gprs_connections[i]);
  }
  gprs_set_context(&gprs_context, PDPTYPE, APN);
  memset(&gprs_statistics, 0, sizeof(gprs_statistics));
  process_start(&sc16is_reader, NULL);
  process_start(&a6at, NULL);
  process_start(&yled, NULL);
}
/*---------------------------------------------------------------------------*/
struct gprs_connection *
alloc_gprs_connection() {
  int i;
  struct gprs_connection *gprsconn;
  for (i = 0; i < GPRS_MAX_CONNECTION; i++) {
    gprsconn = &gprs_connections[i];
    if (!GPRS_CONNECTION_RESERVED(gprsconn)) {
        printf("alloc_gprs_connection -> 0x%x\n", (unsigned) gprsconn);
        GPRS_CONNECTION_RESERVE(gprsconn);
        return gprsconn;
    }
  }
  printf("Cannot alloc gprs connection\n");
  return NULL;
}
/*---------------------------------------------------------------------------*/
static void
free_gprs_connection(struct gprs_connection *gprsconn) {
  GPRS_CONNECTION_RELEASE(gprsconn);
  printf("free_gprs_connection(0x%x)\n", (unsigned) gprsconn);  
  return;
}
/*---------------------------------------------------------------------------*/
static struct gprs_connection *
find_gprs_connection(char connectionid) {
  int i;
  struct gprs_connection *gprsconn;

  for (i = 0; i < GPRS_MAX_CONNECTION; i++) {
    gprsconn = &gprs_connections[i];
    if (gprsconn->connectionid == connectionid) {
      printf("find_gprs_connection -> 0x%x\n", (unsigned) gprsconn);
      return gprsconn;
    }
  }
  printf("Cannot find gprsconn %d\n", connectionid);
  return NULL;
}
/*---------------------------------------------------------------------------*/
static char *tcp_event_str( tcp_socket_gprs_event_t event) {
  switch (event) {
  case TCP_SOCKET_CONNECTED: return("TCP_SOCKET_CONNECTED");
  case TCP_SOCKET_CLOSED: return("TCP_SOCKET_CLOSED"); 
  case TCP_SOCKET_TIMEDOUT: return("TCP_SOCKET_TIMEDOUT"); 
  case TCP_SOCKET_ABORTED: return("TCP_SOCKET_ABORTED"); 
  case TCP_SOCKET_DATA_SENT: return("TCP_SOCKET_DATA_SENT"); 
  default: return("Unknown ");
  }
}

static void
call_event(struct tcp_socket_gprs *s, tcp_socket_gprs_event_t event)
{
  if(s != NULL && s->event_callback != NULL) {
    printf("call_event(%s)\n", tcp_event_str(event));
    s->event_callback(s, s->ptr, event);
  }
}
/*---------------------------------------------------------------------------*/

static void
gprs_call_event(struct gprs_connection *gprsconn, gprs_conn_event_t event)
{
  if(gprsconn != NULL && gprsconn->event_callback != NULL) {
    printf("gprs_call_event(%s)\n", tcp_event_str(event));
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

  printf("gprs_register(0x%x)\n", (unsigned)gconn);
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

  printf("gprs_unregister(0x%x)\n", (unsigned)gconn);
  gconn->socket = NULL;
  gconn->callback_arg = NULL;
  free_gprs_connection(gconn);
  return 0;
}
/*---------------------------------------------------------------------------*/
struct gprs_connection *
gprs_connection(struct gprs_connection *gprsconn,
                const char *proto, const char *ipaddr, uint16_t port,
                struct tcp_socket_gprs *socket) {

  printf("gprs_connection(0x%x)\n", (unsigned)gprsconn);
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
  printf("gprs_send(0x%x)\n", (unsigned) gprsconn);
  enqueue_event(a6at_gprs_send, gprsconn);
}
/*---------------------------------------------------------------------------*/
void
gprs_close(struct tcp_socket_gprs *socket) {
  printf("gprs_close(0x%x)\n", (unsigned) socket);
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
PT_THREAD(wait_ciprcv_callback(struct pt *pt, struct at_wait *at, int c));
static
PT_THREAD(wait_tcpclosed_callback(struct pt *pt, struct at_wait *at, int c));
static
PT_THREAD(wait_gpsrd_callback(struct pt *pt, struct at_wait *at, int c));

static
struct at_wait wait_ciprcv = {"+CIPRCV:", wait_ciprcv_callback, at_match_byte};
static
struct at_wait wait_ok = {"OK", wait_readline_pt, at_match_byte};
static
struct at_wait wait_error = {"ERROR", NULL, at_match_byte};
static
struct at_wait wait_connectok = {"CONNECT OK", NULL, at_match_byte};
static
struct at_wait wait_cmeerror = {"+CME ERROR:", wait_readline_pt, at_match_byte};
static
struct at_wait wait_commandnoresponse = {"COMMAND NO RESPONSE!", NULL, at_match_byte};
static
struct at_wait wait_sendprompt = {">", NULL, at_match_byte};
static
struct at_wait wait_tcpclosed = {"+TCPCLOSED:", wait_tcpclosed_callback, at_match_byte};
static
struct at_wait wait_gpsrd = {"$GPRMC,", wait_gpsrd_callback, at_match_byte};
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

  /* Consume the colon ':' */
  PT_YIELD(pt); 

  /* Get length as a decimal number followed by a comma ','
   */
  nbytes = 0;
  while (c != ',') {
    if (!isdigit(c)) {
      /* Error: bad len */
      printf("ciprcv_callback: bad len 0x%x %d (nbytes %d)\n", c, c, nbytes);
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
  gprsconn = find_gprs_connection(GPRS_CONNID);
  if (gprsconn) {
    gprsconn->input_callback(gprsconn, gprsconn->callback_arg, rcvdata, nbytes);
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
    gprsconn = find_gprs_connection(GPRS_CONNID);
    if (gprsconn && gprsconn->socket) {
      call_event(gprsconn->socket, TCP_SOCKET_CLOSED);
    }
  }
  return ret;
}

static
PT_THREAD(wait_gpsrd_callback(struct pt *pt, struct at_wait *at, int c)) {

  char ret, *valid;
  /* 
     unsigned int year, mon, day, hour, min, sec; 
     char  foo[6], c[1];
  */
  char *p, *time, *s1, *s2, *date;

  /* 
     A7 NMEA sentence
     PGSV,3,2,11,20,52,092,23,10,33,169,,08,23,291,,30,11,344,*7B*
     $GPGSV,3,3,11,07,17,318,,26,22,189,,21,55,086,*4B*
     $GPRMC,112547.000,A,5951.05612,N,01736.86381,E,0.00,0.00,300182,,,A*6D*
     $GPVTG,0.00,T,,
     M,0.00,N,0.00,K,A*3D*
  */

  ret = wait_readlines_pt(pt, at, c);
  printf("GPS=%s\n", at_line);

  const char *delim = " \t\r,";
  time = strtok((char *)&at_line[0], (const char *) delim);

  if(time) printf("time %s\n", time);
  
  valid = strtok(NULL, delim);
  if(valid) printf("valid %s\n", valid);
  
  p = strtok(NULL, delim);
  if(p) {
    status.lat = strtod(p, NULL);
    status.lat = status.lat/100.;
  }

  s1 = strtok(NULL, delim);
  if(p) printf("s1 %s\n", s1);

  p = strtok(NULL, delim);
  if(p) {
    status.longi = strtod(p, NULL);
    status.longi = status.longi/100.;
  }

  s2 = strtok(NULL, delim);
  if(p) printf("s2 %s\n", s2);

  p = strtok(NULL, delim);
  if(p) status.speed = strtod(p, NULL);
#define KNOT_TO_KMPH 1.852 
  status.speed = status.speed * KNOT_TO_KMPH;

  p = strtok(NULL, delim);
  if(p) status.course = strtod(p, NULL);

  date = strtok(NULL, delim);
  if(p) printf("date %s\n", date);

  if((!strcmp(valid, "A"))) {
    /* Fix */
    printf("valid=%s lon=%-lf lat=%-lf speed=%-6.2lf course=%-lf\n", 
	   valid, status.lat, status.longi, status.speed, status.course);
    /* year += 2000; */
  } 
  else {
    /* No fix */
    sc16is_gpio_set_bit(G_LED_RED);
    status.longi = -1;
    status.lat = -1;
    status.speed = -1;
    status.course = -1;
  }

#if 0
    /* Old. Decode $GPRMC sentense */
    res = sscanf(at_line,"%2d%2d%2d.%3c,%1c,%lf,%1c,%lf,%1c,%lf,%lf,%2d%2d%2d",
		 &hour, &min, &sec, foo, &valid, &lat, c, &longi, c, &speed, &course, &day, &mon, &year);
#endif    
      //nmew_time(year, mon, day, hour, min, sec );
  return ret;
}

static void
wait_init() {
  /* The following are to detect async events -- permanently active */
  atwait_start_atlist(1, &wait_ciprcv, &wait_tcpclosed, &wait_gpsrd, NULL);

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
    putchar(c);
}

int
module_init(uint32_t baud)
{
  if(i2c_probed & I2C_SC16IS) {
  printf("Here is module_init(%ld)\n",  baud);

    sc16is_init();
    //sc16is_gpio_set_dir(G_RESET | G_PWR | G_U_5V_CTRL | G_SET | G_LED_YELLOW | G_LED_RED | G_GPIO7);
    sc16is_gpio_set_dir(G_RESET | G_PWR | G_U_5V_CTRL | G_LED_YELLOW | G_LED_RED | G_GPIO7);
    sc16is_gpio_set((G_LED_RED|G_LED_YELLOW));
    sc16is_uart_set_speed(baud);
    /* sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_FCR, SC16IS_FCR_FIFO_BIT); */
    status.state = GPRS_STATE_IDLE;
    sc16is_tx((uint8_t *)"AT", sizeof("AT"-1));
    return 1;
  }
  return 0;
}

PROCESS_THREAD(sc16is_reader, ev, data)
{
  static struct pt wait_pt;
  static int len;
  static uint8_t buf[200];

  PROCESS_BEGIN();

  wait_init();
  PT_INIT(&wait_pt);
  
  while(1) {
    PROCESS_PAUSE();
    if( i2c_probed & I2C_SC16IS ) {
      len = sc16is_rx(buf, sizeof(buf));
      if (len) {
        static int i;
        uint8_t c;
        for (i = 0; i < len; i++) {
          c = buf[i];
          dumpchar(c);
          wait_fsm_pt(&wait_pt, c);
        }
      }
    }
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

static size_t sendbuf_remain;
#define sendbuf(buf, len) { \
  sendbuf_remain = len; \
  while (sendbuf_remain > 0) { \
    if (atwait_matching) {	    \
      printf("MATCHING\n"); \
      PROCESS_WAIT_UNTIL(atwait_matching == 0); \
      printf("DONE MATCHING\n"); \
    } \
    sendbuf_remain -= sc16is_tx(&(buf)[len - sendbuf_remain], sendbuf_remain); \
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
    sendbuf_remain -= sc16is_tx(&(buf)[len - sendbuf_remain], sendbuf_remain); \
    continue; \
  } \
}

#define GPRS_EVENT(E) (E >= a6at_gprs_init && E <= at_match_event)

//#define ATSTR(str) sendbuf_fun(((unsigned char *) str), strlen(str))
//#define ATSTR(str) sendbuf(((unsigned char *) str), strlen(str))
#define ATSTR(str) {printf("-->%s\n", str); sendbuf(((unsigned char *) str), strlen(str))}
#define PT_ATSTR(str) {printf("-->%s\n", str); pt_sendbuf(((unsigned char *) str), strlen(str))}
#define ATBUF(buf, len) sendbuf(buf, len)
#define PT_ATBUF(buf, len) pt_sendbuf(buf, len)

static void
enqueue_event(process_event_t ev, void *data); 

struct etimer et;

char *eventstr(process_event_t ev) {
  static char buf[64]; 
    if (ev == a6at_gprs_init) sprintf(buf, " a6at_gprs_init (%d)", ev); 
    else if (ev == a6at_gprs_connection) sprintf(buf, "a6at_gprs_connection (%d)", ev); 
    else if (ev == a6at_gprs_send) sprintf(buf, "a6at_gprs_send (%d)", ev); 
    else if (ev == a6at_gprs_close) sprintf(buf, "a6at_gprs_close (%d)", ev); 
    else if (ev == at_match_event) sprintf(buf, "at_match_event (%d)", ev); 
    else if (ev == sc16is_input_event) sprintf(buf, "sc16is_input_event (%d)", ev); 
    else sprintf(buf, "unknown)(%d)", ev); 
  return buf;
}
    
static void
event_init() {
  a6at_gprs_init = process_alloc_event();
  a6at_gprs_connection = process_alloc_event();
  a6at_gprs_send = process_alloc_event();
  a6at_gprs_close = process_alloc_event();  
  
  sc16is_input_event = process_alloc_event();
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

/*---------------------------------------------------------------------------*/
/* read_csq
 * Protothread to read rssi/csq with AT commands. Store result
 * in status struct
 */
static
PT_THREAD(read_csq(struct pt *pt)) {
  struct at_wait *at;

  PT_BEGIN(pt);
  PT_ATSTR("AT+CSQ\r");
  atwait_record_on();
  PT_ATWAIT2(5, &wait_ok);
  atwait_record_off();

  if (at == NULL)
    gprs_statistics.at_timeouts += 1;
  else {
    char *csq;
    csq = strstr(at_line, "+CSQ:") + strlen("+CSQ:");    
    status.rssi = atoi((char *) csq /*foundbuf*/);
  }
  PT_END(pt);
}

/*---------------------------------------------------------------------------*/
/* init_module
 * Protothread to initialize mobile data radio unit.
 */
static
PT_THREAD(init_module(struct pt *pt)) {
  struct at_wait *at;
  uint8_t s;

  PT_BEGIN(pt);

  printf("INit module\n");

#ifdef NBIOT   
  module_init(115200);
  //module_init(9600);
#else
  module_init(115200);
#endif    

  set_board_5v(0); /* Power cycle the board */
  PT_DELAY(2);
  set_board_5v(1);
  PT_DELAY(2);
  s = sc16is_gpio_get();
  printf("LOOP GPIO=0x%02x\n", sc16is_gpio_get());
  clr_bit(&s, G_PWR);
  set_bit(&s, G_RESET);

  set_bit(&s, G_LED_RED); /*OFF */
  set_bit(&s, G_LED_YELLOW);
  sc16is_gpio_set(s);
  PT_DELAY(2);
  clr_bit(&s, G_RESET);
  sc16is_gpio_set(s);
  /* start */
  PT_DELAY(2);
  printf("Start Radio\n");
  s = sc16is_gpio_get();
  set_bit(&s, G_PWR);
  sc16is_gpio_set(s);


  PT_ATSTR("ATI\r");
  PT_DELAY(5);
  status.state = GPRS_STATE_IDLE;
  PT_DELAY(10);
  { static int i;
    for (i = 0; i < 10; i++) {
      PT_ATSTR("ATI\r");
      PT_ATWAIT2(10, &wait_ok);
      if (at == &wait_ok)
        break;
    }
  }
      
  printf("Done INit module\n");
  
  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
/* apn_register
 * Protothread to register with APN. Wait until status confirms
 * we are registered, then update state.
 */
static
PT_THREAD(apn_register(struct pt *pt)) {
  struct at_wait *at;

  PT_BEGIN(pt);

  static uint8_t waiting;
  waiting = 0;
  while (waiting < GPRS_APN_REGISTER_TIMEOUT) {
    static uint8_t creg;

    PT_ATSTR("AT+CREG?\r");
    atwait_record_on();
    PT_ATWAIT2(10, &wait_ok);
    atwait_record_off();
    if (at == NULL)
      goto timeout;
    int n;
    /* AT+CREG?
     *  +CREG: 0,0*
     */
    if (1 != (n = sscanf(at_line, "%*[^:]: %*d,%hhd", &creg))) {
      printf("scan cref fail %d\n", n);
      break;
    }
    if (creg == 1 || creg == 5 || creg == 10) {/* Wait for registration */
      status.state = GPRS_STATE_REGISTERED;
      PT_EXIT(pt); 
    }
    /* Registration failed. Delay and try again */
    PT_DELAY(GPRS_APN_REGISTER_REATTEMPT);
    waiting += GPRS_APN_REGISTER_REATTEMPT;
  }
  /* Timeout expired without registering */
  status.state = GPRS_STATE_NONE;
  gprs_statistics.resets += 1;
  PT_EXIT(pt);
 timeout:
  gprs_statistics.at_timeouts += 1;
  status.state = GPRS_STATE_NONE;
  PT_END(pt);
}

/*---------------------------------------------------------------------------*/
/* apn_activate
 * Protothread to activate context. Wait until status confirms
 * we are activated, then update state.
 */
static
PT_THREAD(apn_activate(struct pt *pt)) {
  struct at_wait *at;
  static char str[80];
  static int major_tries, minor_tries;
  
  PT_BEGIN(pt);

  /* Then activate context */
  again:
  major_tries = 0;
  while (major_tries++ < 10 && status.state != GPRS_STATE_ACTIVE) {
    static struct gprs_context *gcontext;
    gcontext = &gprs_context;
    /* Deactivate PDP context */
    sprintf(str, "AT+CSTT=\"%s\", \"\", \"\"\r", gcontext->apn); /* Start task and set APN */
    PT_ATSTR(str);   
    PT_ATWAIT2(20, &wait_ok);
      
    PT_ATSTR("AT+CGATT=1\r");
    PT_ATWAIT2(5, &wait_ok);
    PT_ATSTR("AT+CIPMUX=0\r");
    PT_ATWAIT2(5, &wait_ok);
      
    minor_tries = 0;
    while (minor_tries++ < 10) {
      sprintf(str, "AT+CGDCONT=1,%s,%s\r", gcontext->pdptype, gcontext->apn); /* Set PDP (Packet Data Protocol) context */
      PT_ATSTR(str);
      PT_ATWAIT2(5, &wait_ok);
      PT_ATSTR("AT+CGACT=1,1\r");       /* Sometimes fails with +CME ERROR:148 -- seen when brought up initially, then it seems to work */
      PT_ATWAIT2(20, &wait_ok,  &wait_cmeerror);
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
      PT_DELAY(5);
    }

    if (minor_tries++ >= 10)
      continue;
    minor_tries = 0;
    while (minor_tries++ < 10) {
      PT_ATSTR("AT+CIPSTATUS?\r"); 
      atwait_record_on();
      PT_ATWAIT2(60, &wait_ok);
      atwait_record_off();
      if (at == &wait_ok) {
        if (strstr((char *) at_line, "0,IP GPRSACT") != NULL) {
          gcontext->active = 1;
          status.state = GPRS_STATE_ACTIVE;
          printf("ACTIVATED\n");
          break;
        }
        else {
          PT_DELAY(5);
        }
      }
    }
    if (minor_tries++ >= 10) {
      /* Failed to activate */
      PT_ATSTR("AT+CIPSHUT\r");
      PT_ATWAIT2(10, &wait_ok);
      continue;
    }

  } /* Context activated */
  if (major_tries >= 10) {
    gprs_statistics.resets += 1;
    goto again;
  }
  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
/*
 * get_moduleinfo
 *
 * Protothread to read module info using AT commands.
 * For now, identify module type.
 */
static
PT_THREAD(get_moduleinfo(struct pt *pt)) {
  struct at_wait *at;

  PT_BEGIN(pt);

  status.module = GPRS_MODULE_UNNKOWN;
  PT_ATSTR("ATI\r");
  atwait_record_on();
  PT_ATWAIT2(10, &wait_ok);
  atwait_record_off();
  if (at == NULL) {
    goto notfound;
  }
  int i;
  //const char *delim = " \t\r\n,";
  const char *delim = "\r\n";  

  char *p = strtok((char *)&at_line[0], (const char *) delim);
  if (p == NULL)
    goto notfound;

  for(i=0; i <  sizeof(at_line); i++) {
    p = strtok(NULL, delim);
    if (p == NULL) {
      goto notfound;
    }
    
    if(!strncmp(p, "A6", sizeof("A6")-1)) {
      status.module = GPRS_MODULE_A6;

      /* 
         Workaround and fix and
         needs investigation. It seems like the A6 
         module is not compatible with UART sleep mode.
         Can be an A6 firmware issue.
      */
      sc16is_sleep_mode(0);
      break;
    }
    if(0 == strncmp(p, "A7", sizeof("A7"))) {
      status.module = GPRS_MODULE_A7;
      break;
    }
  }
  if (status.module == GPRS_MODULE_UNNKOWN)
    goto notfound;

#ifdef GPRS_CONF_FORCE_A6
  status.module = GPRS_MODULE_A6; /* To avoid GPS with A7 */
#endif
  printf("PT Module version %d\n", status.module);

  if(status.module == GPRS_MODULE_A7) {
    PT_ATSTR("AT+GPS=0\r");
    PT_ATWAIT2(10, &wait_ok);
    if (at == NULL) {
      printf("GPS not disabled\n");
    }
      
    PT_ATSTR("AT+GPS=1\r");
    PT_ATWAIT2(10, &wait_ok);
    if (at == NULL) {
      printf("GPS not enabled\n");
    }
      
    PT_ATSTR("AT+GPSRD=30\r");
    PT_ATWAIT2(10, &wait_gpsrd);
    if (at == NULL) {
      printf("GPSRD not enabled\n");
    }
  }

  
  PT_ATSTR("AT+CNUM\r");
  PT_ATWAIT2(10, &wait_ok);
  PT_EXIT(pt);
  
 notfound:
  printf("module info not found ('%s')\n", at_line);
  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
/*
 * get_ipconfig
 *
 * Protothread to get IP configuration using AT commands:
 * - Get own IP address
 */
static
PT_THREAD(get_ipconfig(struct pt *pt)) {
  struct at_wait *at;
  
  PT_BEGIN(pt);
  /* Get IP address */
  PT_ATSTR("AT+CIFSR\r");
  atwait_record_on();
  PT_ATWAIT2(10, &wait_ok);
  atwait_record_off();
  if (at == NULL) {
    gprs_statistics.at_timeouts += 1;
  }
  else {
    /* Look for something like
     *     +CGCONTRDP: 1,5,"lpwa.telia.iot","10.81.168.254.255.255.255.0" 
     */
    int n;
    int b1, b2, b3, b4;
    char *str;
    //n = sscanf(at_line, "%*[^:]: %*d,%*d,\"%*[-.a-zA-Z0-9_]\",\"%d.%d.%d.%d", &b1, &b2, &b3, &b4);
    str = at_line;
    while (!isdigit(*str) && *str != '\0')
      str++;
    if (*str != '\0') {
      n = sscanf(str, "%d.%d.%d.%d", &b1, &b2, &b3, &b4);
      if (n == 4) {
#if NETSTACK_CONF_WITH_IPV6
        snprintf(status.ipaddr, sizeof(status.ipaddr), "::ffff:%d.%d.%d.%d", b1, b2, b3, b4);
#else
        snprintf(status.ipaddr, sizeof(status.ipaddr), "%d.%d.%d.%d", b1, b2, b3, b4);
#endif 
        PT_EXIT(pt);
      }
    }
  }
  printf("Could not get ipaddr from \"%s\"\n", at_line);
  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
/*
 * connect
 *
 * Protothread to set up TCP connection with AT commands
 */
static
PT_THREAD(a6_connect(struct pt *pt, struct gprs_connection * gprsconn)) {
  static struct at_wait *at;
  static int minor_tries;
  char str[80];
  
  PT_BEGIN(pt);
  minor_tries = 0;
  yled_interval = CLOCK_SECOND*2;
  process_post(&yled, 3, &yled_interval);

  while (minor_tries++ < 10) {
    printf("Here is connection %s %s:%d\n", gprsconn->proto, gprsconn->ipaddr, uip_htons(gprsconn->port));
    sprintf(str, "AT+CIPSTART= \"%s\", %s, %d\r", gprsconn->proto, gprsconn->ipaddr, uip_ntohs(gprsconn->port));
    PT_ATSTR(str);
    PT_ATWAIT2(60, &wait_connectok, &wait_cmeerror, &wait_commandnoresponse);
    if (at == &wait_connectok) {
      gprs_statistics.connections += 1;
      call_event(gprsconn->socket, TCP_SOCKET_CONNECTED);
      break;
    }
    else if (at == &wait_commandnoresponse) {
      /* Give it some more time. It happens that the connection succeeds seconds after COMMAND NO RESPONSE! */
      PT_ATWAIT2(15, &wait_connectok);
      if (at == &wait_connectok) {
        gprs_statistics.connections += 1;
        call_event(gprsconn->socket, TCP_SOCKET_CONNECTED);
        yled_interval = CLOCK_SECOND/2;
        process_post(&yled, 3, &yled_interval);
        break;
      }
    }

    /* If we ended up here, we failed to set up connection */
    if (at == NULL) {
      /* Timeout */
      gprs_statistics.connfailed += 1;
      PT_ATSTR("AT+CIPCLOSE\r");
      PT_ATWAIT2(5, &wait_ok);
      PT_ATSTR("AT+CIPSHUT\r");
      PT_ATWAIT2(5, &wait_ok);
      status.state = GPRS_STATE_NONE;
      call_event(gprsconn->socket, TCP_SOCKET_TIMEDOUT);
      break;
    }        
    else if (at == &wait_cmeerror) {
      /* COMMAND NO RESPONSE! timeout. Sometimes it take longer though and have seen COMMAND NON RESPONSE! followed by CONNECT OK */ 
      /* Seen +CME ERROR:53 */
      printf("CIPSTART failed\n");
      PT_ATSTR("AT+CREG?\r");
      PT_ATWAIT2(2, &wait_ok);
      gprs_statistics.at_errors += 1;
      PT_ATSTR("AT+CIPCLOSE\r");
      PT_ATWAIT2(15, &wait_ok);//ATWAIT2(5, &wait_ok, &wait_cmeerror);
      PT_ATSTR("AT+CIPSHUT\r");
      PT_ATWAIT2(15, &wait_ok);//ATWAIT2(5, &wait_ok,  &wait_cmeerror);

      /* Test to cure deadlock when closing/shutting down  --ro */
      if (minor_tries++ > 10) {
        gprs_statistics.connfailed += 1;
        break;
      }
      else {
        continue;
      }
    }
  } /* minor_tries */
  if (minor_tries >= 10) {
    gprs_statistics.connfailed += 1;
    call_event(gprsconn->socket, TCP_SOCKET_TIMEDOUT);
  }
  PT_END(pt);
}

/*---------------------------------------------------------------------------*/
/*
 * send
 *
 * Protothread to send data with AT commands
 */
static
PT_THREAD(a6_send(struct pt *pt, struct gprs_connection * gprsconn)) {
  static struct at_wait *at;
  static uint16_t remain;
  static uint16_t len;
  static uint8_t *ptr;

  PT_BEGIN(pt);
#ifdef GPRS_DEBUG
  printf("A6AT GPRS Send\n");
#endif /* GPRS_DEBUG */

  ptr = gprsconn->output_data_ptr;
  //socket = gprsconn->socket;
  //remain = socket->output_data_len;
  remain = gprsconn->output_data_len;
#if 1
  PT_ATSTR("ATE0\r\n");
  PT_ATWAIT2(5, &wait_ok);
  if (at == NULL)
    goto timeout;
#endif
  while (remain > 0) {
    char buf[20];
    len = (remain <= GPRS_MAX_SEND_LEN ? remain : GPRS_MAX_SEND_LEN);
    sprintf((char *) buf, "AT+CIPSEND=%d\r", len);
    PT_ATSTR((char *) buf); /* sometimes CME ERROR:516 */
    PT_ATWAIT2(5, &wait_ok, &wait_sendprompt, &wait_error);
    if (at == NULL) {
      PT_ATSTR("ATE1\r\n");
      PT_ATWAIT2(5, &wait_ok);
      goto timeout;
    }
    else if (at == &wait_error) {
      goto disconnect;
    }
    PT_ATBUF(&ptr[gprsconn->output_data_len-remain], len);
    PT_ATWAIT2(30, &wait_ok, &wait_commandnoresponse);
    if (at == NULL) {
      goto timeout;
    }        
    if (at == &wait_commandnoresponse) {
      goto disconnect;
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
  PT_ATSTR("ATE1\r\n");
  PT_ATWAIT2(5, &wait_ok);
  PT_EXIT(pt);

 disconnect:
  gprs_statistics.at_errors += 1;
  call_event(gprsconn->socket, TCP_SOCKET_CLOSED);
  PT_EXIT(pt);

 timeout:
    gprs_statistics.at_timeouts += 1;
    call_event(gprsconn->socket, TCP_SOCKET_TIMEDOUT);
    status.state = GPRS_STATE_NONE;
  PT_END(pt);
} 
/*---------------------------------------------------------------------------*/
/*
 * close
 *
 * Protothread to close TCP connection with AT commands
 */
static
PT_THREAD(a6_close(struct pt *pt, struct gprs_connection * gprsconn)) {
  static struct at_wait *at;
  static struct tcp_socket_gprs *socket;
  char str[20];
  PT_BEGIN(pt);
#ifdef GPRS_DEBUG
  printf("A6AT GPRS Close\n");
#endif /* GPRS_DEBUG */

  snprintf(str, sizeof(str), "AT+CIPCLOSE\r");
  PT_ATWAIT2(15, &wait_ok, &wait_cmeerror);
  if (at == NULL) {
    gprs_statistics.at_timeouts += 1;
  }
  printf("Call socket_closed\n");
  call_event(socket, TCP_SOCKET_CLOSED);
  printf("Called socket_closed\n");
  PT_END(pt);
} 
/*---------------------------------------------------------------------------*/
  
PROCESS_THREAD(a6at, ev, data) {
  struct at_wait *at;
  static struct gprs_event *gprs_event;
  static struct gprs_connection *gprsconn;

  PROCESS_BEGIN();
  leds_init();
  event_init();
  event_queue_init();
  
  module_init(115200);
  {
    uint8_t s;

    set_board_5v(0); /* Power cycle the board */
    DELAY(4);
    set_board_5v(1);
    DELAY(2);
    s = sc16is_gpio_get();
    printf("LOOP GPIO=0x%02x\n", sc16is_gpio_get());
    clr_bit(&s, G_PWR);
    set_bit(&s, G_RESET);

    set_bit(&s, G_LED_RED); /*OFF */
    set_bit(&s, G_LED_YELLOW);
    sc16is_gpio_set(s);
    DELAY(2);
    clr_bit(&s, G_RESET);
    sc16is_gpio_set(s);
    /* start */
    DELAY(2);
    printf("Start Radio\n");
    s = sc16is_gpio_get();
    set_bit(&s, G_PWR);
    sc16is_gpio_set(s);

    ATSTR("AT\r");
    DELAY(5);

    status.module = GPRS_MODULE_UNNKOWN;
    ATSTR("ATI\r");
    ATWAIT2(10, &wait_ati);
    if (at == NULL) {
      printf("No module version found\n");
    }
  }
  again:
  if (status.state ==  GPRS_STATE_NONE) {
    ATSPAWN(init_module);
    goto again;
  }
  if (status.state == GPRS_STATE_IDLE) {
    ATSPAWN(apn_register);
    goto again;
  }
  if (status.state == GPRS_STATE_REGISTERED) {
    ATSPAWN(read_csq);
    ATSPAWN(apn_activate);
    ATSPAWN(get_moduleinfo);
    ATSPAWN(get_ipconfig);
    if (status.state != GPRS_STATE_ACTIVE)
      goto again;
    process_post(PROCESS_BROADCAST, a6at_gprs_init, NULL);
    if(status.module == GPRS_MODULE_A7) {
      ATSTR("AT+AGPS=1\r");
      ATWAIT2(25, &wait_ok);
      if (at == NULL) {
        printf("AGPS not enabled\n");
      }
      ATSTR("AT+GPSRD=30\r");
      ATWAIT2(10, &wait_gpsrd);
      if (at == NULL) {
        printf("GPSRD not enabled\n");
      }
    }
    goto again;
  }
  if (status.state == GPRS_STATE_ACTIVE) {
    gprs_event = dequeue_event();
    if (gprs_event == NULL) {
      PROCESS_PAUSE();
      goto again;
    }
    else
      printf("dequeed eveny %d %s\n", gprs_event->ev, eventstr(gprs_event->ev));

    gprsconn = (struct gprs_connection *) gprs_event->data;
    if (gprs_event->ev == a6at_gprs_connection) {
#ifdef GPRS_DEBUG
      printf("A6AT GPRS Connection\n");
#endif /* GPRS_DEBUG */
      ATSPAWN(a6_connect, gprsconn);
    } /* ev == a6at_gprs_connection */
    else if (gprs_event->ev == a6at_gprs_send) {
      ATSPAWN(a6_send, gprsconn);
    } /* ev == a6at_gprs_send */
    else if (gprs_event->ev == a6at_gprs_close) {
      ATSPAWN(a6_close, gprsconn);
    } /* ev == a6at_gprs_close */
#ifdef GPRS_DEBUG
    else {
      printf("A6AT GPRS Unknown event %d\n", gprs_event->ev);
    }
#endif /* GPRS_DEBUG */
  }
  ATSPAWN(read_csq);
  goto again;
  PROCESS_END();
}

static struct etimer yled_timer;

PROCESS_THREAD(yled, ev, data) 
{
  PROCESS_BEGIN();

  sc16is_gpio_set_bit(G_LED_YELLOW);

  while (1) { 
    PROCESS_WAIT_EVENT();
    if(etimer_expired(&yled_timer)) { 
      etimer_stop(&yled_timer); 
      sc16is_gpio_toggle_bit(G_LED_YELLOW);
      etimer_set(&yled_timer, yled_interval);
    }
    else if(ev == 1) {
      etimer_stop(&yled_timer); 
      sc16is_gpio_set_bit(G_LED_YELLOW);
    }
    else if(ev == 2) {
      etimer_stop(&yled_timer); 
      sc16is_gpio_clr_bit(G_LED_YELLOW);
    }
    else if(ev == 3) {
      etimer_stop(&yled_timer); 
      etimer_set(&yled_timer, yled_interval);
    }
  }
  PROCESS_END();
}
