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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "i2c.h"
#include "dev/leds.h"
#include "dev/sc16is/sc16is.h"
#include "tcp-socket-gprs.h"
#include "gprs-a6.h"

#define GP0 1
#define GP1 2
#define GP2 4
#define GP3 8
#define G_RESET GP0
#define G_SLEEP  GP1
#define G_PWR_KEY GP2

#define APN "4g.tele2.se"
#define PDPTYPE "IP"

uint32_t baud;
uint8_t at[] = {'A', 'T', 0xd };

#define GPRS_MAX_CONNECTION 1
struct gprs_connection gprs_connections[GPRS_MAX_CONNECTION];
struct gprs_context gprs_context;

process_event_t sc16is_input_event;
process_event_t at_match_event;

#define min(A, B) ((A) <= (B) ? (A) : (B))
/*---------------------------------------------------------------------------*/
PROCESS(sc16is_reader, "I2C UART input process");
//PROCESS(sc16is_at, "I2C UART AT emitter");
PROCESS(gprs_init_pt, "GPRS Initializer");
PROCESS(a6at, "GPRS A6 module");
extern struct process testapp;
//AUTOSTART_PROCESSES(&sc16is_reader);
AUTOSTART_PROCESSES(&sc16is_reader, &a6at, &testapp); //&sc16is_at);
//AUTOSTART_PROCESSES(&sc16is_reader, &a6at);

/*---------------------------------------------------------------------------*/
static struct gprs_connection *
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
gprs_init() {
  gprs_set_context(&gprs_context, "IP", APN);
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
  process_post(&a6at, a6at_gprs_activate, gcontext);
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Do we need this */
int
gprs_register(struct gprs_connection *gconn,
              struct tcp_socket_gprs *socket,
              void *callback) {

  gconn->socket = socket;
  gconn->callback = callback;
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Do we need this */
int
gprs_unregister(struct gprs_connection *gconn) {

  gconn->socket = NULL;
  return 0;
}
/*---------------------------------------------------------------------------*/
struct gprs_connection *
gprs_connection(const char *proto, const char *ipaddr, uint16_t port,
                struct tcp_socket_gprs *socket) {
  struct gprs_connection *gprsconn;
  if ((gprsconn = alloc_gprs_connection()) == NULL)
    return NULL;
  gprsconn->context = &gprs_context;
  gprsconn->proto = proto;
  gprsconn->socket = socket;
  gprsconn->ipaddr = ipaddr;
  gprsconn->port = port;
  if (process_post(&a6at, a6at_gprs_connection, gprsconn) != PROCESS_ERR_OK) {
    free_gprs_connection(gprsconn);
    return NULL;
  }
  else {
    return gprsconn;
  }
}
/*---------------------------------------------------------------------------*/
void
gprs_send(struct tcp_socket_gprs *socket) {
  (void) process_post(&a6at, a6at_gprs_send, socket->g_c);
}
/*---------------------------------------------------------------------------*/
void
gprs_close(struct tcp_socket_gprs *socket) {
  (void) process_post(&a6at, a6at_gprs_close, socket->g_c);
}
/*---------------------------------------------------------------------------*/

struct at_wait;

typedef char (* at_callback_t)(struct pt *, struct at_wait *, uint8_t *data, int len);

struct at_wait {
  char *str;
  at_callback_t callback;
  uint8_t active;
  uint8_t pos;
  uint8_t remain;  
};
 
static
PT_THREAD(wait_ciprcv_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len));
static
PT_THREAD(wait_basic_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len));
static
PT_THREAD(wait_line_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len));
static
PT_THREAD(wait_cipstatus_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len));
static
PT_THREAD(wait_cmeerror_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len));
static
PT_THREAD(wait_creg_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len));
static
PT_THREAD(wait_tcpclosed_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len));


struct at_wait wait_ciprcv = {"+CIPRCV:", wait_ciprcv_callback};
struct at_wait wait_ok = {"OK", wait_basic_callback};
struct at_wait wait_cipstatus = {"+CIPSTATUS:", wait_cipstatus_callback};
struct at_wait wait_creg = {"+CREG: ", wait_creg_callback};
struct at_wait wait_connectok = {"CONNECT OK", wait_basic_callback};
struct at_wait wait_cmeerror = {"CME ERROR", wait_cmeerror_callback};
struct at_wait wait_commandnoresponse = {"COMMAND NO RESPONSE!", wait_basic_callback};
struct at_wait wait_sendprompt = {">", wait_basic_callback};
struct at_wait wait_tcpclosed = {"+TCPCLOSED:", wait_tcpclosed_callback};
struct at_wait *at_waitlist[] = {&wait_ciprcv, &wait_ok, &wait_cipstatus,
                                 &wait_creg, &wait_connectok, &wait_cmeerror,
                                 &wait_commandnoresponse, &wait_sendprompt,
                                 &wait_tcpclosed};
#define NUMWAIT (sizeof(at_waitlist)/sizeof(struct at_wait *))

struct pt wait_pt;
static char atline[80];

static void
start_at(struct at_wait *at) {
  at->active = 1;
  at->pos = 0;
  at->remain = strlen(at->str);
}

static void
stop_at(struct at_wait *at) {
  at->active = 0;
  at->pos = 0;
}

static void start_atlist(struct at_wait *at, ...) { //char *str, char *end) {
  va_list valist;
  
  va_start(valist, at);
  while (at) {
    printf(" %s ", at->str);
    start_at(at);
    at = va_arg(valist, struct at_wait *);
  }
}

static void stop_atlist(struct at_wait *at, ...) { //char *str, char *end) {
  va_list valist;
  
  va_start(valist, at);
  while (at) {
    stop_at(at);
    at = va_arg(valist, struct at_wait *);
  }
}


static void
wait_init() {
  int i;
  for (i = 0; i < NUMWAIT; i++) {
    stop_at(at_waitlist[i]);
  }
  /* The following two are to detect async events -- always active */
  start_at(&wait_ciprcv);
  start_at(&wait_tcpclosed);  
  PT_INIT(&wait_pt);
}

static
PT_THREAD(wait_basic_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len)) {
  PT_BEGIN(pt);
  process_post(&a6at, at_match_event, at);
  PT_END(pt);
}

static uint8_t rcvdata[GPRS_MAX_RECV_LEN];
static uint16_t rcvlen;

static void
newdata(struct tcp_socket_gprs *s, uint16_t len, uint8_t *dataptr) 
{
  uint16_t /* len,*/ copylen, bytesleft;
  /* uint8_t *dataptr;
     len = uip_datalen();
     dataptr = uip_appdata; */

  /* We have a segment with data coming in. We copy as much data as
     possible into the input buffer and call the input callback
     function. The input callback returns the number of bytes that
     should be retained in the buffer, or zero if all data should be
     consumed. If there is data to be retained, the highest bytes of
     data are copied down into the input buffer. */
  if (0){
    int i;
    printf("newdata %d @0x%x: '", len, (unsigned) dataptr);
    for (i = 0; i < len; i++)
      printf("%c", (char ) dataptr[i]);
    printf("'\n");
  }
  do {
    copylen = MIN(len, s->input_data_maxlen);
    memcpy(s->input_data_ptr, dataptr, copylen);
    if(s->input_callback) {
      bytesleft = s->input_callback(s, s->ptr,
				    s->input_data_ptr, copylen);
    } else {
      bytesleft = 0;
    }
    if(bytesleft > 0) {
      printf("tcp: newdata, bytesleft > 0 (%d) not implemented\n", bytesleft);
    }
    dataptr += copylen;
    len -= copylen;

  } while(len > 0);
}

static
PT_THREAD(wait_ciprcv_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len)) {
  static uint16_t nbytes;
  static int rcvpos;
  int datapos = 0;
  struct gprs_connection *gprsconn;
  
  PT_BEGIN(pt);
  nbytes = 0;
  while (datapos < len && data[datapos] != ',') {
    nbytes = nbytes*10 + (data[datapos++] - '0');
    if (datapos == len) {
      PT_YIELD(pt);
    }
  }
  if (nbytes > 0 && ++datapos == len) {
    PT_YIELD(pt);
  }
  rcvlen = min(nbytes, GPRS_MAX_RECV_LEN);
  rcvpos = 0;
  while (nbytes-- > 0) {
    if (rcvpos < GPRS_MAX_RECV_LEN)
      rcvdata[rcvpos] = data[datapos];
    rcvpos++; datapos++;
    if (datapos == len) {
      PT_YIELD(pt);
    }
  }
  rcvdata[rcvpos] = '\0'; 
  start_at(&wait_ciprcv); /* restart */
  gprsconn = find_gprs_connection();
  if (gprsconn && gprsconn->socket) {
    newdata(gprsconn->socket, rcvlen, rcvdata);
  }
  PT_END(pt);
}

static
PT_THREAD(wait_line_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len)) {
  static int atpos;
  int done = 0;
  int datapos = 0;

  PT_BEGIN(pt);
  atpos = 0; 
  while (done == 0) {
    while (datapos < len && done == 0) {
      if (data[datapos] == '\n' || data[datapos] == '\r') {
        //printf("-seen EOL-");
        done = 1;
      }
      else {
        //printf("-%d %c %d+ ", atpos, data[datapos], datapos);
        atline[atpos++] = (char ) data[datapos++];
        if (atpos == sizeof(atline))
          done = 1;
      }
    }
    if (!done) {
      //printf(" --NOT DONE\n");
      /* Reached end of input data but have not seen end of line. 
       * Yield here and wait for next invocation.
       */ 
      PT_YIELD(pt);
    }
  }
  /* done -- mark end of string */
  atline[atpos] = '\0';
  process_post(&a6at, at_match_event, at);
  PT_END(pt);
}

static
PT_THREAD(wait_tcpclosed_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len)) {
  static int atpos;
  struct gprs_connection *gprsconn;
  int done = 0;
  int datapos = 0;

  printf("Here is tcpclosed\n");
  PT_BEGIN(pt);
  atpos = 0; 
  while (done == 0) {
    while (datapos < len && done == 0) {
      if (data[datapos] == '\n' || data[datapos] == '\r') {
        //printf("-seen EOL-");
        done = 1;
      }
      else {
        //printf("-%d %c %d+ ", atpos, data[datapos], datapos);
        atline[atpos++] = (char ) data[datapos++];
        if (atpos == sizeof(atline))
          done = 1;
      }
    }
    if (!done) {
      //printf(" --NOT DONE\n");
      /* Reached end of input data but have not seen end of line. 
       * Yield here and wait for next invocation.
       */ 
      PT_YIELD(pt);
    }
  }
  /* done -- mark end of string */
  atline[atpos] = '\0';
  gprsconn = find_gprs_connection();
  if (gprsconn && gprsconn->socket) {
    call_event(gprsconn->socket, TCP_SOCKET_CLOSED);
  }
  PT_END(pt);
}

static
PT_THREAD(wait_cipstatus_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len)) {
  return wait_line_callback(pt, at, data, len);
}

static
PT_THREAD(wait_creg_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len)) {
  return wait_line_callback(pt, at, data, len);
}


static
PT_THREAD(wait_cmeerror_callback(struct pt *pt, struct at_wait *at, uint8_t *data, int len)) {
  return wait_line_callback(pt, at, data, len);
}

PT_THREAD(wait_fsm_pt(struct pt *pt, uint8_t *data, unsigned int len)) {
  uint8_t i;
  uint8_t datapos = 0;
  static struct pt subpt;
  static struct at_wait *at;

  PT_BEGIN(pt);
  while (1) {
    if (len > 0) {
      for (datapos = 0; datapos < len; datapos++){
        for (i = 0; i < NUMWAIT; i++) {
          at = at_waitlist[i];
          if (at->active) {
            if (data[datapos] == at->str[at->pos]) {
               /* Matched one char -- was it the last?*/
              if (at->str[++at->pos] == '\0') {
                //printf("FSM matched '%s'\n", at->str);
                PT_INIT(&subpt);
                datapos++; /* consume last matched char */
                while (at->callback(&subpt, at, &data[datapos], len-datapos) != PT_ENDED) {
                  PT_YIELD(pt);
                }
                PT_RESTART(pt);
              }
            }
            else {
              /* Does not match current char. Restart. */
              at->pos = 0;
            }
          }
        }
      }
    }
    PT_YIELD(pt);
  }
  PT_END(pt);
}

#define MAXWAIT 3
#define PERMANENT 0

static struct waitfor {
  uint8_t active;
  unsigned char *str;
  uint8_t pos;
  uint8_t found;  
} waitfor[MAXWAIT];

int len;
uint8_t buf[200];


uint8_t found;
static unsigned char foundbuf[200];
static int foundpos;
static unsigned char atbuf[256];
static uint16_t atpos;

static void startwait(char *str, ...) { //char *str, char *end) {
  va_list valist;
  char *s;
  uint8_t w;
  
  start_at(&wait_ok);

  va_start(valist, str);
  s = str;
  w = PERMANENT;
  while (s && w < MAXWAIT) {
    printf(" %s", s);
    waitfor[w].str = (unsigned char *)s;
    waitfor[w].pos = 0;
    waitfor[w].found = 0;
    w++;
    foundpos = 0;
    s = va_arg(valist, char *);
  }

}


static void
stopwait() {
  waitfor[PERMANENT].str = NULL;
  stop_at(&wait_ok);
}

static unsigned char
*matchwait(unsigned char *bytes) {
  uint8_t pos;
  uint8_t w = 0;  
  for(w = 0; w < MAXWAIT && waitfor[w].str != NULL; w++) {
    if (!waitfor[w].found) {
      pos = 0;
      while (bytes[pos] != 0) {
        if (bytes[pos++] == waitfor[w].str[waitfor[w].pos++]) {
          if (waitfor[w].str[waitfor[w].pos] == 0) {
            /* Found it */
            uint8_t foundpos;
            waitfor[w].found = 1;
            foundpos = 0;
            while (bytes[pos] != 0 && bytes[pos] != '\n' && foundpos < sizeof(foundbuf))
              foundbuf[foundpos++] = bytes[pos++];
            foundbuf[foundpos] = 0;
            //printf("Found @%d \"%s\": remains %d: \"%s\"\n", w, waitfor[w].str, foundpos, foundbuf);
            //printf("Match \"%s\", found \"%s\": remains %d: \"%s\"\n", buf, waitfor[w].str, foundpos, foundbuf);
            return waitfor[w].str;
          }
        }
        else {
          waitfor[w].pos = 0;
        }
      }
    }
  }
  return NULL;
}
      
static void
dumpstr(unsigned char *str) {
  unsigned char *s = str;

  printf("    ");
  while (*s) {
    if (*s == '\n')
      printf("\n    ");
    else
      printf("%c", *s);
    s++;
  }
  printf("\n");
}

static uint8_t *match;
void module_init(void)
{
  if( i2c_probed & I2C_SC16IS ) {
    /* GPIO set output */
    sc16is_gpio_set_dir(G_RESET|G_PWR_KEY|G_SLEEP);

    sc16is_init();
    baud = 115200;
    sc16is_uart_set_speed(baud);
    //sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_FCR, SC16IS_FCR_FIFO_BIT);
    sc16is_tx((uint8_t *)"AT", sizeof("AT"-1));
  }
}



PROCESS_THREAD(sc16is_reader, ev, data)
{
  PROCESS_BEGIN();

#if 0
  sc16is_input_event = process_alloc_event();
  at_match_event = process_alloc_event();  

  module_init();
  leds_init();

  /* Fix baudrate  */
  sc16is_tx(at, sizeof(at));
#endif
  printf ("here is reader\n");
  wait_init();
  atpos = 0;
  memset(atbuf, 0, sizeof(atbuf));
  while(1) {
    PROCESS_PAUSE();
#if 0
    start_atlist(&wait_creg, NULL);
    
    wait_fsm_pt(&wait_pt, (uint8_t *) "AVOK", 2);
    wait_fsm_pt(&wait_pt, (uint8_t *) "+CRE", 4);
    wait_fsm_pt(&wait_pt, (uint8_t *) "G: this is", 10);        
    wait_fsm_pt(&wait_pt, (uint8_t *) "data\n", 5);            
    {
      static struct etimer et;
      etimer_set(&et, CLOCK_SECOND*30);
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
      continue;
    }        
#endif    
    if( i2c_probed & I2C_SC16IS ) {
      len = sc16is_rx(buf, sizeof(buf));
      if (len) {
        buf[len] = '\0';
        dumpstr(buf);
        wait_fsm_pt(&wait_pt, buf, len);
        match = matchwait(buf);
        if (match != NULL) {
          stopwait();
          /* Tell other processes there was a match */
          if (process_post(PROCESS_BROADCAST, at_match_event, match) == PROCESS_ERR_OK) {
            PROCESS_WAIT_EVENT_UNTIL(ev == at_match_event);
          }
        }
      }
    }
  }
  PROCESS_END();
}

static struct etimer et;

static void
sendbuf(unsigned char *buf, size_t len) {
  sc16is_tx(buf, len); 
}

#define ATSTR(str) sendbuf((unsigned char *) str, strlen(str))
#define ATBUF(buf, len) sendbuf(buf, len)

#define ATWAIT2(SEC, ...)  {                    \
  at = NULL; \
  printf("---start_atlist:%d: (%d sec)", __LINE__, SEC);    \
  start_atlist(__VA_ARGS__, NULL);            \
  printf("\n"); \
  etimer_set(&et, (SEC)*CLOCK_SECOND);                         \
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et) || \
                           ev == at_match_event); \
  /* printf("#####WAIYED\n"); */                  \
  if(etimer_expired(&et)) { \
    printf("---timeout:%d\n", __LINE__);                 \
  } \
  else if(ev == at_match_event) { \
    etimer_stop(&et); \
    at = (struct at_wait *) data;                \
    printf("---got %s\n", at->str);\
  } \
  stop_atlist(__VA_ARGS__, NULL);               \
  }


#define ATWAIT(delay, ...)                     \
  res = NULL; \
  printf("---startwait:%d: delay=%d", __LINE__, delay);     \
  startwait(__VA_ARGS__, NULL);            \
  printf("\n");                       \
  etimer_set(&et, delay); \
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et) || \
                           ev == at_match_event); \
  /* printf("#####WAIYED\n"); */                  \
  if(etimer_expired(&et)) { \
    printf("---timeout:%d\n", __LINE__);                 \
    stopwait(); \
    res = NULL; \
  } \
  else if(ev == at_match_event) { \
    etimer_stop(&et); \
    res = (unsigned char *) data;                \
    printf("---got %s\n", res);\
  } 
#undef ATWAIT
#define ATWAIT(delay, ...)

#define DELAY(SEC)   { \
    printf("%d: DELAY(%d)\n", __LINE__, SEC); \
    etimer_set(&et, SEC*CLOCK_SECOND); \
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et)); \
  }

//#define APN "online.telia.se"
#define APN "4g.tele2.se"
#define PDPTYPE "IP"
#define IPADDR  "130.237.22.219"
#define PORTNO 9999


PROCESS_THREAD(a6at, ev, data) {
  //unsigned char *res;
  struct at_wait *at;
  char str[80];
  static struct gprs_connection *gprsconn;
  static struct tcp_socket_gprs *socket;

  
  PROCESS_BEGIN();
  a6at_gprs_init = process_alloc_event();
  a6at_gprs_activate = process_alloc_event();  
  a6at_gprs_connection = process_alloc_event();
  a6at_gprs_send = process_alloc_event();
  a6at_gprs_close = process_alloc_event();  
  
  sc16is_input_event = process_alloc_event();
  at_match_event = process_alloc_event();
  at_recv_event = process_alloc_event();    

  wait_init();
  module_init();
  leds_init();


  gprs_init();


  /* Fix baudrate  */
  sc16is_tx((uint8_t *) "AT", sizeof("at"-1));
  
 again:
  if (0){

  /* Reset module */
  printf("Resetting... gpio == 0x%x\n", sc16is_gpio_get());
  sc16is_gpio_set(sc16is_gpio_get()|G_RESET); /* Reset on */
  printf("Initializing... gpio == 0x%x\n", sc16is_gpio_get());
  /* Power on */
  sc16is_gpio_set(sc16is_gpio_get()|G_PWR_KEY|G_SLEEP); /* Power key on, no sleep */
  printf("Probing2... gpio == 0x%x\n", sc16is_gpio_get());
  DELAY(5);
  sc16is_gpio_set(sc16is_gpio_get()&~G_PWR_KEY); /* Toggle power key (power will remain on) */
  printf("Probing3... gpio == 0x%x\n", sc16is_gpio_get());
  ATWAIT2(30, &wait_ok);
  ATWAIT(CLOCK_SECOND*30, "OK");
  goto again;
  }

  {
    printf("Resetting... gpio == 0x%x\n", sc16is_gpio_get());
    sc16is_gpio_set(sc16is_gpio_get()|G_RESET); /* Reset on */
    printf("Sleeping... gpio == 0x%x\n", sc16is_gpio_get());
    DELAY(5);ATWAIT(CLOCK_SECOND*5, "KOKO");
    sc16is_gpio_set((sc16is_gpio_get()&~G_RESET)|G_PWR_KEY|G_SLEEP); /* Toggle reset, power key on, no sleep */
    printf("Probing2... gpio == 0x%x\n", sc16is_gpio_get());
    DELAY(5);ATWAIT(CLOCK_SECOND*5, "KOKO");
    sc16is_gpio_set(sc16is_gpio_get()&~G_PWR_KEY); /* Toggle power key (power will remain on) */
    printf("Probing3... gpio == 0x%x\n", sc16is_gpio_get());
    ATSTR("AT\r"); ATWAIT(CLOCK_SECOND*10, "OK");
    DELAY(5);    ATWAIT(CLOCK_SECOND*10, "KOOK");
  }

  /* Wait for registration status to become 1 (local registration)
   * or 5 (roaming)
   */
  while (1) {
    static uint8_t creg;
    DELAY(2);
    ATSTR("AT+CIPSTATUS?\r");
    ATWAIT2(10, &wait_cipstatus);
    ATWAIT(CLOCK_SECOND*10, "+CIPSTATUS:0,");
    ATSTR("AT+CREG?\r");
    ATWAIT2(10, &wait_creg);
    ATWAIT(CLOCK_SECOND*10, "+CREG: 1,");
    if (at == NULL)
      continue;
    creg = atoi((char *) atline /*foundbuf*/);
    printf("creg atoi(\"%s\") -> %d\n", atline, creg);
    if (creg == 1 || creg == 5 || creg == 10) /* Wait for status == 1 (local registration) */
      break; /* Wait for status == 1 (local registration) */    
  }
  printf("Done---- init\n");
  process_post(PROCESS_BROADCAST, a6at_gprs_init, NULL);

  while(1) {
  nextcommand:
    PROCESS_WAIT_EVENT();
    if (ev == a6at_gprs_activate) {
      printf("GPRS Activate\n");
      static struct gprs_context *gcontext;

      gcontext = (struct gprs_context *) data;
      /* Deactivate PDP context */
      sprintf(str, "AT+CSTT=\"%s\", \"\", \"\"\r", gcontext->apn); /* Start task and set APN */
      ATSTR(str);   
      ATWAIT2(20, &wait_ok);
      ATWAIT(CLOCK_SECOND*20, "OK");

      ATSTR("AT+CGATT=1\r"); ATWAIT2(5, &wait_ok);
      ATWAIT(CLOCK_SECOND*5, "OK"); /* Service attach */

      ATSTR("AT+CIPMUX=0\r"); ATWAIT2(5, &wait_ok);
      ATWAIT(CLOCK_SECOND*5, "OK"); /* Single IP connection */

      ATSTR("AT+CREG?\r");      ATWAIT2(1, &wait_ok);
      ATWAIT(CLOCK_SECOND, "OK");
  
      sprintf(str, "AT+CGDCONT=1,%s,%s\r", gcontext->pdptype, gcontext->apn); /* Set PDP (Packet Data Protocol) context */
      ATSTR(str);        ATWAIT2(5, &wait_ok);
      ATWAIT(CLOCK_SECOND*5, "OK");
      ATSTR("AT+CREG?\r");      ATWAIT2(1, &wait_ok);
      ATWAIT(CLOCK_SECOND, "OK");
      ATSTR("AT+CGACT=1,1\r");       ATWAIT2(20, &wait_ok);
      ATWAIT(CLOCK_SECOND*20, "OK"); /* Activate context */

      ATSTR("AT+CREG?\r");       ATWAIT2(2, &wait_ok);
      ATWAIT(CLOCK_SECOND*2, "OK");
      
      while (1) {
        ATSTR("AT+CIPSTATUS?\r"); 
        ATWAIT2(60, &wait_cipstatus);
        ATWAIT(CLOCK_SECOND*60, "+CIPSTATUS:0,");
        if (at == &wait_cipstatus) {
          if (strncmp((char *) atline /*foundbuf*/, "IP GPRSACT", strlen("IP GPRSACT")) == 0) {
            printf("GPRS is active\n");
            gcontext->active = 1;
            //process_post(PROCESS_BROADCAST, a6at_gprs_active, NULL);
            break;
          }
        }
        printf("GPRS not active\n");
        printf("Got gpio 0x%x\n", sc16is_gpio_get());
        ATSTR("AT+CREG?\r");       ATWAIT2(2, &wait_ok);
        ATWAIT(CLOCK_SECOND*2, "OK");
      }
    } /* ev == a6at_gprs_activate */
    else if (ev == a6at_gprs_connection) {
      gprsconn = (struct gprs_connection *) data;

    try:
      printf("Here is connection %s %s:%d\n", gprsconn->proto, gprsconn->ipaddr, uip_htons(gprsconn->port));

      ATSTR("AT+CREG?\r");       ATWAIT2(2, &wait_ok);
      ATWAIT(CLOCK_SECOND*2, "OK");
      ATSTR("AT+CIPSTATUS?\r");       ATWAIT2(2, &wait_ok);
      ATWAIT(CLOCK_SECOND*2, "OK");

      sprintf(str, "AT+CIPSTART= \"%s\", %s, %d\r", gprsconn->proto, gprsconn->ipaddr, uip_ntohs(gprsconn->port));
      ATSTR(str);
      ATWAIT2(60, &wait_connectok, &wait_cmeerror, &wait_commandnoresponse);
      ATWAIT(CLOCK_SECOND*60, "CONNECT OK", "CME ERROR", "COMMAND NO RESPONSE!");
      if (at == &wait_connectok) {
        call_event(gprsconn->socket, TCP_SOCKET_CONNECTED);
        goto nextcommand;
      }
      else if (at == &wait_commandnoresponse) {
        /* Give it some more time. It happens that the connection succeeds after COMMAND NO RESPONSE! */
        ATWAIT2(15, &wait_connectok);
        ATWAIT(CLOCK_SECOND*15, "CONNECT OK");
        if (at == &wait_connectok) {
          call_event(gprsconn->socket, TCP_SOCKET_CONNECTED);
          goto nextcommand;
        }
      }
      /* If we ended up here, we failed to set up connection */
      if (at == NULL) {
        /* Timeout */
          ATSTR("AT+CIPCLOSE\r");
          ATWAIT2(5, &wait_ok);
          ATWAIT(CLOCK_SECOND*5, "OK");
          ATSTR("AT+CIPSHUT\r");
          ATWAIT2(5, &wait_ok);
          ATWAIT(CLOCK_SECOND*5, "OK");
          call_event(gprsconn->socket, TCP_SOCKET_TIMEDOUT);
          goto nextcommand;
      }        
      else if (at == &wait_cmeerror) {
        /* COMMAND NO RESPONSE! timeout. Sometimes it take longer though and have seen COMMAND NON RESPONSE! followed by CONNECT OK */ 
        /* Seen +CME ERROR:53 */
        /* After CME ERROR:53, seen CIPSTATUS stuck at IP START */
        printf("CIPSTART failed\n");
        ATSTR("AT+CREG?\r");           ATWAIT2(2, &wait_ok);
        ATWAIT(CLOCK_SECOND*2, "OK");    
        ATSTR("AT+CIPCLOSE\r");            ATWAIT2(5, &wait_ok);
        ATWAIT(CLOCK_SECOND*5, "OK");
        ATSTR("AT+CIPSHUT\r");            ATWAIT2(5, &wait_ok);
        ATWAIT(CLOCK_SECOND*5, "OK");
        goto try;
      }

      /* */;
    } /* ev == a6at_gprs_connection */
    else if (ev == a6at_gprs_send) {
      static uint16_t remain;
      static uint16_t len;

      gprsconn = (struct gprs_connection *) data;
      socket = gprsconn->socket;
      remain = socket->output_data_len;
      while (remain > 0) {
        
        len = (remain <= GPRS_MAX_SEND_LEN ? remain : GPRS_MAX_SEND_LEN);
        sprintf((char *) buf, "AT+CIPSEND=%d\r", len);
        ATSTR((char *) buf); /* sometimes CME ERROR:516 */
        ATWAIT2(2, &wait_sendprompt);
        ATWAIT(CLOCK_SECOND*5, ">");
        if (at == NULL) {
          printf("NO SENDPROMPT\n");
          goto failed;
        }
        if (0){
          int i;
          printf("ATBUF: '");
          for (i = 0; i < len; i++) {
            printf("%c", socket->output_data_ptr[socket->output_data_len-remain+len]);
          }
          printf("'\n");
        }
        ATBUF(&socket->output_data_ptr[socket->output_data_len-remain], len);
#ifdef notdef
        sprintf((char *) buf, "AT+CIPSEND=%d,", len-1);
        ATSTR((char *) buf); /* sometimes CME ERROR:516 */
        ATBUF(&socket->output_data_ptr[socket->output_data_len-remain], len);
        ATSTR((char *) "\r"); /* sometimes CME ERROR:516 */
#endif 
        ATWAIT2(30, &wait_ok, &wait_commandnoresponse);
        if (at == NULL || at == &wait_commandnoresponse) {
          goto failed;
        }        

        ATWAIT(CLOCK_SECOND*10, "OK");
        if (remain > len) {
          memcpy(&socket->output_data_ptr[0],
                 &socket->output_data_ptr[len],
                 remain - len);
        }
        socket->output_data_len -= len;
        remain -= len;
      }
      call_event(socket, TCP_SOCKET_DATA_SENT);
    } /* ev == a6at_gprs_send */
    else if (ev == a6at_gprs_close) {
      gprsconn = (struct gprs_connection *) data;
      socket = gprsconn->socket;

      ATSTR("AT+CIPCLOSE\r");
      ATWAIT2(15, &wait_ok);
      ATWAIT(CLOCK_SECOND*15, "OK");
      if (at == &wait_ok) {
        call_event(socket, TCP_SOCKET_CLOSED);
      }
      else {
        call_event(socket, TCP_SOCKET_TIMEDOUT);
      }
    } /* ev == a6at_gprs_close */
    continue;
  failed:
    /* Timeout */
    ATSTR("AT+CIPCLOSE\r");
    ATWAIT2(5, &wait_ok);
    ATWAIT(CLOCK_SECOND*5, "OK");
    ATSTR("AT+CIPSHUT\r");
    ATWAIT2(5, &wait_ok);
    ATWAIT(CLOCK_SECOND*5, "OK");
    call_event(gprsconn->socket, TCP_SOCKET_TIMEDOUT);
  }
  PROCESS_END();
}

/* Events:
+TCPCLOSED,0
+CIPRCV:4,hej
*/


PROCESS_THREAD(gprs_init_pt, ev, data) {
  unsigned char *res;
  char str[80];

  PROCESS_BEGIN();
 again:
  do {
    printf("Probing... gpio == 0x%x\n", sc16is_gpio_get());
    sc16is_gpio_set(sc16is_gpio_get()|G_PWR_KEY|G_SLEEP); /* Power key on, no sleep */
    printf("Probing2... gpio == 0x%x\n", sc16is_gpio_get());
    ATWAIT(CLOCK_SECOND*5, "KOKO");
    sc16is_gpio_set(sc16is_gpio_get()&~G_PWR_KEY); /* Toggle power key (power will remain on) */
    printf("Probing3... gpio == 0x%x\n", sc16is_gpio_get());
    ATSTR("AT\r"); ATWAIT(CLOCK_SECOND*10, "OK");
  }
  while (res == NULL);

  {
    printf("Resetting... gpio == 0x%x\n", sc16is_gpio_get());
    sc16is_gpio_set(sc16is_gpio_get()|G_RESET); /* Reset on */
    printf("Sleeping... gpio == 0x%x\n", sc16is_gpio_get());
    ATWAIT(CLOCK_SECOND*5, "KOKO");
    sc16is_gpio_set((sc16is_gpio_get()&~G_RESET)|G_PWR_KEY|G_SLEEP); /* Toggle reset, power key on, no sleep */
    printf("Probing2... gpio == 0x%x\n", sc16is_gpio_get());
    ATWAIT(CLOCK_SECOND*5, "KOKO");
    sc16is_gpio_set(sc16is_gpio_get()&~G_PWR_KEY); /* Toggle power key (power will remain on) */
    printf("Probing3... gpio == 0x%x\n", sc16is_gpio_get());
    ATSTR("AT\r"); ATWAIT(CLOCK_SECOND*10, "OK");

  }

 do {
    ATWAIT(CLOCK_SECOND*2, "KOKO");
    ATSTR("AT+CIPSTATUS?\r");
    ATWAIT(CLOCK_SECOND*60, "+CIPSTATUS:0,");
    ATSTR("AT+CREG?\r");
    ATWAIT(CLOCK_SECOND*10, "+CREG: 1,");
 } while (atoi((char *) atline /*foundbuf*/) != 1 && atoi((char *) atline /*foundbuf*/) != 5); /* Wait for status == 1 (local registration) */


  if (1){
    ATSTR("AT+CIPSTATUS?\r");
    ATWAIT(CLOCK_SECOND*10, "+CIPSTATUS:0,");
    ATSTR("AT+CGACT=1,0\r");  ATWAIT(CLOCK_SECOND*15, "OK"); 
    ATSTR("AT+CGACT=0,0\r");  ATWAIT(CLOCK_SECOND*15, "OK");
    ATSTR("AT+CGACT=0\r");  ATWAIT(CLOCK_SECOND*15, "OK");     
    if ((res == NULL) || (strncmp((char *) atline/* foundbuf*/, "IP START", strlen("IP START")) == 0)) { /* IP CLOSE OK?*/
      printf("IP STATUS not OK\n");
      ATSTR("AT+CGACT=1,0\r");  ATWAIT(CLOCK_SECOND*15, "OK"); 
      ATSTR("AT+CGACT=0,0\r");  ATWAIT(CLOCK_SECOND*15, "OK");
      ATSTR("AT+CGACT=0\r");  ATWAIT(CLOCK_SECOND*15, "OK");     
      ATSTR("AT+CIPSHUT=0\r"); ATWAIT(CLOCK_SECOND*15, "OK"); 
      ATSTR("AT+CGATT=0\r"); ATWAIT(CLOCK_SECOND*15, "OK"); 
      ATSTR("AT+CIPSTATUS?\r"); ATWAIT(CLOCK_SECOND*10, "OK");
      ATSTR("AT+CREG?\r"); ATWAIT(CLOCK_SECOND*10, "+CREG: 1,");
    }  
  }
  if (0) {
    ATSTR("AT+CREG?\r");
    ATWAIT(CLOCK_SECOND*10, "+CREG: 1,");
    if (atoi((char *) atline /*foundbuf*/) == 5) {
      printf("Roaming mode\n");
      ATSTR("AT+CGATT=0\r");
      ATWAIT(CLOCK_SECOND*10, "OK"); /* Echo on */
      ATSTR("AT+CREG?\r");
      ATWAIT(CLOCK_SECOND*10, "KOKO");
      ATSTR("AT+CREG?\r");
      ATWAIT(CLOCK_SECOND*10, "KOKO");
    }
  }  
  ATSTR("ATE1\r"); ATWAIT(CLOCK_SECOND*10, "OK"); /* Echo on */

  sprintf(str, "AT+CSTT=\"%s\", \"\", \"\"\r", APN); /* Start task and set APN */
  ATSTR(str);   ATWAIT(CLOCK_SECOND*20, "OK");

  ATSTR("AT+CREG?\r");ATWAIT(CLOCK_SECOND, "OK");

  ATSTR("AT+CGATT=1\r"); ATWAIT(CLOCK_SECOND*5, "OK"); /* Service attach */

  ATSTR("AT+CIPMUX=0\r"); ATWAIT(CLOCK_SECOND*5, "OK"); /* Single IP connection */

  ATSTR("AT+CREG?\r");ATWAIT(CLOCK_SECOND, "OK");
  
  sprintf(str, "AT+CGDCONT=1,%s,%s\r", PDPTYPE, APN); /* Set PDP (Packet Data Protocol) context */
  ATSTR(str);  ATWAIT(CLOCK_SECOND*5, "OK");
  ATSTR("AT+CREG?\r");ATWAIT(CLOCK_SECOND, "OK");
  ATSTR("AT+CGACT=1,1\r"); ATWAIT(CLOCK_SECOND*20, "OK"); /* Activate context */

  ATSTR("AT+CREG?\r"); ATWAIT(CLOCK_SECOND*2, "OK");
  ATSTR("AT+CIPSTATUS?\r"); 
  ATWAIT(CLOCK_SECOND*60, "+CIPSTATUS:0,");
  if (res != NULL && strcmp((char *) res, "+CIPSTATUS:0,") == 0) {
    if (strncmp((char *) atline /*foundbuf*/, "IP GPRSACT", strlen("IP GPRSACT")) == 0) {
      printf("GPRS is active\n");
      PROCESS_EXIT();
    }
    else {

      printf("GPRS not active\n");
      printf("Got gpio 0x%x\n", sc16is_gpio_get());
      ATSTR("AT+CREG?\r"); ATWAIT(CLOCK_SECOND*2, "OK");
      goto again;
    }
  }
  else {
    printf("No CIPSTATUS\n");
    goto again;
  }

  PROCESS_END();
}

#if 0
static int err;
extern struct process testapp;
PROCESS_THREAD(sc16is_at, ev, data)
{
  unsigned char *res;
  char str[80];
  static struct gprs_context gprs_context;

  err = 0;
  PROCESS_BEGIN();
  printf("HERE IS SC16it_at\n");
  //module_init();
  printf("Wait firsr 10\n");
  etimer_set(&et, CLOCK_SECOND*10);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

  printf("Wait 10 %d\n", CLOCK_SECOND);
  ATWAIT(CLOCK_SECOND*10, "ook");
  
  process_start(&testapp, NULL);
  process_start(&a6at, NULL);
  printf("PROCESS EXITING\n");
  PROCESS_EXIT();
  PROCESS_WAIT_EVENT_UNTIL(ev == a6at_gprs_init);
  printf("Here is at proc\n");

  gprs_set_context(&gprs_context, PDPTYPE, APN);
  printf("Did set context\n");
  PROCESS_WAIT_UNTIL(gprs_context.active);
  printf("CONTEXT is ACTIVE\n");
  ATWAIT(CLOCK_SECOND*10, "ook");
  gprs_connection("TCP", IPADDR, PORTNO);
  
 again:
#if 0
  process_start(&gprs_init_pt, NULL);
  PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_EXITED && data == &gprs_init_pt);  
  printf("GPRS INITIED\n");
  ATWAIT(CLOCK_SECOND*30, "LOOK");
#endif
  printf("GPIO: 0x%x\n\n", sc16is_gpio_get());
  ATSTR("AT+CREG?\r"); ATWAIT(CLOCK_SECOND*2, "OK");
  ATSTR("AT+CIPSTATUS?\r"); ATWAIT(CLOCK_SECOND*2, "OK");
  ATWAIT(CLOCK_SECOND*5, "KOKO");

 moreconnect:
  sprintf(str, "AT+CIPSTART= \"TCP\", %s, %d\r", IPADDR, PORTNO);
  ATSTR(str); ATWAIT(CLOCK_SECOND*120, "OK", "CME ERROR");
  if (res == NULL || (0 == strcmp((char *) res, "CME ERROR"))) {
    /* Seen +CME ERROR:53 */
    /* After CME ERROR:53, seen CIPSTATUS stuck at IP START */
    printf("CIPSTART failed\n");
    ATSTR("AT+CREG?\r"); ATWAIT(CLOCK_SECOND*2, "OK");    
    ATSTR("AT+CIPCLOSE\r");  ATWAIT(CLOCK_SECOND*5, "OK");
    goto again;
  }
  
  {
    static int i;
    static unsigned char buf[32];

    err = 0;
    for (i = 0; i < sizeof(strings)/sizeof(char *); i++) {
      ATSTR("AT+CREG?\r");ATWAIT(CLOCK_SECOND*5, "OK");
      ATSTR("AT+CIPSTATUS?\r");  ATWAIT(CLOCK_SECOND*5, "OK");
      //ATSTR("AT+CGACT=1,1\r"); ATWAIT(CLOCK_SECOND*20, "OK");

      sprintf((char *) buf, "AT+CIPSEND=%d\r", strlen(strings[i]));
      ATSTR((char *) buf); /* sometimes CME ERROR:516 */
      ATWAIT(CLOCK_SECOND*2, ">");
      ATSTR((char *) strings[i]);
      /* sometimes CME ERROR:516 */
      /* Sometimes COMMAND NO RESPONSE -- timeout after c:a 20 sec */
      ATSTR("\r");
      ATWAIT(CLOCK_SECOND*5, "OK");
      if (err || (res == NULL) || (strcmp((char *)res, "OK") != 0)) {
        err = 1;
        printf("SEND ERROR --------\n");
        ATWAIT(CLOCK_SECOND*240, "OK");
        break;
      }
      ATSTR("AT+CREG?\r"); ATWAIT(CLOCK_SECOND*5, "OK");
      ATSTR("AT+CIPSTATUS?\r");  ATWAIT(CLOCK_SECOND*5, "OK");
    }
  }
  ATSTR("AT+CREG?\r"); ATWAIT(CLOCK_SECOND*5, "OK");
  ATSTR("AT+CIPSTATUS?\r");  ATWAIT(CLOCK_SECOND*5, "OK");

  ATSTR("AT+CIPCLOSE\r");  ATWAIT(CLOCK_SECOND*5, "OK"); /* Close connection */
  if (res == NULL) { /* close failed */
    printf("CLOSE FAILED SEND ERROR ----------\n");
    ATWAIT(CLOCK_SECOND*240, "OK");
    goto again;
  }
    
  //ATSTR("AT+CIPSHUT\r");  ATWAIT(CLOCK_SECOND*5, "OK"); /* Shutdown PDP context */

  ATSTR("AT+CREG?\r");ATWAIT(CLOCK_SECOND*5, "OK");
  ATSTR("AT+CIPSTATUS?\r");  ATWAIT(CLOCK_SECOND*5, "OK");
  ATSTR("AT+CCID\r");  ATWAIT(CLOCK_SECOND*5, "OK");

  ATSTR("AT+CEER\r");  ATWAIT(CLOCK_SECOND*5, "OK");
  ATSTR("AT+CSQ\r"); ATWAIT(CLOCK_SECOND*5, "OK");

  ATSTR("AT+CIFSR\r");  ATWAIT(CLOCK_SECOND*5, "OK");

  ATSTR("AT+CGDCONT=?\r"); ATWAIT(CLOCK_SECOND*5, "OK");
  ATSTR("AT+CGDCONT?\r"); ATWAIT(CLOCK_SECOND*5, "OK");  
  ATWAIT(CLOCK_SECOND*20, "KOKO"); /* delay */
  goto moreconnect;

  PROCESS_END();
}
#endif
