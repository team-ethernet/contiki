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
  int i;
  for (i = 0; i < GPRS_MAX_CONNECTION; i++) { 
    GPRS_CONNECTION_RELEASE(&gprs_connections[i]);
  }
  gprs_set_context(&gprs_context, PDPTYPE, APN);
  memset(&gprs_statistics, 0, sizeof(gprs_statistics));
  process_start(&uart_reader, NULL);
  process_start(&a6at, NULL);
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
PT_THREAD(wait_csonmi_callback(struct pt *pt, struct at_wait *at, int c));
static
PT_THREAD(wait_ciprcv_callback(struct pt *pt, struct at_wait *at, int c));
static
PT_THREAD(wait_tcpclosed_callback(struct pt *pt, struct at_wait *at, int c));

static
struct at_wait wait_csonmi = {"+CSONMI:", wait_csonmi_callback, at_match_byte};
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
  gprsconn = find_gprs_connection(0);
  if (gprsconn) {
    gprsconn->input_callback(gprsconn, gprsconn->callback_arg, rcvdata, nbytes);
  }
  PT_END(pt);
}

/*
 * +CSONMI: sock,len,<data>
 * hex: +CSONMI: 0,8,20020000 * 
 * bin: +CSONMI: 0,4,**** * 
 */
static
PT_THREAD(wait_csonmi_callback(struct pt *pt, struct at_wait *at, int c)) {
  static uint8_t rcvdata[GPRS_MAX_RECV_LEN];
  static uint16_t rcvlen;
  static short int nbytes;
  static uint8_t csock;
  static int rcvpos;

  struct gprs_connection *gprsconn;
  
  PT_BEGIN(pt);
  atwait_record_on();
  while (c != ',') {
    PT_YIELD(pt);
  }
  PT_YIELD(pt);
  while (c != ',') {
    PT_YIELD(pt);
  }
  atwait_record_off();  
  PT_YIELD(pt);
  printf("CSONMI bug '%s'\n", at_line);
  int nm;
  if ((nm = sscanf(at_line, " %hhd,%hd,", &csock, &nbytes)) != 2)
    printf("received not mahc %d", nm);
  printf("received %d byteson sock %d\n", nbytes, csock);

#ifdef SIM7020_RECVHEX
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
#else
  /* Data is binary */
  rcvlen = nbytes;
  if (rcvlen > GPRS_MAX_RECV_LEN)
    rcvlen = GPRS_MAX_RECV_LEN;

  rcvpos = 0;
  /* Get one byte at a time, but don't store more than rcvlen */
  while (nbytes-- > 0) {
    if (rcvpos < rcvlen) {
      rcvdata[rcvpos++] = (uint8_t) c;
    }
    if (nbytes > 0)
      PT_YIELD(pt);
  }
#endif /* SIM7020_RECVHEX */

  restart_at(&wait_csonmi); /* restart */
  gprsconn = find_gprs_connection(csock);
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
  struct gprs_connection *gprsconn;
  static struct pt rlpt;
  uint8_t csock, cerr;
  
  PT_BEGIN(pt);
  atwait_record_on();
  PT_INIT(&rlpt);
  while (wait_readline_pt(&rlpt, at, c) < PT_EXITED) {
    PT_YIELD(pt);
  }
  atwait_record_off();
  restart_at(&wait_tcpclosed); /* restart */
  if (2 != sscanf(at_line, "%hhd,%hhd", &csock, &cerr)) {
    printf("tcpclosed: csock fail\n");
    PT_EXIT(pt);
  }
  gprsconn = find_gprs_connection(csock);
  if (gprsconn && gprsconn->socket) {
    call_event(gprsconn->socket, TCP_SOCKET_CLOSED);
  }
  PT_END(pt);
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
#define PT_ATBUF(buf, len) pt_sendbuf(buf, len)

static void
enqueue_event(process_event_t ev, void *data); 

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
  PT_ATWAIT2(5, &wait_csq);
  atwait_record_off();

  if (at == NULL)
    gprs_statistics.at_timeouts += 1;
  else {
    char *csq;
    csq = strstr(at_line, "+CSQ:") + strlen("+CSQ:");    
    status.rssi = atoi((char *) csq /*foundbuf*/);
    printf("Got CSQ: %d\n", status.rssi);
    PT_ATWAIT2(5, &wait_ok);
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

  PT_BEGIN(pt);

  PT_ATSTR("AT+CRESET\r");
  PT_ATWAIT2(10, &wait_ok);
  PT_ATSTR("AT+CSORCVFLAG?\r");
  PT_ATWAIT2(10, &wait_ok);
  /* Receive data in hex */
#ifdef SIM7020_RECVHEX
  /* Receive data as hex string */
  PT_ATSTR("AT+CSORCVFLAG=0\r");
#else  
  /* Receive binary data */
  PT_ATSTR("AT+CSORCVFLAG=1\r"); 
#endif /* SIM7020_RECVHEX */
  PT_ATWAIT2(10, &wait_ok);
  if (at == NULL) {
    status.state = GPRS_STATE_NONE;
    gprs_statistics.at_timeouts += 1;
  }
  else 
    status.state = GPRS_STATE_IDLE;
  PT_DELAY(10);
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
  static char str[80];

  PT_BEGIN(pt);

#ifdef NB_IOT_TELIA	
  PT_ATSTR("AT+COPS=1,2,\"24001\"\r");
  PT_ATWAIT2(10, &wait_ok);
  if (at == NULL) goto timeout;
    
  PT_ATSTR("AT+CFUN=0\r");
  PT_ATWAIT2(10, &wait_ok);
  if (at == NULL) goto timeout;
  
  sprintf(str, "*MCGDEFCONT=\"%s\",%s\r", PDPTYPE, APN); /* Set PDP (Packet Data Protocol) context */
  PT_ATSTR(str);
  PT_ATWAIT2(10, &wait_ok);
  if (at == NULL) goto timeout;
  
  PT_ATSTR("AT+CFUN=1\r");
  PT_ATWAIT2(10, &wait_ok);

  PT_ATSPAWN(read_csq);
#endif /* NB_IOT_TELIA */

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
    printf("No module version found\n");
  }
  else {
    char *ver;
    ver = strstr(at_line, "SIM7020E ");
    if (ver != NULL) {
      status.module = GPRS_MODULE_SIM7020E;
    }
    else {
      printf("No module version in '%s'\n", at_line);
    }
  }
  PT_ATSTR("AT+GSV\r");
  PT_ATWAIT2(10, &wait_ok);
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
  PT_ATSTR("AT+CGCONTRDP\r");
  atwait_record_on();
  PT_ATWAIT2(10, &wait_ok);
  atwait_record_off();
  if (at == NULL) {
    gprs_statistics.at_timeouts += 1;
    printf("No IP address found\n");
  }
  else {
    /* Look for something like
     *     +CGCONTRDP: 1,5,"lpwa.telia.iot","10.81.168.254.255.255.255.0" 
     */
    int n;
    int b1, b2, b3, b4;
    n = sscanf(at_line, "%*[^:]: %*d,%*d,\"%*[-.a-zA-Z0-9_]\",\"%d.%d.%d.%d", &b1, &b2, &b3, &b4);
    if (n == 4) {
#if NETSTACK_CONF_WITH_IPV6
      snprintf(status.ipaddr, sizeof(status.ipaddr), "::ffff:%d.%d.%d.%d", b1, b2, b3, b4);
#else
      snprintf(status.ipaddr, sizeof(status.ipaddr), "%d.%d.%d.%d", b1, b2, b3, b4);
#endif 
    }
    else
      printf("Could not get ipaddr (%d)\n", n);
  }
  PT_END(pt);
}
/*---------------------------------------------------------------------------*/
/*
 * connect
 *
 * Protothread to set up TCP connection with AT commands
 */
#define apa
#ifdef apa
static
PT_THREAD(sim7020_connect(struct pt *pt, struct gprs_connection * gprsconn)) {
  static struct at_wait *at;
  static int minor_tries;
  char str[80];
  
  PT_BEGIN(pt);
  minor_tries = 0;
  while (minor_tries++ < 10) {
	
    printf("Here is connection %s %s:%d\n", gprsconn->proto, gprsconn->ipaddr, uip_htons(gprsconn->port));
    PT_ATSTR("AT+CSOC=1,1,1\r");
    atwait_record_on();
    PT_ATWAIT2(10, &wait_ok, &wait_error);
    atwait_record_off();
    uint8_t sockid;
    if (at != &wait_ok || 1 != sscanf(at_line, "%*[^:]: %hhd", &sockid)) {
      break;
    }
    printf("Got socket no %u\n", (unsigned) sockid);
    gprsconn->connectionid = sockid;
    
    sprintf(str, "AT+CSOCON=%d,%d,\"%s\"\r", gprsconn->connectionid, uip_ntohs(gprsconn->port), gprsconn->ipaddr);
    PT_ATSTR(str);
    PT_ATWAIT2(60, &wait_ok, &wait_error);        
    if (at == &wait_ok) {
      gprs_statistics.connections += 1;
      call_event(gprsconn->socket, TCP_SOCKET_CONNECTED);
      break;
    }
    /* If we ended up here, we failed to set up connection */
    if (at == NULL) {
      /* Timeout */
      gprs_statistics.connfailed += 1;
      status.state = GPRS_STATE_NONE;
      call_event(gprsconn->socket, TCP_SOCKET_TIMEDOUT);
      break;
    }        
    else if (at == &wait_error) {
      /* COMMAND NO RESPONSE! timeout. Sometimes it take longer though and have seen COMMAND NON RESPONSE! followed by CONNECT OK */ 
      /* Seen +CME ERROR:53 */
      printf("CIPSTART failed\n");
      PT_ATSTR("AT+CREG?\r");
      PT_ATWAIT2(2, &wait_ok);
      gprs_statistics.at_errors += 1;
      sprintf(str, "AT+CSOCLO=%d\r", gprsconn->connectionid);
      PT_ATSTR(str);
      PT_ATWAIT2(15, &wait_ok);//ATWAIT2(5, &wait_ok, &wait_cmeerror);

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
    break;
  }
  PT_END(pt);
}
#endif /* apa*/

/*---------------------------------------------------------------------------*/
/*
 * send
 *
 * Protothread to send data with AT commands
 */
static
PT_THREAD(sim7020_send(struct pt *pt, struct gprs_connection * gprsconn)) {
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
#if 0
  PT_ATSTR("ATE0\r\n");
  PT_ATWAIT2(5, &wait_ok);
  if (at == NULL)
    goto timeout;
#endif
  while (remain > 0) {
    len = (remain <= GPRS_MAX_SEND_LEN ? remain : GPRS_MAX_SEND_LEN);
    printf("Send %d bytes @0x%x\n", len, (unsigned) &ptr[gprsconn->output_data_len-remain]);
    sprintf((char *) buf, "AT+CSODSEND=%d,%d\r", gprsconn->connectionid, len);
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
    PT_ATWAIT2(30, &wait_ok, &wait_error, &wait_dataaccept);
    if (at == NULL || at == &wait_error) {
      gprs_statistics.at_timeouts += 1;
      PT_EXIT(pt);
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
#if 0
  PT_ATSTR("ATE1\r\n");
  PT_ATWAIT2(5, &wait_ok);
#endif
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
PT_THREAD(sim7020_close(struct pt *pt, struct gprs_connection * gprsconn)) {
  static struct at_wait *at;
  static struct tcp_socket_gprs *socket;

  PT_BEGIN(pt);
#ifdef GPRS_DEBUG
  printf("A6AT GPRS Close\n");
#endif /* GPRS_DEBUG */

        
  PT_ATSTR("AT+CIPCLOSE\r");
  PT_ATWAIT2(15, &wait_ok, &wait_cmeerror);
  if (at == &wait_ok) {
    call_event(socket, TCP_SOCKET_CLOSED); 
  }
  else {
    printf("Call socket_closed\n");
    gprs_statistics.at_timeouts += 1;
    call_event(socket, TCP_SOCKET_CLOSED);
    printf("Called socket_closed\n");
  }
  PT_END(pt);
} 


PROCESS_THREAD(a6at, ev, data) {
  //unsigned char *res;
  struct at_wait *at;

  PROCESS_BEGIN();
  leds_init();
  event_init();
  event_queue_init();

 again:
  ATSPAWN(init_module);

  ATSPAWN(apn_register);
  if (status.state < GPRS_STATE_REGISTERED) {
    printf ("Not ready...\n");
    ATSPAWN(read_csq);
    goto again;
  }

  ATSPAWN(get_moduleinfo);
  ATSPAWN(get_ipconfig);

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

    gprsconn = (struct gprs_connection *) gprs_event->data;
    if (gprsconn == NULL) {
      printf("NULL gprsconnection for %s\n", eventstr(gprs_event->ev));
      goto nextcommand;
    }
    socket = gprsconn->socket;
    if (socket == NULL) {
      printf("NULL socket for %s\n", eventstr(gprs_event->ev));
      goto nextcommand;
    }
    else if (gprs_event->ev == a6at_gprs_send) {
      ATSPAWN(sim7020_send, gprsconn);
    } 
    else if (gprs_event->ev == a6at_gprs_connection) {
      ATSPAWN(sim7020_connect, gprsconn);
    }
    else if (gprs_event->ev == a6at_gprs_close) {
      ATSPAWN(sim7020_close, gprsconn);
    } 
#ifdef GPRS_DEBUG
    else {
      printf("A6AT GPRS Unknown event %d\n", gprs_event->ev);
    }
#endif /* GPRS_DEBUG */

    {
      static struct pt _pt;
      PT_INIT(&_pt);
      while (read_csq(&_pt) < PT_EXITED) {
        PROCESS_PAUSE();
      }
    }
    continue;

    /* Timeout */
    ATSTR("AT+CIPCLOSE\r");
    ATWAIT2(5, &wait_ok, &wait_cmeerror);
    ATSTR("AT+CIPSHUT\r");
    ATWAIT2(5, &wait_ok, &wait_cmeerror);
    call_event(gprsconn->socket, TCP_SOCKET_TIMEDOUT);
  }
  PROCESS_END();
}
