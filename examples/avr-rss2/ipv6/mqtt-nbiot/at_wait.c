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

#include "at_wait.h"

static uint8_t at_numwait;
static uint8_t at_numwait_permanent;

void
atwait_init() {
  at_match_event = process_alloc_event();
  at_numwait = 0;
  at_numwait_permanent = 0;
  atwait_matching = 0;

}

/*
 * Simple match -- matched input string (like "OK"). Post 
 * an event to signal the match.
 */
PT_THREAD(wait_simple_callback(struct pt *pt, struct at_wait *at, int c)) {
  PT_BEGIN(pt);
  process_post(at->process, at_match_event, at);
  PT_END(pt);
}

/*
 * FSM protothread to read characters until end of line
 * Characters are stored into at_line buffer, and atpos is the
 * number of chars in the buffer
 */ 
PT_THREAD(wait_readline_pt(struct pt *pt, struct at_wait *at, int c)) {
  static int atpos;

  PT_BEGIN(pt);

  atpos = 0; 
  while (c != '\n' && c != '\r') {
    if (atpos < sizeof(at_line)-1)
      at_line[atpos++] = (uint8_t) c;
    PT_YIELD(pt);
  }
  /* done -- mark end of string */
  at_line[atpos] = '\0';
  PT_END(pt);
}

PT_THREAD(wait_readlines_pt(struct pt *pt, struct at_wait *at, int c)) {
  static int atpos;

  PT_BEGIN(pt);
  atpos = 0;
  while (1) {
    if (c == '\n' || c == '\r') {
      /* Do nothing???*/;
    }
    else {
      at_line[atpos++] = (char ) c;
      if (atpos == sizeof(at_line)-1)
        break;
      if (c == 0)
        break;
    }
    PT_YIELD(pt);
  }
  /* done -- mark end of string */
  at_line[atpos] = '\0';
  PT_END(pt);
}

/*
 * Callback function to read the remainder of the line until
 * EOL, and the post an event to signal the match
 */
PT_THREAD(wait_readline_callback(struct pt *pt, struct at_wait *at, int c)) {
  char ret;
  ret = wait_readline_pt(pt, at, c);
  if (ret == PT_ENDED)
    process_post(at->process, at_match_event, at);
  return ret;
}

/*
 * Callback function to read all lines until full buffer is full
 * and the post an event to signal the match
 */
PT_THREAD(wait_readlines_callback(struct pt *pt, struct at_wait *at, int c)) {
  char ret;
  ret = wait_readlines_pt(pt, at, c);
  if (ret == PT_ENDED)
    process_post(at->process, at_match_event, at);
  return ret;
}

/*
 * Match functions. Return a negative number if current data string does
 * not match search pattern. If there is a match, return number of bytes that 
 * were consumed in last call.
 */

/* Match byte: check one byte at a time */
int at_match_byte(struct at_wait *at, int c) {
  if (c == at->str[at->pos]) {
    at->pos += 1;
    if (at->str[at->pos] == '\0') {
      return 1;
    }
  }
  else { 
    at->pos = 0;
  }
  return -1;
}

void
start_at(struct at_wait *at) {
  if (at_numwait >= MAXWAIT) {
    printf("Error starting %s: too many at_waits (%d)\n", at->str, at_numwait);
    {
      int i;
      for (i = 0; i < at_numwait; i++)
        printf("%d: %s\n", i, at_waitlist2[i]->str);
    }
    return;
  }
  at_waitlist2[at_numwait] = at;
  at_numwait++;
  at->pos = 0;
  at->process = PROCESS_CURRENT();
}

void
restart_at(struct at_wait *at) {
  at->pos = 0;
}

void
atwait_start_permanent(struct at_wait *at, ...) {
  va_list valist;
  struct at_wait *at1;
  
  va_start(valist, at);
  at1 = at;
  while (at1) {
    printf(" '%s' ", at1->str);
    start_at(at1);
    at1 = va_arg(valist, struct at_wait *);
  }
  at_numwait_permanent = at_numwait;
}

static void
vstart_atlist(struct at_wait *at, va_list valist) {
  
  at = va_arg(valist, struct at_wait *);
  while (at) {
    printf(" '%s' ", at->str);
    start_at(at);
    at = va_arg(valist, struct at_wait *);
  }
}

/*
 * Main loop for input matching. Take one byte at a time, and call each
 * active matching state machine. 
 * If one state mechine returns a non-negative value, it is a complete match. 
 * Then call the corresponding callback protothread with the remaining
 * input data.
 */

/* Flag for locking AT commands. Set to non-zero when trigger has been
 * matched, and callback function is active and consuming input. Don't 
 * issue new AT commands while in this state; it would garble the
 * the input.
 */

PT_THREAD(wait_fsm_pt(struct pt *pt, int c)) {
  static uint8_t i;
  static struct pt subpt;
  static struct at_wait *at;
  int match;
    
  PT_BEGIN(pt);

  while (1) {
    atwait_matching = 0;
    for (i = 0; i < at_numwait; i++) {
      at = at_waitlist2[i];
      match = at->match(at, c);
      if (match >= 0) {
        /* An async (permanent) event? Then mark that it is active right now
         * so that other events can be postponed 
         */
        if (i < at_numwait_permanent)
          atwait_matching = 1;
        if (match > 0)
          PT_YIELD(pt); /* Consume char and wait for next */
        /* Run callback protothread -- loop until it has finished (PT_ENDED or PT_EXITED) */
        PT_INIT(&subpt);
        while (at->callback(&subpt, at, c) == PT_YIELDED) {
          PT_YIELD(pt);
        }
        atwait_matching = 0;
      }
    }
    /* Yield and wait for next input */
    PT_YIELD(pt);
  }
  PT_END(pt);
}

PT_THREAD(atwait(int lineno, struct pt *pt, process_event_t ev,
                 process_data_t data, struct at_wait **atp, int seconds, ...)) {
//PT_THREAD(atwait(struct pt *pt, process_event_t ev,
//                 process_data_t data, struct at_wait **atp, int seconds, ...)) {
  va_list valist;
  struct at_wait *at;
  static struct etimer et;
  
  PT_BEGIN(pt);

  printf("---start_atlist:%d: ", lineno);
  va_start(valist, seconds);
  at = va_arg(valist, struct at_wait *);
  while (at) {
    printf(" '%s' ", at->str);
    start_at(at);
    at = va_arg(valist, struct at_wait *);
  }
  printf("\n"); 

  
  etimer_set(&et, (seconds)*CLOCK_SECOND);
  while (1) {
    PT_YIELD(pt);
    if(etimer_expired(&et)) {
      printf("---pt_timeout:%d\n", lineno);
      *atp = NULL;
      break; 
    } 
    else if(ev == at_match_event) {
      etimer_stop(&et); 
      *atp = (struct at_wait *) data;
      printf("\n---pt_got:%d '%s'\n", lineno, (*atp)->str); 
      break;
    } 
  } 

  /* remove dynamic events, and only keep permanent */
  at_numwait = at_numwait_permanent;

  PT_END(pt);
}
