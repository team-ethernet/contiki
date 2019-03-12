/*
 * Copyright (c) 2007, Swedish Institute of Computer Science.
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
 */

/**
 * \file
 *         AVR-specific rtimer code
 *         Defaults to Timer3 for those ATMEGAs that have it.
 *         If Timer3 not present Timer1 will be used.
 * \author
 *         Fredrik Osterlind <fros@sics.se>
 *         Joakim Eriksson <joakime@sics.se>
 *
 *         Make simplier: Constant rate for timer2 use TIMER2_OVF
 *         Synchronizing strategy for RTC and Xtal timer.
 *         Robert Olsson <roolss@kth.se>
 */

/* OBS: 8 seconds maximum time! */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

#include "sys/energest.h"
#include "sys/clock.h"
#include "sys/rtimer.h"
#include "rtimer-arch.h"

#if defined(__AVR_ATmega1284P__)
#define ETIMSK TIMSK3
#define ETIFR TIFR3
#define TICIE3 ICIE3

//Has no 'C', so we just set it to B. The code doesn't really use C so this
//is safe to do but lets it compile. Probably should enable the warning if
//it is ever used on other platforms.
//#warning no OCIE3C in timer3 architecture, hopefully it won't be needed!

#define OCIE3C	OCIE3B
#define OCF3C	OCF3B
#endif

#if defined(__AVR_ATmega1281__) || defined(__AVR_AT90USB1287__) || defined(__AVR_ATmega128RFA1__) || defined(__AVR_ATmega128RFR2__) || defined(__AVR_ATmega256RFR2__)
#define ETIMSK TIMSK3
#define ETIFR TIFR3
#define TICIE3 ICIE3
#endif

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega644__)
#define TIMSK TIMSK1
#define TICIE1 ICIE1
#define TIFR TIFR1
#endif

/* Track flow through rtimer interrupts*/
#if DEBUGFLOWSIZE&&0
extern uint8_t debugflowsize,debugflow[DEBUGFLOWSIZE];
#define DEBUGFLOW(c) if (debugflowsize<(DEBUGFLOWSIZE-1)) debugflow[debugflowsize++]=c
#else
#define DEBUGFLOW(c)
#endif

/*---------------------------------------------------------------------------*/
#if defined(TCNT3) && RTIMER_ARCH_PRESCALER
ISR (TIMER3_COMPA_vect) {
  DEBUGFLOW('/');
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  /* Disable rtimer interrupts */
  ETIMSK &= ~((1 << OCIE3A) | (1 << OCIE3B) | (1 << TOIE3) |
      (1 << TICIE3) | (1 << OCIE3C));

#if RTIMER_CONF_NESTED_INTERRUPTS
  /* Enable nested interrupts. Allows radio interrupt during rtimer interrupt. */
  /* All interrupts are enabled including recursive rtimer, so use with caution */
  sei();
#endif

  /* Call rtimer callback */
  rtimer_run_next();

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
  DEBUGFLOW('\\');
}

#elif RTIMER_ARCH_PRESCALER
#warning "No Timer3 in rtimer-arch.c - using Timer1 instead"
ISR (TIMER1_COMPA_vect) {
  DEBUGFLOW('/');
  TIMSK &= ~((1<<TICIE1)|(1<<OCIE1A)|(1<<OCIE1B)|(1<<TOIE1));

  rtimer_run_next();
  DEBUGFLOW('\\');
}

#endif
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
#if RTIMER_ARCH_PRESCALER
  /* Disable interrupts (store old state) */
  uint8_t sreg;
  sreg = SREG;
  cli ();

#ifdef TCNT3
  /* Disable all timer functions */
  ETIMSK &= ~((1 << OCIE3A) | (1 << OCIE3B) | (1 << TOIE3) |
      (1 << TICIE3) | (1 << OCIE3C));
  /* Write 1s to clear existing timer function flags */
  ETIFR |= (1 << ICF3) | (1 << OCF3A) | (1 << OCF3B) | (1 << TOV3) |
  (1 << OCF3C); 

  /* Default timer behaviour */
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR3C = 0;

  /* Reset counter */
  TCNT3 = 0;

#if RTIMER_ARCH_PRESCALER==1024
  TCCR3B |= 5;
#elif RTIMER_ARCH_PRESCALER==256
  TCCR3B |= 4;
#elif RTIMER_ARCH_PRESCALER==64
  TCCR3B |= 3;
#elif RTIMER_ARCH_PRESCALER==8
  TCCR3B |= 2;
#elif RTIMER_ARCH_PRESCALER==1
  TCCR3B |= 1;
#else
#error Timer3 PRESCALER factor not supported.
#endif

#elif RTIMER_ARCH_PRESCALER
  /* Leave timer1 alone if PRESCALER set to zero */
  /* Obviously you can not then use rtimers */

  TIMSK &= ~((1<<TICIE1)|(1<<OCIE1A)|(1<<OCIE1B)|(1<<TOIE1));
  TIFR |= (1 << ICF1) | (1 << OCF1A) | (1 << OCF1B) | (1 << TOV1);

  /* Default timer behaviour */
  TCCR1A = 0;
  TCCR1B = 0;

  /* Reset counter */
  TCNT1 = 0;

  /* Start clock */
#if RTIMER_ARCH_PRESCALER==1024
  TCCR1B |= 5;
#elif RTIMER_ARCH_PRESCALER==256
  TCCR1B |= 4;
#elif RTIMER_ARCH_PRESCALER==64
  TCCR1B |= 3;
#elif RTIMER_ARCH_PRESCALER==8
  TCCR1B |= 2;
#elif RTIMER_ARCH_PRESCALER==1
  TCCR1B |= 1;
#else
#error Timer1 PRESCALER factor not supported.
#endif

#endif /* TCNT3 */

  /* Restore interrupt state */
  SREG = sreg;
#endif /* RTIMER_ARCH_PRESCALER */
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
#if RTIMER_ARCH_PRESCALER
  /* Disable interrupts (store old state) */
  uint8_t sreg;
#if RTIMER_ARCH_CORRECTION 
#define ABS(x) (x < 0 ? -x : x)
  if(t > ABS(RTIMER_ARCH_CORRECTION)) t += RTIMER_ARCH_CORRECTION;
#endif
  
  sreg = SREG;
  cli ();
  DEBUGFLOW(':');
#ifdef TCNT3
  /* Set compare register */
  OCR3A = t;
  /* Write 1s to clear all timer function flags */
  ETIFR |= (1 << ICF3) | (1 << OCF3A) | (1 << OCF3B) | (1 << TOV3) |
  (1 << OCF3C);
  /* Enable interrupt on OCR3A match */
  ETIMSK |= (1 << OCIE3A);

#elif RTIMER_ARCH_PRESCALER
  /* Set compare register */
  OCR1A = t;
  TIFR |= (1 << ICF1) | (1 << OCF1A) | (1 << OCF1B) | (1 << TOV1);
  TIMSK |= (1 << OCIE1A);

#endif

  /* Restore interrupt state */
  SREG = sreg;
#endif /* RTIMER_ARCH_PRESCALER */
}

#ifdef RDC_CONF_MCU_SLEEP
extern long sleepseconds;
uint16_t sleep_ticks;
int16_t  t2_tick;
volatile uint8_t clock_tick_pending;
rtimer_clock_t t1_tmp, t2, t3, t4;
int16_t t1;
/*---------------------------------------------------------------------------*/
void
rtimer_arch_sleep(rtimer_clock_t howlong)
{
  uint8_t sreg;
/* 
 * Deep Sleep for howlong rtimer ticks. This will stop all timers except
 * for TIMER2 which can be clocked using an external crystal.
 * Unfortunately this is an 8 bit timer; a lower prescaler gives higher
 * precision but smaller maximum sleep time.
 *
 *
 * t1      - time in rtime ticks to sync with RTC
 * t2      - time in rtime ticks running on RTC (rtimer stopped)
 * t2_tick - number of RTC ticks for t2
 * t3      - time in rtime ticks starting after t2
 * t4      - timer buzz at wake-up
 *
 *  howlong = t1 + t2 + t3
 *
 */

#include <avr/sleep.h>
#include <dev/watchdog.h>
  
  sreg = SREG;
  cli();
  clock_tick_pending = 0;
  SREG = sreg;

  t1_tmp = RTIMER_NOW();
  while(clock_tick_pending == 0) {
    watchdog_periodic();
  };

  //if( RTIMER_CLOCK_LT(RTIMER_NOW(), t1);

  t1 = RTIMER_CLOCK_DIFF(RTIMER_NOW(), t1_tmp);

  if(t1 < 0) { 
    t1 = 0;
    goto done;
  }

  if(howlong <= t1)
    goto done;
  
  TCCR3B = 0;

  t2_tick = howlong / (RTIMER_SECOND/CLOCK_SECOND);

  if(t2_tick > 0)
    t2_tick--;
  
  t2 = t2_tick *  (RTIMER_SECOND/CLOCK_SECOND);
  t4 = t2_tick;

  while(t2_tick > 0)  {
    watchdog_periodic();
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
    sleep_mode();
    /* Only clock timer interrupts is considered */
    sreg = SREG;
    cli();
    if(clock_tick_pending) {
      t2_tick--;
      clock_tick_pending = 0;
    }
    SREG = sreg;
  }

  t3 = howlong - t2 - t1 -t4;
  if(howlong > t3) {
    clock_delay_usec(t3 * 1000000/RTIMER_SECOND);
    //clock_delay_usec(t3 * 16);
  }

done:;
  
  
/* Adjust rtimer ticks if rtimer is enabled. TIMER3 is preferred, else TIMER1 */
#if RTIMER_ARCH_PRESCALER
#ifdef TCNT3
  TCNT3 += (howlong - t1 );
#else
  TCNT1 += (howlong - t1 );
#endif
#endif
    TCCR3B = 4;

    sleep_ticks += t4;

    while(sleep_ticks >= CLOCK_SECOND) {
      sleep_ticks -= CLOCK_SECOND;
      sleepseconds++;
    }
}
#endif /* RDC_CONF_MCU_SLEEP */

