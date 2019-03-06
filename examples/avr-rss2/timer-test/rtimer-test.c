/**
 * \file
 *         rtimer test focus on RDC_CONF_MCU_SLEEP
 *
 * \author
 *         Robert Olsson <roolss@kth.se>
 * \heavily based on:
 *         Zach Shelby <zach@sensinode.com> (Original)
 *         George Oikonomou - <oikonomou@users.sourceforge.net> (rtimer code)
 *
 */

/* Version for 16 bit timers Robert Olsson */
/* mcu_sleep test added */

#include "contiki.h"
#include "sys/clock.h"
#include "sys/rtimer.h"
#include "dev/leds.h"
#include <dev/watchdog.h>
#include <stdio.h>
#include <avr/wdt.h>

#define RTIMER_RES 1000000/RTIMER_SECOND 
static struct etimer et;

static struct rtimer rt;
volatile rtimer_clock_t rt_now, rt_for, rt_mcu_sleep;
static clock_time_t ct;

static uint16_t i;

extern rtimer_clock_t t1, t2, t3, t4;
/*---------------------------------------------------------------------------*/
PROCESS(clock_test_process, "Clock test process");
AUTOSTART_PROCESSES(&clock_test_process);
/*---------------------------------------------------------------------------*/

#if RDC_CONF_MCU_SLEEP

void
rt_callback_mcu_sleep(struct rtimer *t, void *ptr)
{
  /* Sleep wait */
  rtimer_arch_sleep(rt_mcu_sleep);
  rt_now = RTIMER_NOW();
  ct = clock_time();
  leds_on(LEDS_RED);
}
#endif

#ifdef CONTIKI_TARGET_AVR_RSS2
static void
stop_avr_wdt(void)
{
    cli();
    watchdog_stop();
    wdt_disable();
    /* Clear WDRF in MCUSR*/
    MCUSR &= ~(1<<WDRF);
    /* Write logical one to WDCE and WDE */
    /* Keep old prescaler setting to prevent unintentional time-out */
    WDTCSR |= (1<<WDCE) | (1<<WDE);
    /* Turn off WDT */
    WDTCSR = 0x00;
    sei();
}
#endif

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(clock_test_process, ev, data)
{
  PROCESS_BEGIN();
#ifdef CONTIKI_TARGET_AVR_RSS2
  stop_avr_wdt();
#endif
  leds_init();

  etimer_set(&et,  CLOCK_SECOND);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  etimer_reset(&et);

  printf("RTIMER_SECOND = (%lu rtimer ticks):\n", RTIMER_SECOND);
  printf("CLOCK_SECOND = (%i):\n", CLOCK_SECOND);
  printf("sizeof rtimer_clock_t = %d\n", sizeof(rtimer_clock_t));
  printf("sizeof clock_time_t = %d\n", sizeof(clock_time_t));

 #if RDC_CONF_MCU_SLEEP
  printf("Rtimer Test MCU sleep, 1 sec (%lu rtimer ticks):\n", RTIMER_SECOND);

  for(rt_mcu_sleep=300; rt_mcu_sleep <= 30000; rt_mcu_sleep+=300) {
    i = 4;

    while(i < 5) {
      /* One sec extra for etimer to finish befor rtimer */
      etimer_set(&et, CLOCK_SECOND*3); 
      ct = clock_time();
      rt_now = RTIMER_NOW();
      rt_for = rt_now + RTIMER_SECOND*1;
      printf("now=%-u clock=%-lu scheduled_for=%u\n", rt_now, ct, rt_for + rt_mcu_sleep);
      if(rtimer_set(&rt, rt_for, 1, (rtimer_callback_t) rt_callback_mcu_sleep, NULL) !=
	 RTIMER_OK) {
	printf("Error setting\n");
      }
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
      printf("run=%-u clock=%-lu ", rt_now, ct);
      printf("diff=%-d sleep_mcu=%-u ", rt_now-(rt_for+rt_mcu_sleep), rt_mcu_sleep);
      printf(" %-u %-u %-u  %-u\n", t1, t2, t3, t4);
      i++;
    }
  }
#endif  

  while(1) {
    etimer_set(&et, CLOCK_SECOND);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    etimer_reset(&et);
    leds_toggle(LEDS_YELLOW);
    leds_toggle(LEDS_RED);
  }
  PROCESS_END();
}
