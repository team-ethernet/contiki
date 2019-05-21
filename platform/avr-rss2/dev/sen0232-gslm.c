/*
   Copyright (c) 2019,
   Anton Bothin,
   Erik Flink,
   Nelly Friman,
   Jacob Klasmark,
   Valter Lundegårdh,
   Isak Olsson,
   Andreas Sjödin,
   Carina Wickström.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:
   1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
   3. The names of the authors may not be used to endorse or promote
   products derived from this software without specific prior
   written permission.

   THIS SOFTWARE IS PROVIDED BY THE AUTHORS ``AS IS'' AND ANY EXPRESS
   OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY
   DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
   GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "contiki.h"
#include "lib/sensors.h"
#include "rss2.h"
#include "pwr.h"
#include "adc.h"
#include "sen0232-gslm.h"

#define MULTIPLY_VALUE 50

const struct sensors_sensor sen0232_gslm;

void
sen0232_gslm_init(void)
{
  pwr_1_init();
}
void
sen0232_gslm_disable(void)
{
  pwr_1_disable();
}
/*
 * Turn power on 175 ticks before reading
 * clock_wait(time) waits time*8 ms
 */
int
value(int type)
{
  pwr_1_on();
  clock_wait(175);
  int dBVal = ((int)(adc_read_a1() * MULTIPLY_VALUE));
  pwr_1_off();
  return (int)dBVal;
}
static int
status(int type)
{
  return 0;
}
static int
configure(int type, int c)
{
  return 0;
}
SENSORS_SENSOR(sen0232_gslm, "sen0232_gslm", value, configure, status);