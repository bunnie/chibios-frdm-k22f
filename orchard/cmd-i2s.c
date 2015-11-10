/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "shell.h"
#include "chprintf.h"

#include "orchard-shell.h"
#include "orchard-i2s.h"

void cmd_i2s(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: i2s\r\n");
    return;
  }
  
  //  chprintf(chp, "Starting I2S for testing...wish me luck!\r\n" );
  i2sStartRx(&I2SD1);
  //chprintf(chp, "started.\n\r");

}

orchard_command("i2s", cmd_i2s);

void cmd_i2sdump(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void) argv;

  int i;

  if (argc > 0) {
    chprintf(chp, "Usage: d\r\n");
    return;
  }

  for( i = 0; i < 16; i++ ) {
    if( i % 8 == 0 )
      chprintf( chp, "\n\r%3x: ", i );
    chprintf( chp, "%08x ", rx_savebuf[i] );
  }
  chprintf( chp, "\n\r" );
}

orchard_command("d", cmd_i2sdump);

void cmd_i2sstat(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void) argv;

  if (argc > 0) {
    chprintf(chp, "Usage: stat\r\n");
    return;
  }

  chprintf( chp, "rx ints: %d, handler calls: %d\n\r", rx_int_count, rx_handler_count );
}

orchard_command("stat", cmd_i2sstat);
