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

#include "orchard-events.h"
#include "orchard-shell.h"
#include "orchard-i2s.h"
#include "orchard-sd.h"

event_source_t i2s_full_event;
event_source_t i2s_reset_event;
static thread_t *i2sthr = NULL;

#define DATA_OFFSET_START  (1024 * 512) // starting block for data writes
#define DATA_LEN_BYTES     (2 * 1024 * 1024) // amount of data to save in bytes, about 10 seconds of audio

static uint32_t sd_offset = DATA_OFFSET_START;

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
    chprintf(chp, "Data is MSB-aligned, 18-bit wide 2's comp with zero pad on the LSB to 32 bits\r\n");
    return;
  }

  for( i = 0; i < 16; i++ ) {
    if( i % 8 == 0 )
      chprintf( chp, "\n\r%3x: ", i );
    chprintf( chp, "%8ld ", rx_savebuf[i] >> 14 );
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

  chprintf( chp, "rx ints: %d, handler calls: %d, sd_offset: %d\n\r", 
	    rx_int_count, rx_handler_count, sd_offset);
}

orchard_command("stat", cmd_i2sstat);

static void i2s_full_handler(eventid_t id) {
  (void)id;
  
  if( sd_offset < (DATA_OFFSET_START + DATA_LEN_BYTES) ) {
    if( !HAL_SUCCESS == 
	MMCD1.vmt->write(&MMCD1, sd_offset / MMCSD_BLOCK_SIZE, 
			 (uint8_t *) rx_savebuf, NUM_RX_SAMPLES * sizeof(int32_t) / MMCSD_BLOCK_SIZE) ) {
      chprintf(stream, "mmc_write failed\n\r");
      return;
    }
    sd_offset += NUM_RX_SAMPLES * sizeof(int32_t);
  }
  // if we're done recording, return doing nothing
}

static void i2s_reset_handler(eventid_t id) {
  (void)id;

  sd_offset = DATA_OFFSET_START;
}


static THD_WORKING_AREA(waOrchardI2SThread, 0x900); // more stack
static THD_FUNCTION(orchard_i2s_thread, arg) {

  (void)arg;
  struct evt_table orchard_i2s_events;

  chRegSetThreadName("Orchard I2S");

  evtTableInit(orchard_i2s_events, 32);
  evtTableHook(orchard_i2s_events, i2s_full_event, i2s_full_handler);
  evtTableHook(orchard_i2s_events, i2s_reset_event, i2s_reset_handler);

  while (!chThdShouldTerminateX())
    chEvtDispatch(evtHandlers(orchard_i2s_events), chEvtWaitOne(ALL_EVENTS));

  evtTableUnhook(orchard_i2s_events, i2s_reset_event, i2s_reset_handler);
  evtTableUnhook(orchard_i2s_events, i2s_full_event, i2s_full_handler);

  chSysLock();
  // chEvtBroadcastI(&orchard_i2s_terminated);
  chThdExitS(MSG_OK);
}

void cmd_i2sthr(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void) argv;
  
  if (argc > 0) {
    chprintf(chp, "Usage: i2sthr\r\n");
    return;
  }

  if( i2sthr ) {
    chprintf(chp, "Thread already started!\n\r");
    return;
  }

  if( !HAL_SUCCESS == MMCD1.vmt->connect(&MMCD1) )
    chprintf(chp, "mmcConnect() failed\n\r");

  // init our event
  chEvtObjectInit(&i2s_full_event);
  chEvtObjectInit(&i2s_reset_event);

  i2sthr = chThdCreateStatic(waOrchardI2SThread,
			     sizeof(waOrchardI2SThread),
			     (LOWPRIO + 2),
			     orchard_i2s_thread,
			     NULL);

}

orchard_command("i2sthr", cmd_i2sthr);


void cmd_i2sevt(BaseSequentialStream *chp, int argc, char *argv[]) {
  (void) argv;

  if (argc > 0) {
    chprintf(chp, "Usage: i2sevt\r\n");
    return;
  }

  chEvtBroadcast(&i2s_reset_event);
}

orchard_command("i2sevt", cmd_i2sevt);

