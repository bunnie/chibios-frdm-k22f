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

#include <string.h>

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

#define ENDURANCE_TEST 1

#define ENDURANCE_OFFSET   2048 // stick it at offset 2048

#define DATA_OFFSET_START  (1024 * 512) // starting block for data writes
#if ENDURANCE_TEST
#define DATA_LEN_BYTES     (32 * 1024 * 1024) // amount of data to save in bytes, about 300 seconds of audio
#else
#define DATA_LEN_BYTES     (2 * 1024 * 1024) // amount of data to save in bytes, about 10 seconds of audio
#endif

static uint32_t sd_offset = DATA_OFFSET_START;

static uint32_t write_iters = 0;
static uint32_t endurance_val = 0;
static uint8_t endure_sector[512];

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

  chprintf( chp, "rx ints: %d, handler calls: %d, sd_offset: %d, this run iters: %d total iters: %d\n\r", 
	    rx_int_count, rx_handler_count, sd_offset, write_iters, endurance_val);
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
  } else {
#if ENDURANCE_TEST
    // if we're done recording, loop again
    sd_offset = DATA_OFFSET_START;
    write_iters++;
    endurance_val++;
    // update the endurance value sector
    memcpy(endure_sector, &endurance_val, sizeof(uint32_t));
    MMCD1.vmt->write(&MMCD1, ENDURANCE_OFFSET / MMCSD_BLOCK_SIZE, 
		     (uint8_t *) endure_sector, 1);
#endif
    // if not doing endurance testing, do nothing
  }
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

  // read in the endurance sector
  if( !HAL_SUCCESS == MMCD1.vmt->read(&MMCD1, ENDURANCE_OFFSET / MMCSD_BLOCK_SIZE, (uint8_t *) endure_sector, 1) ) {
    chprintf(chp, "mmc_read failed\n\r");
  }
  memcpy(&endurance_val, endure_sector, sizeof(uint32_t));

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

