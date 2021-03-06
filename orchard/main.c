/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

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
#include "hal.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"

#include "shell.h"
#include "chprintf.h"

#include "orchard.h"
#include "orchard-shell.h"
#include "orchard-events.h"
#include "orchard-i2s.h"
#include "orchard-sd.h"

#include <string.h>

struct evt_table orchard_events;

static const I2CConfig i2c_config = {
  100000
};

void i2s_handler(I2SDriver *i2sp, size_t offset, size_t n);
int32_t rx_samples[NUM_RX_SAMPLES];
int32_t rx_savebuf[NUM_RX_SAMPLES];
uint32_t rx_handler_count = 0;

static const SPIConfig spi_config = {
  NULL,
  GPIOC,
  4, 
  KINETIS_SPI_TAR_SYSCLK_DIV_8(8)
};

static const MMCConfig mmc_config = { 
  &SPID1,
  &spi_config,
  &spi_config
};


MMCDriver MMCD1;

static I2SConfig i2s_config = {
  NULL,
  rx_samples,
  NUM_RX_SAMPLES,
  i2s_handler,
  { // sai_tx_state
    {48000u, 12288000 /*mclk freq*/, 32, kSaiStereo},  // mclk must be at least 2x bitclock
    NULL,
    0,
    0,
    NULL,
    NULL,
    kSaiModeAsync,
    0,
    4,
    kSaiMaster,
    kSaiBusI2SType,
    //    NULL,  // semaphore_t
    FALSE,
    0,
  },
  { // sai_rx_state
    {48000u, 12288000 /*mclk freq*/, 32, kSaiStereo},
    (uint8_t *) rx_samples,  // regardless fo sample size, driver thinks of this as char stream...for now.
    NUM_RX_SAMPLES,
    0,
    NULL,
    NULL,
    kSaiModeAsync,
    0,
    4,
    kSaiMaster,
    kSaiBusI2SType,
    //    NULL,  // semaphore_t
    FALSE,
    0,
  },
  { // tx_userconfig
    kSaiMclkSourceSysclk,
    0,
    kSaiModeAsync,
    kSaiBusI2SType,
    kSaiMaster,
    kSaiBclkSourceMclkDiv,
    4,
    0,
  },
  { // rx_userconfig
    kSaiMclkSourceSysclk,
    0,
    kSaiModeAsync,
    kSaiBusI2SType,
    kSaiMaster,
    kSaiBclkSourceMclkDiv,
    4,
    0,
  }
};

extern event_source_t i2s_full_event;

void i2s_handler(I2SDriver *i2sp, size_t offset, size_t n) {
  (void) i2sp;
  (void) offset;
  (void) n;
  
  // for now just copy it into the save buffer over and over again.
  // in the future, this would then kick off a SPI MMC data write event to save out the blocks
  rx_handler_count++;
  memcpy( rx_savebuf, rx_samples, NUM_RX_SAMPLES * sizeof(uint32_t) );
  // kick out an event to write data to disk
  //  chSysLockFromISR();
  chEvtBroadcastI(&i2s_full_event);
  // chSysUnlockFromISR();

}

static void shell_termination_handler(eventid_t id) {
  static int i = 1;
  (void)id;

  chprintf(stream, "\r\nRespawning shell (shell #%d, event %d)\r\n", ++i, id);
  orchardShellRestart();
}


extern int print_hex(BaseSequentialStream *chp,
                     const void *block, int count, uint32_t start);

static void print_mcu_info(void) {
  uint32_t sdid = SIM->SDID;
  const char *famid[] = {
    "KL0%d (low-end)",
    "KL1%d (basic)",
    "KL2%d (USB)",
    "KL3%d (Segment LCD)",
    "KL4%d (USB and Segment LCD)",
  };
  const uint8_t ram[] = {
    0,
    1,
    2,
    4,
    8,
    16,
    32,
    64,
  };

  const uint8_t pins[] = {
    16,
    24,
    32,
    36,
    48,
    64,
    80,
  };

  if (((sdid >> 20) & 15) != 1) {
    chprintf(stream, "Device is not Kinetis KL-series\r\n");
    return;
  }

  chprintf(stream, famid[(sdid >> 28) & 15], (sdid >> 24) & 15);
  chprintf(stream, " with %d kB of ram detected"
                   " (Rev: %04x  Die: %04x  Pins: %d).\r\n",
                   ram[(sdid >> 16) & 15],
                   (sdid >> 12) & 15,
                   (sdid >> 7) & 31,
                   pins[(sdid >> 0) & 15]);
}

/*
 * Application entry point.
 */
int main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  i2sStart(&I2SD1, (const I2SConfig *) &i2s_config);
  // spiStart(&SPID1, &spi_config);  // start handled by mmcStart
  spiObjectInit(&SPID1);
  mmcObjectInit(&MMCD1);
  mmcStart(&MMCD1, &mmc_config); // driver, config

  evtTableInit(orchard_events, 32);

  orchardShellInit();

  orchardEventsStart();

  evtTableHook(orchard_events, shell_terminated, shell_termination_handler);

  chprintf(stream, "\r\n\r\nOrchard shell.  Based on build %s\r\n", gitversion);
  print_mcu_info();

  chprintf(stream, "\r\nSystem core clock: %d Hz\r\n", mk22f12_get_system_clock());

  orchardShellRestart();

  while (TRUE)
    chEvtDispatch(evtHandlers(orchard_events), chEvtWaitOne(ALL_EVENTS));
}

