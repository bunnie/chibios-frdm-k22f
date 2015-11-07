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

#define K22x_MCUCONF

#define KINETIS_XTAL_FREQUENCY      8000000UL

/* Enable clock initialization by HAL */
#define KINETIS_NO_INIT             FALSE

/*
 * HAL driver system settings.
 */

/* Select the MCU clocking mode below by enabling the appropriate block. */

/* PEE mode - external 8 MHz crystal with PLL for 48 MHz core/system clock. */
#if 1
#define KINETIS_MCG_MODE            KINETIS_MCG_MODE_PEE
#define KINETIS_XTAL_FREQUENCY      8000000UL
#define KINETIS_SYSCLK_FREQUENCY    48000000UL // note that 48MHz is hard-coded in hal_lld.c
#endif

/* FEI mode - 48 MHz with internal 32.768 kHz oscillator */
#if 0
#define KINETIS_MCG_MODE            KINETIS_MCG_MODE_FEI
#define KINETIS_MCG_FLL_DMX32       1           /* Fine-tune for 32.768 kHz */
#define KINETIS_MCG_FLL_DRS         1           /* 1464x FLL factor */
#define KINETIS_SYSCLK_FREQUENCY    47972352UL  /* 32.768 kHz * 1464 (~48 MHz) */
#endif /* 0 */

/* FEE mode - 24 MHz with external 32.768 kHz crystal */
#if 0
#define KINETIS_MCG_MODE            KINETIS_MCG_MODE_FEE
#define KINETIS_MCG_FLL_DMX32       1           /* Fine-tune for 32.768 kHz */
#define KINETIS_MCG_FLL_DRS         0           /* 732x FLL factor */
#define KINETIS_MCG_FLL_OUTDIV1     1           /* Divide 48 MHz FLL by 1 => 24 MHz */
#define KINETIS_MCG_FLL_OUTDIV4     2           /* Divide OUTDIV1 output by 2 => 12 MHz */
#define KINETIS_SYSCLK_FREQUENCY    23986176UL  /* 32.768 kHz*732 (~24 MHz) */
#define KINETIS_UART0_CLOCK_FREQ    (32768 * 732) /* FLL output */
#define KINETIS_UART0_CLOCK_SRC     1           /* Select FLL clock */
#define KINETIS_BUSCLK_FREQUENCY    (KINETIS_SYSCLK_FREQUENCY / KINETIS_MCG_FLL_OUTDIV4)
#endif /* 0 */

/* FEE mode - 48 MHz */
#if 0
#define KINETIS_MCG_MODE            KINETIS_MCG_MODE_FEE
#define KINETIS_MCG_FLL_DMX32       1           /* Fine-tune for 32.768 kHz */
#define KINETIS_MCG_FLL_DRS         1           /* 1464x FLL factor */
#define KINETIS_MCG_FLL_OUTDIV1     1           /* Divide 48 MHz FLL by 1 => 48 MHz */
#define KINETIS_MCG_FLL_OUTDIV4     2           /* Divide OUTDIV1 output by 2 => 24 MHz */
#define KINETIS_SYSCLK_FREQUENCY    47972352UL  /* 32.768 kHz * 1464 (~48 MHz) */
#endif /* 0 */

/*
 * SERIAL driver system settings.
 */
#define KINETIS_SERIAL_USE_UART1              TRUE

/*
 * EXTI driver system settings.
 */
#define KINETIS_EXTI_NUM_CHANNELS               8
#define KINETIS_EXT_PORTA_IRQ_PRIORITY          3
#define KINETIS_EXT_PORTB_IRQ_PRIORITY          3
#define KINETIS_EXT_PORTC_IRQ_PRIORITY          3
#define KINETIS_EXT_PORTD_IRQ_PRIORITY          3
#define KINETIS_EXT_PORTE_IRQ_PRIORITY          3

/*
 * SPI system settings.
 */
#define KINETIS_SPI_USE_SPI0                    TRUE
#define KINETIS_SPI_USE_SPI1                    TRUE
#define KINETIS_SPI_SPI0_IRQ_PRIORITY           3
#define KINETIS_SPI_SPI1_IRQ_PRIORITY           3

/*
 * I2C system settings.
 */
#define KINETIS_I2C_USE_I2C0                    TRUE
#define KINETIS_I2C_USE_I2C1                    TRUE
#define KINETIS_I2C_I2C0_PRIORITY               2
#define KINETIS_I2C_I2C1_PRIORITY               2

/*
 * ADC driver system settings.
 */
#define KINETIS_ADC_USE_ADC0                  FALSE

/*
 * I2S driver system settings.
 */
#define KINETIS_I2S_USE_I2S1                  TRUE
#define KINETIS_SPI_TX_PRIORITY               1
#define KINETIS_SPI_RX_PRIORITY               1

/*
 * Processor specific widths of each port.
 * Smaller numbers can be used if only lower pins in a port are being used to
 * generate interrupts. Can be set to 0 if a port is unused.
 */


/* KL22FN512  */
#define KINETIS_EXT_PORTA_WIDTH                 32
#define KINETIS_EXT_PORTB_WIDTH                 32
#define KINETIS_EXT_PORTC_WIDTH                 32
#define KINETIS_EXT_PORTD_WIDTH                 32
#define KINETIS_EXT_PORTE_WIDTH                 32

#ifndef KINETIS_EXT_PORTA_WIDTH
#define KINETIS_EXT_PORTA_WIDTH                 0
#endif

#ifndef KINETIS_EXT_PORTB_WIDTH
#define KINETIS_EXT_PORTB_WIDTH                 0
#endif

#ifndef KINETIS_EXT_PORTC_WIDTH
#define KINETIS_EXT_PORTC_WIDTH                 0
#endif

#ifndef KINETIS_EXT_PORTD_WIDTH
#define KINETIS_EXT_PORTD_WIDTH                 0
#endif

#ifndef KINETIS_EXT_PORTE_WIDTH
#define KINETIS_EXT_PORTE_WIDTH                 0
#endif
