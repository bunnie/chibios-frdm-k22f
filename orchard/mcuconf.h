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

/* MCG mode constants */

#define MCG_MODE_FEI                   0U
#define MCG_MODE_FBI                   1U
#define MCG_MODE_BLPI                  2U
#define MCG_MODE_FEE                   3U
#define MCG_MODE_FBE                   4U
#define MCG_MODE_BLPE                  5U
#define MCG_MODE_PBE                   6U
#define MCG_MODE_PEE                   7U

/* Select the MCU clocking mode below by enabling the appropriate block. */

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


#define CLOCK_SETUP 4
#if (CLOCK_SETUP == 4)  // FSL clock setup
#define CPU_XTAL_CLK_HZ                8000000u            /* Value of the external crystal or oscillator clock frequency in Hz */
#define CPU_XTAL32k_CLK_HZ             32768u              /* Value of the external 32k crystal or oscillator clock frequency in Hz */
#define CPU_INT_SLOW_CLK_HZ            32768u              /* Value of the slow internal oscillator clock frequency in Hz  */
#define CPU_INT_FAST_CLK_HZ            4000000u            /* Value of the fast internal oscillator clock frequency in Hz  */
#define CPU_INT_IRC_CLK_HZ             48000000u           /* Value of the 48M internal oscillator clock frequency in Hz  */

/* RTC oscillator setting */
/* RTC_CR: SC2P=0,SC4P=0,SC8P=0,SC16P=0,CLKO=1,OSCE=1,WPS=0,UM=0,SUP=0,WPE=0,SWR=0 */
#define SYSTEM_RTC_CR_VALUE            0x0300U             /* RTC_CR */

/* Low power mode enable */
/* SMC_PMPROT: AHSRUN=1,AVLP=1,ALLS=1,AVLLS=1 */
#define SYSTEM_SMC_PMPROT_VALUE        0xAAU               /* SMC_PMPROT */
 #define KINETIS_MCG_MODE            KINETIS_MCG_MODE_PEE
 #define KINETIS_XTAL_FREQUENCY      8000000UL
  #define KINETIS_SYSCLK_FREQUENCY     120000000UL 

  #define DEFAULT_SYSTEM_CLOCK         120000000u          /* Default System clock value */
  #define MCG_MODE                     MCG_MODE_PEE /* Clock generator mode */
  /* MCG_C1: CLKS=0,FRDIV=3,IREFS=0,IRCLKEN=1,IREFSTEN=0 */
  #define SYSTEM_MCG_C1_VALUE          0x1AU               /* MCG_C1 */
  /* MCG_C2: LOCRE0=0,FCFTRIM=0,RANGE=2,HGO=0,EREFS=1,LP=0,IRCS=0 */
#define SYSTEM_MCG_C2_VALUE          0x24U               /* MCG_C2 */  //(b6)
  /* MCG_C4: DMX32=0,DRST_DRS=0,FCTRIM=0,SCFTRIM=0 */
#define SYSTEM_MCG_C4_VALUE          0x00U               /* MCG_C4 */  //(11)
  /* MCG_SC: ATME=0,ATMS=0,ATMF=0,FLTPRSRV=0,FCRDIV=0,LOCS0=0 */
  #define SYSTEM_MCG_SC_VALUE          0x00U               /* MCG_SC */
/* MCG_C5: PLLCLKEN0=0,PLLSTEN0=0,PRDIV0=1 */
#define SYSTEM_MCG_C5_VALUE          0x01U               /* MCG_C5 */  //(01)
/* MCG_C6: LOLIE0=0,PLLS=1,CME0=0,VDIV0=6 */
#define SYSTEM_MCG_C6_VALUE          0x46U               /* MCG_C6 */  //(06)
/* MCG_C7: OSCSEL=0 */
#define SYSTEM_MCG_C7_VALUE          0x00U               /* MCG_C7 */  //(00)
/* OSC_CR: ERCLKEN=1,EREFSTEN=0,SC2P=0,SC4P=0,SC8P=0,SC16P=0 */
  #define SYSTEM_OSC_CR_VALUE          0x80U               /* OSC_CR */
/* SMC_PMCTRL: RUNM=3,STOPA=0,STOPM=0 */
  #define SYSTEM_SMC_PMCTRL_VALUE      0x60U               /* SMC_PMCTRL */
/* SIM_CLKDIV1: OUTDIV1=0,OUTDIV2=1,OUTDIV3=1,OUTDIV4=4 */
  #define SYSTEM_SIM_CLKDIV1_VALUE     0x01140000U         /* SIM_CLKDIV1 */
/* SIM_CLKDIV2: USBDIV=4,USBFRAC=1 */
  #define SYSTEM_SIM_CLKDIV2_VALUE     0x09U               /* SIM_CLKDIV2 */
/* SIM_SOPT1: USBREGEN=0,USBSSTBY=0,USBVSTBY=0,OSC32KSEL=2,OSC32KOUT=0,RAMSIZE=0 */
  #define SYSTEM_SIM_SOPT1_VALUE       0x00080000U         /* SIM_SOPT1 */
/* SIM_SOPT2: LPUARTSRC=0,USBSRC=0,PLLFLLSEL=1,TRACECLKSEL=0,FBSL=0,CLKOUTSEL=0,RTCCLKOUTSEL=0 */
  #define SYSTEM_SIM_SOPT2_VALUE       0x00010000U         /* SIM_SOPT2 */
#else
/* PEE mode - external 8 MHz crystal with PLL for 48 MHz core/system clock. */
 #define KINETIS_MCG_MODE            KINETIS_MCG_MODE_PEE
 #define KINETIS_XTAL_FREQUENCY      8000000UL
 #define KINETIS_SYSCLK_FREQUENCY    48000000UL // note that 48MHz is hard-coded in hal_lld.c
 #define DEFAULT_SYSTEM_CLOCK KINETIS_SYSCLK_FREQUENCY
#endif

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
