/*
    ChibiOS - Copyright (C) 2014-2015 Fabio Utzig

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

/**
 * @file    templates/hal_lld.c
 * @brief   HAL Driver subsystem low level driver source template.
 *
 * @addtogroup HAL
 * @{
 */

#include "hal.h"
#include "mcuconf.h"
#include "mk22f12_extensions.h"
#include "mcg_lld.h"

extern uint32_t SystemCoreClock;
extern uint32_t g_xtal0ClkFreq;

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

#ifdef __CC_ARM
__attribute__ ((section(".ARM.__at_0x400")))
#else
__attribute__ ((section(".cfmconfig")))
#endif
const uint8_t _cfm[0x10] = {
  0xFF,  /* NV_BACKKEY3: KEY=0xFF */
  0xFF,  /* NV_BACKKEY2: KEY=0xFF */
  0xFF,  /* NV_BACKKEY1: KEY=0xFF */
  0xFF,  /* NV_BACKKEY0: KEY=0xFF */
  0xFF,  /* NV_BACKKEY7: KEY=0xFF */
  0xFF,  /* NV_BACKKEY6: KEY=0xFF */
  0xFF,  /* NV_BACKKEY5: KEY=0xFF */
  0xFF,  /* NV_BACKKEY4: KEY=0xFF */
  0xFF,  /* NV_FPROT3: PROT=0xFF */
  0xFF,  /* NV_FPROT2: PROT=0xFF */
  0xFF,  /* NV_FPROT1: PROT=0xFF */
  0xFF,  /* NV_FPROT0: PROT=0xFF */
  0x7E,  /* NV_FSEC: KEYEN=1,MEEN=3,FSLACC=3,SEC=2 */
  0xFF,  /* NV_FOPT: ??=1,??=1,FAST_INIT=1,LPBOOT1=1,RESET_PIN_CFG=1,
                      NMI_DIS=1,EZPORT_DIS=1,LPBOOT0=1 */
  0xFF,
  0xFF
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level HAL driver initialization.
 * @todo    Use a macro to define the system clock frequency.
 *
 * @notapi
 */
void hal_lld_init(void) {

}

#if 0
/* Function to initialize OSC0 base on board configuration. */
static void BOARD_InitOsc0(void)  // mostly here for reference
{
  // OSC0 configuration.
    osc_user_config_t osc0Config =
      {
        .freq                = OSC0_XTAL_FREQ,
        .hgo                 = MCG_HGO0,
        .range               = MCG_RANGE0,
        .erefs               = MCG_EREFS0,
        .enableCapacitor2p   = OSC0_SC2P_ENABLE_CONFIG,
        .enableCapacitor4p   = OSC0_SC4P_ENABLE_CONFIG,
        .enableCapacitor8p   = OSC0_SC8P_ENABLE_CONFIG,
        .enableCapacitor16p  = OSC0_SC16P_ENABLE_CONFIG,
      };

    CLOCK_SYS_OscInit(0U, &osc0Config);
}
#endif

/**
 * @brief   MK22f12 clock initialization.
 * @note    All the involved constants come from the file @p board.h.
 * @note    This function is meant to be invoked early during the system
 *          initialization, it is usually invoked from the file
 *          @p board.c.
 * @todo    This function needs to be more generic.
 *
 * @special
 */
void mk22f12_clock_init(void) {
#if !KINETIS_NO_INIT

#if KINETIS_MCG_MODE == KINETIS_MCG_MODE_PEE
  uint32_t ratio, frdiv;
  uint32_t ratios[] = { 32, 64, 128, 256, 512, 1024, 1280, 1536 };
  int ratio_quantity = sizeof(ratios) / sizeof(ratios[0]);
  int i;
#endif /* KINETIS_MCG_MODE == KINETIS_MCG_MODE_PEE */

  /* Disable the watchdog */
  WDOG->UNLOCK = 0xC520;
  WDOG->UNLOCK = 0xD928;
  WDOG->STCTRLH &= ~WDOG_STCTRLH_WDOGEN;

  SIM->SCGC5 |= SIM_SCGC5_PORTA |
                SIM_SCGC5_PORTB |
                SIM_SCGC5_PORTC |
                SIM_SCGC5_PORTD |
                SIM_SCGC5_PORTE;

#ifndef CLOCK_SETUP
#if KINETIS_MCG_MODE == KINETIS_MCG_MODE_FEI

  /* Configure FEI mode */
  MCG->C4 = MCG_C4_DRST_DRS(KINETIS_MCG_FLL_DRS) |
            (KINETIS_MCG_FLL_DMX32 ? MCG_C4_DMX32 : 0);

#endif /* KINETIS_MCG_MODE == KINETIS_MCG_MODE_FEI */

#if KINETIS_MCG_MODE == KINETIS_MCG_MODE_PEE

  /* EXTAL0 and XTAL0 */
  PORTA->PCR[18] = 0;
  PORTA->PCR[19] = 0;
  
  /*
   * Start in FEI mode
   */

  /* Disable capacitors for crystal */
  OSC->CR = 0;

  /* TODO: need to add more flexible calculation, specially regarding
   *       divisors which may not be available depending on the XTAL
   *       frequency, which would required other registers to be modified.
   */
  /* Enable OSC, low power mode */
  MCG->C2 = MCG_C2_LOCRE0 | MCG_C2_EREFS0;
  if (KINETIS_XTAL_FREQUENCY > 8000000)
    MCG->C2 |= MCG_C2_RANGE0(2);
  else
    MCG->C2 |= MCG_C2_RANGE0(1);

  frdiv = 7;
  ratio = KINETIS_XTAL_FREQUENCY / 31250;
  for (i = 0; i < ratio_quantity; ++i) {
    if (ratio == ratios[i]) {
      frdiv = i;
      break;
    }
  }

  /* Switch to crystal as clock source, FLL input of 31.25 KHz */
  MCG->C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(frdiv);

  /* Wait for crystal oscillator to begin */
  while (!(MCG->S & MCG_S_OSCINIT0));

  /* Wait for the FLL to use the oscillator */
  while (MCG->S & MCG_S_IREFST);

  /* Wait for the MCGOUTCLK to use the oscillator */
  while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST(2));

  /*
   * Now in FBE mode
   */

  /* Config PLL input for 2 MHz */
  MCG->C5 = MCG_C5_PRDIV0((KINETIS_XTAL_FREQUENCY / 2000000) - 1);

  /* Config PLL for 96 MHz output */
  MCG->C6 = MCG_C6_PLLS | MCG_C6_VDIV0(0);

  /* Wait for PLL to start using crystal as its input */
  while (!(MCG->S & MCG_S_PLLST));

  /* Wait for PLL to lock */
  while (!(MCG->S & MCG_S_LOCK0));

  /*
   * Now in PBE mode
   */

  /* Switch to PLL as clock source */
  MCG->C1 = MCG_C1_CLKS(0);

  /* Wait for PLL clock to be used */
  while ((MCG->S & MCG_S_CLKST_MASK) != MCG_S_CLKST_PLL);

  /*
   * Now in PEE mode
   */
#endif /* KINETIS_MCG_MODE == KINETIS_MCG_MODE_PEE */





#else /* ifndef CLOCK_SETUP */   ///////////////////////////////////

  /* EXTAL0 and XTAL0 */

  PORTA->PCR[18] = 0;
  PORTA->PCR[19] = 0;

  /*
   * Start in FEI mode
   */

  /* Disable capacitors for crystal */
  OSC->CR = 0;

  /* TODO: need to add more flexible calculation, specially regarding
   *       divisors which may not be available depending on the XTAL
   *       frequency, which would required other registers to be modified.
   */
  /* Enable OSC, low power mode */
  MCG->C2 = MCG_C2_LOCRE0 | MCG_C2_EREFS0;
  if (KINETIS_XTAL_FREQUENCY > 8000000)
    MCG->C2 |= MCG_C2_RANGE0(2);
  else
    MCG->C2 |= MCG_C2_RANGE0(1);

  frdiv = 7;
  ratio = KINETIS_XTAL_FREQUENCY / 31250;
  for (i = 0; i < ratio_quantity; ++i) {
    if (ratio == ratios[i]) {
      frdiv = i;
      break;
    }
  }

  /* Switch to crystal as clock source, FLL input of 31.25 KHz */
  MCG->C1 = MCG_C1_CLKS(2) | MCG_C1_FRDIV(frdiv);

  /* Wait for crystal oscillator to begin */
  while (!(MCG->S & MCG_S_OSCINIT0));

  if((RCM->SRS0 & RCM_SRS0_WAKEUP_MASK) != 0x00U)
  {
    if((PMC->REGSC & PMC_REGSC_ACKISO_MASK) != 0x00U)
    {
       PMC->REGSC |= PMC_REGSC_ACKISO_MASK; /* Release hold with ACKISO:  Only has an effect if recovering from VLLSx.*/
    }
  } else {
#ifdef SYSTEM_RTC_CR_VALUE
    SIM_SCGC6 |= SIM_SCGC6_RTC_MASK;
    if ((RTC_CR & RTC_CR_OSCE_MASK) == 0x00U) { /* Only if the OSCILLATOR is not already enabled */
      RTC_CR = (uint32_t)((RTC_CR & (uint32_t)~(uint32_t)(RTC_CR_SC2P_MASK | RTC_CR_SC4P_MASK | RTC_CR_SC8P_MASK | RTC_CR_SC16P_MASK)) | (uint32_t)SYSTEM_RTC_CR_VALUE);
      RTC_CR |= (uint32_t)RTC_CR_OSCE_MASK;
      RTC_CR &= (uint32_t)~(uint32_t)RTC_CR_CLKO_MASK;
    }
#endif
  }

  /* Power mode protection initialization */
#ifdef SYSTEM_SMC_PMPROT_VALUE
  SMC->PMPROT = SYSTEM_SMC_PMPROT_VALUE;
#endif

  /* High speed run mode enable */
#if (((SYSTEM_SMC_PMCTRL_VALUE) & SMC_PMCTRL_RUNM_MASK) == (0x03U << SMC_PMCTRL_RUNM_SHIFT))
  SMC->PMCTRL = (uint8_t)((SYSTEM_SMC_PMCTRL_VALUE) & (SMC_PMCTRL_RUNM_MASK)); /* Enable HSRUN mode */
  while(SMC->PMSTAT != 0x80U) {        /* Wait until the system is in HSRUN mode */
  }
#endif
  /* System clock initialization */
  /* Internal reference clock trim initialization */
#if defined(SLOW_TRIM_ADDRESS)
  if ( *((uint8_t*)SLOW_TRIM_ADDRESS) != 0xFFU) {                              /* Skip if non-volatile flash memory is erased */
    MCG->C3 = *((uint8_t*)SLOW_TRIM_ADDRESS);
  #endif /* defined(SLOW_TRIM_ADDRESS) */
  #if defined(SLOW_FINE_TRIM_ADDRESS)
    MCG->C4 = (MCG->C4 & ~(MCG_C4_SCFTRIM_MASK)) | ((*((uint8_t*) SLOW_FINE_TRIM_ADDRESS)) & MCG_C4_SCFTRIM_MASK);
  #endif
  #if defined(FAST_TRIM_ADDRESS)
    MCG->C4 = (MCG->C4 & ~(MCG_C4_FCTRIM_MASK)) |((*((uint8_t*) FAST_TRIM_ADDRESS)) & MCG_C4_FCTRIM_MASK);
  #endif
  #if defined(FAST_FINE_TRIM_ADDRESS)
    MCG->C2 = (MCG->C2 & ~(MCG_C2_FCFTRIM_MASK)) | ((*((uint8_t*)FAST_TRIM_ADDRESS)) & MCG_C2_FCFTRIM_MASK);
  #endif /* defined(FAST_FINE_TRIM_ADDRESS) */
#if defined(SLOW_TRIM_ADDRESS)
  }
  #endif /* defined(SLOW_TRIM_ADDRESS) */

  /* Set system prescalers and clock sources */
  SIM->CLKDIV1 = SYSTEM_SIM_CLKDIV1_VALUE; /* Set system prescalers */
  SIM->SOPT1 = ((SIM->SOPT1) & (uint32_t)(~(SIM_SOPT1_OSC32KSEL_MASK))) | ((SYSTEM_SIM_SOPT1_VALUE) & (SIM_SOPT1_OSC32KSEL_MASK)); /* Set 32 kHz clock source (ERCLK32K) */
  SIM->SOPT2 = ((SIM->SOPT2) & (uint32_t)(~(SIM_SOPT2_PLLFLLSEL_MASK))) | ((SYSTEM_SIM_SOPT2_VALUE) & (SIM_SOPT2_PLLFLLSEL_MASK)); /* Selects the high frequency clock for various peripheral clocking options. */
#if ((MCG_MODE == MCG_MODE_FEI) || (MCG_MODE == MCG_MODE_FBI) || (MCG_MODE == MCG_MODE_BLPI))
  /* Set MCG and OSC */
#if  ((((SYSTEM_OSC_CR_VALUE) & OSC_CR_ERCLKEN_MASK) != 0x00U) || ((((SYSTEM_MCG_C5_VALUE) & MCG_C5_PLLCLKEN0_MASK) != 0x00U) && (((SYSTEM_MCG_C7_VALUE) & MCG_C7_OSCSEL_MASK) == 0x00U)))
  /* SIM_SCGC5: PORTA=1 */
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  if (((SYSTEM_MCG_C2_VALUE) & MCG_C2_EREFS_MASK) != 0x00U) {
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  }
#endif
  MCG->SC = SYSTEM_MCG_SC_VALUE;       /* Set SC (fast clock internal reference divider) */
  MCG->C1 = SYSTEM_MCG_C1_VALUE;       /* Set C1 (clock source selection, FLL ext. reference divider, int. reference enable etc.) */
  /* Check that the source of the FLL reference clock is the requested one. */
  if (((SYSTEM_MCG_C1_VALUE) & MCG_C1_IREFS_MASK) != 0x00U) {
    while((MCG->S & MCG_S_IREFST_MASK) == 0x00U) {
    }
  } else {
    while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) {
    }
  }
  MCG->C2 = (MCG->C2 & (uint8_t)(~(MCG_C2_FCFTRIM_MASK))) | (SYSTEM_MCG_C2_VALUE & (uint8_t)(~(MCG_C2_LP_MASK))); /* Set C2 (freq. range, ext. and int. reference selection etc. excluding trim bits; low power bit is set later) */
  MCG->C4 = ((SYSTEM_MCG_C4_VALUE) & (uint8_t)(~(MCG_C4_FCTRIM_MASK | MCG_C4_SCFTRIM_MASK))) | (MCG->C4 & (MCG_C4_FCTRIM_MASK | MCG_C4_SCFTRIM_MASK)); /* Set C4 (FLL output; trim values not changed) */
  OSC->CR = SYSTEM_OSC_CR_VALUE;       /* Set OSC_CR (OSCERCLK enable, oscillator capacitor load) */
  MCG->C7 = SYSTEM_MCG_C7_VALUE;       /* Set C7 (OSC Clock Select) */
  #if (MCG_MODE == MCG_MODE_BLPI)
  /* BLPI specific */
  MCG->C2 |= (MCG_C2_LP_MASK);         /* Disable FLL and PLL in bypass mode */
  #endif

#else /* MCG_MODE */
  /* Set MCG and OSC */
#if  (((SYSTEM_OSC_CR_VALUE) & OSC_CR_ERCLKEN_MASK) != 0x00U) || (((SYSTEM_MCG_C7_VALUE) & MCG_C7_OSCSEL_MASK) == 0x00U)
  /* SIM_SCGC5: PORTA=1 */
  SIM_SCGC5 |= SIM_SCGC5_PORTA_MASK;
  /* PORTA_PCR18: ISF=0,MUX=0 */
  PORTA_PCR18 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  if (((SYSTEM_MCG_C2_VALUE) & MCG_C2_EREFS_MASK) != 0x00U) {
  /* PORTA_PCR19: ISF=0,MUX=0 */
  PORTA_PCR19 &= (uint32_t)~(uint32_t)((PORT_PCR_ISF_MASK | PORT_PCR_MUX(0x07)));
  }
#endif
  MCG->SC = SYSTEM_MCG_SC_VALUE;       /* Set SC (fast clock internal reference divider) */
  MCG->C2 = (MCG->C2 & (uint8_t)(~(MCG_C2_FCFTRIM_MASK))) | (SYSTEM_MCG_C2_VALUE & (uint8_t)(~(MCG_C2_LP_MASK))); /* Set C2 (freq. range, ext. and int. reference selection etc. excluding trim bits; low power bit is set later) */
  OSC->CR = SYSTEM_OSC_CR_VALUE;       /* Set OSC_CR (OSCERCLK enable, oscillator capacitor load) */
  MCG->C7 = SYSTEM_MCG_C7_VALUE;       /* Set C7 (OSC Clock Select) */
  #if (MCG_MODE == MCG_MODE_PEE)
  MCG->C1 = (SYSTEM_MCG_C1_VALUE) | MCG_C1_CLKS(0x02); /* Set C1 (clock source selection, FLL ext. reference divider, int. reference enable etc.) - PBE mode*/
  #else
  MCG->C1 = SYSTEM_MCG_C1_VALUE;       /* Set C1 (clock source selection, FLL ext. reference divider, int. reference enable etc.) */
  #endif
  if ((((SYSTEM_MCG_C2_VALUE) & MCG_C2_EREFS_MASK) != 0x00U) && (((SYSTEM_MCG_C7_VALUE) & MCG_C7_OSCSEL_MASK) == 0x00U)) {
    while((MCG->S & MCG_S_OSCINIT0_MASK) == 0x00U) { /* Check that the oscillator is running */
    }
  }
  /* Check that the source of the FLL reference clock is the requested one. */
  if (((SYSTEM_MCG_C1_VALUE) & MCG_C1_IREFS_MASK) != 0x00U) {
    while((MCG->S & MCG_S_IREFST_MASK) == 0x00U) {
    }
  } else {
    while((MCG->S & MCG_S_IREFST_MASK) != 0x00U) {
    }
  }
  MCG->C4 = ((SYSTEM_MCG_C4_VALUE)  & (uint8_t)(~(MCG_C4_FCTRIM_MASK | MCG_C4_SCFTRIM_MASK))) | (MCG->C4 & (MCG_C4_FCTRIM_MASK | MCG_C4_SCFTRIM_MASK)); /* Set C4 (FLL output; trim values not changed) */
#endif /* MCG_MODE */

  /* Common for all MCG modes */

  /* PLL clock can be used to generate clock for some devices regardless of clock generator (MCGOUTCLK) mode. */
  MCG->C5 = (SYSTEM_MCG_C5_VALUE) & (uint8_t)(~(MCG_C5_PLLCLKEN0_MASK)); /* Set C5 (PLL settings, PLL reference divider etc.) */
  MCG->C6 = (SYSTEM_MCG_C6_VALUE) & (uint8_t)~(MCG_C6_PLLS_MASK); /* Set C6 (PLL select, VCO divider etc.) */
  if ((SYSTEM_MCG_C5_VALUE) & MCG_C5_PLLCLKEN0_MASK) {
    MCG->C5 |= MCG_C5_PLLCLKEN0_MASK;  /* PLL clock enable in mode other than PEE or PBE */
  }
  /* BLPE, PEE and PBE MCG mode specific */

#if (MCG_MODE == MCG_MODE_BLPE)
  MCG->C2 |= (MCG_C2_LP_MASK);         /* Disable FLL and PLL in bypass mode */
#elif ((MCG_MODE == MCG_MODE_PBE) || (MCG_MODE == MCG_MODE_PEE))
  MCG->C6 |= (MCG_C6_PLLS_MASK);       /* Set C6 (PLL select, VCO divider etc.) */
  while((MCG->S & MCG_S_LOCK0_MASK) == 0x00U) { /* Wait until PLL is locked*/
  }
  #if (MCG_MODE == MCG_MODE_PEE)
  MCG->C1 &= (uint8_t)~(MCG_C1_CLKS_MASK);
  #endif
#endif
#if ((MCG_MODE == MCG_MODE_FEI) || (MCG_MODE == MCG_MODE_FEE))
  while((MCG->S & MCG_S_CLKST_MASK) != 0x00U) { /* Wait until output of the FLL is selected */
  }
  /* Use LPTMR to wait for 1ms for FLL clock stabilization */
  SIM_SCGC5 |= SIM_SCGC5_LPTMR_MASK;   /* Alow software control of LPMTR */
  LPTMR0->CMR = LPTMR_CMR_COMPARE(0);  /* Default 1 LPO tick */
  LPTMR0->CSR = (LPTMR_CSR_TCF_MASK | LPTMR_CSR_TPS(0x00));
  LPTMR0->PSR = (LPTMR_PSR_PCS(0x01) | LPTMR_PSR_PBYP_MASK); /* Clock source: LPO, Prescaler bypass enable */
  LPTMR0->CSR = LPTMR_CSR_TEN_MASK;    /* LPMTR enable */
  while((LPTMR0_CSR & LPTMR_CSR_TCF_MASK) == 0u) {
  }
  LPTMR0_CSR = 0x00;                   /* Disable LPTMR */
  SIM_SCGC5 &= (uint32_t)~(uint32_t)SIM_SCGC5_LPTMR_MASK;
#elif ((MCG_MODE == MCG_MODE_FBI) || (MCG_MODE == MCG_MODE_BLPI))
  while((MCG->S & MCG_S_CLKST_MASK) != 0x04U) { /* Wait until internal reference clock is selected as MCG output */
  }
#elif ((MCG_MODE == MCG_MODE_FBE) || (MCG_MODE == MCG_MODE_PBE) || (MCG_MODE == MCG_MODE_BLPE))
  while((MCG->S & MCG_S_CLKST_MASK) != 0x08U) { /* Wait until external reference clock is selected as MCG output */
  }
#elif (MCG_MODE == MCG_MODE_PEE)
  while((MCG->S & MCG_S_CLKST_MASK) != 0x0CU) { /* Wait until output of the PLL is selected */
  }
#endif
#if (((SYSTEM_SMC_PMCTRL_VALUE) & SMC_PMCTRL_RUNM_MASK) == (0x02U << SMC_PMCTRL_RUNM_SHIFT))
  SMC->PMCTRL = (uint8_t)((SYSTEM_SMC_PMCTRL_VALUE) & (SMC_PMCTRL_RUNM_MASK)); /* Enable VLPR mode */
  while(SMC->PMSTAT != 0x04U) {        /* Wait until the system is in VLPR mode */
  }
#endif

#if defined(SYSTEM_SIM_CLKDIV2_VALUE)
  SIM->CLKDIV2 = ((SIM->CLKDIV2) & (uint32_t)(~(SIM_CLKDIV2_USBFRAC_MASK | SIM_CLKDIV2_USBDIV_MASK))) | ((SYSTEM_SIM_CLKDIV2_VALUE) & (SIM_CLKDIV2_USBFRAC_MASK | SIM_CLKDIV2_USBDIV_MASK)); /* Selects the USB clock divider. */
#endif

  /* PLL loss of lock interrupt request initialization */
  if (((SYSTEM_MCG_C6_VALUE) & MCG_C6_LOLIE0_MASK) != 0U) {
    NVIC_EnableIRQ(MCG_IRQn);          /* Enable PLL loss of lock interrupt request */
  }

  mk22f12_get_system_clock();  // update the default clock setting
#endif /* ifdef CLOCK_SETUP */

#endif /* !KINETIS_NO_INIT */
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

uint32_t mk22f12_get_system_clock (void) {

  uint32_t MCGOUTClock;                /* Variable to store output clock frequency of the MCG module */
  uint16_t Divider;

  if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x00U) {
    /* Output of FLL or PLL is selected */
    if ((MCG->C6 & MCG_C6_PLLS_MASK) == 0x00U) {
      /* FLL is selected */
      if ((MCG->C1 & MCG_C1_IREFS_MASK) == 0x00U) {
        /* External reference clock is selected */
        switch (MCG->C7 & MCG_C7_OSCSEL_MASK) {
        case 0x00U:
          MCGOUTClock = CPU_XTAL_CLK_HZ; /* System oscillator drives MCG clock */
          break;
        case 0x01U:
          MCGOUTClock = CPU_XTAL32k_CLK_HZ; /* RTC 32 kHz oscillator drives MCG clock */
          break;
        case 0x02U:
        default:
          MCGOUTClock = CPU_INT_IRC_CLK_HZ; /* IRC 48MHz oscillator drives MCG clock */
          break;
        }
        if (((MCG->C2 & MCG_C2_RANGE_MASK) != 0x00U) && ((MCG->C7 & MCG_C7_OSCSEL_MASK) != 0x01U)) {
          switch (MCG->C1 & MCG_C1_FRDIV_MASK) {
          case 0x38U:
            Divider = 1536U;
            break;
          case 0x30U:
            Divider = 1280U;
            break;
          default:
            Divider = (uint16_t)(32LU << ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT));
            break;
          }
        } else {/* ((MCG->C2 & MCG_C2_RANGE_MASK) != 0x00U) */
          Divider = (uint16_t)(1LU << ((MCG->C1 & MCG_C1_FRDIV_MASK) >> MCG_C1_FRDIV_SHIFT));
        }
        MCGOUTClock = (MCGOUTClock / Divider); /* Calculate the divided FLL reference clock */
      } else { /* (!((MCG->C1 & MCG_C1_IREFS_MASK) == 0x00U)) */
        MCGOUTClock = CPU_INT_SLOW_CLK_HZ; /* The slow internal reference clock is selected */
      } /* (!((MCG->C1 & MCG_C1_IREFS_MASK) == 0x00U)) */
      /* Select correct multiplier to calculate the MCG output clock  */
      switch (MCG->C4 & (MCG_C4_DMX32_MASK | MCG_C4_DRST_DRS_MASK)) {
        case 0x00U:
          MCGOUTClock *= 640U;
          break;
        case 0x20U:
          MCGOUTClock *= 1280U;
          break;
        case 0x40U:
          MCGOUTClock *= 1920U;
          break;
        case 0x60U:
          MCGOUTClock *= 2560U;
          break;
        case 0x80U:
          MCGOUTClock *= 732U;
          break;
        case 0xA0U:
          MCGOUTClock *= 1464U;
          break;
        case 0xC0U:
          MCGOUTClock *= 2197U;
          break;
        case 0xE0U:
          MCGOUTClock *= 2929U;
          break;
        default:
          break;
      }
    } else { /* (!((MCG->C6 & MCG_C6_PLLS_MASK) == 0x00U)) */
      /* PLL is selected */
      Divider = (((uint16_t)MCG->C5 & MCG_C5_PRDIV0_MASK) + 0x01U);
      MCGOUTClock = (uint32_t)(CPU_XTAL_CLK_HZ / Divider); /* Calculate the PLL reference clock */
      Divider = (((uint16_t)MCG->C6 & MCG_C6_VDIV0_MASK) + 24U);
      MCGOUTClock *= Divider;          /* Calculate the MCG output clock */
    } /* (!((MCG->C6 & MCG_C6_PLLS_MASK) == 0x00U)) */
  } else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x40U) {
    /* Internal reference clock is selected */
    if ((MCG->C2 & MCG_C2_IRCS_MASK) == 0x00U) {
      MCGOUTClock = CPU_INT_SLOW_CLK_HZ; /* Slow internal reference clock selected */
    } else { /* (!((MCG->C2 & MCG_C2_IRCS_MASK) == 0x00U)) */
      Divider = (uint16_t)(0x01LU << ((MCG->SC & MCG_SC_FCRDIV_MASK) >> MCG_SC_FCRDIV_SHIFT));
      MCGOUTClock = (uint32_t) (CPU_INT_FAST_CLK_HZ / Divider); /* Fast internal reference clock selected */
    } /* (!((MCG->C2 & MCG_C2_IRCS_MASK) == 0x00U)) */
  } else if ((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80U) {
    /* External reference clock is selected */
    switch (MCG->C7 & MCG_C7_OSCSEL_MASK) {
    case 0x00U:
      MCGOUTClock = CPU_XTAL_CLK_HZ;   /* System oscillator drives MCG clock */
      break;
    case 0x01U:
      MCGOUTClock = CPU_XTAL32k_CLK_HZ; /* RTC 32 kHz oscillator drives MCG clock */
      break;
    case 0x02U:
    default:
      MCGOUTClock = CPU_INT_IRC_CLK_HZ; /* IRC 48MHz oscillator drives MCG clock */
      break;
    }
  } else { /* (!((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80U)) */
    /* Reserved value */
    return 0;
  } /* (!((MCG->C1 & MCG_C1_CLKS_MASK) == 0x80U)) */
  SystemCoreClock = (MCGOUTClock / (0x01U + ((SIM->CLKDIV1 & SIM_CLKDIV1_OUTDIV1_MASK) >> SIM_CLKDIV1_OUTDIV1_SHIFT)));
  return SystemCoreClock;
}
/** @} */
