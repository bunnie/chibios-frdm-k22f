/*
** ###################################################################
**     Compilers:           Keil ARM C/C++ Compiler
**                          Freescale C/C++ for Embedded ARM
**                          GNU C Compiler
**                          GNU C Compiler - CodeSourcery Sourcery G++
**                          IAR ANSI C/C++ Compiler for ARM
**
**     Reference manual:    K22P121M120SF7RM, Rev. 1, March 24, 2014
**     Version:             rev. 2.8, 2015-02-19
**     Build:               b150225
**
**     Abstract:
**         CMSIS Peripheral Access Layer for MK22F51212
**
**     Copyright (c) 1997 - 2015 Freescale Semiconductor, Inc.
**     All rights reserved.
**
**     Redistribution and use in source and binary forms, with or without modification,
**     are permitted provided that the following conditions are met:
**
**     o Redistributions of source code must retain the above copyright notice, this list
**       of conditions and the following disclaimer.
**
**     o Redistributions in binary form must reproduce the above copyright notice, this
**       list of conditions and the following disclaimer in the documentation and/or
**       other materials provided with the distribution.
**
**     o Neither the name of Freescale Semiconductor, Inc. nor the names of its
**       contributors may be used to endorse or promote products derived from this
**       software without specific prior written permission.
**
**     THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
**     ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
**     WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
**     DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
**     ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
**     (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
**     LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
**     ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
**     (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
**     SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**     http:                 www.freescale.com
**     mail:                 support@freescale.com
**
**     Revisions:
**     - rev. 1.0 (2013-07-23)
**         Initial version.
**     - rev. 1.1 (2013-09-17)
**         RM rev. 0.4 update.
**     - rev. 2.0 (2013-10-29)
**         Register accessor macros added to the memory map.
**         Symbols for Processor Expert memory map compatibility added to the memory map.
**         Startup file for gcc has been updated according to CMSIS 3.2.
**         System initialization updated.
**     - rev. 2.1 (2013-10-30)
**         Definition of BITBAND macros updated to support peripherals with 32-bit acces disabled.
**     - rev. 2.2 (2013-12-20)
**         Update according to reference manual rev. 0.6,
**     - rev. 2.3 (2014-01-13)
**         Update according to reference manual rev. 0.61,
**     - rev. 2.4 (2014-02-10)
**         The declaration of clock configurations has been moved to separate header file system_MK22F51212.h
**     - rev. 2.5 (2014-05-06)
**         Update according to reference manual rev. 1.0,
**         Update of system and startup files.
**         Module access macro module_BASES replaced by module_BASE_PTRS.
**     - rev. 2.6 (2014-08-28)
**         Update of system files - default clock configuration changed.
**         Update of startup files - possibility to override DefaultISR added.
**     - rev. 2.7 (2014-10-14)
**         Interrupt INT_LPTimer renamed to INT_LPTMR0, interrupt INT_Watchdog renamed to INT_WDOG_EWM.
**     - rev. 2.8 (2015-02-19)
**         Renamed interrupt vector LLW to LLWU.
**
** ###################################################################
*/

/*!
 * @file MK22F51212.h
 * @version 2.8
 * @date 2015-02-19
 * @brief CMSIS Peripheral Access Layer for MK22F51212
 *
 * CMSIS Peripheral Access Layer for MK22F51212
 */


/* ----------------------------------------------------------------------------
   -- MCU activation
   ---------------------------------------------------------------------------- */

/* Prevention from multiple including the same memory map */
#if !defined(MK22F51212_H_)  /* Check if memory map has not been already included */
#define MK22F51212_H_
#define MCU_MK22F51212

/* Check if another memory map has not been also included */
#if (defined(MCU_ACTIVE))
  #error MK22F51212 memory map: There is already included another memory map. Only one memory map can be included.
#endif /* (defined(MCU_ACTIVE)) */
#define MCU_ACTIVE

#include <stdint.h>

/** Memory map major version (memory maps with equal major version number are
 * compatible) */
#define MCU_MEM_MAP_VERSION 0x0200u
/** Memory map minor version */
#define MCU_MEM_MAP_VERSION_MINOR 0x0008u

/**
 * @brief Macro to calculate address of an aliased word in the peripheral
 *        bitband area for a peripheral register and bit (bit band region 0x40000000 to
 *        0x400FFFFF).
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return  Address of the aliased word in the peripheral bitband area.
 */
#define BITBAND_REGADDR(Reg,Bit) (0x42000000u + (32u*((uint32_t)&(Reg) - (uint32_t)0x40000000u)) + (4u*((uint32_t)(Bit))))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 32bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG32(Reg,Bit) (*((uint32_t volatile*)(BITBAND_REGADDR(Reg,Bit))))
#define BITBAND_REG(Reg,Bit) (BITBAND_REG32(Reg,Bit))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 16bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG16(Reg,Bit) (*((uint16_t volatile*)(BITBAND_REGADDR(Reg,Bit))))
/**
 * @brief Macro to access a single bit of a peripheral register (bit band region
 *        0x40000000 to 0x400FFFFF) using the bit-band alias region access. Can
 *        be used for peripherals with 8bit access allowed.
 * @param Reg Register to access.
 * @param Bit Bit number to access.
 * @return Value of the targeted bit in the bit band region.
 */
#define BITBAND_REG8(Reg,Bit) (*((uint8_t volatile*)(BITBAND_REGADDR(Reg,Bit))))

/* ----------------------------------------------------------------------------
   -- Interrupt vector numbers
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Interrupt_vector_numbers Interrupt vector numbers
 * @{
 */

/** Interrupt Number Definitions */
#define NUMBER_OF_INT_VECTORS 102                /**< Number of interrupts in the Vector table */

typedef enum IRQn {
  /* Auxiliary constants */
  NotAvail_IRQn                = -128,             /**< Not available device specific interrupt */

  /* Core interrupts */
  NonMaskableInt_IRQn          = -14,              /**< Non Maskable Interrupt */
  HardFault_IRQn               = -13,              /**< Cortex-M4 SV Hard Fault Interrupt */
  MemoryManagement_IRQn        = -12,              /**< Cortex-M4 Memory Management Interrupt */
  BusFault_IRQn                = -11,              /**< Cortex-M4 Bus Fault Interrupt */
  UsageFault_IRQn              = -10,              /**< Cortex-M4 Usage Fault Interrupt */
  SVCall_IRQn                  = -5,               /**< Cortex-M4 SV Call Interrupt */
  DebugMonitor_IRQn            = -4,               /**< Cortex-M4 Debug Monitor Interrupt */
  PendSV_IRQn                  = -2,               /**< Cortex-M4 Pend SV Interrupt */
  SysTick_IRQn                 = -1,               /**< Cortex-M4 System Tick Interrupt */

  /* Device specific interrupts */
  DMA0_IRQn                    = 0,                /**< DMA Channel 0 Transfer Complete */
  DMA1_IRQn                    = 1,                /**< DMA Channel 1 Transfer Complete */
  DMA2_IRQn                    = 2,                /**< DMA Channel 2 Transfer Complete */
  DMA3_IRQn                    = 3,                /**< DMA Channel 3 Transfer Complete */
  DMA4_IRQn                    = 4,                /**< DMA Channel 4 Transfer Complete */
  DMA5_IRQn                    = 5,                /**< DMA Channel 5 Transfer Complete */
  DMA6_IRQn                    = 6,                /**< DMA Channel 6 Transfer Complete */
  DMA7_IRQn                    = 7,                /**< DMA Channel 7 Transfer Complete */
  DMA8_IRQn                    = 8,                /**< DMA Channel 8 Transfer Complete */
  DMA9_IRQn                    = 9,                /**< DMA Channel 9 Transfer Complete */
  DMA10_IRQn                   = 10,               /**< DMA Channel 10 Transfer Complete */
  DMA11_IRQn                   = 11,               /**< DMA Channel 11 Transfer Complete */
  DMA12_IRQn                   = 12,               /**< DMA Channel 12 Transfer Complete */
  DMA13_IRQn                   = 13,               /**< DMA Channel 13 Transfer Complete */
  DMA14_IRQn                   = 14,               /**< DMA Channel 14 Transfer Complete */
  DMA15_IRQn                   = 15,               /**< DMA Channel 15 Transfer Complete */
  DMA_Error_IRQn               = 16,               /**< DMA Error Interrupt */
  MCM_IRQn                     = 17,               /**< Normal Interrupt */
  FTF_IRQn                     = 18,               /**< FTFA Command complete interrupt */
  Read_Collision_IRQn          = 19,               /**< Read Collision Interrupt */
  LVD_LVW_IRQn                 = 20,               /**< Low Voltage Detect, Low Voltage Warning */
  LLWU_IRQn                    = 21,               /**< Low Leakage Wakeup Unit */
  WDOG_EWM_IRQn                = 22,               /**< WDOG Interrupt */
  RNG_IRQn                     = 23,               /**< RNG Interrupt */
  I2C0_IRQn                    = 24,               /**< I2C0 interrupt */
  I2C1_IRQn                    = 25,               /**< I2C1 interrupt */
  SPI0_IRQn                    = 26,               /**< SPI0 Interrupt */
  SPI1_IRQn                    = 27,               /**< SPI1 Interrupt */
  I2S0_Tx_IRQn                 = 28,               /**< I2S0 transmit interrupt */
  I2S0_Rx_IRQn                 = 29,               /**< I2S0 receive interrupt */
  LPUART0_IRQn                 = 30,               /**< LPUART0 status/error interrupt */
  UART0Status_IRQn             = 31,               /**< UART0 Receive/Transmit interrupt */
  UART0_ERR_IRQn               = 32,               /**< UART0 Error interrupt */
  UART1Status_IRQn             = 33,               /**< UART1 Receive/Transmit interrupt */
  UART1_ERR_IRQn               = 34,               /**< UART1 Error interrupt */
  UART2Status_IRQn             = 35,               /**< UART2 Receive/Transmit interrupt */
  UART2_ERR_IRQn               = 36,               /**< UART2 Error interrupt */
  Reserved53_IRQn              = 37,               /**< Reserved interrupt 53 */
  Reserved54_IRQn              = 38,               /**< Reserved interrupt 54 */
  ADC0_IRQn                    = 39,               /**< ADC0 interrupt */
  CMP0_IRQn                    = 40,               /**< CMP0 interrupt */
  CMP1_IRQn                    = 41,               /**< CMP1 interrupt */
  FTM0_IRQn                    = 42,               /**< FTM0 fault, overflow and channels interrupt */
  FTM1_IRQn                    = 43,               /**< FTM1 fault, overflow and channels interrupt */
  FTM2_IRQn                    = 44,               /**< FTM2 fault, overflow and channels interrupt */
  Reserved61_IRQn              = 45,               /**< Reserved interrupt 61 */
  RTC_IRQn                     = 46,               /**< RTC interrupt */
  RTC_Seconds_IRQn             = 47,               /**< RTC seconds interrupt */
  PIT0_IRQn                    = 48,               /**< PIT timer channel 0 interrupt */
  PIT1_IRQn                    = 49,               /**< PIT timer channel 1 interrupt */
  PIT2_IRQn                    = 50,               /**< PIT timer channel 2 interrupt */
  PIT3_IRQn                    = 51,               /**< PIT timer channel 3 interrupt */
  PDB0_IRQn                    = 52,               /**< PDB0 Interrupt */
  USB0_IRQn                    = 53,               /**< USB0 interrupt */
  Reserved70_IRQn              = 54,               /**< Reserved interrupt 70 */
  Reserved71_IRQn              = 55,               /**< Reserved interrupt 71 */
  DAC0_IRQn                    = 56,               /**< DAC0 interrupt */
  MCG_IRQn                     = 57,               /**< MCG Interrupt */
  LPTMR0_IRQn                  = 58,               /**< LPTimer interrupt */
  PINA_IRQn                   = 59,               /**< Port A interrupt */
  PINB_IRQn                   = 60,               /**< Port B interrupt */
  PINC_IRQn                   = 61,               /**< Port C interrupt */
  PIND_IRQn                   = 62,               /**< Port D interrupt */
  PINE_IRQn                   = 63,               /**< Port E interrupt */
  SWI_IRQn                     = 64,               /**< Software interrupt */
  Reserved81_IRQn              = 65,               /**< Reserved interrupt 81 */
  Reserved82_IRQn              = 66,               /**< Reserved interrupt 82 */
  Reserved83_IRQn              = 67,               /**< Reserved interrupt 83 */
  Reserved84_IRQn              = 68,               /**< Reserved interrupt 84 */
  Reserved85_IRQn              = 69,               /**< Reserved interrupt 85 */
  Reserved86_IRQn              = 70,               /**< Reserved interrupt 86 */
  FTM3_IRQn                    = 71,               /**< FTM3 fault, overflow and channels interrupt */
  DAC1_IRQn                    = 72,               /**< DAC1 interrupt */
  ADC1_IRQn                    = 73,               /**< ADC1 interrupt */
  Reserved90_IRQn              = 74,               /**< Reserved Interrupt 90 */
  Reserved91_IRQn              = 75,               /**< Reserved Interrupt 91 */
  Reserved92_IRQn              = 76,               /**< Reserved Interrupt 92 */
  Reserved93_IRQn              = 77,               /**< Reserved Interrupt 93 */
  Reserved94_IRQn              = 78,               /**< Reserved Interrupt 94 */
  Reserved95_IRQn              = 79,               /**< Reserved Interrupt 95 */
  Reserved96_IRQn              = 80,               /**< Reserved Interrupt 96 */
  Reserved97_IRQn              = 81,               /**< Reserved Interrupt 97 */
  Reserved98_IRQn              = 82,               /**< Reserved Interrupt 98 */
  Reserved99_IRQn              = 83,               /**< Reserved Interrupt 99 */
  Reserved100_IRQn             = 84,               /**< Reserved Interrupt 100 */
  Reserved101_IRQn             = 85                /**< Reserved Interrupt 101 */
} IRQn_Type;

/*!
 * @}
 */ /* end of group Interrupt_vector_numbers */


/* ----------------------------------------------------------------------------
   -- Cortex M4 Core Configuration
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Cortex_Core_Configuration Cortex M4 Core Configuration
 * @{
 */

#define __MPU_PRESENT                  0         /**< Defines if an MPU is present or not */
#define __NVIC_PRIO_BITS               4         /**< Number of priority bits implemented in the NVIC */
#define __Vendor_SysTickConfig         0         /**< Vendor specific implementation of SysTickConfig is defined */
#define __FPU_PRESENT                  1         /**< Defines if an FPU is present or not */

#include "core_cm4.h"                  /* Core Peripheral Access Layer */
// #include "system_MK22F51212.h"         /* Device specific configuration file */

/*!
 * @}
 */ /* end of group Cortex_Core_Configuration */


/* ----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{
 */


/*
** Start of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #pragma push
  #pragma anon_unions
#elif defined(__CWCC__)
  #pragma push
  #pragma cpp_extensions on
#elif defined(__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=extended
#else
  #error Not supported compiler type
#endif

/* ----------------------------------------------------------------------------
   -- ADC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Peripheral_Access_Layer ADC Peripheral Access Layer
 * @{
 */

/** ADC - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC1[2];                            /**< ADC Status and Control Registers 1, array offset: 0x0, array step: 0x4 */
  __IO uint32_t CFG1;                              /**< ADC Configuration Register 1, offset: 0x8 */
  __IO uint32_t CFG2;                              /**< ADC Configuration Register 2, offset: 0xC */
  __I  uint32_t R[2];                              /**< ADC Data Result Register, array offset: 0x10, array step: 0x4 */
  __IO uint32_t CV1;                               /**< Compare Value Registers, offset: 0x18 */
  __IO uint32_t CV2;                               /**< Compare Value Registers, offset: 0x1C */
  __IO uint32_t SC2;                               /**< Status and Control Register 2, offset: 0x20 */
  __IO uint32_t SC3;                               /**< Status and Control Register 3, offset: 0x24 */
  __IO uint32_t OFS;                               /**< ADC Offset Correction Register, offset: 0x28 */
  __IO uint32_t PG;                                /**< ADC Plus-Side Gain Register, offset: 0x2C */
  __IO uint32_t MG;                                /**< ADC Minus-Side Gain Register, offset: 0x30 */
  __IO uint32_t CLPD;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x34 */
  __IO uint32_t CLPS;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x38 */
  __IO uint32_t CLP4;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x3C */
  __IO uint32_t CLP3;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x40 */
  __IO uint32_t CLP2;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x44 */
  __IO uint32_t CLP1;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x48 */
  __IO uint32_t CLP0;                              /**< ADC Plus-Side General Calibration Value Register, offset: 0x4C */
       uint8_t RESERVED_0[4];
  __IO uint32_t CLMD;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x54 */
  __IO uint32_t CLMS;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x58 */
  __IO uint32_t CLM4;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x5C */
  __IO uint32_t CLM3;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x60 */
  __IO uint32_t CLM2;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x64 */
  __IO uint32_t CLM1;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x68 */
  __IO uint32_t CLM0;                              /**< ADC Minus-Side General Calibration Value Register, offset: 0x6C */
} ADC_TypeDef, *ADC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- ADC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Accessor_Macros ADC - Register accessor macros
 * @{
 */


/* ADC - Register accessors */
#define ADC_SC1_REG(base,index)                  ((base)->SC1[index])
#define ADC_SC1_COUNT                            2
#define ADC_CFG1_REG(base)                       ((base)->CFG1)
#define ADC_CFG2_REG(base)                       ((base)->CFG2)
#define ADC_R_REG(base,index)                    ((base)->R[index])
#define ADC_R_COUNT                              2
#define ADC_CV1_REG(base)                        ((base)->CV1)
#define ADC_CV2_REG(base)                        ((base)->CV2)
#define ADC_SC2_REG(base)                        ((base)->SC2)
#define ADC_SC3_REG(base)                        ((base)->SC3)
#define ADC_OFS_REG(base)                        ((base)->OFS)
#define ADC_PG_REG(base)                         ((base)->PG)
#define ADC_MG_REG(base)                         ((base)->MG)
#define ADC_CLPD_REG(base)                       ((base)->CLPD)
#define ADC_CLPS_REG(base)                       ((base)->CLPS)
#define ADC_CLP4_REG(base)                       ((base)->CLP4)
#define ADC_CLP3_REG(base)                       ((base)->CLP3)
#define ADC_CLP2_REG(base)                       ((base)->CLP2)
#define ADC_CLP1_REG(base)                       ((base)->CLP1)
#define ADC_CLP0_REG(base)                       ((base)->CLP0)
#define ADC_CLMD_REG(base)                       ((base)->CLMD)
#define ADC_CLMS_REG(base)                       ((base)->CLMS)
#define ADC_CLM4_REG(base)                       ((base)->CLM4)
#define ADC_CLM3_REG(base)                       ((base)->CLM3)
#define ADC_CLM2_REG(base)                       ((base)->CLM2)
#define ADC_CLM1_REG(base)                       ((base)->CLM1)
#define ADC_CLM0_REG(base)                       ((base)->CLM0)

/*!
 * @}
 */ /* end of group ADC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- ADC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Masks ADC Register Masks
 * @{
 */

/* SC1 Bit Fields */
#define ADC_SC1_ADCH_MASK                        0x1Fu
#define ADC_SC1_ADCH_SHIFT                       0
#define ADC_SC1_ADCH_WIDTH                       5
#define ADC_SC1_ADCH(x)                          (((uint32_t)(((uint32_t)(x))<<ADC_SC1_ADCH_SHIFT))&ADC_SC1_ADCH_MASK)
#define ADC_SC1_DIFF		((uint32_t)0x20u)
#define ADC_SC1_AIEN		((uint32_t)0x40u)
#define ADC_SC1_COCO		((uint32_t)0x80u)
/* CFG1 Bit Fields */
#define ADC_CFG1_ADICLK_MASK                     0x3u
#define ADC_CFG1_ADICLK_SHIFT                    0
#define ADC_CFG1_ADICLK_WIDTH                    2
#define ADC_CFG1_ADICLK(x)                       (((uint32_t)(((uint32_t)(x))<<ADC_CFG1_ADICLK_SHIFT))&ADC_CFG1_ADICLK_MASK)
#define ADC_CFG1_MODE_MASK                       0xCu
#define ADC_CFG1_MODE_SHIFT                      2
#define ADC_CFG1_MODE_WIDTH                      2
#define ADC_CFG1_MODE(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CFG1_MODE_SHIFT))&ADC_CFG1_MODE_MASK)
#define ADC_CFG1_ADLSMP		((uint32_t)0x10u)
#define ADC_CFG1_ADIV_MASK                       0x60u
#define ADC_CFG1_ADIV_SHIFT                      5
#define ADC_CFG1_ADIV_WIDTH                      2
#define ADC_CFG1_ADIV(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CFG1_ADIV_SHIFT))&ADC_CFG1_ADIV_MASK)
#define ADC_CFG1_ADLPC		((uint32_t)0x80u)
/* CFG2 Bit Fields */
#define ADC_CFG2_ADLSTS_MASK                     0x3u
#define ADC_CFG2_ADLSTS_SHIFT                    0
#define ADC_CFG2_ADLSTS_WIDTH                    2
#define ADC_CFG2_ADLSTS(x)                       (((uint32_t)(((uint32_t)(x))<<ADC_CFG2_ADLSTS_SHIFT))&ADC_CFG2_ADLSTS_MASK)
#define ADC_CFG2_ADHSC		((uint32_t)0x4u)
#define ADC_CFG2_ADACKEN		((uint32_t)0x8u)
#define ADC_CFG2_MUXSEL		((uint32_t)0x10u)
/* R Bit Fields */
#define ADC_R_D_MASK                             0xFFFFu
#define ADC_R_D_SHIFT                            0
#define ADC_R_D_WIDTH                            16
#define ADC_R_D(x)                               (((uint32_t)(((uint32_t)(x))<<ADC_R_D_SHIFT))&ADC_R_D_MASK)
/* CV1 Bit Fields */
#define ADC_CV1_CV_MASK                          0xFFFFu
#define ADC_CV1_CV_SHIFT                         0
#define ADC_CV1_CV_WIDTH                         16
#define ADC_CV1_CV(x)                            (((uint32_t)(((uint32_t)(x))<<ADC_CV1_CV_SHIFT))&ADC_CV1_CV_MASK)
/* CV2 Bit Fields */
#define ADC_CV2_CV_MASK                          0xFFFFu
#define ADC_CV2_CV_SHIFT                         0
#define ADC_CV2_CV_WIDTH                         16
#define ADC_CV2_CV(x)                            (((uint32_t)(((uint32_t)(x))<<ADC_CV2_CV_SHIFT))&ADC_CV2_CV_MASK)
/* SC2 Bit Fields */
#define ADC_SC2_REFSEL_MASK                      0x3u
#define ADC_SC2_REFSEL_SHIFT                     0
#define ADC_SC2_REFSEL_WIDTH                     2
#define ADC_SC2_REFSEL(x)                        (((uint32_t)(((uint32_t)(x))<<ADC_SC2_REFSEL_SHIFT))&ADC_SC2_REFSEL_MASK)
#define ADC_SC2_DMAEN		((uint32_t)0x4u)
#define ADC_SC2_ACREN		((uint32_t)0x8u)
#define ADC_SC2_ACFGT		((uint32_t)0x10u)
#define ADC_SC2_ACFE		((uint32_t)0x20u)
#define ADC_SC2_ADTRG		((uint32_t)0x40u)
#define ADC_SC2_ADACT		((uint32_t)0x80u)
/* SC3 Bit Fields */
#define ADC_SC3_AVGS_MASK                        0x3u
#define ADC_SC3_AVGS_SHIFT                       0
#define ADC_SC3_AVGS_WIDTH                       2
#define ADC_SC3_AVGS(x)                          (((uint32_t)(((uint32_t)(x))<<ADC_SC3_AVGS_SHIFT))&ADC_SC3_AVGS_MASK)
#define ADC_SC3_AVGE		((uint32_t)0x4u)
#define ADC_SC3_ADCO		((uint32_t)0x8u)
#define ADC_SC3_CALF		((uint32_t)0x40u)
#define ADC_SC3_CAL		((uint32_t)0x80u)
/* OFS Bit Fields */
#define ADC_OFS_OFS_MASK                         0xFFFFu
#define ADC_OFS_OFS_SHIFT                        0
#define ADC_OFS_OFS_WIDTH                        16
#define ADC_OFS_OFS(x)                           (((uint32_t)(((uint32_t)(x))<<ADC_OFS_OFS_SHIFT))&ADC_OFS_OFS_MASK)
/* PG Bit Fields */
#define ADC_PG_PG_MASK                           0xFFFFu
#define ADC_PG_PG_SHIFT                          0
#define ADC_PG_PG_WIDTH                          16
#define ADC_PG_PG(x)                             (((uint32_t)(((uint32_t)(x))<<ADC_PG_PG_SHIFT))&ADC_PG_PG_MASK)
/* MG Bit Fields */
#define ADC_MG_MG_MASK                           0xFFFFu
#define ADC_MG_MG_SHIFT                          0
#define ADC_MG_MG_WIDTH                          16
#define ADC_MG_MG(x)                             (((uint32_t)(((uint32_t)(x))<<ADC_MG_MG_SHIFT))&ADC_MG_MG_MASK)
/* CLPD Bit Fields */
#define ADC_CLPD_CLPD_MASK                       0x3Fu
#define ADC_CLPD_CLPD_SHIFT                      0
#define ADC_CLPD_CLPD_WIDTH                      6
#define ADC_CLPD_CLPD(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLPD_CLPD_SHIFT))&ADC_CLPD_CLPD_MASK)
/* CLPS Bit Fields */
#define ADC_CLPS_CLPS_MASK                       0x3Fu
#define ADC_CLPS_CLPS_SHIFT                      0
#define ADC_CLPS_CLPS_WIDTH                      6
#define ADC_CLPS_CLPS(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLPS_CLPS_SHIFT))&ADC_CLPS_CLPS_MASK)
/* CLP4 Bit Fields */
#define ADC_CLP4_CLP4_MASK                       0x3FFu
#define ADC_CLP4_CLP4_SHIFT                      0
#define ADC_CLP4_CLP4_WIDTH                      10
#define ADC_CLP4_CLP4(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLP4_CLP4_SHIFT))&ADC_CLP4_CLP4_MASK)
/* CLP3 Bit Fields */
#define ADC_CLP3_CLP3_MASK                       0x1FFu
#define ADC_CLP3_CLP3_SHIFT                      0
#define ADC_CLP3_CLP3_WIDTH                      9
#define ADC_CLP3_CLP3(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLP3_CLP3_SHIFT))&ADC_CLP3_CLP3_MASK)
/* CLP2 Bit Fields */
#define ADC_CLP2_CLP2_MASK                       0xFFu
#define ADC_CLP2_CLP2_SHIFT                      0
#define ADC_CLP2_CLP2_WIDTH                      8
#define ADC_CLP2_CLP2(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLP2_CLP2_SHIFT))&ADC_CLP2_CLP2_MASK)
/* CLP1 Bit Fields */
#define ADC_CLP1_CLP1_MASK                       0x7Fu
#define ADC_CLP1_CLP1_SHIFT                      0
#define ADC_CLP1_CLP1_WIDTH                      7
#define ADC_CLP1_CLP1(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLP1_CLP1_SHIFT))&ADC_CLP1_CLP1_MASK)
/* CLP0 Bit Fields */
#define ADC_CLP0_CLP0_MASK                       0x3Fu
#define ADC_CLP0_CLP0_SHIFT                      0
#define ADC_CLP0_CLP0_WIDTH                      6
#define ADC_CLP0_CLP0(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLP0_CLP0_SHIFT))&ADC_CLP0_CLP0_MASK)
/* CLMD Bit Fields */
#define ADC_CLMD_CLMD_MASK                       0x3Fu
#define ADC_CLMD_CLMD_SHIFT                      0
#define ADC_CLMD_CLMD_WIDTH                      6
#define ADC_CLMD_CLMD(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLMD_CLMD_SHIFT))&ADC_CLMD_CLMD_MASK)
/* CLMS Bit Fields */
#define ADC_CLMS_CLMS_MASK                       0x3Fu
#define ADC_CLMS_CLMS_SHIFT                      0
#define ADC_CLMS_CLMS_WIDTH                      6
#define ADC_CLMS_CLMS(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLMS_CLMS_SHIFT))&ADC_CLMS_CLMS_MASK)
/* CLM4 Bit Fields */
#define ADC_CLM4_CLM4_MASK                       0x3FFu
#define ADC_CLM4_CLM4_SHIFT                      0
#define ADC_CLM4_CLM4_WIDTH                      10
#define ADC_CLM4_CLM4(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLM4_CLM4_SHIFT))&ADC_CLM4_CLM4_MASK)
/* CLM3 Bit Fields */
#define ADC_CLM3_CLM3_MASK                       0x1FFu
#define ADC_CLM3_CLM3_SHIFT                      0
#define ADC_CLM3_CLM3_WIDTH                      9
#define ADC_CLM3_CLM3(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLM3_CLM3_SHIFT))&ADC_CLM3_CLM3_MASK)
/* CLM2 Bit Fields */
#define ADC_CLM2_CLM2_MASK                       0xFFu
#define ADC_CLM2_CLM2_SHIFT                      0
#define ADC_CLM2_CLM2_WIDTH                      8
#define ADC_CLM2_CLM2(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLM2_CLM2_SHIFT))&ADC_CLM2_CLM2_MASK)
/* CLM1 Bit Fields */
#define ADC_CLM1_CLM1_MASK                       0x7Fu
#define ADC_CLM1_CLM1_SHIFT                      0
#define ADC_CLM1_CLM1_WIDTH                      7
#define ADC_CLM1_CLM1(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLM1_CLM1_SHIFT))&ADC_CLM1_CLM1_MASK)
/* CLM0 Bit Fields */
#define ADC_CLM0_CLM0_MASK                       0x3Fu
#define ADC_CLM0_CLM0_SHIFT                      0
#define ADC_CLM0_CLM0_WIDTH                      6
#define ADC_CLM0_CLM0(x)                         (((uint32_t)(((uint32_t)(x))<<ADC_CLM0_CLM0_SHIFT))&ADC_CLM0_CLM0_MASK)

/*!
 * @}
 */ /* end of group ADC_Register_Masks */


/* ADC - Peripheral instance base addresses */
/** Peripheral ADC0 base address */
#define ADC0_BASE                                (0x4003B000u)
/** Peripheral ADC0 base pointer */
#define ADC0                                     ((ADC_TypeDef *)ADC0_BASE)
#define ADC0_BASE_PTR                            (ADC0)
/** Peripheral ADC1 base address */
#define ADC1_BASE                                (0x40027000u)
/** Peripheral ADC1 base pointer */
#define ADC1                                     ((ADC_TypeDef *)ADC1_BASE)
#define ADC1_BASE_PTR                            (ADC1)
/** Array initializer of ADC peripheral base addresses */
#define ADC_BASE_ADDRS                           { ADC0_BASE, ADC1_BASE }
/** Array initializer of ADC peripheral base pointers */
#define ADC_BASE_PTRS                            { ADC0, ADC1 }
/** Interrupt vectors for the ADC peripheral type */
#define ADC_IRQS                                 { ADC0_IRQn, ADC1_IRQn }

/* ----------------------------------------------------------------------------
   -- ADC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup ADC_Register_Accessor_Macros ADC - Register accessor macros
 * @{
 */


/* ADC - Register instance definitions */
/* ADC0 */
#define ADC0_SC1A                                ADC_SC1_REG(ADC0,0)
#define ADC0_SC1B                                ADC_SC1_REG(ADC0,1)
#define ADC0_CFG1                                ADC_CFG1_REG(ADC0)
#define ADC0_CFG2                                ADC_CFG2_REG(ADC0)
#define ADC0_RA                                  ADC_R_REG(ADC0,0)
#define ADC0_RB                                  ADC_R_REG(ADC0,1)
#define ADC0_CV1                                 ADC_CV1_REG(ADC0)
#define ADC0_CV2                                 ADC_CV2_REG(ADC0)
#define ADC0_SC2                                 ADC_SC2_REG(ADC0)
#define ADC0_SC3                                 ADC_SC3_REG(ADC0)
#define ADC0_OFS                                 ADC_OFS_REG(ADC0)
#define ADC0_PG                                  ADC_PG_REG(ADC0)
#define ADC0_MG                                  ADC_MG_REG(ADC0)
#define ADC0_CLPD                                ADC_CLPD_REG(ADC0)
#define ADC0_CLPS                                ADC_CLPS_REG(ADC0)
#define ADC0_CLP4                                ADC_CLP4_REG(ADC0)
#define ADC0_CLP3                                ADC_CLP3_REG(ADC0)
#define ADC0_CLP2                                ADC_CLP2_REG(ADC0)
#define ADC0_CLP1                                ADC_CLP1_REG(ADC0)
#define ADC0_CLP0                                ADC_CLP0_REG(ADC0)
#define ADC0_CLMD                                ADC_CLMD_REG(ADC0)
#define ADC0_CLMS                                ADC_CLMS_REG(ADC0)
#define ADC0_CLM4                                ADC_CLM4_REG(ADC0)
#define ADC0_CLM3                                ADC_CLM3_REG(ADC0)
#define ADC0_CLM2                                ADC_CLM2_REG(ADC0)
#define ADC0_CLM1                                ADC_CLM1_REG(ADC0)
#define ADC0_CLM0                                ADC_CLM0_REG(ADC0)
/* ADC1 */
#define ADC1_SC1A                                ADC_SC1_REG(ADC1,0)
#define ADC1_SC1B                                ADC_SC1_REG(ADC1,1)
#define ADC1_CFG1                                ADC_CFG1_REG(ADC1)
#define ADC1_CFG2                                ADC_CFG2_REG(ADC1)
#define ADC1_RA                                  ADC_R_REG(ADC1,0)
#define ADC1_RB                                  ADC_R_REG(ADC1,1)
#define ADC1_CV1                                 ADC_CV1_REG(ADC1)
#define ADC1_CV2                                 ADC_CV2_REG(ADC1)
#define ADC1_SC2                                 ADC_SC2_REG(ADC1)
#define ADC1_SC3                                 ADC_SC3_REG(ADC1)
#define ADC1_OFS                                 ADC_OFS_REG(ADC1)
#define ADC1_PG                                  ADC_PG_REG(ADC1)
#define ADC1_MG                                  ADC_MG_REG(ADC1)
#define ADC1_CLPD                                ADC_CLPD_REG(ADC1)
#define ADC1_CLPS                                ADC_CLPS_REG(ADC1)
#define ADC1_CLP4                                ADC_CLP4_REG(ADC1)
#define ADC1_CLP3                                ADC_CLP3_REG(ADC1)
#define ADC1_CLP2                                ADC_CLP2_REG(ADC1)
#define ADC1_CLP1                                ADC_CLP1_REG(ADC1)
#define ADC1_CLP0                                ADC_CLP0_REG(ADC1)
#define ADC1_CLMD                                ADC_CLMD_REG(ADC1)
#define ADC1_CLMS                                ADC_CLMS_REG(ADC1)
#define ADC1_CLM4                                ADC_CLM4_REG(ADC1)
#define ADC1_CLM3                                ADC_CLM3_REG(ADC1)
#define ADC1_CLM2                                ADC_CLM2_REG(ADC1)
#define ADC1_CLM1                                ADC_CLM1_REG(ADC1)
#define ADC1_CLM0                                ADC_CLM0_REG(ADC1)

/* ADC - Register array accessors */
#define ADC0_SC1(index)                          ADC_SC1_REG(ADC0,index)
#define ADC1_SC1(index)                          ADC_SC1_REG(ADC1,index)
#define ADC0_R(index)                            ADC_R_REG(ADC0,index)
#define ADC1_R(index)                            ADC_R_REG(ADC1,index)

/*!
 * @}
 */ /* end of group ADC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group ADC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CMP Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Peripheral_Access_Layer CMP Peripheral Access Layer
 * @{
 */

/** CMP - Register Layout Typedef */
typedef struct {
  __IO uint8_t CR0;                                /**< CMP Control Register 0, offset: 0x0 */
  __IO uint8_t CR1;                                /**< CMP Control Register 1, offset: 0x1 */
  __IO uint8_t FPR;                                /**< CMP Filter Period Register, offset: 0x2 */
  __IO uint8_t SCR;                                /**< CMP Status and Control Register, offset: 0x3 */
  __IO uint8_t DACCR;                              /**< DAC Control Register, offset: 0x4 */
  __IO uint8_t MUXCR;                              /**< MUX Control Register, offset: 0x5 */
} CMP_TypeDef, *CMP_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CMP - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Accessor_Macros CMP - Register accessor macros
 * @{
 */


/* CMP - Register accessors */
#define CMP_CR0_REG(base)                        ((base)->CR0)
#define CMP_CR1_REG(base)                        ((base)->CR1)
#define CMP_FPR_REG(base)                        ((base)->FPR)
#define CMP_SCR_REG(base)                        ((base)->SCR)
#define CMP_DACCR_REG(base)                      ((base)->DACCR)
#define CMP_MUXCR_REG(base)                      ((base)->MUXCR)

/*!
 * @}
 */ /* end of group CMP_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CMP Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Masks CMP Register Masks
 * @{
 */

/* CR0 Bit Fields */
#define CMP_CR0_HYSTCTR_MASK                     0x3u
#define CMP_CR0_HYSTCTR_SHIFT                    0
#define CMP_CR0_HYSTCTR_WIDTH                    2
#define CMP_CR0_HYSTCTR(x)                       (((uint8_t)(((uint8_t)(x))<<CMP_CR0_HYSTCTR_SHIFT))&CMP_CR0_HYSTCTR_MASK)
#define CMP_CR0_FILTER_CNT_MASK                  0x70u
#define CMP_CR0_FILTER_CNT_SHIFT                 4
#define CMP_CR0_FILTER_CNT_WIDTH                 3
#define CMP_CR0_FILTER_CNT(x)                    (((uint8_t)(((uint8_t)(x))<<CMP_CR0_FILTER_CNT_SHIFT))&CMP_CR0_FILTER_CNT_MASK)
/* CR1 Bit Fields */
#define CMP_CR1_EN		((uint8_t)0x1u)
#define CMP_CR1_OPE		((uint8_t)0x2u)
#define CMP_CR1_COS		((uint8_t)0x4u)
#define CMP_CR1_INV		((uint8_t)0x8u)
#define CMP_CR1_PMODE		((uint8_t)0x10u)
#define CMP_CR1_TRIGM		((uint8_t)0x20u)
#define CMP_CR1_WE		((uint8_t)0x40u)
#define CMP_CR1_SE		((uint8_t)0x80u)
/* FPR Bit Fields */
#define CMP_FPR_FILT_PER_MASK                    0xFFu
#define CMP_FPR_FILT_PER_SHIFT                   0
#define CMP_FPR_FILT_PER_WIDTH                   8
#define CMP_FPR_FILT_PER(x)                      (((uint8_t)(((uint8_t)(x))<<CMP_FPR_FILT_PER_SHIFT))&CMP_FPR_FILT_PER_MASK)
/* SCR Bit Fields */
#define CMP_SCR_COUT		((uint8_t)0x1u)
#define CMP_SCR_CFF		((uint8_t)0x2u)
#define CMP_SCR_CFR		((uint8_t)0x4u)
#define CMP_SCR_IEF		((uint8_t)0x8u)
#define CMP_SCR_IER		((uint8_t)0x10u)
#define CMP_SCR_DMAEN		((uint8_t)0x40u)
/* DACCR Bit Fields */
#define CMP_DACCR_VOSEL_MASK                     0x3Fu
#define CMP_DACCR_VOSEL_SHIFT                    0
#define CMP_DACCR_VOSEL_WIDTH                    6
#define CMP_DACCR_VOSEL(x)                       (((uint8_t)(((uint8_t)(x))<<CMP_DACCR_VOSEL_SHIFT))&CMP_DACCR_VOSEL_MASK)
#define CMP_DACCR_VRSEL		((uint8_t)0x40u)
#define CMP_DACCR_DACEN		((uint8_t)0x80u)
/* MUXCR Bit Fields */
#define CMP_MUXCR_MSEL_MASK                      0x7u
#define CMP_MUXCR_MSEL_SHIFT                     0
#define CMP_MUXCR_MSEL_WIDTH                     3
#define CMP_MUXCR_MSEL(x)                        (((uint8_t)(((uint8_t)(x))<<CMP_MUXCR_MSEL_SHIFT))&CMP_MUXCR_MSEL_MASK)
#define CMP_MUXCR_PSEL_MASK                      0x38u
#define CMP_MUXCR_PSEL_SHIFT                     3
#define CMP_MUXCR_PSEL_WIDTH                     3
#define CMP_MUXCR_PSEL(x)                        (((uint8_t)(((uint8_t)(x))<<CMP_MUXCR_PSEL_SHIFT))&CMP_MUXCR_PSEL_MASK)

/*!
 * @}
 */ /* end of group CMP_Register_Masks */


/* CMP - Peripheral instance base addresses */
/** Peripheral CMP0 base address */
#define CMP0_BASE                                (0x40073000u)
/** Peripheral CMP0 base pointer */
#define CMP0                                     ((CMP_TypeDef *)CMP0_BASE)
#define CMP0_BASE_PTR                            (CMP0)
/** Peripheral CMP1 base address */
#define CMP1_BASE                                (0x40073008u)
/** Peripheral CMP1 base pointer */
#define CMP1                                     ((CMP_TypeDef *)CMP1_BASE)
#define CMP1_BASE_PTR                            (CMP1)
/** Array initializer of CMP peripheral base addresses */
#define CMP_BASE_ADDRS                           { CMP0_BASE, CMP1_BASE }
/** Array initializer of CMP peripheral base pointers */
#define CMP_BASE_PTRS                            { CMP0, CMP1 }
/** Interrupt vectors for the CMP peripheral type */
#define CMP_IRQS                                 { CMP0_IRQn, CMP1_IRQn }

/* ----------------------------------------------------------------------------
   -- CMP - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CMP_Register_Accessor_Macros CMP - Register accessor macros
 * @{
 */


/* CMP - Register instance definitions */
/* CMP0 */
#define CMP0_CR0                                 CMP_CR0_REG(CMP0)
#define CMP0_CR1                                 CMP_CR1_REG(CMP0)
#define CMP0_FPR                                 CMP_FPR_REG(CMP0)
#define CMP0_SCR                                 CMP_SCR_REG(CMP0)
#define CMP0_DACCR                               CMP_DACCR_REG(CMP0)
#define CMP0_MUXCR                               CMP_MUXCR_REG(CMP0)
/* CMP1 */
#define CMP1_CR0                                 CMP_CR0_REG(CMP1)
#define CMP1_CR1                                 CMP_CR1_REG(CMP1)
#define CMP1_FPR                                 CMP_FPR_REG(CMP1)
#define CMP1_SCR                                 CMP_SCR_REG(CMP1)
#define CMP1_DACCR                               CMP_DACCR_REG(CMP1)
#define CMP1_MUXCR                               CMP_MUXCR_REG(CMP1)

/*!
 * @}
 */ /* end of group CMP_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group CMP_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- CRC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Peripheral_Access_Layer CRC Peripheral Access Layer
 * @{
 */

/** CRC - Register Layout Typedef */
typedef struct {
  union {                                          /* offset: 0x0 */
    struct {                                         /* offset: 0x0 */
      __IO uint16_t DATAL;                             /**< CRC_DATAL register., offset: 0x0 */
      __IO uint16_t DATAH;                             /**< CRC_DATAH register., offset: 0x2 */
    } ACCESS16BIT;
    __IO uint32_t DATA;                              /**< CRC Data register, offset: 0x0 */
    struct {                                         /* offset: 0x0 */
      __IO uint8_t DATALL;                             /**< CRC_DATALL register., offset: 0x0 */
      __IO uint8_t DATALU;                             /**< CRC_DATALU register., offset: 0x1 */
      __IO uint8_t DATAHL;                             /**< CRC_DATAHL register., offset: 0x2 */
      __IO uint8_t DATAHU;                             /**< CRC_DATAHU register., offset: 0x3 */
    } ACCESS8BIT;
  };
  union {                                          /* offset: 0x4 */
    struct {                                         /* offset: 0x4 */
      __IO uint16_t GPOLYL;                            /**< CRC_GPOLYL register., offset: 0x4 */
      __IO uint16_t GPOLYH;                            /**< CRC_GPOLYH register., offset: 0x6 */
    } GPOLY_ACCESS16BIT;
    __IO uint32_t GPOLY;                             /**< CRC Polynomial register, offset: 0x4 */
    struct {                                         /* offset: 0x4 */
      __IO uint8_t GPOLYLL;                            /**< CRC_GPOLYLL register., offset: 0x4 */
      __IO uint8_t GPOLYLU;                            /**< CRC_GPOLYLU register., offset: 0x5 */
      __IO uint8_t GPOLYHL;                            /**< CRC_GPOLYHL register., offset: 0x6 */
      __IO uint8_t GPOLYHU;                            /**< CRC_GPOLYHU register., offset: 0x7 */
    } GPOLY_ACCESS8BIT;
  };
  union {                                          /* offset: 0x8 */
    __IO uint32_t CTRL;                              /**< CRC Control register, offset: 0x8 */
    struct {                                         /* offset: 0x8 */
           uint8_t RESERVED_0[3];
      __IO uint8_t CTRLHU;                             /**< CRC_CTRLHU register., offset: 0xB */
    } CTRL_ACCESS8BIT;
  };
} CRC_TypeDef, *CRC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- CRC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Accessor_Macros CRC - Register accessor macros
 * @{
 */


/* CRC - Register accessors */
#define CRC_DATAL_REG(base)                      ((base)->ACCESS16BIT.DATAL)
#define CRC_DATAH_REG(base)                      ((base)->ACCESS16BIT.DATAH)
#define CRC_DATA_REG(base)                       ((base)->DATA)
#define CRC_DATALL_REG(base)                     ((base)->ACCESS8BIT.DATALL)
#define CRC_DATALU_REG(base)                     ((base)->ACCESS8BIT.DATALU)
#define CRC_DATAHL_REG(base)                     ((base)->ACCESS8BIT.DATAHL)
#define CRC_DATAHU_REG(base)                     ((base)->ACCESS8BIT.DATAHU)
#define CRC_GPOLYL_REG(base)                     ((base)->GPOLY_ACCESS16BIT.GPOLYL)
#define CRC_GPOLYH_REG(base)                     ((base)->GPOLY_ACCESS16BIT.GPOLYH)
#define CRC_GPOLY_REG(base)                      ((base)->GPOLY)
#define CRC_GPOLYLL_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYLL)
#define CRC_GPOLYLU_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYLU)
#define CRC_GPOLYHL_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYHL)
#define CRC_GPOLYHU_REG(base)                    ((base)->GPOLY_ACCESS8BIT.GPOLYHU)
#define CRC_CTRL_REG(base)                       ((base)->CTRL)
#define CRC_CTRLHU_REG(base)                     ((base)->CTRL_ACCESS8BIT.CTRLHU)

/*!
 * @}
 */ /* end of group CRC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- CRC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Masks CRC Register Masks
 * @{
 */

/* DATAL Bit Fields */
#define CRC_DATAL_DATAL_MASK                     0xFFFFu
#define CRC_DATAL_DATAL_SHIFT                    0
#define CRC_DATAL_DATAL_WIDTH                    16
#define CRC_DATAL_DATAL(x)                       (((uint16_t)(((uint16_t)(x))<<CRC_DATAL_DATAL_SHIFT))&CRC_DATAL_DATAL_MASK)
/* DATAH Bit Fields */
#define CRC_DATAH_DATAH_MASK                     0xFFFFu
#define CRC_DATAH_DATAH_SHIFT                    0
#define CRC_DATAH_DATAH_WIDTH                    16
#define CRC_DATAH_DATAH(x)                       (((uint16_t)(((uint16_t)(x))<<CRC_DATAH_DATAH_SHIFT))&CRC_DATAH_DATAH_MASK)
/* DATA Bit Fields */
#define CRC_DATA_LL_MASK                         0xFFu
#define CRC_DATA_LL_SHIFT                        0
#define CRC_DATA_LL_WIDTH                        8
#define CRC_DATA_LL(x)                           (((uint32_t)(((uint32_t)(x))<<CRC_DATA_LL_SHIFT))&CRC_DATA_LL_MASK)
#define CRC_DATA_LU_MASK                         0xFF00u
#define CRC_DATA_LU_SHIFT                        8
#define CRC_DATA_LU_WIDTH                        8
#define CRC_DATA_LU(x)                           (((uint32_t)(((uint32_t)(x))<<CRC_DATA_LU_SHIFT))&CRC_DATA_LU_MASK)
#define CRC_DATA_HL_MASK                         0xFF0000u
#define CRC_DATA_HL_SHIFT                        16
#define CRC_DATA_HL_WIDTH                        8
#define CRC_DATA_HL(x)                           (((uint32_t)(((uint32_t)(x))<<CRC_DATA_HL_SHIFT))&CRC_DATA_HL_MASK)
#define CRC_DATA_HU_MASK                         0xFF000000u
#define CRC_DATA_HU_SHIFT                        24
#define CRC_DATA_HU_WIDTH                        8
#define CRC_DATA_HU(x)                           (((uint32_t)(((uint32_t)(x))<<CRC_DATA_HU_SHIFT))&CRC_DATA_HU_MASK)
/* DATALL Bit Fields */
#define CRC_DATALL_DATALL_MASK                   0xFFu
#define CRC_DATALL_DATALL_SHIFT                  0
#define CRC_DATALL_DATALL_WIDTH                  8
#define CRC_DATALL_DATALL(x)                     (((uint8_t)(((uint8_t)(x))<<CRC_DATALL_DATALL_SHIFT))&CRC_DATALL_DATALL_MASK)
/* DATALU Bit Fields */
#define CRC_DATALU_DATALU_MASK                   0xFFu
#define CRC_DATALU_DATALU_SHIFT                  0
#define CRC_DATALU_DATALU_WIDTH                  8
#define CRC_DATALU_DATALU(x)                     (((uint8_t)(((uint8_t)(x))<<CRC_DATALU_DATALU_SHIFT))&CRC_DATALU_DATALU_MASK)
/* DATAHL Bit Fields */
#define CRC_DATAHL_DATAHL_MASK                   0xFFu
#define CRC_DATAHL_DATAHL_SHIFT                  0
#define CRC_DATAHL_DATAHL_WIDTH                  8
#define CRC_DATAHL_DATAHL(x)                     (((uint8_t)(((uint8_t)(x))<<CRC_DATAHL_DATAHL_SHIFT))&CRC_DATAHL_DATAHL_MASK)
/* DATAHU Bit Fields */
#define CRC_DATAHU_DATAHU_MASK                   0xFFu
#define CRC_DATAHU_DATAHU_SHIFT                  0
#define CRC_DATAHU_DATAHU_WIDTH                  8
#define CRC_DATAHU_DATAHU(x)                     (((uint8_t)(((uint8_t)(x))<<CRC_DATAHU_DATAHU_SHIFT))&CRC_DATAHU_DATAHU_MASK)
/* GPOLYL Bit Fields */
#define CRC_GPOLYL_GPOLYL_MASK                   0xFFFFu
#define CRC_GPOLYL_GPOLYL_SHIFT                  0
#define CRC_GPOLYL_GPOLYL_WIDTH                  16
#define CRC_GPOLYL_GPOLYL(x)                     (((uint16_t)(((uint16_t)(x))<<CRC_GPOLYL_GPOLYL_SHIFT))&CRC_GPOLYL_GPOLYL_MASK)
/* GPOLYH Bit Fields */
#define CRC_GPOLYH_GPOLYH_MASK                   0xFFFFu
#define CRC_GPOLYH_GPOLYH_SHIFT                  0
#define CRC_GPOLYH_GPOLYH_WIDTH                  16
#define CRC_GPOLYH_GPOLYH(x)                     (((uint16_t)(((uint16_t)(x))<<CRC_GPOLYH_GPOLYH_SHIFT))&CRC_GPOLYH_GPOLYH_MASK)
/* GPOLY Bit Fields */
#define CRC_GPOLY_LOW_MASK                       0xFFFFu
#define CRC_GPOLY_LOW_SHIFT                      0
#define CRC_GPOLY_LOW_WIDTH                      16
#define CRC_GPOLY_LOW(x)                         (((uint32_t)(((uint32_t)(x))<<CRC_GPOLY_LOW_SHIFT))&CRC_GPOLY_LOW_MASK)
#define CRC_GPOLY_HIGH_MASK                      0xFFFF0000u
#define CRC_GPOLY_HIGH_SHIFT                     16
#define CRC_GPOLY_HIGH_WIDTH                     16
#define CRC_GPOLY_HIGH(x)                        (((uint32_t)(((uint32_t)(x))<<CRC_GPOLY_HIGH_SHIFT))&CRC_GPOLY_HIGH_MASK)
/* GPOLYLL Bit Fields */
#define CRC_GPOLYLL_GPOLYLL_MASK                 0xFFu
#define CRC_GPOLYLL_GPOLYLL_SHIFT                0
#define CRC_GPOLYLL_GPOLYLL_WIDTH                8
#define CRC_GPOLYLL_GPOLYLL(x)                   (((uint8_t)(((uint8_t)(x))<<CRC_GPOLYLL_GPOLYLL_SHIFT))&CRC_GPOLYLL_GPOLYLL_MASK)
/* GPOLYLU Bit Fields */
#define CRC_GPOLYLU_GPOLYLU_MASK                 0xFFu
#define CRC_GPOLYLU_GPOLYLU_SHIFT                0
#define CRC_GPOLYLU_GPOLYLU_WIDTH                8
#define CRC_GPOLYLU_GPOLYLU(x)                   (((uint8_t)(((uint8_t)(x))<<CRC_GPOLYLU_GPOLYLU_SHIFT))&CRC_GPOLYLU_GPOLYLU_MASK)
/* GPOLYHL Bit Fields */
#define CRC_GPOLYHL_GPOLYHL_MASK                 0xFFu
#define CRC_GPOLYHL_GPOLYHL_SHIFT                0
#define CRC_GPOLYHL_GPOLYHL_WIDTH                8
#define CRC_GPOLYHL_GPOLYHL(x)                   (((uint8_t)(((uint8_t)(x))<<CRC_GPOLYHL_GPOLYHL_SHIFT))&CRC_GPOLYHL_GPOLYHL_MASK)
/* GPOLYHU Bit Fields */
#define CRC_GPOLYHU_GPOLYHU_MASK                 0xFFu
#define CRC_GPOLYHU_GPOLYHU_SHIFT                0
#define CRC_GPOLYHU_GPOLYHU_WIDTH                8
#define CRC_GPOLYHU_GPOLYHU(x)                   (((uint8_t)(((uint8_t)(x))<<CRC_GPOLYHU_GPOLYHU_SHIFT))&CRC_GPOLYHU_GPOLYHU_MASK)
/* CTRL Bit Fields */
#define CRC_CTRL_TCRC		((uint32_t)0x1000000u)
#define CRC_CTRL_WAS		((uint32_t)0x2000000u)
#define CRC_CTRL_FXOR		((uint32_t)0x4000000u)
#define CRC_CTRL_TOTR_MASK                       0x30000000u
#define CRC_CTRL_TOTR_SHIFT                      28
#define CRC_CTRL_TOTR_WIDTH                      2
#define CRC_CTRL_TOTR(x)                         (((uint32_t)(((uint32_t)(x))<<CRC_CTRL_TOTR_SHIFT))&CRC_CTRL_TOTR_MASK)
#define CRC_CTRL_TOT_MASK                        0xC0000000u
#define CRC_CTRL_TOT_SHIFT                       30
#define CRC_CTRL_TOT_WIDTH                       2
#define CRC_CTRL_TOT(x)                          (((uint32_t)(((uint32_t)(x))<<CRC_CTRL_TOT_SHIFT))&CRC_CTRL_TOT_MASK)
/* CTRLHU Bit Fields */
#define CRC_CTRLHU_TCRC		((uint8_t)0x1u)
#define CRC_CTRLHU_WAS		((uint8_t)0x2u)
#define CRC_CTRLHU_FXOR		((uint8_t)0x4u)
#define CRC_CTRLHU_TOTR_MASK                     0x30u
#define CRC_CTRLHU_TOTR_SHIFT                    4
#define CRC_CTRLHU_TOTR_WIDTH                    2
#define CRC_CTRLHU_TOTR(x)                       (((uint8_t)(((uint8_t)(x))<<CRC_CTRLHU_TOTR_SHIFT))&CRC_CTRLHU_TOTR_MASK)
#define CRC_CTRLHU_TOT_MASK                      0xC0u
#define CRC_CTRLHU_TOT_SHIFT                     6
#define CRC_CTRLHU_TOT_WIDTH                     2
#define CRC_CTRLHU_TOT(x)                        (((uint8_t)(((uint8_t)(x))<<CRC_CTRLHU_TOT_SHIFT))&CRC_CTRLHU_TOT_MASK)

/*!
 * @}
 */ /* end of group CRC_Register_Masks */


/* CRC - Peripheral instance base addresses */
/** Peripheral CRC base address */
#define CRC_BASE                                 (0x40032000u)
/** Peripheral CRC base pointer */
#define CRC0                                     ((CRC_TypeDef *)CRC_BASE)
#define CRC_BASE_PTR                             (CRC0)
/** Array initializer of CRC peripheral base addresses */
#define CRC_BASE_ADDRS                           { CRC_BASE }
/** Array initializer of CRC peripheral base pointers */
#define CRC_BASE_PTRS                            { CRC0 }

/* ----------------------------------------------------------------------------
   -- CRC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup CRC_Register_Accessor_Macros CRC - Register accessor macros
 * @{
 */


/* CRC - Register instance definitions */
/* CRC */
#define CRC_DATA                                 CRC_DATA_REG(CRC0)
#define CRC_DATAL                                CRC_DATAL_REG(CRC0)
#define CRC_DATALL                               CRC_DATALL_REG(CRC0)
#define CRC_DATALU                               CRC_DATALU_REG(CRC0)
#define CRC_DATAH                                CRC_DATAH_REG(CRC0)
#define CRC_DATAHL                               CRC_DATAHL_REG(CRC0)
#define CRC_DATAHU                               CRC_DATAHU_REG(CRC0)
#define CRC_GPOLY                                CRC_GPOLY_REG(CRC0)
#define CRC_GPOLYL                               CRC_GPOLYL_REG(CRC0)
#define CRC_GPOLYLL                              CRC_GPOLYLL_REG(CRC0)
#define CRC_GPOLYLU                              CRC_GPOLYLU_REG(CRC0)
#define CRC_GPOLYH                               CRC_GPOLYH_REG(CRC0)
#define CRC_GPOLYHL                              CRC_GPOLYHL_REG(CRC0)
#define CRC_GPOLYHU                              CRC_GPOLYHU_REG(CRC0)
#define CRC_CTRL                                 CRC_CTRL_REG(CRC0)
#define CRC_CTRLHU                               CRC_CTRLHU_REG(CRC0)

/*!
 * @}
 */ /* end of group CRC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group CRC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DAC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Peripheral_Access_Layer DAC Peripheral Access Layer
 * @{
 */

/** DAC - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0x2 */
    __IO uint8_t DATL;                               /**< DAC Data Low Register, array offset: 0x0, array step: 0x2 */
    __IO uint8_t DATH;                               /**< DAC Data High Register, array offset: 0x1, array step: 0x2 */
  } DAT[16];
  __IO uint8_t SR;                                 /**< DAC Status Register, offset: 0x20 */
  __IO uint8_t C0;                                 /**< DAC Control Register, offset: 0x21 */
  __IO uint8_t C1;                                 /**< DAC Control Register 1, offset: 0x22 */
  __IO uint8_t C2;                                 /**< DAC Control Register 2, offset: 0x23 */
} DAC_TypeDef, *DAC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DAC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Accessor_Macros DAC - Register accessor macros
 * @{
 */


/* DAC - Register accessors */
#define DAC_DATL_REG(base,index)                 ((base)->DAT[index].DATL)
#define DAC_DATL_COUNT                           16
#define DAC_DATH_REG(base,index)                 ((base)->DAT[index].DATH)
#define DAC_DATH_COUNT                           16
#define DAC_SR_REG(base)                         ((base)->SR)
#define DAC_C0_REG(base)                         ((base)->C0)
#define DAC_C1_REG(base)                         ((base)->C1)
#define DAC_C2_REG(base)                         ((base)->C2)

/*!
 * @}
 */ /* end of group DAC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DAC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Masks DAC Register Masks
 * @{
 */

/* DATL Bit Fields */
#define DAC_DATL_DATA0_MASK                      0xFFu
#define DAC_DATL_DATA0_SHIFT                     0
#define DAC_DATL_DATA0_WIDTH                     8
#define DAC_DATL_DATA0(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_DATL_DATA0_SHIFT))&DAC_DATL_DATA0_MASK)
/* DATH Bit Fields */
#define DAC_DATH_DATA1_MASK                      0xFu
#define DAC_DATH_DATA1_SHIFT                     0
#define DAC_DATH_DATA1_WIDTH                     4
#define DAC_DATH_DATA1(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_DATH_DATA1_SHIFT))&DAC_DATH_DATA1_MASK)
/* SR Bit Fields */
#define DAC_SR_DACBFRPBF		((uint8_t)0x1u)
#define DAC_SR_DACBFRPTF		((uint8_t)0x2u)
#define DAC_SR_DACBFWMF		((uint8_t)0x4u)
/* C0 Bit Fields */
#define DAC_C0_DACBBIEN		((uint8_t)0x1u)
#define DAC_C0_DACBTIEN		((uint8_t)0x2u)
#define DAC_C0_DACBWIEN		((uint8_t)0x4u)
#define DAC_C0_LPEN		((uint8_t)0x8u)
#define DAC_C0_DACSWTRG		((uint8_t)0x10u)
#define DAC_C0_DACTRGSEL		((uint8_t)0x20u)
#define DAC_C0_DACRFS		((uint8_t)0x40u)
#define DAC_C0_DACEN		((uint8_t)0x80u)
/* C1 Bit Fields */
#define DAC_C1_DACBFEN		((uint8_t)0x1u)
#define DAC_C1_DACBFMD_MASK                      0x6u
#define DAC_C1_DACBFMD_SHIFT                     1
#define DAC_C1_DACBFMD_WIDTH                     2
#define DAC_C1_DACBFMD(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_C1_DACBFMD_SHIFT))&DAC_C1_DACBFMD_MASK)
#define DAC_C1_DACBFWM_MASK                      0x18u
#define DAC_C1_DACBFWM_SHIFT                     3
#define DAC_C1_DACBFWM_WIDTH                     2
#define DAC_C1_DACBFWM(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_C1_DACBFWM_SHIFT))&DAC_C1_DACBFWM_MASK)
#define DAC_C1_DMAEN		((uint8_t)0x80u)
/* C2 Bit Fields */
#define DAC_C2_DACBFUP_MASK                      0xFu
#define DAC_C2_DACBFUP_SHIFT                     0
#define DAC_C2_DACBFUP_WIDTH                     4
#define DAC_C2_DACBFUP(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_C2_DACBFUP_SHIFT))&DAC_C2_DACBFUP_MASK)
#define DAC_C2_DACBFRP_MASK                      0xF0u
#define DAC_C2_DACBFRP_SHIFT                     4
#define DAC_C2_DACBFRP_WIDTH                     4
#define DAC_C2_DACBFRP(x)                        (((uint8_t)(((uint8_t)(x))<<DAC_C2_DACBFRP_SHIFT))&DAC_C2_DACBFRP_MASK)

/*!
 * @}
 */ /* end of group DAC_Register_Masks */


/* DAC - Peripheral instance base addresses */
/** Peripheral DAC0 base address */
#define DAC0_BASE                                (0x4003F000u)
/** Peripheral DAC0 base pointer */
#define DAC0                                     ((DAC_TypeDef *)DAC0_BASE)
#define DAC0_BASE_PTR                            (DAC0)
/** Peripheral DAC1 base address */
#define DAC1_BASE                                (0x40028000u)
/** Peripheral DAC1 base pointer */
#define DAC1                                     ((DAC_TypeDef *)DAC1_BASE)
#define DAC1_BASE_PTR                            (DAC1)
/** Array initializer of DAC peripheral base addresses */
#define DAC_BASE_ADDRS                           { DAC0_BASE, DAC1_BASE }
/** Array initializer of DAC peripheral base pointers */
#define DAC_BASE_PTRS                            { DAC0, DAC1 }
/** Interrupt vectors for the DAC peripheral type */
#define DAC_IRQS                                 { DAC0_IRQn, DAC1_IRQn }

/* ----------------------------------------------------------------------------
   -- DAC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DAC_Register_Accessor_Macros DAC - Register accessor macros
 * @{
 */


/* DAC - Register instance definitions */
/* DAC0 */
#define DAC0_DAT0L                               DAC_DATL_REG(DAC0,0)
#define DAC0_DAT0H                               DAC_DATH_REG(DAC0,0)
#define DAC0_DAT1L                               DAC_DATL_REG(DAC0,1)
#define DAC0_DAT1H                               DAC_DATH_REG(DAC0,1)
#define DAC0_DAT2L                               DAC_DATL_REG(DAC0,2)
#define DAC0_DAT2H                               DAC_DATH_REG(DAC0,2)
#define DAC0_DAT3L                               DAC_DATL_REG(DAC0,3)
#define DAC0_DAT3H                               DAC_DATH_REG(DAC0,3)
#define DAC0_DAT4L                               DAC_DATL_REG(DAC0,4)
#define DAC0_DAT4H                               DAC_DATH_REG(DAC0,4)
#define DAC0_DAT5L                               DAC_DATL_REG(DAC0,5)
#define DAC0_DAT5H                               DAC_DATH_REG(DAC0,5)
#define DAC0_DAT6L                               DAC_DATL_REG(DAC0,6)
#define DAC0_DAT6H                               DAC_DATH_REG(DAC0,6)
#define DAC0_DAT7L                               DAC_DATL_REG(DAC0,7)
#define DAC0_DAT7H                               DAC_DATH_REG(DAC0,7)
#define DAC0_DAT8L                               DAC_DATL_REG(DAC0,8)
#define DAC0_DAT8H                               DAC_DATH_REG(DAC0,8)
#define DAC0_DAT9L                               DAC_DATL_REG(DAC0,9)
#define DAC0_DAT9H                               DAC_DATH_REG(DAC0,9)
#define DAC0_DAT10L                              DAC_DATL_REG(DAC0,10)
#define DAC0_DAT10H                              DAC_DATH_REG(DAC0,10)
#define DAC0_DAT11L                              DAC_DATL_REG(DAC0,11)
#define DAC0_DAT11H                              DAC_DATH_REG(DAC0,11)
#define DAC0_DAT12L                              DAC_DATL_REG(DAC0,12)
#define DAC0_DAT12H                              DAC_DATH_REG(DAC0,12)
#define DAC0_DAT13L                              DAC_DATL_REG(DAC0,13)
#define DAC0_DAT13H                              DAC_DATH_REG(DAC0,13)
#define DAC0_DAT14L                              DAC_DATL_REG(DAC0,14)
#define DAC0_DAT14H                              DAC_DATH_REG(DAC0,14)
#define DAC0_DAT15L                              DAC_DATL_REG(DAC0,15)
#define DAC0_DAT15H                              DAC_DATH_REG(DAC0,15)
#define DAC0_SR                                  DAC_SR_REG(DAC0)
#define DAC0_C0                                  DAC_C0_REG(DAC0)
#define DAC0_C1                                  DAC_C1_REG(DAC0)
#define DAC0_C2                                  DAC_C2_REG(DAC0)
/* DAC1 */
#define DAC1_DAT0L                               DAC_DATL_REG(DAC1,0)
#define DAC1_DAT0H                               DAC_DATH_REG(DAC1,0)
#define DAC1_DAT1L                               DAC_DATL_REG(DAC1,1)
#define DAC1_DAT1H                               DAC_DATH_REG(DAC1,1)
#define DAC1_DAT2L                               DAC_DATL_REG(DAC1,2)
#define DAC1_DAT2H                               DAC_DATH_REG(DAC1,2)
#define DAC1_DAT3L                               DAC_DATL_REG(DAC1,3)
#define DAC1_DAT3H                               DAC_DATH_REG(DAC1,3)
#define DAC1_DAT4L                               DAC_DATL_REG(DAC1,4)
#define DAC1_DAT4H                               DAC_DATH_REG(DAC1,4)
#define DAC1_DAT5L                               DAC_DATL_REG(DAC1,5)
#define DAC1_DAT5H                               DAC_DATH_REG(DAC1,5)
#define DAC1_DAT6L                               DAC_DATL_REG(DAC1,6)
#define DAC1_DAT6H                               DAC_DATH_REG(DAC1,6)
#define DAC1_DAT7L                               DAC_DATL_REG(DAC1,7)
#define DAC1_DAT7H                               DAC_DATH_REG(DAC1,7)
#define DAC1_DAT8L                               DAC_DATL_REG(DAC1,8)
#define DAC1_DAT8H                               DAC_DATH_REG(DAC1,8)
#define DAC1_DAT9L                               DAC_DATL_REG(DAC1,9)
#define DAC1_DAT9H                               DAC_DATH_REG(DAC1,9)
#define DAC1_DAT10L                              DAC_DATL_REG(DAC1,10)
#define DAC1_DAT10H                              DAC_DATH_REG(DAC1,10)
#define DAC1_DAT11L                              DAC_DATL_REG(DAC1,11)
#define DAC1_DAT11H                              DAC_DATH_REG(DAC1,11)
#define DAC1_DAT12L                              DAC_DATL_REG(DAC1,12)
#define DAC1_DAT12H                              DAC_DATH_REG(DAC1,12)
#define DAC1_DAT13L                              DAC_DATL_REG(DAC1,13)
#define DAC1_DAT13H                              DAC_DATH_REG(DAC1,13)
#define DAC1_DAT14L                              DAC_DATL_REG(DAC1,14)
#define DAC1_DAT14H                              DAC_DATH_REG(DAC1,14)
#define DAC1_DAT15L                              DAC_DATL_REG(DAC1,15)
#define DAC1_DAT15H                              DAC_DATH_REG(DAC1,15)
#define DAC1_SR                                  DAC_SR_REG(DAC1)
#define DAC1_C0                                  DAC_C0_REG(DAC1)
#define DAC1_C1                                  DAC_C1_REG(DAC1)
#define DAC1_C2                                  DAC_C2_REG(DAC1)

/* DAC - Register array accessors */
#define DAC0_DATL(index)                         DAC_DATL_REG(DAC0,index)
#define DAC1_DATL(index)                         DAC_DATL_REG(DAC1,index)
#define DAC0_DATH(index)                         DAC_DATH_REG(DAC0,index)
#define DAC1_DATH(index)                         DAC_DATH_REG(DAC1,index)

/*!
 * @}
 */ /* end of group DAC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DAC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Peripheral_Access_Layer DMA Peripheral Access Layer
 * @{
 */

typedef struct {                                         /* offset: 0x1000, array step: 0x20 */
    __IO uint32_t SADDR;                             /**< TCD Source Address, array offset: 0x1000, array step: 0x20 */
    __IO uint16_t SOFF;                              /**< TCD Signed Source Address Offset, array offset: 0x1004, array step: 0x20 */
    __IO uint16_t ATTR;                              /**< TCD Transfer Attributes, array offset: 0x1006, array step: 0x20 */
    union {                                          /* offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLNO;                       /**< TCD Minor Byte Count (Minor Loop Disabled), array offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLOFFNO;                    /**< TCD Signed Minor Loop Offset (Minor Loop Enabled and Offset Disabled), array offset: 0x1008, array step: 0x20 */
      __IO uint32_t NBYTES_MLOFFYES;                   /**< TCD Signed Minor Loop Offset (Minor Loop and Offset Enabled), array offset: 0x1008, array step: 0x20 */
    };
    __IO uint32_t SLAST;                             /**< TCD Last Source Address Adjustment, array offset: 0x100C, array step: 0x20 */
    __IO uint32_t DADDR;                             /**< TCD Destination Address, array offset: 0x1010, array step: 0x20 */
    __IO uint16_t DOFF;                              /**< TCD Signed Destination Address Offset, array offset: 0x1014, array step: 0x20 */
    union {                                          /* offset: 0x1016, array step: 0x20 */
      __IO uint16_t CITER_ELINKNO;                     /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x1016, array step: 0x20 */
      __IO uint16_t CITER_ELINKYES;                    /**< TCD Current Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x1016, array step: 0x20 */
    };
    __IO uint32_t DLASTSGA;                         /**< TCD Last Destination Address Adjustment/Scatter Gather Address, array offset: 0x1018, array step: 0x20 */
    __IO uint16_t CSR;                               /**< TCD Control and Status, array offset: 0x101C, array step: 0x20 */
    union {                                          /* offset: 0x101E, array step: 0x20 */
      __IO uint16_t BITER_ELINKNO;                     /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Disabled), array offset: 0x101E, array step: 0x20 */
      __IO uint16_t BITER_ELINKYES;                    /**< TCD Beginning Minor Loop Link, Major Loop Count (Channel Linking Enabled), array offset: 0x101E, array step: 0x20 */
    };
} DMA_TCD_TypeDef;

/** DMA - Register Layout Typedef */
typedef struct {
  __IO uint32_t CR;                                /**< Control Register, offset: 0x0 */
  __I  uint32_t ES;                                /**< Error Status Register, offset: 0x4 */
       uint8_t RESERVED_0[4];
  __IO uint32_t ERQ;                               /**< Enable Request Register, offset: 0xC */
       uint8_t RESERVED_1[4];
  __IO uint32_t EEI;                               /**< Enable Error Interrupt Register, offset: 0x14 */
  __O  uint8_t CEEI;                               /**< Clear Enable Error Interrupt Register, offset: 0x18 */
  __O  uint8_t SEEI;                               /**< Set Enable Error Interrupt Register, offset: 0x19 */
  __O  uint8_t CERQ;                               /**< Clear Enable Request Register, offset: 0x1A */
  __O  uint8_t SERQ;                               /**< Set Enable Request Register, offset: 0x1B */
  __O  uint8_t CDNE;                               /**< Clear DONE Status Bit Register, offset: 0x1C */
  __O  uint8_t SSRT;                               /**< Set START Bit Register, offset: 0x1D */
  __O  uint8_t CERR;                               /**< Clear Error Register, offset: 0x1E */
  __O  uint8_t CINT;                               /**< Clear Interrupt Request Register, offset: 0x1F */
       uint8_t RESERVED_2[4];
  __IO uint32_t INT;                               /**< Interrupt Request Register, offset: 0x24 */
       uint8_t RESERVED_3[4];
  __IO uint32_t ERR;                               /**< Error Register, offset: 0x2C */
       uint8_t RESERVED_4[4];
  __I  uint32_t HRS;                               /**< Hardware Request Status Register, offset: 0x34 */
       uint8_t RESERVED_5[12];
  __IO uint32_t EARS;                              /**< Enable Asynchronous Request in Stop Register, offset: 0x44 */
       uint8_t RESERVED_6[184];
  __IO uint8_t DCHPRI3;                            /**< Channel n Priority Register, offset: 0x100 */
  __IO uint8_t DCHPRI2;                            /**< Channel n Priority Register, offset: 0x101 */
  __IO uint8_t DCHPRI1;                            /**< Channel n Priority Register, offset: 0x102 */
  __IO uint8_t DCHPRI0;                            /**< Channel n Priority Register, offset: 0x103 */
  __IO uint8_t DCHPRI7;                            /**< Channel n Priority Register, offset: 0x104 */
  __IO uint8_t DCHPRI6;                            /**< Channel n Priority Register, offset: 0x105 */
  __IO uint8_t DCHPRI5;                            /**< Channel n Priority Register, offset: 0x106 */
  __IO uint8_t DCHPRI4;                            /**< Channel n Priority Register, offset: 0x107 */
  __IO uint8_t DCHPRI11;                           /**< Channel n Priority Register, offset: 0x108 */
  __IO uint8_t DCHPRI10;                           /**< Channel n Priority Register, offset: 0x109 */
  __IO uint8_t DCHPRI9;                            /**< Channel n Priority Register, offset: 0x10A */
  __IO uint8_t DCHPRI8;                            /**< Channel n Priority Register, offset: 0x10B */
  __IO uint8_t DCHPRI15;                           /**< Channel n Priority Register, offset: 0x10C */
  __IO uint8_t DCHPRI14;                           /**< Channel n Priority Register, offset: 0x10D */
  __IO uint8_t DCHPRI13;                           /**< Channel n Priority Register, offset: 0x10E */
  __IO uint8_t DCHPRI12;                           /**< Channel n Priority Register, offset: 0x10F */
       uint8_t RESERVED_7[3824];
  DMA_TCD_TypeDef TCD[16];
} DMA_TypeDef, *DMA_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DMA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Accessor_Macros DMA - Register accessor macros
 * @{
 */


/* DMA - Register accessors */
#define DMA_CR_REG(base)                         ((base)->CR)
#define DMA_ES_REG(base)                         ((base)->ES)
#define DMA_ERQ_REG(base)                        ((base)->ERQ)
#define DMA_EEI_REG(base)                        ((base)->EEI)
#define DMA_CEEI_REG(base)                       ((base)->CEEI)
#define DMA_SEEI_REG(base)                       ((base)->SEEI)
#define DMA_CERQ_REG(base)                       ((base)->CERQ)
#define DMA_SERQ_REG(base)                       ((base)->SERQ)
#define DMA_CDNE_REG(base)                       ((base)->CDNE)
#define DMA_SSRT_REG(base)                       ((base)->SSRT)
#define DMA_CERR_REG(base)                       ((base)->CERR)
#define DMA_CINT_REG(base)                       ((base)->CINT)
#define DMA_INT_REG(base)                        ((base)->INT)
#define DMA_ERR_REG(base)                        ((base)->ERR)
#define DMA_HRS_REG(base)                        ((base)->HRS)
#define DMA_EARS_REG(base)                       ((base)->EARS)
#define DMA_DCHPRI3_REG(base)                    ((base)->DCHPRI3)
#define DMA_DCHPRI2_REG(base)                    ((base)->DCHPRI2)
#define DMA_DCHPRI1_REG(base)                    ((base)->DCHPRI1)
#define DMA_DCHPRI0_REG(base)                    ((base)->DCHPRI0)
#define DMA_DCHPRI7_REG(base)                    ((base)->DCHPRI7)
#define DMA_DCHPRI6_REG(base)                    ((base)->DCHPRI6)
#define DMA_DCHPRI5_REG(base)                    ((base)->DCHPRI5)
#define DMA_DCHPRI4_REG(base)                    ((base)->DCHPRI4)
#define DMA_DCHPRI11_REG(base)                   ((base)->DCHPRI11)
#define DMA_DCHPRI10_REG(base)                   ((base)->DCHPRI10)
#define DMA_DCHPRI9_REG(base)                    ((base)->DCHPRI9)
#define DMA_DCHPRI8_REG(base)                    ((base)->DCHPRI8)
#define DMA_DCHPRI15_REG(base)                   ((base)->DCHPRI15)
#define DMA_DCHPRI14_REG(base)                   ((base)->DCHPRI14)
#define DMA_DCHPRI13_REG(base)                   ((base)->DCHPRI13)
#define DMA_DCHPRI12_REG(base)                   ((base)->DCHPRI12)
#define DMA_SADDR_REG(base,index)                ((base)->TCD[index].SADDR)
#define DMA_SADDR_COUNT                          16
#define DMA_SOFF_REG(base,index)                 ((base)->TCD[index].SOFF)
#define DMA_SOFF_COUNT                           16
#define DMA_ATTR_REG(base,index)                 ((base)->TCD[index].ATTR)
#define DMA_ATTR_COUNT                           16
#define DMA_NBYTES_MLNO_REG(base,index)          ((base)->TCD[index].NBYTES_MLNO)
#define DMA_NBYTES_MLNO_COUNT                    16
#define DMA_NBYTES_MLOFFNO_REG(base,index)       ((base)->TCD[index].NBYTES_MLOFFNO)
#define DMA_NBYTES_MLOFFNO_COUNT                 16
#define DMA_NBYTES_MLOFFYES_REG(base,index)      ((base)->TCD[index].NBYTES_MLOFFYES)
#define DMA_NBYTES_MLOFFYES_COUNT                16
#define DMA_SLAST_REG(base,index)                ((base)->TCD[index].SLAST)
#define DMA_SLAST_COUNT                          16
#define DMA_DADDR_REG(base,index)                ((base)->TCD[index].DADDR)
#define DMA_DADDR_COUNT                          16
#define DMA_DOFF_REG(base,index)                 ((base)->TCD[index].DOFF)
#define DMA_DOFF_COUNT                           16
#define DMA_CITER_ELINKNO_REG(base,index)        ((base)->TCD[index].CITER_ELINKNO)
#define DMA_CITER_ELINKNO_COUNT                  16
#define DMA_CITER_ELINKYES_REG(base,index)       ((base)->TCD[index].CITER_ELINKYES)
#define DMA_CITER_ELINKYES_COUNT                 16
#define DMA_DLAST_SGA_REG(base,index)            ((base)->TCD[index].DLAST_SGA)
#define DMA_DLAST_SGA_COUNT                      16
#define DMA_CSR_REG(base,index)                  ((base)->TCD[index].CSR)
#define DMA_CSR_COUNT                            16
#define DMA_BITER_ELINKNO_REG(base,index)        ((base)->TCD[index].BITER_ELINKNO)
#define DMA_BITER_ELINKNO_COUNT                  16
#define DMA_BITER_ELINKYES_REG(base,index)       ((base)->TCD[index].BITER_ELINKYES)
#define DMA_BITER_ELINKYES_COUNT                 16

/*!
 * @}
 */ /* end of group DMA_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DMA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Masks DMA Register Masks
 * @{
 */

/* CR Bit Fields */
#define DMA_CR_EDBG		((uint32_t)0x2u)
#define DMA_CR_ERCA		((uint32_t)0x4u)
#define DMA_CR_HOE		((uint32_t)0x10u)
#define DMA_CR_HALT		((uint32_t)0x20u)
#define DMA_CR_CLM		((uint32_t)0x40u)
#define DMA_CR_EMLM		((uint32_t)0x80u)
#define DMA_CR_ECX		((uint32_t)0x10000u)
#define DMA_CR_CX		((uint32_t)0x20000u)
/* ES Bit Fields */
#define DMA_ES_DBE		((uint32_t)0x1u)
#define DMA_ES_SBE		((uint32_t)0x2u)
#define DMA_ES_SGE		((uint32_t)0x4u)
#define DMA_ES_NCE		((uint32_t)0x8u)
#define DMA_ES_DOE		((uint32_t)0x10u)
#define DMA_ES_DAE		((uint32_t)0x20u)
#define DMA_ES_SOE		((uint32_t)0x40u)
#define DMA_ES_SAE		((uint32_t)0x80u)
#define DMA_ES_ERRCHN_MASK                       0xF00u
#define DMA_ES_ERRCHN_SHIFT                      8
#define DMA_ES_ERRCHN_WIDTH                      4
#define DMA_ES_ERRCHN(x)                         (((uint32_t)(((uint32_t)(x))<<DMA_ES_ERRCHN_SHIFT))&DMA_ES_ERRCHN_MASK)
#define DMA_ES_CPE		((uint32_t)0x4000u)
#define DMA_ES_ECX		((uint32_t)0x10000u)
#define DMA_ES_VLD		((uint32_t)0x80000000u)
/* ERQ Bit Fields */
#define DMA_ERQ_ERQ0		((uint32_t)0x1u)
#define DMA_ERQ_ERQ1		((uint32_t)0x2u)
#define DMA_ERQ_ERQ2		((uint32_t)0x4u)
#define DMA_ERQ_ERQ3		((uint32_t)0x8u)
#define DMA_ERQ_ERQ4		((uint32_t)0x10u)
#define DMA_ERQ_ERQ5		((uint32_t)0x20u)
#define DMA_ERQ_ERQ6		((uint32_t)0x40u)
#define DMA_ERQ_ERQ7		((uint32_t)0x80u)
#define DMA_ERQ_ERQ8		((uint32_t)0x100u)
#define DMA_ERQ_ERQ9		((uint32_t)0x200u)
#define DMA_ERQ_ERQ10		((uint32_t)0x400u)
#define DMA_ERQ_ERQ11		((uint32_t)0x800u)
#define DMA_ERQ_ERQ12		((uint32_t)0x1000u)
#define DMA_ERQ_ERQ13		((uint32_t)0x2000u)
#define DMA_ERQ_ERQ14		((uint32_t)0x4000u)
#define DMA_ERQ_ERQ15		((uint32_t)0x8000u)
/* EEI Bit Fields */
#define DMA_EEI_EEI0		((uint32_t)0x1u)
#define DMA_EEI_EEI1		((uint32_t)0x2u)
#define DMA_EEI_EEI2		((uint32_t)0x4u)
#define DMA_EEI_EEI3		((uint32_t)0x8u)
#define DMA_EEI_EEI4		((uint32_t)0x10u)
#define DMA_EEI_EEI5		((uint32_t)0x20u)
#define DMA_EEI_EEI6		((uint32_t)0x40u)
#define DMA_EEI_EEI7		((uint32_t)0x80u)
#define DMA_EEI_EEI8		((uint32_t)0x100u)
#define DMA_EEI_EEI9		((uint32_t)0x200u)
#define DMA_EEI_EEI10		((uint32_t)0x400u)
#define DMA_EEI_EEI11		((uint32_t)0x800u)
#define DMA_EEI_EEI12		((uint32_t)0x1000u)
#define DMA_EEI_EEI13		((uint32_t)0x2000u)
#define DMA_EEI_EEI14		((uint32_t)0x4000u)
#define DMA_EEI_EEI15		((uint32_t)0x8000u)
/* CEEI Bit Fields */
#define DMA_CEEI_CEEI_MASK                       0xFu
#define DMA_CEEI_CEEI_SHIFT                      0
#define DMA_CEEI_CEEI_WIDTH                      4
#define DMA_CEEI_CEEI(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CEEI_CEEI_SHIFT))&DMA_CEEI_CEEI_MASK)
#define DMA_CEEI_CAEE		((uint8_t)0x40u)
#define DMA_CEEI_NOP		((uint8_t)0x80u)
/* SEEI Bit Fields */
#define DMA_SEEI_SEEI_MASK                       0xFu
#define DMA_SEEI_SEEI_SHIFT                      0
#define DMA_SEEI_SEEI_WIDTH                      4
#define DMA_SEEI_SEEI(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SEEI_SEEI_SHIFT))&DMA_SEEI_SEEI_MASK)
#define DMA_SEEI_SAEE		((uint8_t)0x40u)
#define DMA_SEEI_NOP		((uint8_t)0x80u)
/* CERQ Bit Fields */
#define DMA_CERQ_CERQ_MASK                       0xFu
#define DMA_CERQ_CERQ_SHIFT                      0
#define DMA_CERQ_CERQ_WIDTH                      4
#define DMA_CERQ_CERQ(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CERQ_CERQ_SHIFT))&DMA_CERQ_CERQ_MASK)
#define DMA_CERQ_CAER		((uint8_t)0x40u)
#define DMA_CERQ_NOP		((uint8_t)0x80u)
/* SERQ Bit Fields */
#define DMA_SERQ_SERQ_MASK                       0xFu
#define DMA_SERQ_SERQ_SHIFT                      0
#define DMA_SERQ_SERQ_WIDTH                      4
#define DMA_SERQ_SERQ(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SERQ_SERQ_SHIFT))&DMA_SERQ_SERQ_MASK)
#define DMA_SERQ_SAER		((uint8_t)0x40u)
#define DMA_SERQ_NOP		((uint8_t)0x80u)
/* CDNE Bit Fields */
#define DMA_CDNE_CDNE_MASK                       0xFu
#define DMA_CDNE_CDNE_SHIFT                      0
#define DMA_CDNE_CDNE_WIDTH                      4
#define DMA_CDNE_CDNE(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CDNE_CDNE_SHIFT))&DMA_CDNE_CDNE_MASK)
#define DMA_CDNE_CADN		((uint8_t)0x40u)
#define DMA_CDNE_NOP		((uint8_t)0x80u)
/* SSRT Bit Fields */
#define DMA_SSRT_SSRT_MASK                       0xFu
#define DMA_SSRT_SSRT_SHIFT                      0
#define DMA_SSRT_SSRT_WIDTH                      4
#define DMA_SSRT_SSRT(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_SSRT_SSRT_SHIFT))&DMA_SSRT_SSRT_MASK)
#define DMA_SSRT_SAST		((uint8_t)0x40u)
#define DMA_SSRT_NOP		((uint8_t)0x80u)
/* CERR Bit Fields */
#define DMA_CERR_CERR_MASK                       0xFu
#define DMA_CERR_CERR_SHIFT                      0
#define DMA_CERR_CERR_WIDTH                      4
#define DMA_CERR_CERR(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CERR_CERR_SHIFT))&DMA_CERR_CERR_MASK)
#define DMA_CERR_CAEI		((uint8_t)0x40u)
#define DMA_CERR_NOP		((uint8_t)0x80u)
/* CINT Bit Fields */
#define DMA_CINT_CINT_MASK                       0xFu
#define DMA_CINT_CINT_SHIFT                      0
#define DMA_CINT_CINT_WIDTH                      4
#define DMA_CINT_CINT(x)                         (((uint8_t)(((uint8_t)(x))<<DMA_CINT_CINT_SHIFT))&DMA_CINT_CINT_MASK)
#define DMA_CINT_CAIR		((uint8_t)0x40u)
#define DMA_CINT_NOP		((uint8_t)0x80u)
/* INT Bit Fields */
#define DMA_INT_INT0		((uint32_t)0x1u)
#define DMA_INT_INT1		((uint32_t)0x2u)
#define DMA_INT_INT2		((uint32_t)0x4u)
#define DMA_INT_INT3		((uint32_t)0x8u)
#define DMA_INT_INT4		((uint32_t)0x10u)
#define DMA_INT_INT5		((uint32_t)0x20u)
#define DMA_INT_INT6		((uint32_t)0x40u)
#define DMA_INT_INT7		((uint32_t)0x80u)
#define DMA_INT_INT8		((uint32_t)0x100u)
#define DMA_INT_INT9		((uint32_t)0x200u)
#define DMA_INT_INT10		((uint32_t)0x400u)
#define DMA_INT_INT11		((uint32_t)0x800u)
#define DMA_INT_INT12		((uint32_t)0x1000u)
#define DMA_INT_INT13		((uint32_t)0x2000u)
#define DMA_INT_INT14		((uint32_t)0x4000u)
#define DMA_INT_INT15		((uint32_t)0x8000u)
/* ERR Bit Fields */
#define DMA_ERR_ERR0		((uint32_t)0x1u)
#define DMA_ERR_ERR1		((uint32_t)0x2u)
#define DMA_ERR_ERR2		((uint32_t)0x4u)
#define DMA_ERR_ERR3		((uint32_t)0x8u)
#define DMA_ERR_ERR4		((uint32_t)0x10u)
#define DMA_ERR_ERR5		((uint32_t)0x20u)
#define DMA_ERR_ERR6		((uint32_t)0x40u)
#define DMA_ERR_ERR7		((uint32_t)0x80u)
#define DMA_ERR_ERR8		((uint32_t)0x100u)
#define DMA_ERR_ERR9		((uint32_t)0x200u)
#define DMA_ERR_ERR10		((uint32_t)0x400u)
#define DMA_ERR_ERR11		((uint32_t)0x800u)
#define DMA_ERR_ERR12		((uint32_t)0x1000u)
#define DMA_ERR_ERR13		((uint32_t)0x2000u)
#define DMA_ERR_ERR14		((uint32_t)0x4000u)
#define DMA_ERR_ERR15		((uint32_t)0x8000u)
/* HRS Bit Fields */
#define DMA_HRS_HRS0		((uint32_t)0x1u)
#define DMA_HRS_HRS1		((uint32_t)0x2u)
#define DMA_HRS_HRS2		((uint32_t)0x4u)
#define DMA_HRS_HRS3		((uint32_t)0x8u)
#define DMA_HRS_HRS4		((uint32_t)0x10u)
#define DMA_HRS_HRS5		((uint32_t)0x20u)
#define DMA_HRS_HRS6		((uint32_t)0x40u)
#define DMA_HRS_HRS7		((uint32_t)0x80u)
#define DMA_HRS_HRS8		((uint32_t)0x100u)
#define DMA_HRS_HRS9		((uint32_t)0x200u)
#define DMA_HRS_HRS10		((uint32_t)0x400u)
#define DMA_HRS_HRS11		((uint32_t)0x800u)
#define DMA_HRS_HRS12		((uint32_t)0x1000u)
#define DMA_HRS_HRS13		((uint32_t)0x2000u)
#define DMA_HRS_HRS14		((uint32_t)0x4000u)
#define DMA_HRS_HRS15		((uint32_t)0x8000u)
/* EARS Bit Fields */
#define DMA_EARS_EDREQ_0		((uint32_t)0x1u)
#define DMA_EARS_EDREQ_1		((uint32_t)0x2u)
#define DMA_EARS_EDREQ_2		((uint32_t)0x4u)
#define DMA_EARS_EDREQ_3		((uint32_t)0x8u)
#define DMA_EARS_EDREQ_4		((uint32_t)0x10u)
#define DMA_EARS_EDREQ_5		((uint32_t)0x20u)
#define DMA_EARS_EDREQ_6		((uint32_t)0x40u)
#define DMA_EARS_EDREQ_7		((uint32_t)0x80u)
#define DMA_EARS_EDREQ_8		((uint32_t)0x100u)
#define DMA_EARS_EDREQ_9		((uint32_t)0x200u)
#define DMA_EARS_EDREQ_10		((uint32_t)0x400u)
#define DMA_EARS_EDREQ_11		((uint32_t)0x800u)
#define DMA_EARS_EDREQ_12		((uint32_t)0x1000u)
#define DMA_EARS_EDREQ_13		((uint32_t)0x2000u)
#define DMA_EARS_EDREQ_14		((uint32_t)0x4000u)
#define DMA_EARS_EDREQ_15		((uint32_t)0x8000u)
/* DCHPRI3 Bit Fields */
#define DMA_DCHPRI3_CHPRI_MASK                   0xFu
#define DMA_DCHPRI3_CHPRI_SHIFT                  0
#define DMA_DCHPRI3_CHPRI_WIDTH                  4
#define DMA_DCHPRI3_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI3_CHPRI_SHIFT))&DMA_DCHPRI3_CHPRI_MASK)
#define DMA_DCHPRI3_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI3_ECP		((uint8_t)0x80u)
/* DCHPRI2 Bit Fields */
#define DMA_DCHPRI2_CHPRI_MASK                   0xFu
#define DMA_DCHPRI2_CHPRI_SHIFT                  0
#define DMA_DCHPRI2_CHPRI_WIDTH                  4
#define DMA_DCHPRI2_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI2_CHPRI_SHIFT))&DMA_DCHPRI2_CHPRI_MASK)
#define DMA_DCHPRI2_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI2_ECP		((uint8_t)0x80u)
/* DCHPRI1 Bit Fields */
#define DMA_DCHPRI1_CHPRI_MASK                   0xFu
#define DMA_DCHPRI1_CHPRI_SHIFT                  0
#define DMA_DCHPRI1_CHPRI_WIDTH                  4
#define DMA_DCHPRI1_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI1_CHPRI_SHIFT))&DMA_DCHPRI1_CHPRI_MASK)
#define DMA_DCHPRI1_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI1_ECP		((uint8_t)0x80u)
/* DCHPRI0 Bit Fields */
#define DMA_DCHPRI0_CHPRI_MASK                   0xFu
#define DMA_DCHPRI0_CHPRI_SHIFT                  0
#define DMA_DCHPRI0_CHPRI_WIDTH                  4
#define DMA_DCHPRI0_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI0_CHPRI_SHIFT))&DMA_DCHPRI0_CHPRI_MASK)
#define DMA_DCHPRI0_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI0_ECP		((uint8_t)0x80u)
/* DCHPRI7 Bit Fields */
#define DMA_DCHPRI7_CHPRI_MASK                   0xFu
#define DMA_DCHPRI7_CHPRI_SHIFT                  0
#define DMA_DCHPRI7_CHPRI_WIDTH                  4
#define DMA_DCHPRI7_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI7_CHPRI_SHIFT))&DMA_DCHPRI7_CHPRI_MASK)
#define DMA_DCHPRI7_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI7_ECP		((uint8_t)0x80u)
/* DCHPRI6 Bit Fields */
#define DMA_DCHPRI6_CHPRI_MASK                   0xFu
#define DMA_DCHPRI6_CHPRI_SHIFT                  0
#define DMA_DCHPRI6_CHPRI_WIDTH                  4
#define DMA_DCHPRI6_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI6_CHPRI_SHIFT))&DMA_DCHPRI6_CHPRI_MASK)
#define DMA_DCHPRI6_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI6_ECP		((uint8_t)0x80u)
/* DCHPRI5 Bit Fields */
#define DMA_DCHPRI5_CHPRI_MASK                   0xFu
#define DMA_DCHPRI5_CHPRI_SHIFT                  0
#define DMA_DCHPRI5_CHPRI_WIDTH                  4
#define DMA_DCHPRI5_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI5_CHPRI_SHIFT))&DMA_DCHPRI5_CHPRI_MASK)
#define DMA_DCHPRI5_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI5_ECP		((uint8_t)0x80u)
/* DCHPRI4 Bit Fields */
#define DMA_DCHPRI4_CHPRI_MASK                   0xFu
#define DMA_DCHPRI4_CHPRI_SHIFT                  0
#define DMA_DCHPRI4_CHPRI_WIDTH                  4
#define DMA_DCHPRI4_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI4_CHPRI_SHIFT))&DMA_DCHPRI4_CHPRI_MASK)
#define DMA_DCHPRI4_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI4_ECP		((uint8_t)0x80u)
/* DCHPRI11 Bit Fields */
#define DMA_DCHPRI11_CHPRI_MASK                  0xFu
#define DMA_DCHPRI11_CHPRI_SHIFT                 0
#define DMA_DCHPRI11_CHPRI_WIDTH                 4
#define DMA_DCHPRI11_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI11_CHPRI_SHIFT))&DMA_DCHPRI11_CHPRI_MASK)
#define DMA_DCHPRI11_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI11_ECP		((uint8_t)0x80u)
/* DCHPRI10 Bit Fields */
#define DMA_DCHPRI10_CHPRI_MASK                  0xFu
#define DMA_DCHPRI10_CHPRI_SHIFT                 0
#define DMA_DCHPRI10_CHPRI_WIDTH                 4
#define DMA_DCHPRI10_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI10_CHPRI_SHIFT))&DMA_DCHPRI10_CHPRI_MASK)
#define DMA_DCHPRI10_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI10_ECP		((uint8_t)0x80u)
/* DCHPRI9 Bit Fields */
#define DMA_DCHPRI9_CHPRI_MASK                   0xFu
#define DMA_DCHPRI9_CHPRI_SHIFT                  0
#define DMA_DCHPRI9_CHPRI_WIDTH                  4
#define DMA_DCHPRI9_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI9_CHPRI_SHIFT))&DMA_DCHPRI9_CHPRI_MASK)
#define DMA_DCHPRI9_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI9_ECP		((uint8_t)0x80u)
/* DCHPRI8 Bit Fields */
#define DMA_DCHPRI8_CHPRI_MASK                   0xFu
#define DMA_DCHPRI8_CHPRI_SHIFT                  0
#define DMA_DCHPRI8_CHPRI_WIDTH                  4
#define DMA_DCHPRI8_CHPRI(x)                     (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI8_CHPRI_SHIFT))&DMA_DCHPRI8_CHPRI_MASK)
#define DMA_DCHPRI8_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI8_ECP		((uint8_t)0x80u)
/* DCHPRI15 Bit Fields */
#define DMA_DCHPRI15_CHPRI_MASK                  0xFu
#define DMA_DCHPRI15_CHPRI_SHIFT                 0
#define DMA_DCHPRI15_CHPRI_WIDTH                 4
#define DMA_DCHPRI15_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI15_CHPRI_SHIFT))&DMA_DCHPRI15_CHPRI_MASK)
#define DMA_DCHPRI15_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI15_ECP		((uint8_t)0x80u)
/* DCHPRI14 Bit Fields */
#define DMA_DCHPRI14_CHPRI_MASK                  0xFu
#define DMA_DCHPRI14_CHPRI_SHIFT                 0
#define DMA_DCHPRI14_CHPRI_WIDTH                 4
#define DMA_DCHPRI14_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI14_CHPRI_SHIFT))&DMA_DCHPRI14_CHPRI_MASK)
#define DMA_DCHPRI14_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI14_ECP		((uint8_t)0x80u)
/* DCHPRI13 Bit Fields */
#define DMA_DCHPRI13_CHPRI_MASK                  0xFu
#define DMA_DCHPRI13_CHPRI_SHIFT                 0
#define DMA_DCHPRI13_CHPRI_WIDTH                 4
#define DMA_DCHPRI13_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI13_CHPRI_SHIFT))&DMA_DCHPRI13_CHPRI_MASK)
#define DMA_DCHPRI13_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI13_ECP		((uint8_t)0x80u)
/* DCHPRI12 Bit Fields */
#define DMA_DCHPRI12_CHPRI_MASK                  0xFu
#define DMA_DCHPRI12_CHPRI_SHIFT                 0
#define DMA_DCHPRI12_CHPRI_WIDTH                 4
#define DMA_DCHPRI12_CHPRI(x)                    (((uint8_t)(((uint8_t)(x))<<DMA_DCHPRI12_CHPRI_SHIFT))&DMA_DCHPRI12_CHPRI_MASK)
#define DMA_DCHPRI12_DPA		((uint8_t)0x40u)
#define DMA_DCHPRI12_ECP		((uint8_t)0x80u)
/* SADDR Bit Fields */
#define DMA_SADDR_SADDR_MASK                     0xFFFFFFFFu
#define DMA_SADDR_SADDR_SHIFT                    0
#define DMA_SADDR_SADDR_WIDTH                    32
#define DMA_SADDR_SADDR(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_SADDR_SADDR_SHIFT))&DMA_SADDR_SADDR_MASK)
/* SOFF Bit Fields */
#define DMA_SOFF_SOFF_MASK                       0xFFFFu
#define DMA_SOFF_SOFF_SHIFT                      0
#define DMA_SOFF_SOFF_WIDTH                      16
#define DMA_SOFF_SOFF(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_SOFF_SOFF_SHIFT))&DMA_SOFF_SOFF_MASK)
/* ATTR Bit Fields */
#define DMA_ATTR_DSIZE_MASK                      0x7u
#define DMA_ATTR_DSIZE_SHIFT                     0
#define DMA_ATTR_DSIZE_WIDTH                     3
#define DMA_ATTR_DSIZE(x)                        (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_DSIZE_SHIFT))&DMA_ATTR_DSIZE_MASK)
#define DMA_ATTR_DMOD_MASK                       0xF8u
#define DMA_ATTR_DMOD_SHIFT                      3
#define DMA_ATTR_DMOD_WIDTH                      5
#define DMA_ATTR_DMOD(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_DMOD_SHIFT))&DMA_ATTR_DMOD_MASK)
#define DMA_ATTR_SSIZE_MASK                      0x700u
#define DMA_ATTR_SSIZE_SHIFT                     8
#define DMA_ATTR_SSIZE_WIDTH                     3
#define DMA_ATTR_SSIZE(x)                        (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_SSIZE_SHIFT))&DMA_ATTR_SSIZE_MASK)
#define DMA_ATTR_SMOD_MASK                       0xF800u
#define DMA_ATTR_SMOD_SHIFT                      11
#define DMA_ATTR_SMOD_WIDTH                      5
#define DMA_ATTR_SMOD(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_ATTR_SMOD_SHIFT))&DMA_ATTR_SMOD_MASK)
/* NBYTES_MLNO Bit Fields */
#define DMA_NBYTES_MLNO_NBYTES_MASK              0xFFFFFFFFu
#define DMA_NBYTES_MLNO_NBYTES_SHIFT             0
#define DMA_NBYTES_MLNO_NBYTES_WIDTH             32
#define DMA_NBYTES_MLNO_NBYTES(x)                (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLNO_NBYTES_SHIFT))&DMA_NBYTES_MLNO_NBYTES_MASK)
/* NBYTES_MLOFFNO Bit Fields */
#define DMA_NBYTES_MLOFFNO_NBYTES_MASK           0x3FFFFFFFu
#define DMA_NBYTES_MLOFFNO_NBYTES_SHIFT          0
#define DMA_NBYTES_MLOFFNO_NBYTES_WIDTH          30
#define DMA_NBYTES_MLOFFNO_NBYTES(x)             (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFNO_NBYTES_SHIFT))&DMA_NBYTES_MLOFFNO_NBYTES_MASK)
#define DMA_NBYTES_MLOFFNO_DMLOE		((uint32_t)0x40000000u)
#define DMA_NBYTES_MLOFFNO_SMLOE		((uint32_t)0x80000000u)
/* NBYTES_MLOFFYES Bit Fields */
#define DMA_NBYTES_MLOFFYES_NBYTES_MASK          0x3FFu
#define DMA_NBYTES_MLOFFYES_NBYTES_SHIFT         0
#define DMA_NBYTES_MLOFFYES_NBYTES_WIDTH         10
#define DMA_NBYTES_MLOFFYES_NBYTES(x)            (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFYES_NBYTES_SHIFT))&DMA_NBYTES_MLOFFYES_NBYTES_MASK)
#define DMA_NBYTES_MLOFFYES_MLOFF_MASK           0x3FFFFC00u
#define DMA_NBYTES_MLOFFYES_MLOFF_SHIFT          10
#define DMA_NBYTES_MLOFFYES_MLOFF_WIDTH          20
#define DMA_NBYTES_MLOFFYES_MLOFF(x)             (((uint32_t)(((uint32_t)(x))<<DMA_NBYTES_MLOFFYES_MLOFF_SHIFT))&DMA_NBYTES_MLOFFYES_MLOFF_MASK)
#define DMA_NBYTES_MLOFFYES_DMLOE		((uint32_t)0x40000000u)
#define DMA_NBYTES_MLOFFYES_SMLOE		((uint32_t)0x80000000u)
/* SLAST Bit Fields */
#define DMA_SLAST_SLAST_MASK                     0xFFFFFFFFu
#define DMA_SLAST_SLAST_SHIFT                    0
#define DMA_SLAST_SLAST_WIDTH                    32
#define DMA_SLAST_SLAST(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_SLAST_SLAST_SHIFT))&DMA_SLAST_SLAST_MASK)
/* DADDR Bit Fields */
#define DMA_DADDR_DADDR_MASK                     0xFFFFFFFFu
#define DMA_DADDR_DADDR_SHIFT                    0
#define DMA_DADDR_DADDR_WIDTH                    32
#define DMA_DADDR_DADDR(x)                       (((uint32_t)(((uint32_t)(x))<<DMA_DADDR_DADDR_SHIFT))&DMA_DADDR_DADDR_MASK)
/* DOFF Bit Fields */
#define DMA_DOFF_DOFF_MASK                       0xFFFFu
#define DMA_DOFF_DOFF_SHIFT                      0
#define DMA_DOFF_DOFF_WIDTH                      16
#define DMA_DOFF_DOFF(x)                         (((uint16_t)(((uint16_t)(x))<<DMA_DOFF_DOFF_SHIFT))&DMA_DOFF_DOFF_MASK)
/* CITER_ELINKNO Bit Fields */
#define DMA_CITER_ELINKNO_CITER_MASK             0x7FFFu
#define DMA_CITER_ELINKNO_CITER_SHIFT            0
#define DMA_CITER_ELINKNO_CITER_WIDTH            15
#define DMA_CITER_ELINKNO_CITER(x)               (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKNO_CITER_SHIFT))&DMA_CITER_ELINKNO_CITER_MASK)
#define DMA_CITER_ELINKNO_ELINK		((uint16_t)0x8000u)
/* CITER_ELINKYES Bit Fields */
#define DMA_CITER_ELINKYES_CITER_MASK            0x1FFu
#define DMA_CITER_ELINKYES_CITER_SHIFT           0
#define DMA_CITER_ELINKYES_CITER_WIDTH           9
#define DMA_CITER_ELINKYES_CITER(x)              (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKYES_CITER_SHIFT))&DMA_CITER_ELINKYES_CITER_MASK)
#define DMA_CITER_ELINKYES_LINKCH_MASK           0x1E00u
#define DMA_CITER_ELINKYES_LINKCH_SHIFT          9
#define DMA_CITER_ELINKYES_LINKCH_WIDTH          4
#define DMA_CITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x))<<DMA_CITER_ELINKYES_LINKCH_SHIFT))&DMA_CITER_ELINKYES_LINKCH_MASK)
#define DMA_CITER_ELINKYES_ELINK		((uint16_t)0x8000u)
/* DLAST_SGA Bit Fields */
#define DMA_DLAST_SGA_DLASTSGA_MASK              0xFFFFFFFFu
#define DMA_DLAST_SGA_DLASTSGA_SHIFT             0
#define DMA_DLAST_SGA_DLASTSGA_WIDTH             32
#define DMA_DLAST_SGA_DLASTSGA(x)                (((uint32_t)(((uint32_t)(x))<<DMA_DLAST_SGA_DLASTSGA_SHIFT))&DMA_DLAST_SGA_DLASTSGA_MASK)
/* CSR Bit Fields */
#define DMA_CSR_START_MASK                       0x1u
#define DMA_CSR_START_SHIFT                      0
#define DMA_CSR_INTMAJOR_MASK                    0x2u
#define DMA_CSR_INTMAJOR_SHIFT                   1
#define DMA_CSR_INTHALF_MASK                     0x4u
#define DMA_CSR_INTHALF_SHIFT                    2
#define DMA_CSR_DREQ_MASK                        0x8u
#define DMA_CSR_DREQ_SHIFT                       3
#define DMA_CSR_ESG_MASK                         0x10u
#define DMA_CSR_ESG_SHIFT                        4
#define DMA_CSR_MAJORELINK_MASK                  0x20u
#define DMA_CSR_MAJORELINK_SHIFT                 5
#define DMA_CSR_ACTIVE_MASK                      0x40u
#define DMA_CSR_ACTIVE_SHIFT                     6
#define DMA_CSR_DONE_MASK                        0x80u
#define DMA_CSR_DONE_SHIFT                       7
#define DMA_CSR_MAJORLINKCH_MASK                 0xF00u
#define DMA_CSR_MAJORLINKCH_SHIFT                8
#define DMA_CSR_START		((uint16_t)0x1u)
#define DMA_CSR_INTMAJOR		((uint16_t)0x2u)
#define DMA_CSR_INTHALF		((uint16_t)0x4u)
#define DMA_CSR_DREQ		((uint16_t)0x8u)
#define DMA_CSR_ESG		((uint16_t)0x10u)
#define DMA_CSR_MAJORELINK		((uint16_t)0x20u)
#define DMA_CSR_ACTIVE		((uint16_t)0x40u)
#define DMA_CSR_DONE		((uint16_t)0x80u)
#define DMA_CSR_MAJORLINKCH_MASK                 0xF00u
#define DMA_CSR_MAJORLINKCH_SHIFT                8
#define DMA_CSR_MAJORLINKCH_WIDTH                4
#define DMA_CSR_MAJORLINKCH(x)                   (((uint16_t)(((uint16_t)(x))<<DMA_CSR_MAJORLINKCH_SHIFT))&DMA_CSR_MAJORLINKCH_MASK)
#define DMA_CSR_BWC_MASK                         0xC000u
#define DMA_CSR_BWC_SHIFT                        14
#define DMA_CSR_BWC_WIDTH                        2
#define DMA_CSR_BWC(x)                           (((uint16_t)(((uint16_t)(x))<<DMA_CSR_BWC_SHIFT))&DMA_CSR_BWC_MASK)
/* BITER_ELINKNO Bit Fields */
#define DMA_BITER_ELINKNO_BITER_MASK             0x7FFFu
#define DMA_BITER_ELINKNO_BITER_SHIFT            0
#define DMA_BITER_ELINKNO_BITER_WIDTH            15
#define DMA_BITER_ELINKNO_BITER(x)               (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKNO_BITER_SHIFT))&DMA_BITER_ELINKNO_BITER_MASK)
#define DMA_BITER_ELINKNO_ELINK		((uint16_t)0x8000u)
/* BITER_ELINKYES Bit Fields */
#define DMA_BITER_ELINKYES_BITER_MASK            0x1FFu
#define DMA_BITER_ELINKYES_BITER_SHIFT           0
#define DMA_BITER_ELINKYES_BITER_WIDTH           9
#define DMA_BITER_ELINKYES_BITER(x)              (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_BITER_SHIFT))&DMA_BITER_ELINKYES_BITER_MASK)
#define DMA_BITER_ELINKYES_LINKCH_MASK           0x1E00u
#define DMA_BITER_ELINKYES_LINKCH_SHIFT          9
#define DMA_BITER_ELINKYES_LINKCH_WIDTH          4
#define DMA_BITER_ELINKYES_LINKCH(x)             (((uint16_t)(((uint16_t)(x))<<DMA_BITER_ELINKYES_LINKCH_SHIFT))&DMA_BITER_ELINKYES_LINKCH_MASK)
#define DMA_BITER_ELINKYES_ELINK		((uint16_t)0x8000u)

/*!
 * @}
 */ /* end of group DMA_Register_Masks */


/* DMA - Peripheral instance base addresses */
/** Peripheral DMA base address */
#define DMA_BASE                                 (0x40008000u)
/** Peripheral DMA base pointer */
#define DMA0                                     ((DMA_TypeDef *)DMA_BASE)
#define DMA                                      ((DMA_TypeDef *)DMA_BASE) // for chibios compatibility
#define DMA_BASE_PTR                             (DMA0)
/** Array initializer of DMA peripheral base addresses */
#define DMA_BASE_ADDRS                           { DMA_BASE }
/** Array initializer of DMA peripheral base pointers */
#define DMA_BASE_PTRS                            { DMA0 }
/** Interrupt vectors for the DMA peripheral type */
#define DMA_CHN_IRQS                             { DMA0_IRQn, DMA1_IRQn, DMA2_IRQn, DMA3_IRQn, DMA4_IRQn, DMA5_IRQn, DMA6_IRQn, DMA7_IRQn, DMA8_IRQn, DMA9_IRQn, DMA10_IRQn, DMA11_IRQn, DMA12_IRQn, DMA13_IRQn, DMA14_IRQn, DMA15_IRQn }
#define DMA_ERROR_IRQS                           { DMA_Error_IRQn }

/* ----------------------------------------------------------------------------
   -- DMA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMA_Register_Accessor_Macros DMA - Register accessor macros
 * @{
 */


/* DMA - Register instance definitions */
/* DMA */
#define DMA_CR                                   DMA_CR_REG(DMA0)
#define DMA_ES                                   DMA_ES_REG(DMA0)
#define DMA_ERQ                                  DMA_ERQ_REG(DMA0)
#define DMA_EEI                                  DMA_EEI_REG(DMA0)
#define DMA_CEEI                                 DMA_CEEI_REG(DMA0)
#define DMA_SEEI                                 DMA_SEEI_REG(DMA0)
#define DMA_CERQ                                 DMA_CERQ_REG(DMA0)
#define DMA_SERQ                                 DMA_SERQ_REG(DMA0)
#define DMA_CDNE                                 DMA_CDNE_REG(DMA0)
#define DMA_SSRT                                 DMA_SSRT_REG(DMA0)
#define DMA_CERR                                 DMA_CERR_REG(DMA0)
#define DMA_CINT                                 DMA_CINT_REG(DMA0)
#define DMA_INT                                  DMA_INT_REG(DMA0)
#define DMA_ERR                                  DMA_ERR_REG(DMA0)
#define DMA_HRS                                  DMA_HRS_REG(DMA0)
#define DMA_EARS                                 DMA_EARS_REG(DMA0)
#define DMA_DCHPRI3                              DMA_DCHPRI3_REG(DMA0)
#define DMA_DCHPRI2                              DMA_DCHPRI2_REG(DMA0)
#define DMA_DCHPRI1                              DMA_DCHPRI1_REG(DMA0)
#define DMA_DCHPRI0                              DMA_DCHPRI0_REG(DMA0)
#define DMA_DCHPRI7                              DMA_DCHPRI7_REG(DMA0)
#define DMA_DCHPRI6                              DMA_DCHPRI6_REG(DMA0)
#define DMA_DCHPRI5                              DMA_DCHPRI5_REG(DMA0)
#define DMA_DCHPRI4                              DMA_DCHPRI4_REG(DMA0)
#define DMA_DCHPRI11                             DMA_DCHPRI11_REG(DMA0)
#define DMA_DCHPRI10                             DMA_DCHPRI10_REG(DMA0)
#define DMA_DCHPRI9                              DMA_DCHPRI9_REG(DMA0)
#define DMA_DCHPRI8                              DMA_DCHPRI8_REG(DMA0)
#define DMA_DCHPRI15                             DMA_DCHPRI15_REG(DMA0)
#define DMA_DCHPRI14                             DMA_DCHPRI14_REG(DMA0)
#define DMA_DCHPRI13                             DMA_DCHPRI13_REG(DMA0)
#define DMA_DCHPRI12                             DMA_DCHPRI12_REG(DMA0)
#define DMA_TCD0_SADDR                           DMA_SADDR_REG(DMA0,0)
#define DMA_TCD0_SOFF                            DMA_SOFF_REG(DMA0,0)
#define DMA_TCD0_ATTR                            DMA_ATTR_REG(DMA0,0)
#define DMA_TCD0_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,0)
#define DMA_TCD0_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,0)
#define DMA_TCD0_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,0)
#define DMA_TCD0_SLAST                           DMA_SLAST_REG(DMA0,0)
#define DMA_TCD0_DADDR                           DMA_DADDR_REG(DMA0,0)
#define DMA_TCD0_DOFF                            DMA_DOFF_REG(DMA0,0)
#define DMA_TCD0_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,0)
#define DMA_TCD0_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,0)
#define DMA_TCD0_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,0)
#define DMA_TCD0_CSR                             DMA_CSR_REG(DMA0,0)
#define DMA_TCD0_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,0)
#define DMA_TCD0_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,0)
#define DMA_TCD1_SADDR                           DMA_SADDR_REG(DMA0,1)
#define DMA_TCD1_SOFF                            DMA_SOFF_REG(DMA0,1)
#define DMA_TCD1_ATTR                            DMA_ATTR_REG(DMA0,1)
#define DMA_TCD1_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,1)
#define DMA_TCD1_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,1)
#define DMA_TCD1_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,1)
#define DMA_TCD1_SLAST                           DMA_SLAST_REG(DMA0,1)
#define DMA_TCD1_DADDR                           DMA_DADDR_REG(DMA0,1)
#define DMA_TCD1_DOFF                            DMA_DOFF_REG(DMA0,1)
#define DMA_TCD1_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,1)
#define DMA_TCD1_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,1)
#define DMA_TCD1_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,1)
#define DMA_TCD1_CSR                             DMA_CSR_REG(DMA0,1)
#define DMA_TCD1_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,1)
#define DMA_TCD1_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,1)
#define DMA_TCD2_SADDR                           DMA_SADDR_REG(DMA0,2)
#define DMA_TCD2_SOFF                            DMA_SOFF_REG(DMA0,2)
#define DMA_TCD2_ATTR                            DMA_ATTR_REG(DMA0,2)
#define DMA_TCD2_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,2)
#define DMA_TCD2_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,2)
#define DMA_TCD2_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,2)
#define DMA_TCD2_SLAST                           DMA_SLAST_REG(DMA0,2)
#define DMA_TCD2_DADDR                           DMA_DADDR_REG(DMA0,2)
#define DMA_TCD2_DOFF                            DMA_DOFF_REG(DMA0,2)
#define DMA_TCD2_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,2)
#define DMA_TCD2_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,2)
#define DMA_TCD2_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,2)
#define DMA_TCD2_CSR                             DMA_CSR_REG(DMA0,2)
#define DMA_TCD2_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,2)
#define DMA_TCD2_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,2)
#define DMA_TCD3_SADDR                           DMA_SADDR_REG(DMA0,3)
#define DMA_TCD3_SOFF                            DMA_SOFF_REG(DMA0,3)
#define DMA_TCD3_ATTR                            DMA_ATTR_REG(DMA0,3)
#define DMA_TCD3_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,3)
#define DMA_TCD3_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,3)
#define DMA_TCD3_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,3)
#define DMA_TCD3_SLAST                           DMA_SLAST_REG(DMA0,3)
#define DMA_TCD3_DADDR                           DMA_DADDR_REG(DMA0,3)
#define DMA_TCD3_DOFF                            DMA_DOFF_REG(DMA0,3)
#define DMA_TCD3_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,3)
#define DMA_TCD3_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,3)
#define DMA_TCD3_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,3)
#define DMA_TCD3_CSR                             DMA_CSR_REG(DMA0,3)
#define DMA_TCD3_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,3)
#define DMA_TCD3_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,3)
#define DMA_TCD4_SADDR                           DMA_SADDR_REG(DMA0,4)
#define DMA_TCD4_SOFF                            DMA_SOFF_REG(DMA0,4)
#define DMA_TCD4_ATTR                            DMA_ATTR_REG(DMA0,4)
#define DMA_TCD4_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,4)
#define DMA_TCD4_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,4)
#define DMA_TCD4_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,4)
#define DMA_TCD4_SLAST                           DMA_SLAST_REG(DMA0,4)
#define DMA_TCD4_DADDR                           DMA_DADDR_REG(DMA0,4)
#define DMA_TCD4_DOFF                            DMA_DOFF_REG(DMA0,4)
#define DMA_TCD4_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,4)
#define DMA_TCD4_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,4)
#define DMA_TCD4_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,4)
#define DMA_TCD4_CSR                             DMA_CSR_REG(DMA0,4)
#define DMA_TCD4_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,4)
#define DMA_TCD4_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,4)
#define DMA_TCD5_SADDR                           DMA_SADDR_REG(DMA0,5)
#define DMA_TCD5_SOFF                            DMA_SOFF_REG(DMA0,5)
#define DMA_TCD5_ATTR                            DMA_ATTR_REG(DMA0,5)
#define DMA_TCD5_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,5)
#define DMA_TCD5_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,5)
#define DMA_TCD5_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,5)
#define DMA_TCD5_SLAST                           DMA_SLAST_REG(DMA0,5)
#define DMA_TCD5_DADDR                           DMA_DADDR_REG(DMA0,5)
#define DMA_TCD5_DOFF                            DMA_DOFF_REG(DMA0,5)
#define DMA_TCD5_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,5)
#define DMA_TCD5_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,5)
#define DMA_TCD5_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,5)
#define DMA_TCD5_CSR                             DMA_CSR_REG(DMA0,5)
#define DMA_TCD5_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,5)
#define DMA_TCD5_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,5)
#define DMA_TCD6_SADDR                           DMA_SADDR_REG(DMA0,6)
#define DMA_TCD6_SOFF                            DMA_SOFF_REG(DMA0,6)
#define DMA_TCD6_ATTR                            DMA_ATTR_REG(DMA0,6)
#define DMA_TCD6_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,6)
#define DMA_TCD6_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,6)
#define DMA_TCD6_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,6)
#define DMA_TCD6_SLAST                           DMA_SLAST_REG(DMA0,6)
#define DMA_TCD6_DADDR                           DMA_DADDR_REG(DMA0,6)
#define DMA_TCD6_DOFF                            DMA_DOFF_REG(DMA0,6)
#define DMA_TCD6_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,6)
#define DMA_TCD6_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,6)
#define DMA_TCD6_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,6)
#define DMA_TCD6_CSR                             DMA_CSR_REG(DMA0,6)
#define DMA_TCD6_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,6)
#define DMA_TCD6_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,6)
#define DMA_TCD7_SADDR                           DMA_SADDR_REG(DMA0,7)
#define DMA_TCD7_SOFF                            DMA_SOFF_REG(DMA0,7)
#define DMA_TCD7_ATTR                            DMA_ATTR_REG(DMA0,7)
#define DMA_TCD7_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,7)
#define DMA_TCD7_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,7)
#define DMA_TCD7_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,7)
#define DMA_TCD7_SLAST                           DMA_SLAST_REG(DMA0,7)
#define DMA_TCD7_DADDR                           DMA_DADDR_REG(DMA0,7)
#define DMA_TCD7_DOFF                            DMA_DOFF_REG(DMA0,7)
#define DMA_TCD7_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,7)
#define DMA_TCD7_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,7)
#define DMA_TCD7_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,7)
#define DMA_TCD7_CSR                             DMA_CSR_REG(DMA0,7)
#define DMA_TCD7_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,7)
#define DMA_TCD7_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,7)
#define DMA_TCD8_SADDR                           DMA_SADDR_REG(DMA0,8)
#define DMA_TCD8_SOFF                            DMA_SOFF_REG(DMA0,8)
#define DMA_TCD8_ATTR                            DMA_ATTR_REG(DMA0,8)
#define DMA_TCD8_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,8)
#define DMA_TCD8_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,8)
#define DMA_TCD8_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,8)
#define DMA_TCD8_SLAST                           DMA_SLAST_REG(DMA0,8)
#define DMA_TCD8_DADDR                           DMA_DADDR_REG(DMA0,8)
#define DMA_TCD8_DOFF                            DMA_DOFF_REG(DMA0,8)
#define DMA_TCD8_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,8)
#define DMA_TCD8_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,8)
#define DMA_TCD8_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,8)
#define DMA_TCD8_CSR                             DMA_CSR_REG(DMA0,8)
#define DMA_TCD8_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,8)
#define DMA_TCD8_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,8)
#define DMA_TCD9_SADDR                           DMA_SADDR_REG(DMA0,9)
#define DMA_TCD9_SOFF                            DMA_SOFF_REG(DMA0,9)
#define DMA_TCD9_ATTR                            DMA_ATTR_REG(DMA0,9)
#define DMA_TCD9_NBYTES_MLNO                     DMA_NBYTES_MLNO_REG(DMA0,9)
#define DMA_TCD9_NBYTES_MLOFFNO                  DMA_NBYTES_MLOFFNO_REG(DMA0,9)
#define DMA_TCD9_NBYTES_MLOFFYES                 DMA_NBYTES_MLOFFYES_REG(DMA0,9)
#define DMA_TCD9_SLAST                           DMA_SLAST_REG(DMA0,9)
#define DMA_TCD9_DADDR                           DMA_DADDR_REG(DMA0,9)
#define DMA_TCD9_DOFF                            DMA_DOFF_REG(DMA0,9)
#define DMA_TCD9_CITER_ELINKNO                   DMA_CITER_ELINKNO_REG(DMA0,9)
#define DMA_TCD9_CITER_ELINKYES                  DMA_CITER_ELINKYES_REG(DMA0,9)
#define DMA_TCD9_DLASTSGA                        DMA_DLAST_SGA_REG(DMA0,9)
#define DMA_TCD9_CSR                             DMA_CSR_REG(DMA0,9)
#define DMA_TCD9_BITER_ELINKNO                   DMA_BITER_ELINKNO_REG(DMA0,9)
#define DMA_TCD9_BITER_ELINKYES                  DMA_BITER_ELINKYES_REG(DMA0,9)
#define DMA_TCD10_SADDR                          DMA_SADDR_REG(DMA0,10)
#define DMA_TCD10_SOFF                           DMA_SOFF_REG(DMA0,10)
#define DMA_TCD10_ATTR                           DMA_ATTR_REG(DMA0,10)
#define DMA_TCD10_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,10)
#define DMA_TCD10_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,10)
#define DMA_TCD10_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,10)
#define DMA_TCD10_SLAST                          DMA_SLAST_REG(DMA0,10)
#define DMA_TCD10_DADDR                          DMA_DADDR_REG(DMA0,10)
#define DMA_TCD10_DOFF                           DMA_DOFF_REG(DMA0,10)
#define DMA_TCD10_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,10)
#define DMA_TCD10_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,10)
#define DMA_TCD10_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,10)
#define DMA_TCD10_CSR                            DMA_CSR_REG(DMA0,10)
#define DMA_TCD10_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,10)
#define DMA_TCD10_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,10)
#define DMA_TCD11_SADDR                          DMA_SADDR_REG(DMA0,11)
#define DMA_TCD11_SOFF                           DMA_SOFF_REG(DMA0,11)
#define DMA_TCD11_ATTR                           DMA_ATTR_REG(DMA0,11)
#define DMA_TCD11_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,11)
#define DMA_TCD11_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,11)
#define DMA_TCD11_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,11)
#define DMA_TCD11_SLAST                          DMA_SLAST_REG(DMA0,11)
#define DMA_TCD11_DADDR                          DMA_DADDR_REG(DMA0,11)
#define DMA_TCD11_DOFF                           DMA_DOFF_REG(DMA0,11)
#define DMA_TCD11_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,11)
#define DMA_TCD11_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,11)
#define DMA_TCD11_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,11)
#define DMA_TCD11_CSR                            DMA_CSR_REG(DMA0,11)
#define DMA_TCD11_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,11)
#define DMA_TCD11_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,11)
#define DMA_TCD12_SADDR                          DMA_SADDR_REG(DMA0,12)
#define DMA_TCD12_SOFF                           DMA_SOFF_REG(DMA0,12)
#define DMA_TCD12_ATTR                           DMA_ATTR_REG(DMA0,12)
#define DMA_TCD12_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,12)
#define DMA_TCD12_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,12)
#define DMA_TCD12_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,12)
#define DMA_TCD12_SLAST                          DMA_SLAST_REG(DMA0,12)
#define DMA_TCD12_DADDR                          DMA_DADDR_REG(DMA0,12)
#define DMA_TCD12_DOFF                           DMA_DOFF_REG(DMA0,12)
#define DMA_TCD12_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,12)
#define DMA_TCD12_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,12)
#define DMA_TCD12_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,12)
#define DMA_TCD12_CSR                            DMA_CSR_REG(DMA0,12)
#define DMA_TCD12_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,12)
#define DMA_TCD12_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,12)
#define DMA_TCD13_SADDR                          DMA_SADDR_REG(DMA0,13)
#define DMA_TCD13_SOFF                           DMA_SOFF_REG(DMA0,13)
#define DMA_TCD13_ATTR                           DMA_ATTR_REG(DMA0,13)
#define DMA_TCD13_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,13)
#define DMA_TCD13_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,13)
#define DMA_TCD13_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,13)
#define DMA_TCD13_SLAST                          DMA_SLAST_REG(DMA0,13)
#define DMA_TCD13_DADDR                          DMA_DADDR_REG(DMA0,13)
#define DMA_TCD13_DOFF                           DMA_DOFF_REG(DMA0,13)
#define DMA_TCD13_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,13)
#define DMA_TCD13_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,13)
#define DMA_TCD13_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,13)
#define DMA_TCD13_CSR                            DMA_CSR_REG(DMA0,13)
#define DMA_TCD13_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,13)
#define DMA_TCD13_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,13)
#define DMA_TCD14_SADDR                          DMA_SADDR_REG(DMA0,14)
#define DMA_TCD14_SOFF                           DMA_SOFF_REG(DMA0,14)
#define DMA_TCD14_ATTR                           DMA_ATTR_REG(DMA0,14)
#define DMA_TCD14_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,14)
#define DMA_TCD14_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,14)
#define DMA_TCD14_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,14)
#define DMA_TCD14_SLAST                          DMA_SLAST_REG(DMA0,14)
#define DMA_TCD14_DADDR                          DMA_DADDR_REG(DMA0,14)
#define DMA_TCD14_DOFF                           DMA_DOFF_REG(DMA0,14)
#define DMA_TCD14_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,14)
#define DMA_TCD14_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,14)
#define DMA_TCD14_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,14)
#define DMA_TCD14_CSR                            DMA_CSR_REG(DMA0,14)
#define DMA_TCD14_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,14)
#define DMA_TCD14_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,14)
#define DMA_TCD15_SADDR                          DMA_SADDR_REG(DMA0,15)
#define DMA_TCD15_SOFF                           DMA_SOFF_REG(DMA0,15)
#define DMA_TCD15_ATTR                           DMA_ATTR_REG(DMA0,15)
#define DMA_TCD15_NBYTES_MLNO                    DMA_NBYTES_MLNO_REG(DMA0,15)
#define DMA_TCD15_NBYTES_MLOFFNO                 DMA_NBYTES_MLOFFNO_REG(DMA0,15)
#define DMA_TCD15_NBYTES_MLOFFYES                DMA_NBYTES_MLOFFYES_REG(DMA0,15)
#define DMA_TCD15_SLAST                          DMA_SLAST_REG(DMA0,15)
#define DMA_TCD15_DADDR                          DMA_DADDR_REG(DMA0,15)
#define DMA_TCD15_DOFF                           DMA_DOFF_REG(DMA0,15)
#define DMA_TCD15_CITER_ELINKNO                  DMA_CITER_ELINKNO_REG(DMA0,15)
#define DMA_TCD15_CITER_ELINKYES                 DMA_CITER_ELINKYES_REG(DMA0,15)
#define DMA_TCD15_DLASTSGA                       DMA_DLAST_SGA_REG(DMA0,15)
#define DMA_TCD15_CSR                            DMA_CSR_REG(DMA0,15)
#define DMA_TCD15_BITER_ELINKNO                  DMA_BITER_ELINKNO_REG(DMA0,15)
#define DMA_TCD15_BITER_ELINKYES                 DMA_BITER_ELINKYES_REG(DMA0,15)

/* DMA - Register array accessors */
#define DMA_SADDR(index)                         DMA_SADDR_REG(DMA0,index)
#define DMA_SOFF(index)                          DMA_SOFF_REG(DMA0,index)
#define DMA_ATTR(index)                          DMA_ATTR_REG(DMA0,index)
#define DMA_NBYTES_MLNO(index)                   DMA_NBYTES_MLNO_REG(DMA0,index)
#define DMA_NBYTES_MLOFFNO(index)                DMA_NBYTES_MLOFFNO_REG(DMA0,index)
#define DMA_NBYTES_MLOFFYES(index)               DMA_NBYTES_MLOFFYES_REG(DMA0,index)
#define DMA_SLAST(index)                         DMA_SLAST_REG(DMA0,index)
#define DMA_DADDR(index)                         DMA_DADDR_REG(DMA0,index)
#define DMA_DOFF(index)                          DMA_DOFF_REG(DMA0,index)
#define DMA_CITER_ELINKNO(index)                 DMA_CITER_ELINKNO_REG(DMA0,index)
#define DMA_CITER_ELINKYES(index)                DMA_CITER_ELINKYES_REG(DMA0,index)
#define DMA_DLAST_SGA(index)                     DMA_DLAST_SGA_REG(DMA0,index)
#define DMA_CSR(index)                           DMA_CSR_REG(DMA0,index)
#define DMA_BITER_ELINKNO(index)                 DMA_BITER_ELINKNO_REG(DMA0,index)
#define DMA_BITER_ELINKYES(index)                DMA_BITER_ELINKYES_REG(DMA0,index)

/*!
 * @}
 */ /* end of group DMA_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DMA_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- DMAMUX Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Peripheral_Access_Layer DMAMUX Peripheral Access Layer
 * @{
 */

/** DMAMUX - Register Layout Typedef */
typedef struct {
  __IO uint8_t CHCFG[16];                          /**< Channel Configuration register, array offset: 0x0, array step: 0x1 */
} DMAMUX_TypeDef, *DMAMUX_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- DMAMUX - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Accessor_Macros DMAMUX - Register accessor macros
 * @{
 */


/* DMAMUX - Register accessors */
#define DMAMUX_CHCFG_REG(base,index)             ((base)->CHCFG[index])
#define DMAMUX_CHCFG_COUNT                       16

/*!
 * @}
 */ /* end of group DMAMUX_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- DMAMUX Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Masks DMAMUX Register Masks
 * @{
 */

/* CHCFG Bit Fields */
#define DMAMUX_CHCFG_SOURCE_MASK                 0x3Fu
#define DMAMUX_CHCFG_SOURCE_SHIFT                0
#define DMAMUX_CHCFG_SOURCE_WIDTH                6
#define DMAMUX_CHCFG_SOURCE(x)                   (((uint8_t)(((uint8_t)(x))<<DMAMUX_CHCFG_SOURCE_SHIFT))&DMAMUX_CHCFG_SOURCE_MASK)
#define DMAMUX_CHCFG_TRIG		((uint8_t)0x40u)
#define DMAMUX_CHCFG_ENBL		((uint8_t)0x80u)

/********  Bits definition for DMAMUX_CHCFGn register  **********/
#define DMAMUX_CHCFGn_ENBL           ((uint8_t)((uint8_t)1 << 7))  /*!< DMA Channel Enable */
#define DMAMUX_CHCFGn_TRIG           ((uint8_t)((uint8_t)1 << 6))  /*!< DMA Channel Trigger Enable */
#define DMAMUX_CHCFGn_SOURCE_SHIFT   0                                                                                      /*!< DMA Channel Source (Slot) (shift) */
#define DMAMUX_CHCFGn_SOURCE_MASK    ((uint8_t)((uint8_t)0x3F << DMAMUX_CHCFGn_SOURCE_SHIFT))                               /*!< DMA Channel Source (Slot) (mask) */
#define DMAMUX_CHCFGn_SOURCE(x)      ((uint8_t)(((uint8_t)(x) << DMAMUX_CHCFGn_SOURCE_SHIFT) & DMAMUX_CHCFGn_SOURCE_MASK))  /*!< DMA Channel Source (Slot) */

/*!
 * @}
 */ /* end of group DMAMUX_Register_Masks */


/* DMAMUX - Peripheral instance base addresses */
/** Peripheral DMAMUX base address */
#define DMAMUX_BASE                              (0x40021000u)
/** Peripheral DMAMUX base pointer */
#define DMAMUX                                   ((DMAMUX_TypeDef *)DMAMUX_BASE)
#define DMAMUX_BASE_PTR                          (DMAMUX)
/** Array initializer of DMAMUX peripheral base addresses */
#define DMAMUX_BASE_ADDRS                        { DMAMUX_BASE }
/** Array initializer of DMAMUX peripheral base pointers */
#define DMAMUX_BASE_PTRS                         { DMAMUX }

/* ----------------------------------------------------------------------------
   -- DMAMUX - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup DMAMUX_Register_Accessor_Macros DMAMUX - Register accessor macros
 * @{
 */


/* DMAMUX - Register instance definitions */
/* DMAMUX */
#define DMAMUX_CHCFG0                            DMAMUX_CHCFG_REG(DMAMUX,0)
#define DMAMUX_CHCFG1                            DMAMUX_CHCFG_REG(DMAMUX,1)
#define DMAMUX_CHCFG2                            DMAMUX_CHCFG_REG(DMAMUX,2)
#define DMAMUX_CHCFG3                            DMAMUX_CHCFG_REG(DMAMUX,3)
#define DMAMUX_CHCFG4                            DMAMUX_CHCFG_REG(DMAMUX,4)
#define DMAMUX_CHCFG5                            DMAMUX_CHCFG_REG(DMAMUX,5)
#define DMAMUX_CHCFG6                            DMAMUX_CHCFG_REG(DMAMUX,6)
#define DMAMUX_CHCFG7                            DMAMUX_CHCFG_REG(DMAMUX,7)
#define DMAMUX_CHCFG8                            DMAMUX_CHCFG_REG(DMAMUX,8)
#define DMAMUX_CHCFG9                            DMAMUX_CHCFG_REG(DMAMUX,9)
#define DMAMUX_CHCFG10                           DMAMUX_CHCFG_REG(DMAMUX,10)
#define DMAMUX_CHCFG11                           DMAMUX_CHCFG_REG(DMAMUX,11)
#define DMAMUX_CHCFG12                           DMAMUX_CHCFG_REG(DMAMUX,12)
#define DMAMUX_CHCFG13                           DMAMUX_CHCFG_REG(DMAMUX,13)
#define DMAMUX_CHCFG14                           DMAMUX_CHCFG_REG(DMAMUX,14)
#define DMAMUX_CHCFG15                           DMAMUX_CHCFG_REG(DMAMUX,15)

/* DMAMUX - Register array accessors */
#define DMAMUX_CHCFG(index)                      DMAMUX_CHCFG_REG(DMAMUX,index)

/*!
 * @}
 */ /* end of group DMAMUX_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group DMAMUX_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- EWM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Peripheral_Access_Layer EWM Peripheral Access Layer
 * @{
 */

/** EWM - Register Layout Typedef */
typedef struct {
  __IO uint8_t CTRL;                               /**< Control Register, offset: 0x0 */
  __O  uint8_t SERV;                               /**< Service Register, offset: 0x1 */
  __IO uint8_t CMPL;                               /**< Compare Low Register, offset: 0x2 */
  __IO uint8_t CMPH;                               /**< Compare High Register, offset: 0x3 */
       uint8_t RESERVED_0[1];
  __IO uint8_t CLKPRESCALER;                       /**< Clock Prescaler Register, offset: 0x5 */
} EWM_TypeDef, *EWM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- EWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Accessor_Macros EWM - Register accessor macros
 * @{
 */


/* EWM - Register accessors */
#define EWM_CTRL_REG(base)                       ((base)->CTRL)
#define EWM_SERV_REG(base)                       ((base)->SERV)
#define EWM_CMPL_REG(base)                       ((base)->CMPL)
#define EWM_CMPH_REG(base)                       ((base)->CMPH)
#define EWM_CLKPRESCALER_REG(base)               ((base)->CLKPRESCALER)

/*!
 * @}
 */ /* end of group EWM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- EWM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Masks EWM Register Masks
 * @{
 */

/* CTRL Bit Fields */
#define EWM_CTRL_EWMEN		((uint8_t)0x1u)
#define EWM_CTRL_ASSIN		((uint8_t)0x2u)
#define EWM_CTRL_INEN		((uint8_t)0x4u)
#define EWM_CTRL_INTEN		((uint8_t)0x8u)
/* SERV Bit Fields */
#define EWM_SERV_SERVICE_MASK                    0xFFu
#define EWM_SERV_SERVICE_SHIFT                   0
#define EWM_SERV_SERVICE_WIDTH                   8
#define EWM_SERV_SERVICE(x)                      (((uint8_t)(((uint8_t)(x))<<EWM_SERV_SERVICE_SHIFT))&EWM_SERV_SERVICE_MASK)
/* CMPL Bit Fields */
#define EWM_CMPL_COMPAREL_MASK                   0xFFu
#define EWM_CMPL_COMPAREL_SHIFT                  0
#define EWM_CMPL_COMPAREL_WIDTH                  8
#define EWM_CMPL_COMPAREL(x)                     (((uint8_t)(((uint8_t)(x))<<EWM_CMPL_COMPAREL_SHIFT))&EWM_CMPL_COMPAREL_MASK)
/* CMPH Bit Fields */
#define EWM_CMPH_COMPAREH_MASK                   0xFFu
#define EWM_CMPH_COMPAREH_SHIFT                  0
#define EWM_CMPH_COMPAREH_WIDTH                  8
#define EWM_CMPH_COMPAREH(x)                     (((uint8_t)(((uint8_t)(x))<<EWM_CMPH_COMPAREH_SHIFT))&EWM_CMPH_COMPAREH_MASK)
/* CLKPRESCALER Bit Fields */
#define EWM_CLKPRESCALER_CLK_DIV_MASK            0xFFu
#define EWM_CLKPRESCALER_CLK_DIV_SHIFT           0
#define EWM_CLKPRESCALER_CLK_DIV_WIDTH           8
#define EWM_CLKPRESCALER_CLK_DIV(x)              (((uint8_t)(((uint8_t)(x))<<EWM_CLKPRESCALER_CLK_DIV_SHIFT))&EWM_CLKPRESCALER_CLK_DIV_MASK)

/*!
 * @}
 */ /* end of group EWM_Register_Masks */


/* EWM - Peripheral instance base addresses */
/** Peripheral EWM base address */
#define EWM_BASE                                 (0x40061000u)
/** Peripheral EWM base pointer */
#define EWM                                      ((EWM_TypeDef *)EWM_BASE)
#define EWM_BASE_PTR                             (EWM)
/** Array initializer of EWM peripheral base addresses */
#define EWM_BASE_ADDRS                           { EWM_BASE }
/** Array initializer of EWM peripheral base pointers */
#define EWM_BASE_PTRS                            { EWM }
/** Interrupt vectors for the EWM peripheral type */
#define EWM_IRQS                                 { WDOG_EWM_IRQn }

/* ----------------------------------------------------------------------------
   -- EWM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup EWM_Register_Accessor_Macros EWM - Register accessor macros
 * @{
 */


/* EWM - Register instance definitions */
/* EWM */
#define EWM_CTRL                                 EWM_CTRL_REG(EWM)
#define EWM_SERV                                 EWM_SERV_REG(EWM)
#define EWM_CMPL                                 EWM_CMPL_REG(EWM)
#define EWM_CMPH                                 EWM_CMPH_REG(EWM)
#define EWM_CLKPRESCALER                         EWM_CLKPRESCALER_REG(EWM)

/*!
 * @}
 */ /* end of group EWM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group EWM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FB_Peripheral_Access_Layer FB Peripheral Access Layer
 * @{
 */

/** FB - Register Layout Typedef */
typedef struct {
  struct {                                         /* offset: 0x0, array step: 0xC */
    __IO uint32_t CSAR;                              /**< Chip Select Address Register, array offset: 0x0, array step: 0xC */
    __IO uint32_t CSMR;                              /**< Chip Select Mask Register, array offset: 0x4, array step: 0xC */
    __IO uint32_t CSCR;                              /**< Chip Select Control Register, array offset: 0x8, array step: 0xC */
  } CS[6];
       uint8_t RESERVED_0[24];
  __IO uint32_t CSPMCR;                            /**< Chip Select port Multiplexing Control Register, offset: 0x60 */
} FB_TypeDef, *FB_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- FB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FB_Register_Accessor_Macros FB - Register accessor macros
 * @{
 */


/* FB - Register accessors */
#define FB_CSAR_REG(base,index)                  ((base)->CS[index].CSAR)
#define FB_CSAR_COUNT                            6
#define FB_CSMR_REG(base,index)                  ((base)->CS[index].CSMR)
#define FB_CSMR_COUNT                            6
#define FB_CSCR_REG(base,index)                  ((base)->CS[index].CSCR)
#define FB_CSCR_COUNT                            6
#define FB_CSPMCR_REG(base)                      ((base)->CSPMCR)

/*!
 * @}
 */ /* end of group FB_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- FB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FB_Register_Masks FB Register Masks
 * @{
 */

/* CSAR Bit Fields */
#define FB_CSAR_BA_MASK                          0xFFFF0000u
#define FB_CSAR_BA_SHIFT                         16
#define FB_CSAR_BA_WIDTH                         16
#define FB_CSAR_BA(x)                            (((uint32_t)(((uint32_t)(x))<<FB_CSAR_BA_SHIFT))&FB_CSAR_BA_MASK)
/* CSMR Bit Fields */
#define FB_CSMR_V		((uint32_t)0x1u)
#define FB_CSMR_WP		((uint32_t)0x100u)
#define FB_CSMR_BAM_MASK                         0xFFFF0000u
#define FB_CSMR_BAM_SHIFT                        16
#define FB_CSMR_BAM_WIDTH                        16
#define FB_CSMR_BAM(x)                           (((uint32_t)(((uint32_t)(x))<<FB_CSMR_BAM_SHIFT))&FB_CSMR_BAM_MASK)
/* CSCR Bit Fields */
#define FB_CSCR_BSTW		((uint32_t)0x8u)
#define FB_CSCR_BSTR		((uint32_t)0x10u)
#define FB_CSCR_BEM		((uint32_t)0x20u)
#define FB_CSCR_PS_MASK                          0xC0u
#define FB_CSCR_PS_SHIFT                         6
#define FB_CSCR_PS_WIDTH                         2
#define FB_CSCR_PS(x)                            (((uint32_t)(((uint32_t)(x))<<FB_CSCR_PS_SHIFT))&FB_CSCR_PS_MASK)
#define FB_CSCR_AA		((uint32_t)0x100u)
#define FB_CSCR_BLS		((uint32_t)0x200u)
#define FB_CSCR_WS_MASK                          0xFC00u
#define FB_CSCR_WS_SHIFT                         10
#define FB_CSCR_WS_WIDTH                         6
#define FB_CSCR_WS(x)                            (((uint32_t)(((uint32_t)(x))<<FB_CSCR_WS_SHIFT))&FB_CSCR_WS_MASK)
#define FB_CSCR_WRAH_MASK                        0x30000u
#define FB_CSCR_WRAH_SHIFT                       16
#define FB_CSCR_WRAH_WIDTH                       2
#define FB_CSCR_WRAH(x)                          (((uint32_t)(((uint32_t)(x))<<FB_CSCR_WRAH_SHIFT))&FB_CSCR_WRAH_MASK)
#define FB_CSCR_RDAH_MASK                        0xC0000u
#define FB_CSCR_RDAH_SHIFT                       18
#define FB_CSCR_RDAH_WIDTH                       2
#define FB_CSCR_RDAH(x)                          (((uint32_t)(((uint32_t)(x))<<FB_CSCR_RDAH_SHIFT))&FB_CSCR_RDAH_MASK)
#define FB_CSCR_ASET_MASK                        0x300000u
#define FB_CSCR_ASET_SHIFT                       20
#define FB_CSCR_ASET_WIDTH                       2
#define FB_CSCR_ASET(x)                          (((uint32_t)(((uint32_t)(x))<<FB_CSCR_ASET_SHIFT))&FB_CSCR_ASET_MASK)
#define FB_CSCR_EXTS		((uint32_t)0x400000u)
#define FB_CSCR_SWSEN		((uint32_t)0x800000u)
#define FB_CSCR_SWS_MASK                         0xFC000000u
#define FB_CSCR_SWS_SHIFT                        26
#define FB_CSCR_SWS_WIDTH                        6
#define FB_CSCR_SWS(x)                           (((uint32_t)(((uint32_t)(x))<<FB_CSCR_SWS_SHIFT))&FB_CSCR_SWS_MASK)
/* CSPMCR Bit Fields */
#define FB_CSPMCR_GROUP5_MASK                    0xF000u
#define FB_CSPMCR_GROUP5_SHIFT                   12
#define FB_CSPMCR_GROUP5_WIDTH                   4
#define FB_CSPMCR_GROUP5(x)                      (((uint32_t)(((uint32_t)(x))<<FB_CSPMCR_GROUP5_SHIFT))&FB_CSPMCR_GROUP5_MASK)
#define FB_CSPMCR_GROUP4_MASK                    0xF0000u
#define FB_CSPMCR_GROUP4_SHIFT                   16
#define FB_CSPMCR_GROUP4_WIDTH                   4
#define FB_CSPMCR_GROUP4(x)                      (((uint32_t)(((uint32_t)(x))<<FB_CSPMCR_GROUP4_SHIFT))&FB_CSPMCR_GROUP4_MASK)
#define FB_CSPMCR_GROUP3_MASK                    0xF00000u
#define FB_CSPMCR_GROUP3_SHIFT                   20
#define FB_CSPMCR_GROUP3_WIDTH                   4
#define FB_CSPMCR_GROUP3(x)                      (((uint32_t)(((uint32_t)(x))<<FB_CSPMCR_GROUP3_SHIFT))&FB_CSPMCR_GROUP3_MASK)
#define FB_CSPMCR_GROUP2_MASK                    0xF000000u
#define FB_CSPMCR_GROUP2_SHIFT                   24
#define FB_CSPMCR_GROUP2_WIDTH                   4
#define FB_CSPMCR_GROUP2(x)                      (((uint32_t)(((uint32_t)(x))<<FB_CSPMCR_GROUP2_SHIFT))&FB_CSPMCR_GROUP2_MASK)
#define FB_CSPMCR_GROUP1_MASK                    0xF0000000u
#define FB_CSPMCR_GROUP1_SHIFT                   28
#define FB_CSPMCR_GROUP1_WIDTH                   4
#define FB_CSPMCR_GROUP1(x)                      (((uint32_t)(((uint32_t)(x))<<FB_CSPMCR_GROUP1_SHIFT))&FB_CSPMCR_GROUP1_MASK)

/*!
 * @}
 */ /* end of group FB_Register_Masks */


/* FB - Peripheral instance base addresses */
/** Peripheral FB base address */
#define FB_BASE                                  (0x4000C000u)
/** Peripheral FB base pointer */
#define FB                                       ((FB_TypeDef *)FB_BASE)
#define FB_BASE_PTR                              (FB)
/** Array initializer of FB peripheral base addresses */
#define FB_BASE_ADDRS                            { FB_BASE }
/** Array initializer of FB peripheral base pointers */
#define FB_BASE_PTRS                             { FB }

/* ----------------------------------------------------------------------------
   -- FB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FB_Register_Accessor_Macros FB - Register accessor macros
 * @{
 */


/* FB - Register instance definitions */
/* FB */
#define FB_CSAR0                                 FB_CSAR_REG(FB,0)
#define FB_CSMR0                                 FB_CSMR_REG(FB,0)
#define FB_CSCR0                                 FB_CSCR_REG(FB,0)
#define FB_CSAR1                                 FB_CSAR_REG(FB,1)
#define FB_CSMR1                                 FB_CSMR_REG(FB,1)
#define FB_CSCR1                                 FB_CSCR_REG(FB,1)
#define FB_CSAR2                                 FB_CSAR_REG(FB,2)
#define FB_CSMR2                                 FB_CSMR_REG(FB,2)
#define FB_CSCR2                                 FB_CSCR_REG(FB,2)
#define FB_CSAR3                                 FB_CSAR_REG(FB,3)
#define FB_CSMR3                                 FB_CSMR_REG(FB,3)
#define FB_CSCR3                                 FB_CSCR_REG(FB,3)
#define FB_CSAR4                                 FB_CSAR_REG(FB,4)
#define FB_CSMR4                                 FB_CSMR_REG(FB,4)
#define FB_CSCR4                                 FB_CSCR_REG(FB,4)
#define FB_CSAR5                                 FB_CSAR_REG(FB,5)
#define FB_CSMR5                                 FB_CSMR_REG(FB,5)
#define FB_CSCR5                                 FB_CSCR_REG(FB,5)
#define FB_CSPMCR                                FB_CSPMCR_REG(FB)

/* FB - Register array accessors */
#define FB_CSAR(index)                           FB_CSAR_REG(FB,index)
#define FB_CSMR(index)                           FB_CSMR_REG(FB,index)
#define FB_CSCR(index)                           FB_CSCR_REG(FB,index)

/*!
 * @}
 */ /* end of group FB_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group FB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Peripheral_Access_Layer FMC Peripheral Access Layer
 * @{
 */

/** FMC - Register Layout Typedef */
typedef struct {
  __IO uint32_t PFAPR;                             /**< Flash Access Protection Register, offset: 0x0 */
  __IO uint32_t PFB0CR;                            /**< Flash Bank 0 Control Register, offset: 0x4 */
  __IO uint32_t PFB1CR;                            /**< Flash Bank 1 Control Register, offset: 0x8 */
       uint8_t RESERVED_0[244];
  __IO uint32_t TAGVDW0S[8];                       /**< Cache Tag Storage, array offset: 0x100, array step: 0x4 */
  __IO uint32_t TAGVDW1S[8];                       /**< Cache Tag Storage, array offset: 0x120, array step: 0x4 */
  __IO uint32_t TAGVDW2S[8];                       /**< Cache Tag Storage, array offset: 0x140, array step: 0x4 */
  __IO uint32_t TAGVDW3S[8];                       /**< Cache Tag Storage, array offset: 0x160, array step: 0x4 */
       uint8_t RESERVED_1[128];
  struct {                                         /* offset: 0x200, array step: index*0x40, index2*0x8 */
    __IO uint32_t DATA_U;                            /**< Cache Data Storage (upper word), array offset: 0x200, array step: index*0x40, index2*0x8 */
    __IO uint32_t DATA_L;                            /**< Cache Data Storage (lower word), array offset: 0x204, array step: index*0x40, index2*0x8 */
  } SET[4][8];
} FMC_TypeDef, *FMC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- FMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Accessor_Macros FMC - Register accessor macros
 * @{
 */


/* FMC - Register accessors */
#define FMC_PFAPR_REG(base)                      ((base)->PFAPR)
#define FMC_PFB0CR_REG(base)                     ((base)->PFB0CR)
#define FMC_PFB1CR_REG(base)                     ((base)->PFB1CR)
#define FMC_TAGVDW0S_REG(base,index)             ((base)->TAGVDW0S[index])
#define FMC_TAGVDW0S_COUNT                       8
#define FMC_TAGVDW1S_REG(base,index)             ((base)->TAGVDW1S[index])
#define FMC_TAGVDW1S_COUNT                       8
#define FMC_TAGVDW2S_REG(base,index)             ((base)->TAGVDW2S[index])
#define FMC_TAGVDW2S_COUNT                       8
#define FMC_TAGVDW3S_REG(base,index)             ((base)->TAGVDW3S[index])
#define FMC_TAGVDW3S_COUNT                       8
#define FMC_DATA_U_REG(base,index,index2)        ((base)->SET[index][index2].DATA_U)
#define FMC_DATA_U_COUNT                         4
#define FMC_DATA_U_COUNT2                        8
#define FMC_DATA_L_REG(base,index,index2)        ((base)->SET[index][index2].DATA_L)
#define FMC_DATA_L_COUNT                         4
#define FMC_DATA_L_COUNT2                        8

/*!
 * @}
 */ /* end of group FMC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- FMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Masks FMC Register Masks
 * @{
 */

/* PFAPR Bit Fields */
#define FMC_PFAPR_M0AP_MASK                      0x3u
#define FMC_PFAPR_M0AP_SHIFT                     0
#define FMC_PFAPR_M0AP_WIDTH                     2
#define FMC_PFAPR_M0AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M0AP_SHIFT))&FMC_PFAPR_M0AP_MASK)
#define FMC_PFAPR_M1AP_MASK                      0xCu
#define FMC_PFAPR_M1AP_SHIFT                     2
#define FMC_PFAPR_M1AP_WIDTH                     2
#define FMC_PFAPR_M1AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M1AP_SHIFT))&FMC_PFAPR_M1AP_MASK)
#define FMC_PFAPR_M2AP_MASK                      0x30u
#define FMC_PFAPR_M2AP_SHIFT                     4
#define FMC_PFAPR_M2AP_WIDTH                     2
#define FMC_PFAPR_M2AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M2AP_SHIFT))&FMC_PFAPR_M2AP_MASK)
#define FMC_PFAPR_M3AP_MASK                      0xC0u
#define FMC_PFAPR_M3AP_SHIFT                     6
#define FMC_PFAPR_M3AP_WIDTH                     2
#define FMC_PFAPR_M3AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M3AP_SHIFT))&FMC_PFAPR_M3AP_MASK)
#define FMC_PFAPR_M4AP_MASK                      0x300u
#define FMC_PFAPR_M4AP_SHIFT                     8
#define FMC_PFAPR_M4AP_WIDTH                     2
#define FMC_PFAPR_M4AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M4AP_SHIFT))&FMC_PFAPR_M4AP_MASK)
#define FMC_PFAPR_M5AP_MASK                      0xC00u
#define FMC_PFAPR_M5AP_SHIFT                     10
#define FMC_PFAPR_M5AP_WIDTH                     2
#define FMC_PFAPR_M5AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M5AP_SHIFT))&FMC_PFAPR_M5AP_MASK)
#define FMC_PFAPR_M6AP_MASK                      0x3000u
#define FMC_PFAPR_M6AP_SHIFT                     12
#define FMC_PFAPR_M6AP_WIDTH                     2
#define FMC_PFAPR_M6AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M6AP_SHIFT))&FMC_PFAPR_M6AP_MASK)
#define FMC_PFAPR_M7AP_MASK                      0xC000u
#define FMC_PFAPR_M7AP_SHIFT                     14
#define FMC_PFAPR_M7AP_WIDTH                     2
#define FMC_PFAPR_M7AP(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFAPR_M7AP_SHIFT))&FMC_PFAPR_M7AP_MASK)
#define FMC_PFAPR_M0PFD		((uint32_t)0x10000u)
#define FMC_PFAPR_M1PFD		((uint32_t)0x20000u)
#define FMC_PFAPR_M2PFD		((uint32_t)0x40000u)
#define FMC_PFAPR_M3PFD		((uint32_t)0x80000u)
#define FMC_PFAPR_M4PFD		((uint32_t)0x100000u)
#define FMC_PFAPR_M5PFD		((uint32_t)0x200000u)
#define FMC_PFAPR_M6PFD		((uint32_t)0x400000u)
#define FMC_PFAPR_M7PFD		((uint32_t)0x800000u)
/* PFB0CR Bit Fields */
#define FMC_PFB0CR_B0SEBE		((uint32_t)0x1u)
#define FMC_PFB0CR_B0IPE		((uint32_t)0x2u)
#define FMC_PFB0CR_B0DPE		((uint32_t)0x4u)
#define FMC_PFB0CR_B0ICE		((uint32_t)0x8u)
#define FMC_PFB0CR_B0DCE		((uint32_t)0x10u)
#define FMC_PFB0CR_CRC_MASK                      0xE0u
#define FMC_PFB0CR_CRC_SHIFT                     5
#define FMC_PFB0CR_CRC_WIDTH                     3
#define FMC_PFB0CR_CRC(x)                        (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_CRC_SHIFT))&FMC_PFB0CR_CRC_MASK)
#define FMC_PFB0CR_B0MW_MASK                     0x60000u
#define FMC_PFB0CR_B0MW_SHIFT                    17
#define FMC_PFB0CR_B0MW_WIDTH                    2
#define FMC_PFB0CR_B0MW(x)                       (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0MW_SHIFT))&FMC_PFB0CR_B0MW_MASK)
#define FMC_PFB0CR_S_B_INV		((uint32_t)0x80000u)
#define FMC_PFB0CR_CINV_WAY_MASK                 0xF00000u
#define FMC_PFB0CR_CINV_WAY_SHIFT                20
#define FMC_PFB0CR_CINV_WAY_WIDTH                4
#define FMC_PFB0CR_CINV_WAY(x)                   (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_CINV_WAY_SHIFT))&FMC_PFB0CR_CINV_WAY_MASK)
#define FMC_PFB0CR_CLCK_WAY_MASK                 0xF000000u
#define FMC_PFB0CR_CLCK_WAY_SHIFT                24
#define FMC_PFB0CR_CLCK_WAY_WIDTH                4
#define FMC_PFB0CR_CLCK_WAY(x)                   (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_CLCK_WAY_SHIFT))&FMC_PFB0CR_CLCK_WAY_MASK)
#define FMC_PFB0CR_B0RWSC_MASK                   0xF0000000u
#define FMC_PFB0CR_B0RWSC_SHIFT                  28
#define FMC_PFB0CR_B0RWSC_WIDTH                  4
#define FMC_PFB0CR_B0RWSC(x)                     (((uint32_t)(((uint32_t)(x))<<FMC_PFB0CR_B0RWSC_SHIFT))&FMC_PFB0CR_B0RWSC_MASK)
/* PFB1CR Bit Fields */
#define FMC_PFB1CR_B1SEBE		((uint32_t)0x1u)
#define FMC_PFB1CR_B1IPE		((uint32_t)0x2u)
#define FMC_PFB1CR_B1DPE		((uint32_t)0x4u)
#define FMC_PFB1CR_B1ICE		((uint32_t)0x8u)
#define FMC_PFB1CR_B1DCE		((uint32_t)0x10u)
#define FMC_PFB1CR_B1MW_MASK                     0x60000u
#define FMC_PFB1CR_B1MW_SHIFT                    17
#define FMC_PFB1CR_B1MW_WIDTH                    2
#define FMC_PFB1CR_B1MW(x)                       (((uint32_t)(((uint32_t)(x))<<FMC_PFB1CR_B1MW_SHIFT))&FMC_PFB1CR_B1MW_MASK)
#define FMC_PFB1CR_B1RWSC_MASK                   0xF0000000u
#define FMC_PFB1CR_B1RWSC_SHIFT                  28
#define FMC_PFB1CR_B1RWSC_WIDTH                  4
#define FMC_PFB1CR_B1RWSC(x)                     (((uint32_t)(((uint32_t)(x))<<FMC_PFB1CR_B1RWSC_SHIFT))&FMC_PFB1CR_B1RWSC_MASK)
/* TAGVDW0S Bit Fields */
#define FMC_TAGVDW0S_valid		((uint32_t)0x1u)
#define FMC_TAGVDW0S_tag_MASK                    0x7FFE0u
#define FMC_TAGVDW0S_tag_SHIFT                   5
#define FMC_TAGVDW0S_tag_WIDTH                   14
#define FMC_TAGVDW0S_tag(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW0S_tag_SHIFT))&FMC_TAGVDW0S_tag_MASK)
/* TAGVDW1S Bit Fields */
#define FMC_TAGVDW1S_valid		((uint32_t)0x1u)
#define FMC_TAGVDW1S_tag_MASK                    0x7FFE0u
#define FMC_TAGVDW1S_tag_SHIFT                   5
#define FMC_TAGVDW1S_tag_WIDTH                   14
#define FMC_TAGVDW1S_tag(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW1S_tag_SHIFT))&FMC_TAGVDW1S_tag_MASK)
/* TAGVDW2S Bit Fields */
#define FMC_TAGVDW2S_valid		((uint32_t)0x1u)
#define FMC_TAGVDW2S_tag_MASK                    0x7FFE0u
#define FMC_TAGVDW2S_tag_SHIFT                   5
#define FMC_TAGVDW2S_tag_WIDTH                   14
#define FMC_TAGVDW2S_tag(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW2S_tag_SHIFT))&FMC_TAGVDW2S_tag_MASK)
/* TAGVDW3S Bit Fields */
#define FMC_TAGVDW3S_valid		((uint32_t)0x1u)
#define FMC_TAGVDW3S_tag_MASK                    0x7FFE0u
#define FMC_TAGVDW3S_tag_SHIFT                   5
#define FMC_TAGVDW3S_tag_WIDTH                   14
#define FMC_TAGVDW3S_tag(x)                      (((uint32_t)(((uint32_t)(x))<<FMC_TAGVDW3S_tag_SHIFT))&FMC_TAGVDW3S_tag_MASK)
/* DATA_U Bit Fields */
#define FMC_DATA_U_data_MASK                     0xFFFFFFFFu
#define FMC_DATA_U_data_SHIFT                    0
#define FMC_DATA_U_data_WIDTH                    32
#define FMC_DATA_U_data(x)                       (((uint32_t)(((uint32_t)(x))<<FMC_DATA_U_data_SHIFT))&FMC_DATA_U_data_MASK)
/* DATA_L Bit Fields */
#define FMC_DATA_L_data_MASK                     0xFFFFFFFFu
#define FMC_DATA_L_data_SHIFT                    0
#define FMC_DATA_L_data_WIDTH                    32
#define FMC_DATA_L_data(x)                       (((uint32_t)(((uint32_t)(x))<<FMC_DATA_L_data_SHIFT))&FMC_DATA_L_data_MASK)

/*!
 * @}
 */ /* end of group FMC_Register_Masks */


/* FMC - Peripheral instance base addresses */
/** Peripheral FMC base address */
#define FMC_BASE                                 (0x4001F000u)
/** Peripheral FMC base pointer */
#define FMC                                      ((FMC_TypeDef *)FMC_BASE)
#define FMC_BASE_PTR                             (FMC)
/** Array initializer of FMC peripheral base addresses */
#define FMC_BASE_ADDRS                           { FMC_BASE }
/** Array initializer of FMC peripheral base pointers */
#define FMC_BASE_PTRS                            { FMC }

/* ----------------------------------------------------------------------------
   -- FMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FMC_Register_Accessor_Macros FMC - Register accessor macros
 * @{
 */


/* FMC - Register instance definitions */
/* FMC */
#define FMC_PFAPR                                FMC_PFAPR_REG(FMC)
#define FMC_PFB0CR                               FMC_PFB0CR_REG(FMC)
#define FMC_PFB1CR                               FMC_PFB1CR_REG(FMC)
#define FMC_TAGVDW0S0                            FMC_TAGVDW0S_REG(FMC,0)
#define FMC_TAGVDW0S1                            FMC_TAGVDW0S_REG(FMC,1)
#define FMC_TAGVDW0S2                            FMC_TAGVDW0S_REG(FMC,2)
#define FMC_TAGVDW0S3                            FMC_TAGVDW0S_REG(FMC,3)
#define FMC_TAGVDW0S4                            FMC_TAGVDW0S_REG(FMC,4)
#define FMC_TAGVDW0S5                            FMC_TAGVDW0S_REG(FMC,5)
#define FMC_TAGVDW0S6                            FMC_TAGVDW0S_REG(FMC,6)
#define FMC_TAGVDW0S7                            FMC_TAGVDW0S_REG(FMC,7)
#define FMC_TAGVDW1S0                            FMC_TAGVDW1S_REG(FMC,0)
#define FMC_TAGVDW1S1                            FMC_TAGVDW1S_REG(FMC,1)
#define FMC_TAGVDW1S2                            FMC_TAGVDW1S_REG(FMC,2)
#define FMC_TAGVDW1S3                            FMC_TAGVDW1S_REG(FMC,3)
#define FMC_TAGVDW1S4                            FMC_TAGVDW1S_REG(FMC,4)
#define FMC_TAGVDW1S5                            FMC_TAGVDW1S_REG(FMC,5)
#define FMC_TAGVDW1S6                            FMC_TAGVDW1S_REG(FMC,6)
#define FMC_TAGVDW1S7                            FMC_TAGVDW1S_REG(FMC,7)
#define FMC_TAGVDW2S0                            FMC_TAGVDW2S_REG(FMC,0)
#define FMC_TAGVDW2S1                            FMC_TAGVDW2S_REG(FMC,1)
#define FMC_TAGVDW2S2                            FMC_TAGVDW2S_REG(FMC,2)
#define FMC_TAGVDW2S3                            FMC_TAGVDW2S_REG(FMC,3)
#define FMC_TAGVDW2S4                            FMC_TAGVDW2S_REG(FMC,4)
#define FMC_TAGVDW2S5                            FMC_TAGVDW2S_REG(FMC,5)
#define FMC_TAGVDW2S6                            FMC_TAGVDW2S_REG(FMC,6)
#define FMC_TAGVDW2S7                            FMC_TAGVDW2S_REG(FMC,7)
#define FMC_TAGVDW3S0                            FMC_TAGVDW3S_REG(FMC,0)
#define FMC_TAGVDW3S1                            FMC_TAGVDW3S_REG(FMC,1)
#define FMC_TAGVDW3S2                            FMC_TAGVDW3S_REG(FMC,2)
#define FMC_TAGVDW3S3                            FMC_TAGVDW3S_REG(FMC,3)
#define FMC_TAGVDW3S4                            FMC_TAGVDW3S_REG(FMC,4)
#define FMC_TAGVDW3S5                            FMC_TAGVDW3S_REG(FMC,5)
#define FMC_TAGVDW3S6                            FMC_TAGVDW3S_REG(FMC,6)
#define FMC_TAGVDW3S7                            FMC_TAGVDW3S_REG(FMC,7)
#define FMC_DATAW0S0U                            FMC_DATA_U_REG(FMC,0,0)
#define FMC_DATAW0S0L                            FMC_DATA_L_REG(FMC,0,0)
#define FMC_DATAW0S1U                            FMC_DATA_U_REG(FMC,0,1)
#define FMC_DATAW0S1L                            FMC_DATA_L_REG(FMC,0,1)
#define FMC_DATAW0S2U                            FMC_DATA_U_REG(FMC,0,2)
#define FMC_DATAW0S2L                            FMC_DATA_L_REG(FMC,0,2)
#define FMC_DATAW0S3U                            FMC_DATA_U_REG(FMC,0,3)
#define FMC_DATAW0S3L                            FMC_DATA_L_REG(FMC,0,3)
#define FMC_DATAW0S4U                            FMC_DATA_U_REG(FMC,0,4)
#define FMC_DATAW0S4L                            FMC_DATA_L_REG(FMC,0,4)
#define FMC_DATAW0S5U                            FMC_DATA_U_REG(FMC,0,5)
#define FMC_DATAW0S5L                            FMC_DATA_L_REG(FMC,0,5)
#define FMC_DATAW0S6U                            FMC_DATA_U_REG(FMC,0,6)
#define FMC_DATAW0S6L                            FMC_DATA_L_REG(FMC,0,6)
#define FMC_DATAW0S7U                            FMC_DATA_U_REG(FMC,0,7)
#define FMC_DATAW0S7L                            FMC_DATA_L_REG(FMC,0,7)
#define FMC_DATAW1S0U                            FMC_DATA_U_REG(FMC,1,0)
#define FMC_DATAW1S0L                            FMC_DATA_L_REG(FMC,1,0)
#define FMC_DATAW1S1U                            FMC_DATA_U_REG(FMC,1,1)
#define FMC_DATAW1S1L                            FMC_DATA_L_REG(FMC,1,1)
#define FMC_DATAW1S2U                            FMC_DATA_U_REG(FMC,1,2)
#define FMC_DATAW1S2L                            FMC_DATA_L_REG(FMC,1,2)
#define FMC_DATAW1S3U                            FMC_DATA_U_REG(FMC,1,3)
#define FMC_DATAW1S3L                            FMC_DATA_L_REG(FMC,1,3)
#define FMC_DATAW1S4U                            FMC_DATA_U_REG(FMC,1,4)
#define FMC_DATAW1S4L                            FMC_DATA_L_REG(FMC,1,4)
#define FMC_DATAW1S5U                            FMC_DATA_U_REG(FMC,1,5)
#define FMC_DATAW1S5L                            FMC_DATA_L_REG(FMC,1,5)
#define FMC_DATAW1S6U                            FMC_DATA_U_REG(FMC,1,6)
#define FMC_DATAW1S6L                            FMC_DATA_L_REG(FMC,1,6)
#define FMC_DATAW1S7U                            FMC_DATA_U_REG(FMC,1,7)
#define FMC_DATAW1S7L                            FMC_DATA_L_REG(FMC,1,7)
#define FMC_DATAW2S0U                            FMC_DATA_U_REG(FMC,2,0)
#define FMC_DATAW2S0L                            FMC_DATA_L_REG(FMC,2,0)
#define FMC_DATAW2S1U                            FMC_DATA_U_REG(FMC,2,1)
#define FMC_DATAW2S1L                            FMC_DATA_L_REG(FMC,2,1)
#define FMC_DATAW2S2U                            FMC_DATA_U_REG(FMC,2,2)
#define FMC_DATAW2S2L                            FMC_DATA_L_REG(FMC,2,2)
#define FMC_DATAW2S3U                            FMC_DATA_U_REG(FMC,2,3)
#define FMC_DATAW2S3L                            FMC_DATA_L_REG(FMC,2,3)
#define FMC_DATAW2S4U                            FMC_DATA_U_REG(FMC,2,4)
#define FMC_DATAW2S4L                            FMC_DATA_L_REG(FMC,2,4)
#define FMC_DATAW2S5U                            FMC_DATA_U_REG(FMC,2,5)
#define FMC_DATAW2S5L                            FMC_DATA_L_REG(FMC,2,5)
#define FMC_DATAW2S6U                            FMC_DATA_U_REG(FMC,2,6)
#define FMC_DATAW2S6L                            FMC_DATA_L_REG(FMC,2,6)
#define FMC_DATAW2S7U                            FMC_DATA_U_REG(FMC,2,7)
#define FMC_DATAW2S7L                            FMC_DATA_L_REG(FMC,2,7)
#define FMC_DATAW3S0U                            FMC_DATA_U_REG(FMC,3,0)
#define FMC_DATAW3S0L                            FMC_DATA_L_REG(FMC,3,0)
#define FMC_DATAW3S1U                            FMC_DATA_U_REG(FMC,3,1)
#define FMC_DATAW3S1L                            FMC_DATA_L_REG(FMC,3,1)
#define FMC_DATAW3S2U                            FMC_DATA_U_REG(FMC,3,2)
#define FMC_DATAW3S2L                            FMC_DATA_L_REG(FMC,3,2)
#define FMC_DATAW3S3U                            FMC_DATA_U_REG(FMC,3,3)
#define FMC_DATAW3S3L                            FMC_DATA_L_REG(FMC,3,3)
#define FMC_DATAW3S4U                            FMC_DATA_U_REG(FMC,3,4)
#define FMC_DATAW3S4L                            FMC_DATA_L_REG(FMC,3,4)
#define FMC_DATAW3S5U                            FMC_DATA_U_REG(FMC,3,5)
#define FMC_DATAW3S5L                            FMC_DATA_L_REG(FMC,3,5)
#define FMC_DATAW3S6U                            FMC_DATA_U_REG(FMC,3,6)
#define FMC_DATAW3S6L                            FMC_DATA_L_REG(FMC,3,6)
#define FMC_DATAW3S7U                            FMC_DATA_U_REG(FMC,3,7)
#define FMC_DATAW3S7L                            FMC_DATA_L_REG(FMC,3,7)

/* FMC - Register array accessors */
#define FMC_TAGVDW0S(index)                      FMC_TAGVDW0S_REG(FMC,index)
#define FMC_TAGVDW1S(index)                      FMC_TAGVDW1S_REG(FMC,index)
#define FMC_TAGVDW2S(index)                      FMC_TAGVDW2S_REG(FMC,index)
#define FMC_TAGVDW3S(index)                      FMC_TAGVDW3S_REG(FMC,index)
#define FMC_DATA_U(index,index2)                 FMC_DATA_U_REG(FMC,index,index2)
#define FMC_DATA_L(index,index2)                 FMC_DATA_L_REG(FMC,index,index2)

/*!
 * @}
 */ /* end of group FMC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group FMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FTFA Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Peripheral_Access_Layer FTFA Peripheral Access Layer
 * @{
 */

/** FTFA - Register Layout Typedef */
typedef struct {
  __IO uint8_t FSTAT;                              /**< Flash Status Register, offset: 0x0 */
  __IO uint8_t FCNFG;                              /**< Flash Configuration Register, offset: 0x1 */
  __I  uint8_t FSEC;                               /**< Flash Security Register, offset: 0x2 */
  __I  uint8_t FOPT;                               /**< Flash Option Register, offset: 0x3 */
  __IO uint8_t FCCOB3;                             /**< Flash Common Command Object Registers, offset: 0x4 */
  __IO uint8_t FCCOB2;                             /**< Flash Common Command Object Registers, offset: 0x5 */
  __IO uint8_t FCCOB1;                             /**< Flash Common Command Object Registers, offset: 0x6 */
  __IO uint8_t FCCOB0;                             /**< Flash Common Command Object Registers, offset: 0x7 */
  __IO uint8_t FCCOB7;                             /**< Flash Common Command Object Registers, offset: 0x8 */
  __IO uint8_t FCCOB6;                             /**< Flash Common Command Object Registers, offset: 0x9 */
  __IO uint8_t FCCOB5;                             /**< Flash Common Command Object Registers, offset: 0xA */
  __IO uint8_t FCCOB4;                             /**< Flash Common Command Object Registers, offset: 0xB */
  __IO uint8_t FCCOBB;                             /**< Flash Common Command Object Registers, offset: 0xC */
  __IO uint8_t FCCOBA;                             /**< Flash Common Command Object Registers, offset: 0xD */
  __IO uint8_t FCCOB9;                             /**< Flash Common Command Object Registers, offset: 0xE */
  __IO uint8_t FCCOB8;                             /**< Flash Common Command Object Registers, offset: 0xF */
  __IO uint8_t FPROT3;                             /**< Program Flash Protection Registers, offset: 0x10 */
  __IO uint8_t FPROT2;                             /**< Program Flash Protection Registers, offset: 0x11 */
  __IO uint8_t FPROT1;                             /**< Program Flash Protection Registers, offset: 0x12 */
  __IO uint8_t FPROT0;                             /**< Program Flash Protection Registers, offset: 0x13 */
       uint8_t RESERVED_0[4];
  __I  uint8_t XACCH3;                             /**< Execute-only Access Registers, offset: 0x18 */
  __I  uint8_t XACCH2;                             /**< Execute-only Access Registers, offset: 0x19 */
  __I  uint8_t XACCH1;                             /**< Execute-only Access Registers, offset: 0x1A */
  __I  uint8_t XACCH0;                             /**< Execute-only Access Registers, offset: 0x1B */
  __I  uint8_t XACCL3;                             /**< Execute-only Access Registers, offset: 0x1C */
  __I  uint8_t XACCL2;                             /**< Execute-only Access Registers, offset: 0x1D */
  __I  uint8_t XACCL1;                             /**< Execute-only Access Registers, offset: 0x1E */
  __I  uint8_t XACCL0;                             /**< Execute-only Access Registers, offset: 0x1F */
  __I  uint8_t SACCH3;                             /**< Supervisor-only Access Registers, offset: 0x20 */
  __I  uint8_t SACCH2;                             /**< Supervisor-only Access Registers, offset: 0x21 */
  __I  uint8_t SACCH1;                             /**< Supervisor-only Access Registers, offset: 0x22 */
  __I  uint8_t SACCH0;                             /**< Supervisor-only Access Registers, offset: 0x23 */
  __I  uint8_t SACCL3;                             /**< Supervisor-only Access Registers, offset: 0x24 */
  __I  uint8_t SACCL2;                             /**< Supervisor-only Access Registers, offset: 0x25 */
  __I  uint8_t SACCL1;                             /**< Supervisor-only Access Registers, offset: 0x26 */
  __I  uint8_t SACCL0;                             /**< Supervisor-only Access Registers, offset: 0x27 */
  __I  uint8_t FACSS;                              /**< Flash Access Segment Size Register, offset: 0x28 */
       uint8_t RESERVED_1[2];
  __I  uint8_t FACSN;                              /**< Flash Access Segment Number Register, offset: 0x2B */
} FTFA_TypeDef, *FTFA_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- FTFA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Register_Accessor_Macros FTFA - Register accessor macros
 * @{
 */


/* FTFA - Register accessors */
#define FTFA_FSTAT_REG(base)                     ((base)->FSTAT)
#define FTFA_FCNFG_REG(base)                     ((base)->FCNFG)
#define FTFA_FSEC_REG(base)                      ((base)->FSEC)
#define FTFA_FOPT_REG(base)                      ((base)->FOPT)
#define FTFA_FCCOB3_REG(base)                    ((base)->FCCOB3)
#define FTFA_FCCOB2_REG(base)                    ((base)->FCCOB2)
#define FTFA_FCCOB1_REG(base)                    ((base)->FCCOB1)
#define FTFA_FCCOB0_REG(base)                    ((base)->FCCOB0)
#define FTFA_FCCOB7_REG(base)                    ((base)->FCCOB7)
#define FTFA_FCCOB6_REG(base)                    ((base)->FCCOB6)
#define FTFA_FCCOB5_REG(base)                    ((base)->FCCOB5)
#define FTFA_FCCOB4_REG(base)                    ((base)->FCCOB4)
#define FTFA_FCCOBB_REG(base)                    ((base)->FCCOBB)
#define FTFA_FCCOBA_REG(base)                    ((base)->FCCOBA)
#define FTFA_FCCOB9_REG(base)                    ((base)->FCCOB9)
#define FTFA_FCCOB8_REG(base)                    ((base)->FCCOB8)
#define FTFA_FPROT3_REG(base)                    ((base)->FPROT3)
#define FTFA_FPROT2_REG(base)                    ((base)->FPROT2)
#define FTFA_FPROT1_REG(base)                    ((base)->FPROT1)
#define FTFA_FPROT0_REG(base)                    ((base)->FPROT0)
#define FTFA_XACCH3_REG(base)                    ((base)->XACCH3)
#define FTFA_XACCH2_REG(base)                    ((base)->XACCH2)
#define FTFA_XACCH1_REG(base)                    ((base)->XACCH1)
#define FTFA_XACCH0_REG(base)                    ((base)->XACCH0)
#define FTFA_XACCL3_REG(base)                    ((base)->XACCL3)
#define FTFA_XACCL2_REG(base)                    ((base)->XACCL2)
#define FTFA_XACCL1_REG(base)                    ((base)->XACCL1)
#define FTFA_XACCL0_REG(base)                    ((base)->XACCL0)
#define FTFA_SACCH3_REG(base)                    ((base)->SACCH3)
#define FTFA_SACCH2_REG(base)                    ((base)->SACCH2)
#define FTFA_SACCH1_REG(base)                    ((base)->SACCH1)
#define FTFA_SACCH0_REG(base)                    ((base)->SACCH0)
#define FTFA_SACCL3_REG(base)                    ((base)->SACCL3)
#define FTFA_SACCL2_REG(base)                    ((base)->SACCL2)
#define FTFA_SACCL1_REG(base)                    ((base)->SACCL1)
#define FTFA_SACCL0_REG(base)                    ((base)->SACCL0)
#define FTFA_FACSS_REG(base)                     ((base)->FACSS)
#define FTFA_FACSN_REG(base)                     ((base)->FACSN)

/*!
 * @}
 */ /* end of group FTFA_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- FTFA Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Register_Masks FTFA Register Masks
 * @{
 */

/* FSTAT Bit Fields */
#define FTFA_FSTAT_MGSTAT0		((uint8_t)0x1u)
#define FTFA_FSTAT_FPVIOL		((uint8_t)0x10u)
#define FTFA_FSTAT_ACCERR		((uint8_t)0x20u)
#define FTFA_FSTAT_RDCOLERR		((uint8_t)0x40u)
#define FTFA_FSTAT_CCIF		((uint8_t)0x80u)
/* FCNFG Bit Fields */
#define FTFA_FCNFG_ERSSUSP		((uint8_t)0x10u)
#define FTFA_FCNFG_ERSAREQ		((uint8_t)0x20u)
#define FTFA_FCNFG_RDCOLLIE		((uint8_t)0x40u)
#define FTFA_FCNFG_CCIE		((uint8_t)0x80u)
/* FSEC Bit Fields */
#define FTFA_FSEC_SEC_MASK                       0x3u
#define FTFA_FSEC_SEC_SHIFT                      0
#define FTFA_FSEC_SEC_WIDTH                      2
#define FTFA_FSEC_SEC(x)                         (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_SEC_SHIFT))&FTFA_FSEC_SEC_MASK)
#define FTFA_FSEC_FSLACC_MASK                    0xCu
#define FTFA_FSEC_FSLACC_SHIFT                   2
#define FTFA_FSEC_FSLACC_WIDTH                   2
#define FTFA_FSEC_FSLACC(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_FSLACC_SHIFT))&FTFA_FSEC_FSLACC_MASK)
#define FTFA_FSEC_MEEN_MASK                      0x30u
#define FTFA_FSEC_MEEN_SHIFT                     4
#define FTFA_FSEC_MEEN_WIDTH                     2
#define FTFA_FSEC_MEEN(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_MEEN_SHIFT))&FTFA_FSEC_MEEN_MASK)
#define FTFA_FSEC_KEYEN_MASK                     0xC0u
#define FTFA_FSEC_KEYEN_SHIFT                    6
#define FTFA_FSEC_KEYEN_WIDTH                    2
#define FTFA_FSEC_KEYEN(x)                       (((uint8_t)(((uint8_t)(x))<<FTFA_FSEC_KEYEN_SHIFT))&FTFA_FSEC_KEYEN_MASK)
/* FOPT Bit Fields */
#define FTFA_FOPT_OPT_MASK                       0xFFu
#define FTFA_FOPT_OPT_SHIFT                      0
#define FTFA_FOPT_OPT_WIDTH                      8
#define FTFA_FOPT_OPT(x)                         (((uint8_t)(((uint8_t)(x))<<FTFA_FOPT_OPT_SHIFT))&FTFA_FOPT_OPT_MASK)
/* FCCOB3 Bit Fields */
#define FTFA_FCCOB3_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB3_CCOBn_SHIFT                  0
#define FTFA_FCCOB3_CCOBn_WIDTH                  8
#define FTFA_FCCOB3_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB3_CCOBn_SHIFT))&FTFA_FCCOB3_CCOBn_MASK)
/* FCCOB2 Bit Fields */
#define FTFA_FCCOB2_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB2_CCOBn_SHIFT                  0
#define FTFA_FCCOB2_CCOBn_WIDTH                  8
#define FTFA_FCCOB2_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB2_CCOBn_SHIFT))&FTFA_FCCOB2_CCOBn_MASK)
/* FCCOB1 Bit Fields */
#define FTFA_FCCOB1_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB1_CCOBn_SHIFT                  0
#define FTFA_FCCOB1_CCOBn_WIDTH                  8
#define FTFA_FCCOB1_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB1_CCOBn_SHIFT))&FTFA_FCCOB1_CCOBn_MASK)
/* FCCOB0 Bit Fields */
#define FTFA_FCCOB0_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB0_CCOBn_SHIFT                  0
#define FTFA_FCCOB0_CCOBn_WIDTH                  8
#define FTFA_FCCOB0_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB0_CCOBn_SHIFT))&FTFA_FCCOB0_CCOBn_MASK)
/* FCCOB7 Bit Fields */
#define FTFA_FCCOB7_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB7_CCOBn_SHIFT                  0
#define FTFA_FCCOB7_CCOBn_WIDTH                  8
#define FTFA_FCCOB7_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB7_CCOBn_SHIFT))&FTFA_FCCOB7_CCOBn_MASK)
/* FCCOB6 Bit Fields */
#define FTFA_FCCOB6_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB6_CCOBn_SHIFT                  0
#define FTFA_FCCOB6_CCOBn_WIDTH                  8
#define FTFA_FCCOB6_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB6_CCOBn_SHIFT))&FTFA_FCCOB6_CCOBn_MASK)
/* FCCOB5 Bit Fields */
#define FTFA_FCCOB5_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB5_CCOBn_SHIFT                  0
#define FTFA_FCCOB5_CCOBn_WIDTH                  8
#define FTFA_FCCOB5_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB5_CCOBn_SHIFT))&FTFA_FCCOB5_CCOBn_MASK)
/* FCCOB4 Bit Fields */
#define FTFA_FCCOB4_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB4_CCOBn_SHIFT                  0
#define FTFA_FCCOB4_CCOBn_WIDTH                  8
#define FTFA_FCCOB4_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB4_CCOBn_SHIFT))&FTFA_FCCOB4_CCOBn_MASK)
/* FCCOBB Bit Fields */
#define FTFA_FCCOBB_CCOBn_MASK                   0xFFu
#define FTFA_FCCOBB_CCOBn_SHIFT                  0
#define FTFA_FCCOBB_CCOBn_WIDTH                  8
#define FTFA_FCCOBB_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOBB_CCOBn_SHIFT))&FTFA_FCCOBB_CCOBn_MASK)
/* FCCOBA Bit Fields */
#define FTFA_FCCOBA_CCOBn_MASK                   0xFFu
#define FTFA_FCCOBA_CCOBn_SHIFT                  0
#define FTFA_FCCOBA_CCOBn_WIDTH                  8
#define FTFA_FCCOBA_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOBA_CCOBn_SHIFT))&FTFA_FCCOBA_CCOBn_MASK)
/* FCCOB9 Bit Fields */
#define FTFA_FCCOB9_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB9_CCOBn_SHIFT                  0
#define FTFA_FCCOB9_CCOBn_WIDTH                  8
#define FTFA_FCCOB9_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB9_CCOBn_SHIFT))&FTFA_FCCOB9_CCOBn_MASK)
/* FCCOB8 Bit Fields */
#define FTFA_FCCOB8_CCOBn_MASK                   0xFFu
#define FTFA_FCCOB8_CCOBn_SHIFT                  0
#define FTFA_FCCOB8_CCOBn_WIDTH                  8
#define FTFA_FCCOB8_CCOBn(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FCCOB8_CCOBn_SHIFT))&FTFA_FCCOB8_CCOBn_MASK)
/* FPROT3 Bit Fields */
#define FTFA_FPROT3_PROT_MASK                    0xFFu
#define FTFA_FPROT3_PROT_SHIFT                   0
#define FTFA_FPROT3_PROT_WIDTH                   8
#define FTFA_FPROT3_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT3_PROT_SHIFT))&FTFA_FPROT3_PROT_MASK)
/* FPROT2 Bit Fields */
#define FTFA_FPROT2_PROT_MASK                    0xFFu
#define FTFA_FPROT2_PROT_SHIFT                   0
#define FTFA_FPROT2_PROT_WIDTH                   8
#define FTFA_FPROT2_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT2_PROT_SHIFT))&FTFA_FPROT2_PROT_MASK)
/* FPROT1 Bit Fields */
#define FTFA_FPROT1_PROT_MASK                    0xFFu
#define FTFA_FPROT1_PROT_SHIFT                   0
#define FTFA_FPROT1_PROT_WIDTH                   8
#define FTFA_FPROT1_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT1_PROT_SHIFT))&FTFA_FPROT1_PROT_MASK)
/* FPROT0 Bit Fields */
#define FTFA_FPROT0_PROT_MASK                    0xFFu
#define FTFA_FPROT0_PROT_SHIFT                   0
#define FTFA_FPROT0_PROT_WIDTH                   8
#define FTFA_FPROT0_PROT(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FPROT0_PROT_SHIFT))&FTFA_FPROT0_PROT_MASK)
/* XACCH3 Bit Fields */
#define FTFA_XACCH3_XA_MASK                      0xFFu
#define FTFA_XACCH3_XA_SHIFT                     0
#define FTFA_XACCH3_XA_WIDTH                     8
#define FTFA_XACCH3_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCH3_XA_SHIFT))&FTFA_XACCH3_XA_MASK)
/* XACCH2 Bit Fields */
#define FTFA_XACCH2_XA_MASK                      0xFFu
#define FTFA_XACCH2_XA_SHIFT                     0
#define FTFA_XACCH2_XA_WIDTH                     8
#define FTFA_XACCH2_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCH2_XA_SHIFT))&FTFA_XACCH2_XA_MASK)
/* XACCH1 Bit Fields */
#define FTFA_XACCH1_XA_MASK                      0xFFu
#define FTFA_XACCH1_XA_SHIFT                     0
#define FTFA_XACCH1_XA_WIDTH                     8
#define FTFA_XACCH1_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCH1_XA_SHIFT))&FTFA_XACCH1_XA_MASK)
/* XACCH0 Bit Fields */
#define FTFA_XACCH0_XA_MASK                      0xFFu
#define FTFA_XACCH0_XA_SHIFT                     0
#define FTFA_XACCH0_XA_WIDTH                     8
#define FTFA_XACCH0_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCH0_XA_SHIFT))&FTFA_XACCH0_XA_MASK)
/* XACCL3 Bit Fields */
#define FTFA_XACCL3_XA_MASK                      0xFFu
#define FTFA_XACCL3_XA_SHIFT                     0
#define FTFA_XACCL3_XA_WIDTH                     8
#define FTFA_XACCL3_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCL3_XA_SHIFT))&FTFA_XACCL3_XA_MASK)
/* XACCL2 Bit Fields */
#define FTFA_XACCL2_XA_MASK                      0xFFu
#define FTFA_XACCL2_XA_SHIFT                     0
#define FTFA_XACCL2_XA_WIDTH                     8
#define FTFA_XACCL2_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCL2_XA_SHIFT))&FTFA_XACCL2_XA_MASK)
/* XACCL1 Bit Fields */
#define FTFA_XACCL1_XA_MASK                      0xFFu
#define FTFA_XACCL1_XA_SHIFT                     0
#define FTFA_XACCL1_XA_WIDTH                     8
#define FTFA_XACCL1_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCL1_XA_SHIFT))&FTFA_XACCL1_XA_MASK)
/* XACCL0 Bit Fields */
#define FTFA_XACCL0_XA_MASK                      0xFFu
#define FTFA_XACCL0_XA_SHIFT                     0
#define FTFA_XACCL0_XA_WIDTH                     8
#define FTFA_XACCL0_XA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_XACCL0_XA_SHIFT))&FTFA_XACCL0_XA_MASK)
/* SACCH3 Bit Fields */
#define FTFA_SACCH3_SA_MASK                      0xFFu
#define FTFA_SACCH3_SA_SHIFT                     0
#define FTFA_SACCH3_SA_WIDTH                     8
#define FTFA_SACCH3_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCH3_SA_SHIFT))&FTFA_SACCH3_SA_MASK)
/* SACCH2 Bit Fields */
#define FTFA_SACCH2_SA_MASK                      0xFFu
#define FTFA_SACCH2_SA_SHIFT                     0
#define FTFA_SACCH2_SA_WIDTH                     8
#define FTFA_SACCH2_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCH2_SA_SHIFT))&FTFA_SACCH2_SA_MASK)
/* SACCH1 Bit Fields */
#define FTFA_SACCH1_SA_MASK                      0xFFu
#define FTFA_SACCH1_SA_SHIFT                     0
#define FTFA_SACCH1_SA_WIDTH                     8
#define FTFA_SACCH1_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCH1_SA_SHIFT))&FTFA_SACCH1_SA_MASK)
/* SACCH0 Bit Fields */
#define FTFA_SACCH0_SA_MASK                      0xFFu
#define FTFA_SACCH0_SA_SHIFT                     0
#define FTFA_SACCH0_SA_WIDTH                     8
#define FTFA_SACCH0_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCH0_SA_SHIFT))&FTFA_SACCH0_SA_MASK)
/* SACCL3 Bit Fields */
#define FTFA_SACCL3_SA_MASK                      0xFFu
#define FTFA_SACCL3_SA_SHIFT                     0
#define FTFA_SACCL3_SA_WIDTH                     8
#define FTFA_SACCL3_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCL3_SA_SHIFT))&FTFA_SACCL3_SA_MASK)
/* SACCL2 Bit Fields */
#define FTFA_SACCL2_SA_MASK                      0xFFu
#define FTFA_SACCL2_SA_SHIFT                     0
#define FTFA_SACCL2_SA_WIDTH                     8
#define FTFA_SACCL2_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCL2_SA_SHIFT))&FTFA_SACCL2_SA_MASK)
/* SACCL1 Bit Fields */
#define FTFA_SACCL1_SA_MASK                      0xFFu
#define FTFA_SACCL1_SA_SHIFT                     0
#define FTFA_SACCL1_SA_WIDTH                     8
#define FTFA_SACCL1_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCL1_SA_SHIFT))&FTFA_SACCL1_SA_MASK)
/* SACCL0 Bit Fields */
#define FTFA_SACCL0_SA_MASK                      0xFFu
#define FTFA_SACCL0_SA_SHIFT                     0
#define FTFA_SACCL0_SA_WIDTH                     8
#define FTFA_SACCL0_SA(x)                        (((uint8_t)(((uint8_t)(x))<<FTFA_SACCL0_SA_SHIFT))&FTFA_SACCL0_SA_MASK)
/* FACSS Bit Fields */
#define FTFA_FACSS_SGSIZE_MASK                   0xFFu
#define FTFA_FACSS_SGSIZE_SHIFT                  0
#define FTFA_FACSS_SGSIZE_WIDTH                  8
#define FTFA_FACSS_SGSIZE(x)                     (((uint8_t)(((uint8_t)(x))<<FTFA_FACSS_SGSIZE_SHIFT))&FTFA_FACSS_SGSIZE_MASK)
/* FACSN Bit Fields */
#define FTFA_FACSN_NUMSG_MASK                    0xFFu
#define FTFA_FACSN_NUMSG_SHIFT                   0
#define FTFA_FACSN_NUMSG_WIDTH                   8
#define FTFA_FACSN_NUMSG(x)                      (((uint8_t)(((uint8_t)(x))<<FTFA_FACSN_NUMSG_SHIFT))&FTFA_FACSN_NUMSG_MASK)

/*!
 * @}
 */ /* end of group FTFA_Register_Masks */


/* FTFA - Peripheral instance base addresses */
/** Peripheral FTFA base address */
#define FTFA_BASE                                (0x40020000u)
/** Peripheral FTFA base pointer */
#define FTFA                                     ((FTFA_TypeDef *)FTFA_BASE)
#define FTFA_BASE_PTR                            (FTFA)
/** Array initializer of FTFA peripheral base addresses */
#define FTFA_BASE_ADDRS                          { FTFA_BASE }
/** Array initializer of FTFA peripheral base pointers */
#define FTFA_BASE_PTRS                           { FTFA }
/** Interrupt vectors for the FTFA peripheral type */
#define FTFA_COMMAND_COMPLETE_IRQS               { FTF_IRQn }
#define FTFA_READ_COLLISION_IRQS                 { Read_Collision_IRQn }

/* ----------------------------------------------------------------------------
   -- FTFA - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTFA_Register_Accessor_Macros FTFA - Register accessor macros
 * @{
 */


/* FTFA - Register instance definitions */
/* FTFA */
#define FTFA_FSTAT                               FTFA_FSTAT_REG(FTFA)
#define FTFA_FCNFG                               FTFA_FCNFG_REG(FTFA)
#define FTFA_FSEC                                FTFA_FSEC_REG(FTFA)
#define FTFA_FOPT                                FTFA_FOPT_REG(FTFA)
#define FTFA_FCCOB3                              FTFA_FCCOB3_REG(FTFA)
#define FTFA_FCCOB2                              FTFA_FCCOB2_REG(FTFA)
#define FTFA_FCCOB1                              FTFA_FCCOB1_REG(FTFA)
#define FTFA_FCCOB0                              FTFA_FCCOB0_REG(FTFA)
#define FTFA_FCCOB7                              FTFA_FCCOB7_REG(FTFA)
#define FTFA_FCCOB6                              FTFA_FCCOB6_REG(FTFA)
#define FTFA_FCCOB5                              FTFA_FCCOB5_REG(FTFA)
#define FTFA_FCCOB4                              FTFA_FCCOB4_REG(FTFA)
#define FTFA_FCCOBB                              FTFA_FCCOBB_REG(FTFA)
#define FTFA_FCCOBA                              FTFA_FCCOBA_REG(FTFA)
#define FTFA_FCCOB9                              FTFA_FCCOB9_REG(FTFA)
#define FTFA_FCCOB8                              FTFA_FCCOB8_REG(FTFA)
#define FTFA_FPROT3                              FTFA_FPROT3_REG(FTFA)
#define FTFA_FPROT2                              FTFA_FPROT2_REG(FTFA)
#define FTFA_FPROT1                              FTFA_FPROT1_REG(FTFA)
#define FTFA_FPROT0                              FTFA_FPROT0_REG(FTFA)
#define FTFA_XACCH3                              FTFA_XACCH3_REG(FTFA)
#define FTFA_XACCH2                              FTFA_XACCH2_REG(FTFA)
#define FTFA_XACCH1                              FTFA_XACCH1_REG(FTFA)
#define FTFA_XACCH0                              FTFA_XACCH0_REG(FTFA)
#define FTFA_XACCL3                              FTFA_XACCL3_REG(FTFA)
#define FTFA_XACCL2                              FTFA_XACCL2_REG(FTFA)
#define FTFA_XACCL1                              FTFA_XACCL1_REG(FTFA)
#define FTFA_XACCL0                              FTFA_XACCL0_REG(FTFA)
#define FTFA_SACCH3                              FTFA_SACCH3_REG(FTFA)
#define FTFA_SACCH2                              FTFA_SACCH2_REG(FTFA)
#define FTFA_SACCH1                              FTFA_SACCH1_REG(FTFA)
#define FTFA_SACCH0                              FTFA_SACCH0_REG(FTFA)
#define FTFA_SACCL3                              FTFA_SACCL3_REG(FTFA)
#define FTFA_SACCL2                              FTFA_SACCL2_REG(FTFA)
#define FTFA_SACCL1                              FTFA_SACCL1_REG(FTFA)
#define FTFA_SACCL0                              FTFA_SACCL0_REG(FTFA)
#define FTFA_FACSS                               FTFA_FACSS_REG(FTFA)
#define FTFA_FACSN                               FTFA_FACSN_REG(FTFA)

/*!
 * @}
 */ /* end of group FTFA_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group FTFA_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- FTM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Peripheral_Access_Layer FTM Peripheral Access Layer
 * @{
 */

/** FTM - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC;                                /**< Status And Control, offset: 0x0 */
  __IO uint32_t CNT;                               /**< Counter, offset: 0x4 */
  __IO uint32_t MOD;                               /**< Modulo, offset: 0x8 */
  struct {                                         /* offset: 0xC, array step: 0x8 */
    __IO uint32_t CnSC;                              /**< Channel (n) Status And Control, array offset: 0xC, array step: 0x8 */
    __IO uint32_t CnV;                               /**< Channel (n) Value, array offset: 0x10, array step: 0x8 */
  } CONTROLS[8];
  __IO uint32_t CNTIN;                             /**< Counter Initial Value, offset: 0x4C */
  __IO uint32_t STATUS;                            /**< Capture And Compare Status, offset: 0x50 */
  __IO uint32_t MODE;                              /**< Features Mode Selection, offset: 0x54 */
  __IO uint32_t SYNC;                              /**< Synchronization, offset: 0x58 */
  __IO uint32_t OUTINIT;                           /**< Initial State For Channels Output, offset: 0x5C */
  __IO uint32_t OUTMASK;                           /**< Output Mask, offset: 0x60 */
  __IO uint32_t COMBINE;                           /**< Function For Linked Channels, offset: 0x64 */
  __IO uint32_t DEADTIME;                          /**< Deadtime Insertion Control, offset: 0x68 */
  __IO uint32_t EXTTRIG;                           /**< FTM External Trigger, offset: 0x6C */
  __IO uint32_t POL;                               /**< Channels Polarity, offset: 0x70 */
  __IO uint32_t FMS;                               /**< Fault Mode Status, offset: 0x74 */
  __IO uint32_t FILTER;                            /**< Input Capture Filter Control, offset: 0x78 */
  __IO uint32_t FLTCTRL;                           /**< Fault Control, offset: 0x7C */
  __IO uint32_t QDCTRL;                            /**< Quadrature Decoder Control And Status, offset: 0x80 */
  __IO uint32_t CONF;                              /**< Configuration, offset: 0x84 */
  __IO uint32_t FLTPOL;                            /**< FTM Fault Input Polarity, offset: 0x88 */
  __IO uint32_t SYNCONF;                           /**< Synchronization Configuration, offset: 0x8C */
  __IO uint32_t INVCTRL;                           /**< FTM Inverting Control, offset: 0x90 */
  __IO uint32_t SWOCTRL;                           /**< FTM Software Output Control, offset: 0x94 */
  __IO uint32_t PWMLOAD;                           /**< FTM PWM Load, offset: 0x98 */
} FTM_TypeDef, *FTM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- FTM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Accessor_Macros FTM - Register accessor macros
 * @{
 */


/* FTM - Register accessors */
#define FTM_SC_REG(base)                         ((base)->SC)
#define FTM_CNT_REG(base)                        ((base)->CNT)
#define FTM_MOD_REG(base)                        ((base)->MOD)
#define FTM_CnSC_REG(base,index)                 ((base)->CONTROLS[index].CnSC)
#define FTM_CnSC_COUNT                           8
#define FTM_CnV_REG(base,index)                  ((base)->CONTROLS[index].CnV)
#define FTM_CnV_COUNT                            8
#define FTM_CNTIN_REG(base)                      ((base)->CNTIN)
#define FTM_STATUS_REG(base)                     ((base)->STATUS)
#define FTM_MODE_REG(base)                       ((base)->MODE)
#define FTM_SYNC_REG(base)                       ((base)->SYNC)
#define FTM_OUTINIT_REG(base)                    ((base)->OUTINIT)
#define FTM_OUTMASK_REG(base)                    ((base)->OUTMASK)
#define FTM_COMBINE_REG(base)                    ((base)->COMBINE)
#define FTM_DEADTIME_REG(base)                   ((base)->DEADTIME)
#define FTM_EXTTRIG_REG(base)                    ((base)->EXTTRIG)
#define FTM_POL_REG(base)                        ((base)->POL)
#define FTM_FMS_REG(base)                        ((base)->FMS)
#define FTM_FILTER_REG(base)                     ((base)->FILTER)
#define FTM_FLTCTRL_REG(base)                    ((base)->FLTCTRL)
#define FTM_QDCTRL_REG(base)                     ((base)->QDCTRL)
#define FTM_CONF_REG(base)                       ((base)->CONF)
#define FTM_FLTPOL_REG(base)                     ((base)->FLTPOL)
#define FTM_SYNCONF_REG(base)                    ((base)->SYNCONF)
#define FTM_INVCTRL_REG(base)                    ((base)->INVCTRL)
#define FTM_SWOCTRL_REG(base)                    ((base)->SWOCTRL)
#define FTM_PWMLOAD_REG(base)                    ((base)->PWMLOAD)

/*!
 * @}
 */ /* end of group FTM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- FTM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Masks FTM Register Masks
 * @{
 */

/* SC Bit Fields */
#define FTM_SC_PS_MASK                           0x7u
#define FTM_SC_PS_SHIFT                          0
#define FTM_SC_PS_WIDTH                          3
#define FTM_SC_PS(x)                             (((uint32_t)(((uint32_t)(x))<<FTM_SC_PS_SHIFT))&FTM_SC_PS_MASK)
#define FTM_SC_CLKS_MASK                         0x18u
#define FTM_SC_CLKS_SHIFT                        3
#define FTM_SC_CLKS_WIDTH                        2
#define FTM_SC_CLKS(x)                           (((uint32_t)(((uint32_t)(x))<<FTM_SC_CLKS_SHIFT))&FTM_SC_CLKS_MASK)
#define FTM_SC_CPWMS		((uint32_t)0x20u)
#define FTM_SC_TOIE		((uint32_t)0x40u)
#define FTM_SC_TOF		((uint32_t)0x80u)
/* CNT Bit Fields */
#define FTM_CNT_COUNT_MASK                       0xFFFFu
#define FTM_CNT_COUNT_SHIFT                      0
#define FTM_CNT_COUNT_WIDTH                      16
#define FTM_CNT_COUNT(x)                         (((uint32_t)(((uint32_t)(x))<<FTM_CNT_COUNT_SHIFT))&FTM_CNT_COUNT_MASK)
/* MOD Bit Fields */
#define FTM_MOD_MOD_MASK                         0xFFFFu
#define FTM_MOD_MOD_SHIFT                        0
#define FTM_MOD_MOD_WIDTH                        16
#define FTM_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x))<<FTM_MOD_MOD_SHIFT))&FTM_MOD_MOD_MASK)
/* CnSC Bit Fields */
#define FTM_CnSC_DMA		((uint32_t)0x1u)
#define FTM_CnSC_ICRST		((uint32_t)0x2u)
#define FTM_CnSC_ELSA		((uint32_t)0x4u)
#define FTM_CnSC_ELSB		((uint32_t)0x8u)
#define FTM_CnSC_MSA		((uint32_t)0x10u)
#define FTM_CnSC_MSB		((uint32_t)0x20u)
#define FTM_CnSC_CHIE		((uint32_t)0x40u)
#define FTM_CnSC_CHF		((uint32_t)0x80u)
/* CnV Bit Fields */
#define FTM_CnV_VAL_MASK                         0xFFFFu
#define FTM_CnV_VAL_SHIFT                        0
#define FTM_CnV_VAL_WIDTH                        16
#define FTM_CnV_VAL(x)                           (((uint32_t)(((uint32_t)(x))<<FTM_CnV_VAL_SHIFT))&FTM_CnV_VAL_MASK)
/* CNTIN Bit Fields */
#define FTM_CNTIN_INIT_MASK                      0xFFFFu
#define FTM_CNTIN_INIT_SHIFT                     0
#define FTM_CNTIN_INIT_WIDTH                     16
#define FTM_CNTIN_INIT(x)                        (((uint32_t)(((uint32_t)(x))<<FTM_CNTIN_INIT_SHIFT))&FTM_CNTIN_INIT_MASK)
/* STATUS Bit Fields */
#define FTM_STATUS_CH0F		((uint32_t)0x1u)
#define FTM_STATUS_CH1F		((uint32_t)0x2u)
#define FTM_STATUS_CH2F		((uint32_t)0x4u)
#define FTM_STATUS_CH3F		((uint32_t)0x8u)
#define FTM_STATUS_CH4F		((uint32_t)0x10u)
#define FTM_STATUS_CH5F		((uint32_t)0x20u)
#define FTM_STATUS_CH6F		((uint32_t)0x40u)
#define FTM_STATUS_CH7F		((uint32_t)0x80u)
/* MODE Bit Fields */
#define FTM_MODE_FTMEN		((uint32_t)0x1u)
#define FTM_MODE_INIT		((uint32_t)0x2u)
#define FTM_MODE_WPDIS		((uint32_t)0x4u)
#define FTM_MODE_PWMSYNC		((uint32_t)0x8u)
#define FTM_MODE_CAPTEST		((uint32_t)0x10u)
#define FTM_MODE_FAULTM_MASK                     0x60u
#define FTM_MODE_FAULTM_SHIFT                    5
#define FTM_MODE_FAULTM_WIDTH                    2
#define FTM_MODE_FAULTM(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_MODE_FAULTM_SHIFT))&FTM_MODE_FAULTM_MASK)
#define FTM_MODE_FAULTIE		((uint32_t)0x80u)
/* SYNC Bit Fields */
#define FTM_SYNC_CNTMIN		((uint32_t)0x1u)
#define FTM_SYNC_CNTMAX		((uint32_t)0x2u)
#define FTM_SYNC_REINIT		((uint32_t)0x4u)
#define FTM_SYNC_SYNCHOM		((uint32_t)0x8u)
#define FTM_SYNC_TRIG0		((uint32_t)0x10u)
#define FTM_SYNC_TRIG1		((uint32_t)0x20u)
#define FTM_SYNC_TRIG2		((uint32_t)0x40u)
#define FTM_SYNC_SWSYNC		((uint32_t)0x80u)
/* OUTINIT Bit Fields */
#define FTM_OUTINIT_CH0OI		((uint32_t)0x1u)
#define FTM_OUTINIT_CH1OI		((uint32_t)0x2u)
#define FTM_OUTINIT_CH2OI		((uint32_t)0x4u)
#define FTM_OUTINIT_CH3OI		((uint32_t)0x8u)
#define FTM_OUTINIT_CH4OI		((uint32_t)0x10u)
#define FTM_OUTINIT_CH5OI		((uint32_t)0x20u)
#define FTM_OUTINIT_CH6OI		((uint32_t)0x40u)
#define FTM_OUTINIT_CH7OI		((uint32_t)0x80u)
/* OUTMASK Bit Fields */
#define FTM_OUTMASK_CH0OM		((uint32_t)0x1u)
#define FTM_OUTMASK_CH1OM		((uint32_t)0x2u)
#define FTM_OUTMASK_CH2OM		((uint32_t)0x4u)
#define FTM_OUTMASK_CH3OM		((uint32_t)0x8u)
#define FTM_OUTMASK_CH4OM		((uint32_t)0x10u)
#define FTM_OUTMASK_CH5OM		((uint32_t)0x20u)
#define FTM_OUTMASK_CH6OM		((uint32_t)0x40u)
#define FTM_OUTMASK_CH7OM		((uint32_t)0x80u)
/* COMBINE Bit Fields */
#define FTM_COMBINE_COMBINE0		((uint32_t)0x1u)
#define FTM_COMBINE_COMP0		((uint32_t)0x2u)
#define FTM_COMBINE_DECAPEN0		((uint32_t)0x4u)
#define FTM_COMBINE_DECAP0		((uint32_t)0x8u)
#define FTM_COMBINE_DTEN0		((uint32_t)0x10u)
#define FTM_COMBINE_SYNCEN0		((uint32_t)0x20u)
#define FTM_COMBINE_FAULTEN0		((uint32_t)0x40u)
#define FTM_COMBINE_COMBINE1		((uint32_t)0x100u)
#define FTM_COMBINE_COMP1		((uint32_t)0x200u)
#define FTM_COMBINE_DECAPEN1		((uint32_t)0x400u)
#define FTM_COMBINE_DECAP1		((uint32_t)0x800u)
#define FTM_COMBINE_DTEN1		((uint32_t)0x1000u)
#define FTM_COMBINE_SYNCEN1		((uint32_t)0x2000u)
#define FTM_COMBINE_FAULTEN1		((uint32_t)0x4000u)
#define FTM_COMBINE_COMBINE2		((uint32_t)0x10000u)
#define FTM_COMBINE_COMP2		((uint32_t)0x20000u)
#define FTM_COMBINE_DECAPEN2		((uint32_t)0x40000u)
#define FTM_COMBINE_DECAP2		((uint32_t)0x80000u)
#define FTM_COMBINE_DTEN2		((uint32_t)0x100000u)
#define FTM_COMBINE_SYNCEN2		((uint32_t)0x200000u)
#define FTM_COMBINE_FAULTEN2		((uint32_t)0x400000u)
#define FTM_COMBINE_COMBINE3		((uint32_t)0x1000000u)
#define FTM_COMBINE_COMP3		((uint32_t)0x2000000u)
#define FTM_COMBINE_DECAPEN3		((uint32_t)0x4000000u)
#define FTM_COMBINE_DECAP3		((uint32_t)0x8000000u)
#define FTM_COMBINE_DTEN3		((uint32_t)0x10000000u)
#define FTM_COMBINE_SYNCEN3		((uint32_t)0x20000000u)
#define FTM_COMBINE_FAULTEN3		((uint32_t)0x40000000u)
/* DEADTIME Bit Fields */
#define FTM_DEADTIME_DTVAL_MASK                  0x3Fu
#define FTM_DEADTIME_DTVAL_SHIFT                 0
#define FTM_DEADTIME_DTVAL_WIDTH                 6
#define FTM_DEADTIME_DTVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_DEADTIME_DTVAL_SHIFT))&FTM_DEADTIME_DTVAL_MASK)
#define FTM_DEADTIME_DTPS_MASK                   0xC0u
#define FTM_DEADTIME_DTPS_SHIFT                  6
#define FTM_DEADTIME_DTPS_WIDTH                  2
#define FTM_DEADTIME_DTPS(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_DEADTIME_DTPS_SHIFT))&FTM_DEADTIME_DTPS_MASK)
/* EXTTRIG Bit Fields */
#define FTM_EXTTRIG_CH2TRIG		((uint32_t)0x1u)
#define FTM_EXTTRIG_CH3TRIG		((uint32_t)0x2u)
#define FTM_EXTTRIG_CH4TRIG		((uint32_t)0x4u)
#define FTM_EXTTRIG_CH5TRIG		((uint32_t)0x8u)
#define FTM_EXTTRIG_CH0TRIG		((uint32_t)0x10u)
#define FTM_EXTTRIG_CH1TRIG		((uint32_t)0x20u)
#define FTM_EXTTRIG_INITTRIGEN		((uint32_t)0x40u)
#define FTM_EXTTRIG_TRIGF		((uint32_t)0x80u)
/* POL Bit Fields */
#define FTM_POL_POL0		((uint32_t)0x1u)
#define FTM_POL_POL1		((uint32_t)0x2u)
#define FTM_POL_POL2		((uint32_t)0x4u)
#define FTM_POL_POL3		((uint32_t)0x8u)
#define FTM_POL_POL4		((uint32_t)0x10u)
#define FTM_POL_POL5		((uint32_t)0x20u)
#define FTM_POL_POL6		((uint32_t)0x40u)
#define FTM_POL_POL7		((uint32_t)0x80u)
/* FMS Bit Fields */
#define FTM_FMS_FAULTF0		((uint32_t)0x1u)
#define FTM_FMS_FAULTF1		((uint32_t)0x2u)
#define FTM_FMS_FAULTF2		((uint32_t)0x4u)
#define FTM_FMS_FAULTF3		((uint32_t)0x8u)
#define FTM_FMS_FAULTIN		((uint32_t)0x20u)
#define FTM_FMS_WPEN		((uint32_t)0x40u)
#define FTM_FMS_FAULTF		((uint32_t)0x80u)
/* FILTER Bit Fields */
#define FTM_FILTER_CH0FVAL_MASK                  0xFu
#define FTM_FILTER_CH0FVAL_SHIFT                 0
#define FTM_FILTER_CH0FVAL_WIDTH                 4
#define FTM_FILTER_CH0FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH0FVAL_SHIFT))&FTM_FILTER_CH0FVAL_MASK)
#define FTM_FILTER_CH1FVAL_MASK                  0xF0u
#define FTM_FILTER_CH1FVAL_SHIFT                 4
#define FTM_FILTER_CH1FVAL_WIDTH                 4
#define FTM_FILTER_CH1FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH1FVAL_SHIFT))&FTM_FILTER_CH1FVAL_MASK)
#define FTM_FILTER_CH2FVAL_MASK                  0xF00u
#define FTM_FILTER_CH2FVAL_SHIFT                 8
#define FTM_FILTER_CH2FVAL_WIDTH                 4
#define FTM_FILTER_CH2FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH2FVAL_SHIFT))&FTM_FILTER_CH2FVAL_MASK)
#define FTM_FILTER_CH3FVAL_MASK                  0xF000u
#define FTM_FILTER_CH3FVAL_SHIFT                 12
#define FTM_FILTER_CH3FVAL_WIDTH                 4
#define FTM_FILTER_CH3FVAL(x)                    (((uint32_t)(((uint32_t)(x))<<FTM_FILTER_CH3FVAL_SHIFT))&FTM_FILTER_CH3FVAL_MASK)
/* FLTCTRL Bit Fields */
#define FTM_FLTCTRL_FAULT0EN		((uint32_t)0x1u)
#define FTM_FLTCTRL_FAULT1EN		((uint32_t)0x2u)
#define FTM_FLTCTRL_FAULT2EN		((uint32_t)0x4u)
#define FTM_FLTCTRL_FAULT3EN		((uint32_t)0x8u)
#define FTM_FLTCTRL_FFLTR0EN		((uint32_t)0x10u)
#define FTM_FLTCTRL_FFLTR1EN		((uint32_t)0x20u)
#define FTM_FLTCTRL_FFLTR2EN		((uint32_t)0x40u)
#define FTM_FLTCTRL_FFLTR3EN		((uint32_t)0x80u)
#define FTM_FLTCTRL_FFVAL_MASK                   0xF00u
#define FTM_FLTCTRL_FFVAL_SHIFT                  8
#define FTM_FLTCTRL_FFVAL_WIDTH                  4
#define FTM_FLTCTRL_FFVAL(x)                     (((uint32_t)(((uint32_t)(x))<<FTM_FLTCTRL_FFVAL_SHIFT))&FTM_FLTCTRL_FFVAL_MASK)
/* QDCTRL Bit Fields */
#define FTM_QDCTRL_QUADEN		((uint32_t)0x1u)
#define FTM_QDCTRL_TOFDIR		((uint32_t)0x2u)
#define FTM_QDCTRL_QUADIR		((uint32_t)0x4u)
#define FTM_QDCTRL_QUADMODE		((uint32_t)0x8u)
#define FTM_QDCTRL_PHBPOL		((uint32_t)0x10u)
#define FTM_QDCTRL_PHAPOL		((uint32_t)0x20u)
#define FTM_QDCTRL_PHBFLTREN		((uint32_t)0x40u)
#define FTM_QDCTRL_PHAFLTREN		((uint32_t)0x80u)
/* CONF Bit Fields */
#define FTM_CONF_NUMTOF_MASK                     0x1Fu
#define FTM_CONF_NUMTOF_SHIFT                    0
#define FTM_CONF_NUMTOF_WIDTH                    5
#define FTM_CONF_NUMTOF(x)                       (((uint32_t)(((uint32_t)(x))<<FTM_CONF_NUMTOF_SHIFT))&FTM_CONF_NUMTOF_MASK)
#define FTM_CONF_BDMMODE_MASK                    0xC0u
#define FTM_CONF_BDMMODE_SHIFT                   6
#define FTM_CONF_BDMMODE_WIDTH                   2
#define FTM_CONF_BDMMODE(x)                      (((uint32_t)(((uint32_t)(x))<<FTM_CONF_BDMMODE_SHIFT))&FTM_CONF_BDMMODE_MASK)
#define FTM_CONF_GTBEEN		((uint32_t)0x200u)
#define FTM_CONF_GTBEOUT		((uint32_t)0x400u)
/* FLTPOL Bit Fields */
#define FTM_FLTPOL_FLT0POL		((uint32_t)0x1u)
#define FTM_FLTPOL_FLT1POL		((uint32_t)0x2u)
#define FTM_FLTPOL_FLT2POL		((uint32_t)0x4u)
#define FTM_FLTPOL_FLT3POL		((uint32_t)0x8u)
/* SYNCONF Bit Fields */
#define FTM_SYNCONF_HWTRIGMODE		((uint32_t)0x1u)
#define FTM_SYNCONF_CNTINC		((uint32_t)0x4u)
#define FTM_SYNCONF_INVC		((uint32_t)0x10u)
#define FTM_SYNCONF_SWOC		((uint32_t)0x20u)
#define FTM_SYNCONF_SYNCMODE		((uint32_t)0x80u)
#define FTM_SYNCONF_SWRSTCNT		((uint32_t)0x100u)
#define FTM_SYNCONF_SWWRBUF		((uint32_t)0x200u)
#define FTM_SYNCONF_SWOM		((uint32_t)0x400u)
#define FTM_SYNCONF_SWINVC		((uint32_t)0x800u)
#define FTM_SYNCONF_SWSOC		((uint32_t)0x1000u)
#define FTM_SYNCONF_HWRSTCNT		((uint32_t)0x10000u)
#define FTM_SYNCONF_HWWRBUF		((uint32_t)0x20000u)
#define FTM_SYNCONF_HWOM		((uint32_t)0x40000u)
#define FTM_SYNCONF_HWINVC		((uint32_t)0x80000u)
#define FTM_SYNCONF_HWSOC		((uint32_t)0x100000u)
/* INVCTRL Bit Fields */
#define FTM_INVCTRL_INV0EN		((uint32_t)0x1u)
#define FTM_INVCTRL_INV1EN		((uint32_t)0x2u)
#define FTM_INVCTRL_INV2EN		((uint32_t)0x4u)
#define FTM_INVCTRL_INV3EN		((uint32_t)0x8u)
/* SWOCTRL Bit Fields */
#define FTM_SWOCTRL_CH0OC		((uint32_t)0x1u)
#define FTM_SWOCTRL_CH1OC		((uint32_t)0x2u)
#define FTM_SWOCTRL_CH2OC		((uint32_t)0x4u)
#define FTM_SWOCTRL_CH3OC		((uint32_t)0x8u)
#define FTM_SWOCTRL_CH4OC		((uint32_t)0x10u)
#define FTM_SWOCTRL_CH5OC		((uint32_t)0x20u)
#define FTM_SWOCTRL_CH6OC		((uint32_t)0x40u)
#define FTM_SWOCTRL_CH7OC		((uint32_t)0x80u)
#define FTM_SWOCTRL_CH0OCV		((uint32_t)0x100u)
#define FTM_SWOCTRL_CH1OCV		((uint32_t)0x200u)
#define FTM_SWOCTRL_CH2OCV		((uint32_t)0x400u)
#define FTM_SWOCTRL_CH3OCV		((uint32_t)0x800u)
#define FTM_SWOCTRL_CH4OCV		((uint32_t)0x1000u)
#define FTM_SWOCTRL_CH5OCV		((uint32_t)0x2000u)
#define FTM_SWOCTRL_CH6OCV		((uint32_t)0x4000u)
#define FTM_SWOCTRL_CH7OCV		((uint32_t)0x8000u)
/* PWMLOAD Bit Fields */
#define FTM_PWMLOAD_CH0SEL		((uint32_t)0x1u)
#define FTM_PWMLOAD_CH1SEL		((uint32_t)0x2u)
#define FTM_PWMLOAD_CH2SEL		((uint32_t)0x4u)
#define FTM_PWMLOAD_CH3SEL		((uint32_t)0x8u)
#define FTM_PWMLOAD_CH4SEL		((uint32_t)0x10u)
#define FTM_PWMLOAD_CH5SEL		((uint32_t)0x20u)
#define FTM_PWMLOAD_CH6SEL		((uint32_t)0x40u)
#define FTM_PWMLOAD_CH7SEL		((uint32_t)0x80u)
#define FTM_PWMLOAD_LDOK		((uint32_t)0x200u)

/*!
 * @}
 */ /* end of group FTM_Register_Masks */


/* FTM - Peripheral instance base addresses */
/** Peripheral FTM0 base address */
#define FTM0_BASE                                (0x40038000u)
/** Peripheral FTM0 base pointer */
#define FTM0                                     ((FTM_TypeDef *)FTM0_BASE)
#define FTM0_BASE_PTR                            (FTM0)
/** Peripheral FTM1 base address */
#define FTM1_BASE                                (0x40039000u)
/** Peripheral FTM1 base pointer */
#define FTM1                                     ((FTM_TypeDef *)FTM1_BASE)
#define FTM1_BASE_PTR                            (FTM1)
/** Peripheral FTM2 base address */
#define FTM2_BASE                                (0x4003A000u)
/** Peripheral FTM2 base pointer */
#define FTM2                                     ((FTM_TypeDef *)FTM2_BASE)
#define FTM2_BASE_PTR                            (FTM2)
/** Peripheral FTM3 base address */
#define FTM3_BASE                                (0x40026000u)
/** Peripheral FTM3 base pointer */
#define FTM3                                     ((FTM_TypeDef *)FTM3_BASE)
#define FTM3_BASE_PTR                            (FTM3)
/** Array initializer of FTM peripheral base addresses */
#define FTM_BASE_ADDRS                           { FTM0_BASE, FTM1_BASE, FTM2_BASE, FTM3_BASE }
/** Array initializer of FTM peripheral base pointers */
#define FTM_BASE_PTRS                            { FTM0, FTM1, FTM2, FTM3 }
/** Interrupt vectors for the FTM peripheral type */
#define FTM_IRQS                                 { FTM0_IRQn, FTM1_IRQn, FTM2_IRQn, FTM3_IRQn }

/* ----------------------------------------------------------------------------
   -- FTM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup FTM_Register_Accessor_Macros FTM - Register accessor macros
 * @{
 */


/* FTM - Register instance definitions */
/* FTM0 */
#define FTM0_SC                                  FTM_SC_REG(FTM0)
#define FTM0_CNT                                 FTM_CNT_REG(FTM0)
#define FTM0_MOD                                 FTM_MOD_REG(FTM0)
#define FTM0_C0SC                                FTM_CnSC_REG(FTM0,0)
#define FTM0_C0V                                 FTM_CnV_REG(FTM0,0)
#define FTM0_C1SC                                FTM_CnSC_REG(FTM0,1)
#define FTM0_C1V                                 FTM_CnV_REG(FTM0,1)
#define FTM0_C2SC                                FTM_CnSC_REG(FTM0,2)
#define FTM0_C2V                                 FTM_CnV_REG(FTM0,2)
#define FTM0_C3SC                                FTM_CnSC_REG(FTM0,3)
#define FTM0_C3V                                 FTM_CnV_REG(FTM0,3)
#define FTM0_C4SC                                FTM_CnSC_REG(FTM0,4)
#define FTM0_C4V                                 FTM_CnV_REG(FTM0,4)
#define FTM0_C5SC                                FTM_CnSC_REG(FTM0,5)
#define FTM0_C5V                                 FTM_CnV_REG(FTM0,5)
#define FTM0_C6SC                                FTM_CnSC_REG(FTM0,6)
#define FTM0_C6V                                 FTM_CnV_REG(FTM0,6)
#define FTM0_C7SC                                FTM_CnSC_REG(FTM0,7)
#define FTM0_C7V                                 FTM_CnV_REG(FTM0,7)
#define FTM0_CNTIN                               FTM_CNTIN_REG(FTM0)
#define FTM0_STATUS                              FTM_STATUS_REG(FTM0)
#define FTM0_MODE                                FTM_MODE_REG(FTM0)
#define FTM0_SYNC                                FTM_SYNC_REG(FTM0)
#define FTM0_OUTINIT                             FTM_OUTINIT_REG(FTM0)
#define FTM0_OUTMASK                             FTM_OUTMASK_REG(FTM0)
#define FTM0_COMBINE                             FTM_COMBINE_REG(FTM0)
#define FTM0_DEADTIME                            FTM_DEADTIME_REG(FTM0)
#define FTM0_EXTTRIG                             FTM_EXTTRIG_REG(FTM0)
#define FTM0_POL                                 FTM_POL_REG(FTM0)
#define FTM0_FMS                                 FTM_FMS_REG(FTM0)
#define FTM0_FILTER                              FTM_FILTER_REG(FTM0)
#define FTM0_FLTCTRL                             FTM_FLTCTRL_REG(FTM0)
#define FTM0_QDCTRL                              FTM_QDCTRL_REG(FTM0)
#define FTM0_CONF                                FTM_CONF_REG(FTM0)
#define FTM0_FLTPOL                              FTM_FLTPOL_REG(FTM0)
#define FTM0_SYNCONF                             FTM_SYNCONF_REG(FTM0)
#define FTM0_INVCTRL                             FTM_INVCTRL_REG(FTM0)
#define FTM0_SWOCTRL                             FTM_SWOCTRL_REG(FTM0)
#define FTM0_PWMLOAD                             FTM_PWMLOAD_REG(FTM0)
/* FTM1 */
#define FTM1_SC                                  FTM_SC_REG(FTM1)
#define FTM1_CNT                                 FTM_CNT_REG(FTM1)
#define FTM1_MOD                                 FTM_MOD_REG(FTM1)
#define FTM1_C0SC                                FTM_CnSC_REG(FTM1,0)
#define FTM1_C0V                                 FTM_CnV_REG(FTM1,0)
#define FTM1_C1SC                                FTM_CnSC_REG(FTM1,1)
#define FTM1_C1V                                 FTM_CnV_REG(FTM1,1)
#define FTM1_CNTIN                               FTM_CNTIN_REG(FTM1)
#define FTM1_STATUS                              FTM_STATUS_REG(FTM1)
#define FTM1_MODE                                FTM_MODE_REG(FTM1)
#define FTM1_SYNC                                FTM_SYNC_REG(FTM1)
#define FTM1_OUTINIT                             FTM_OUTINIT_REG(FTM1)
#define FTM1_OUTMASK                             FTM_OUTMASK_REG(FTM1)
#define FTM1_COMBINE                             FTM_COMBINE_REG(FTM1)
#define FTM1_DEADTIME                            FTM_DEADTIME_REG(FTM1)
#define FTM1_EXTTRIG                             FTM_EXTTRIG_REG(FTM1)
#define FTM1_POL                                 FTM_POL_REG(FTM1)
#define FTM1_FMS                                 FTM_FMS_REG(FTM1)
#define FTM1_FILTER                              FTM_FILTER_REG(FTM1)
#define FTM1_FLTCTRL                             FTM_FLTCTRL_REG(FTM1)
#define FTM1_QDCTRL                              FTM_QDCTRL_REG(FTM1)
#define FTM1_CONF                                FTM_CONF_REG(FTM1)
#define FTM1_FLTPOL                              FTM_FLTPOL_REG(FTM1)
#define FTM1_SYNCONF                             FTM_SYNCONF_REG(FTM1)
#define FTM1_INVCTRL                             FTM_INVCTRL_REG(FTM1)
#define FTM1_SWOCTRL                             FTM_SWOCTRL_REG(FTM1)
#define FTM1_PWMLOAD                             FTM_PWMLOAD_REG(FTM1)
/* FTM2 */
#define FTM2_SC                                  FTM_SC_REG(FTM2)
#define FTM2_CNT                                 FTM_CNT_REG(FTM2)
#define FTM2_MOD                                 FTM_MOD_REG(FTM2)
#define FTM2_C0SC                                FTM_CnSC_REG(FTM2,0)
#define FTM2_C0V                                 FTM_CnV_REG(FTM2,0)
#define FTM2_C1SC                                FTM_CnSC_REG(FTM2,1)
#define FTM2_C1V                                 FTM_CnV_REG(FTM2,1)
#define FTM2_CNTIN                               FTM_CNTIN_REG(FTM2)
#define FTM2_STATUS                              FTM_STATUS_REG(FTM2)
#define FTM2_MODE                                FTM_MODE_REG(FTM2)
#define FTM2_SYNC                                FTM_SYNC_REG(FTM2)
#define FTM2_OUTINIT                             FTM_OUTINIT_REG(FTM2)
#define FTM2_OUTMASK                             FTM_OUTMASK_REG(FTM2)
#define FTM2_COMBINE                             FTM_COMBINE_REG(FTM2)
#define FTM2_DEADTIME                            FTM_DEADTIME_REG(FTM2)
#define FTM2_EXTTRIG                             FTM_EXTTRIG_REG(FTM2)
#define FTM2_POL                                 FTM_POL_REG(FTM2)
#define FTM2_FMS                                 FTM_FMS_REG(FTM2)
#define FTM2_FILTER                              FTM_FILTER_REG(FTM2)
#define FTM2_FLTCTRL                             FTM_FLTCTRL_REG(FTM2)
#define FTM2_QDCTRL                              FTM_QDCTRL_REG(FTM2)
#define FTM2_CONF                                FTM_CONF_REG(FTM2)
#define FTM2_FLTPOL                              FTM_FLTPOL_REG(FTM2)
#define FTM2_SYNCONF                             FTM_SYNCONF_REG(FTM2)
#define FTM2_INVCTRL                             FTM_INVCTRL_REG(FTM2)
#define FTM2_SWOCTRL                             FTM_SWOCTRL_REG(FTM2)
#define FTM2_PWMLOAD                             FTM_PWMLOAD_REG(FTM2)
/* FTM3 */
#define FTM3_SC                                  FTM_SC_REG(FTM3)
#define FTM3_CNT                                 FTM_CNT_REG(FTM3)
#define FTM3_MOD                                 FTM_MOD_REG(FTM3)
#define FTM3_C0SC                                FTM_CnSC_REG(FTM3,0)
#define FTM3_C0V                                 FTM_CnV_REG(FTM3,0)
#define FTM3_C1SC                                FTM_CnSC_REG(FTM3,1)
#define FTM3_C1V                                 FTM_CnV_REG(FTM3,1)
#define FTM3_C2SC                                FTM_CnSC_REG(FTM3,2)
#define FTM3_C2V                                 FTM_CnV_REG(FTM3,2)
#define FTM3_C3SC                                FTM_CnSC_REG(FTM3,3)
#define FTM3_C3V                                 FTM_CnV_REG(FTM3,3)
#define FTM3_C4SC                                FTM_CnSC_REG(FTM3,4)
#define FTM3_C4V                                 FTM_CnV_REG(FTM3,4)
#define FTM3_C5SC                                FTM_CnSC_REG(FTM3,5)
#define FTM3_C5V                                 FTM_CnV_REG(FTM3,5)
#define FTM3_C6SC                                FTM_CnSC_REG(FTM3,6)
#define FTM3_C6V                                 FTM_CnV_REG(FTM3,6)
#define FTM3_C7SC                                FTM_CnSC_REG(FTM3,7)
#define FTM3_C7V                                 FTM_CnV_REG(FTM3,7)
#define FTM3_CNTIN                               FTM_CNTIN_REG(FTM3)
#define FTM3_STATUS                              FTM_STATUS_REG(FTM3)
#define FTM3_MODE                                FTM_MODE_REG(FTM3)
#define FTM3_SYNC                                FTM_SYNC_REG(FTM3)
#define FTM3_OUTINIT                             FTM_OUTINIT_REG(FTM3)
#define FTM3_OUTMASK                             FTM_OUTMASK_REG(FTM3)
#define FTM3_COMBINE                             FTM_COMBINE_REG(FTM3)
#define FTM3_DEADTIME                            FTM_DEADTIME_REG(FTM3)
#define FTM3_EXTTRIG                             FTM_EXTTRIG_REG(FTM3)
#define FTM3_POL                                 FTM_POL_REG(FTM3)
#define FTM3_FMS                                 FTM_FMS_REG(FTM3)
#define FTM3_FILTER                              FTM_FILTER_REG(FTM3)
#define FTM3_FLTCTRL                             FTM_FLTCTRL_REG(FTM3)
#define FTM3_QDCTRL                              FTM_QDCTRL_REG(FTM3)
#define FTM3_CONF                                FTM_CONF_REG(FTM3)
#define FTM3_FLTPOL                              FTM_FLTPOL_REG(FTM3)
#define FTM3_SYNCONF                             FTM_SYNCONF_REG(FTM3)
#define FTM3_INVCTRL                             FTM_INVCTRL_REG(FTM3)
#define FTM3_SWOCTRL                             FTM_SWOCTRL_REG(FTM3)
#define FTM3_PWMLOAD                             FTM_PWMLOAD_REG(FTM3)

/* FTM - Register array accessors */
#define FTM0_CnSC(index)                         FTM_CnSC_REG(FTM0,index)
#define FTM1_CnSC(index)                         FTM_CnSC_REG(FTM1,index)
#define FTM2_CnSC(index)                         FTM_CnSC_REG(FTM2,index)
#define FTM3_CnSC(index)                         FTM_CnSC_REG(FTM3,index)
#define FTM0_CnV(index)                          FTM_CnV_REG(FTM0,index)
#define FTM1_CnV(index)                          FTM_CnV_REG(FTM1,index)
#define FTM2_CnV(index)                          FTM_CnV_REG(FTM2,index)
#define FTM3_CnV(index)                          FTM_CnV_REG(FTM3,index)

/*!
 * @}
 */ /* end of group FTM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group FTM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- GPIO Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 * @{
 */

/** GPIO - Register Layout Typedef */
typedef struct {
  __IO uint32_t PDOR;                              /**< Port Data Output Register, offset: 0x0 */
  __O  uint32_t PSOR;                              /**< Port Set Output Register, offset: 0x4 */
  __O  uint32_t PCOR;                              /**< Port Clear Output Register, offset: 0x8 */
  __O  uint32_t PTOR;                              /**< Port Toggle Output Register, offset: 0xC */
  __I  uint32_t PDIR;                              /**< Port Data Input Register, offset: 0x10 */
  __IO uint32_t PDDR;                              /**< Port Data Direction Register, offset: 0x14 */
} GPIO_TypeDef, *GPIO_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- GPIO - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Accessor_Macros GPIO - Register accessor macros
 * @{
 */


/* GPIO - Register accessors */
#define GPIO_PDOR_REG(base)                      ((base)->PDOR)
#define GPIO_PSOR_REG(base)                      ((base)->PSOR)
#define GPIO_PCOR_REG(base)                      ((base)->PCOR)
#define GPIO_PTOR_REG(base)                      ((base)->PTOR)
#define GPIO_PDIR_REG(base)                      ((base)->PDIR)
#define GPIO_PDDR_REG(base)                      ((base)->PDDR)

/*!
 * @}
 */ /* end of group GPIO_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- GPIO Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Masks GPIO Register Masks
 * @{
 */

/* PDOR Bit Fields */
#define GPIO_PDOR_PDO_MASK                       0xFFFFFFFFu
#define GPIO_PDOR_PDO_SHIFT                      0
#define GPIO_PDOR_PDO_WIDTH                      32
#define GPIO_PDOR_PDO(x)                         (((uint32_t)(((uint32_t)(x))<<GPIO_PDOR_PDO_SHIFT))&GPIO_PDOR_PDO_MASK)
/* PSOR Bit Fields */
#define GPIO_PSOR_PTSO_MASK                      0xFFFFFFFFu
#define GPIO_PSOR_PTSO_SHIFT                     0
#define GPIO_PSOR_PTSO_WIDTH                     32
#define GPIO_PSOR_PTSO(x)                        (((uint32_t)(((uint32_t)(x))<<GPIO_PSOR_PTSO_SHIFT))&GPIO_PSOR_PTSO_MASK)
/* PCOR Bit Fields */
#define GPIO_PCOR_PTCO_MASK                      0xFFFFFFFFu
#define GPIO_PCOR_PTCO_SHIFT                     0
#define GPIO_PCOR_PTCO_WIDTH                     32
#define GPIO_PCOR_PTCO(x)                        (((uint32_t)(((uint32_t)(x))<<GPIO_PCOR_PTCO_SHIFT))&GPIO_PCOR_PTCO_MASK)
/* PTOR Bit Fields */
#define GPIO_PTOR_PTTO_MASK                      0xFFFFFFFFu
#define GPIO_PTOR_PTTO_SHIFT                     0
#define GPIO_PTOR_PTTO_WIDTH                     32
#define GPIO_PTOR_PTTO(x)                        (((uint32_t)(((uint32_t)(x))<<GPIO_PTOR_PTTO_SHIFT))&GPIO_PTOR_PTTO_MASK)
/* PDIR Bit Fields */
#define GPIO_PDIR_PDI_MASK                       0xFFFFFFFFu
#define GPIO_PDIR_PDI_SHIFT                      0
#define GPIO_PDIR_PDI_WIDTH                      32
#define GPIO_PDIR_PDI(x)                         (((uint32_t)(((uint32_t)(x))<<GPIO_PDIR_PDI_SHIFT))&GPIO_PDIR_PDI_MASK)
/* PDDR Bit Fields */
#define GPIO_PDDR_PDD_MASK                       0xFFFFFFFFu
#define GPIO_PDDR_PDD_SHIFT                      0
#define GPIO_PDDR_PDD_WIDTH                      32
#define GPIO_PDDR_PDD(x)                         (((uint32_t)(((uint32_t)(x))<<GPIO_PDDR_PDD_SHIFT))&GPIO_PDDR_PDD_MASK)

/*!
 * @}
 */ /* end of group GPIO_Register_Masks */


/* GPIO - Peripheral instance base addresses */
/** Peripheral PTA base address */
#define PTA_BASE                                 (0x400FF000u)
/** Peripheral PTA base pointer */
#define PTA                                      ((GPIO_TypeDef *)PTA_BASE)
#define PTA_BASE_PTR                             (PTA)
/** Peripheral PTB base address */
#define PTB_BASE                                 (0x400FF040u)
/** Peripheral PTB base pointer */
#define PTB                                      ((GPIO_TypeDef *)PTB_BASE)
#define PTB_BASE_PTR                             (PTB)
/** Peripheral PTC base address */
#define PTC_BASE                                 (0x400FF080u)
/** Peripheral PTC base pointer */
#define PTC                                      ((GPIO_TypeDef *)PTC_BASE)
#define PTC_BASE_PTR                             (PTC)
/** Peripheral PTD base address */
#define PTD_BASE                                 (0x400FF0C0u)
/** Peripheral PTD base pointer */
#define PTD                                      ((GPIO_TypeDef *)PTD_BASE)
#define PTD_BASE_PTR                             (PTD)
/** Peripheral PTE base address */
#define PTE_BASE                                 (0x400FF100u)
/** Peripheral PTE base pointer */
#define PTE                                      ((GPIO_TypeDef *)PTE_BASE)
#define PTE_BASE_PTR                             (PTE)
/** Array initializer of GPIO peripheral base addresses */
#define GPIO_BASE_ADDRS                          { PTA_BASE, PTB_BASE, PTC_BASE, PTD_BASE, PTE_BASE }
/** Array initializer of GPIO peripheral base pointers */
#define GPIO_BASE_PTRS                           { PTA, PTB, PTC, PTD, PTE }

#define GPIOA  (PTA)
#define GPIOB  (PTB)
#define GPIOC  (PTC)
#define GPIOD  (PTD)
#define GPIOE  (PTE)

/* ----------------------------------------------------------------------------
   -- GPIO - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup GPIO_Register_Accessor_Macros GPIO - Register accessor macros
 * @{
 */


/* GPIO - Register instance definitions */
/* PTA */
#define GPIOA_PDOR                               GPIO_PDOR_REG(PTA)
#define GPIOA_PSOR                               GPIO_PSOR_REG(PTA)
#define GPIOA_PCOR                               GPIO_PCOR_REG(PTA)
#define GPIOA_PTOR                               GPIO_PTOR_REG(PTA)
#define GPIOA_PDIR                               GPIO_PDIR_REG(PTA)
#define GPIOA_PDDR                               GPIO_PDDR_REG(PTA)
/* PTB */
#define GPIOB_PDOR                               GPIO_PDOR_REG(PTB)
#define GPIOB_PSOR                               GPIO_PSOR_REG(PTB)
#define GPIOB_PCOR                               GPIO_PCOR_REG(PTB)
#define GPIOB_PTOR                               GPIO_PTOR_REG(PTB)
#define GPIOB_PDIR                               GPIO_PDIR_REG(PTB)
#define GPIOB_PDDR                               GPIO_PDDR_REG(PTB)
/* PTC */
#define GPIOC_PDOR                               GPIO_PDOR_REG(PTC)
#define GPIOC_PSOR                               GPIO_PSOR_REG(PTC)
#define GPIOC_PCOR                               GPIO_PCOR_REG(PTC)
#define GPIOC_PTOR                               GPIO_PTOR_REG(PTC)
#define GPIOC_PDIR                               GPIO_PDIR_REG(PTC)
#define GPIOC_PDDR                               GPIO_PDDR_REG(PTC)
/* PTD */
#define GPIOD_PDOR                               GPIO_PDOR_REG(PTD)
#define GPIOD_PSOR                               GPIO_PSOR_REG(PTD)
#define GPIOD_PCOR                               GPIO_PCOR_REG(PTD)
#define GPIOD_PTOR                               GPIO_PTOR_REG(PTD)
#define GPIOD_PDIR                               GPIO_PDIR_REG(PTD)
#define GPIOD_PDDR                               GPIO_PDDR_REG(PTD)
/* PTE */
#define GPIOE_PDOR                               GPIO_PDOR_REG(PTE)
#define GPIOE_PSOR                               GPIO_PSOR_REG(PTE)
#define GPIOE_PCOR                               GPIO_PCOR_REG(PTE)
#define GPIOE_PTOR                               GPIO_PTOR_REG(PTE)
#define GPIOE_PDIR                               GPIO_PDIR_REG(PTE)
#define GPIOE_PDDR                               GPIO_PDDR_REG(PTE)

/*!
 * @}
 */ /* end of group GPIO_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group GPIO_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- I2C Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Peripheral_Access_Layer I2C Peripheral Access Layer
 * @{
 */

/** I2C - Register Layout Typedef */
typedef struct {
  __IO uint8_t A1;                                 /**< I2C Address Register 1, offset: 0x0 */
  __IO uint8_t F;                                  /**< I2C Frequency Divider register, offset: 0x1 */
  __IO uint8_t C1;                                 /**< I2C Control Register 1, offset: 0x2 */
  __IO uint8_t S;                                  /**< I2C Status register, offset: 0x3 */
  __IO uint8_t D;                                  /**< I2C Data I/O register, offset: 0x4 */
  __IO uint8_t C2;                                 /**< I2C Control Register 2, offset: 0x5 */
  __IO uint8_t FLT;                                /**< I2C Programmable Input Glitch Filter register, offset: 0x6 */
  __IO uint8_t RA;                                 /**< I2C Range Address register, offset: 0x7 */
  __IO uint8_t SMB;                                /**< I2C SMBus Control and Status register, offset: 0x8 */
  __IO uint8_t A2;                                 /**< I2C Address Register 2, offset: 0x9 */
  __IO uint8_t SLTH;                               /**< I2C SCL Low Timeout Register High, offset: 0xA */
  __IO uint8_t SLTL;                               /**< I2C SCL Low Timeout Register Low, offset: 0xB */
} I2C_TypeDef, *I2C_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- I2C - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Accessor_Macros I2C - Register accessor macros
 * @{
 */


/* I2C - Register accessors */
#define I2C_A1_REG(base)                         ((base)->A1)
#define I2C_F_REG(base)                          ((base)->F)
#define I2C_C1_REG(base)                         ((base)->C1)
#define I2C_S_REG(base)                          ((base)->S)
#define I2C_D_REG(base)                          ((base)->D)
#define I2C_C2_REG(base)                         ((base)->C2)
#define I2C_FLT_REG(base)                        ((base)->FLT)
#define I2C_RA_REG(base)                         ((base)->RA)
#define I2C_SMB_REG(base)                        ((base)->SMB)
#define I2C_A2_REG(base)                         ((base)->A2)
#define I2C_SLTH_REG(base)                       ((base)->SLTH)
#define I2C_SLTL_REG(base)                       ((base)->SLTL)

/*!
 * @}
 */ /* end of group I2C_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- I2C Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Masks I2C Register Masks
 * @{
 */

/* A1 Bit Fields */
#define I2C_A1_AD_MASK                           0xFEu
#define I2C_A1_AD_SHIFT                          1
#define I2C_A1_AD_WIDTH                          7
#define I2C_A1_AD(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_A1_AD_SHIFT))&I2C_A1_AD_MASK)
/* F Bit Fields */
#define I2C_F_ICR_MASK                           0x3Fu
#define I2C_F_ICR_SHIFT                          0
#define I2C_F_ICR_WIDTH                          6
#define I2C_F_ICR(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_F_ICR_SHIFT))&I2C_F_ICR_MASK)
#define I2C_F_MULT_MASK                          0xC0u
#define I2C_F_MULT_SHIFT                         6
#define I2C_F_MULT_WIDTH                         2
#define I2C_F_MULT(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_F_MULT_SHIFT))&I2C_F_MULT_MASK)
/* C1 Bit Fields */
#define I2C_C1_DMAEN		((uint8_t)0x1u)
#define I2C_C1_WUEN		((uint8_t)0x2u)
#define I2C_C1_RSTA		((uint8_t)0x4u)
#define I2C_C1_TXAK		((uint8_t)0x8u)
#define I2C_C1_TX		((uint8_t)0x10u)
#define I2C_C1_MST		((uint8_t)0x20u)
#define I2C_C1_IICIE		((uint8_t)0x40u)
#define I2C_C1_IICEN		((uint8_t)0x80u)
/* S Bit Fields */
#define I2C_S_RXAK		((uint8_t)0x1u)
#define I2C_S_IICIF		((uint8_t)0x2u)
#define I2C_S_SRW		((uint8_t)0x4u)
#define I2C_S_RAM		((uint8_t)0x8u)
#define I2C_S_ARBL		((uint8_t)0x10u)
#define I2C_S_BUSY		((uint8_t)0x20u)
#define I2C_S_IAAS		((uint8_t)0x40u)
#define I2C_S_TCF		((uint8_t)0x80u)
/* D Bit Fields */
#define I2C_D_DATA_MASK                          0xFFu
#define I2C_D_DATA_SHIFT                         0
#define I2C_D_DATA_WIDTH                         8
#define I2C_D_DATA(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_D_DATA_SHIFT))&I2C_D_DATA_MASK)
/* C2 Bit Fields */
#define I2C_C2_AD_MASK                           0x7u
#define I2C_C2_AD_SHIFT                          0
#define I2C_C2_AD_WIDTH                          3
#define I2C_C2_AD(x)                             (((uint8_t)(((uint8_t)(x))<<I2C_C2_AD_SHIFT))&I2C_C2_AD_MASK)
#define I2C_C2_RMEN		((uint8_t)0x8u)
#define I2C_C2_SBRC		((uint8_t)0x10u)
#define I2C_C2_HDRS		((uint8_t)0x20u)
#define I2C_C2_ADEXT		((uint8_t)0x40u)
#define I2C_C2_GCAEN		((uint8_t)0x80u)
/* FLT Bit Fields */
#define I2C_FLT_FLT_MASK                         0xFu
#define I2C_FLT_FLT_SHIFT                        0
#define I2C_FLT_FLT_WIDTH                        4
#define I2C_FLT_FLT(x)                           (((uint8_t)(((uint8_t)(x))<<I2C_FLT_FLT_SHIFT))&I2C_FLT_FLT_MASK)
#define I2C_FLT_STARTF		((uint8_t)0x10u)
#define I2C_FLT_SSIE		((uint8_t)0x20u)
#define I2C_FLT_STOPF		((uint8_t)0x40u)
#define I2C_FLT_SHEN		((uint8_t)0x80u)
/* RA Bit Fields */
#define I2C_RA_RAD_MASK                          0xFEu
#define I2C_RA_RAD_SHIFT                         1
#define I2C_RA_RAD_WIDTH                         7
#define I2C_RA_RAD(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_RA_RAD_SHIFT))&I2C_RA_RAD_MASK)
/* SMB Bit Fields */
#define I2C_SMB_SHTF2IE		((uint8_t)0x1u)
#define I2C_SMB_SHTF2		((uint8_t)0x2u)
#define I2C_SMB_SHTF1		((uint8_t)0x4u)
#define I2C_SMB_SLTF		((uint8_t)0x8u)
#define I2C_SMB_TCKSEL		((uint8_t)0x10u)
#define I2C_SMB_SIICAEN		((uint8_t)0x20u)
#define I2C_SMB_ALERTEN		((uint8_t)0x40u)
#define I2C_SMB_FACK		((uint8_t)0x80u)
/* A2 Bit Fields */
#define I2C_A2_SAD_MASK                          0xFEu
#define I2C_A2_SAD_SHIFT                         1
#define I2C_A2_SAD_WIDTH                         7
#define I2C_A2_SAD(x)                            (((uint8_t)(((uint8_t)(x))<<I2C_A2_SAD_SHIFT))&I2C_A2_SAD_MASK)
/* SLTH Bit Fields */
#define I2C_SLTH_SSLT_MASK                       0xFFu
#define I2C_SLTH_SSLT_SHIFT                      0
#define I2C_SLTH_SSLT_WIDTH                      8
#define I2C_SLTH_SSLT(x)                         (((uint8_t)(((uint8_t)(x))<<I2C_SLTH_SSLT_SHIFT))&I2C_SLTH_SSLT_MASK)
/* SLTL Bit Fields */
#define I2C_SLTL_SSLT_MASK                       0xFFu
#define I2C_SLTL_SSLT_SHIFT                      0
#define I2C_SLTL_SSLT_WIDTH                      8
#define I2C_SLTL_SSLT(x)                         (((uint8_t)(((uint8_t)(x))<<I2C_SLTL_SSLT_SHIFT))&I2C_SLTL_SSLT_MASK)

/*!
 * @}
 */ /* end of group I2C_Register_Masks */


/* I2C - Peripheral instance base addresses */
/** Peripheral I2C0 base address */
#define I2C0_BASE                                (0x40066000u)
/** Peripheral I2C0 base pointer */
#define I2C0                                     ((I2C_TypeDef *)I2C0_BASE)
#define I2C0_BASE_PTR                            (I2C0)
/** Peripheral I2C1 base address */
#define I2C1_BASE                                (0x40067000u)
/** Peripheral I2C1 base pointer */
#define I2C1                                     ((I2C_TypeDef *)I2C1_BASE)
#define I2C1_BASE_PTR                            (I2C1)
/** Array initializer of I2C peripheral base addresses */
#define I2C_BASE_ADDRS                           { I2C0_BASE, I2C1_BASE }
/** Array initializer of I2C peripheral base pointers */
#define I2C_BASE_PTRS                            { I2C0, I2C1 }
/** Interrupt vectors for the I2C peripheral type */
#define I2C_IRQS                                 { I2C0_IRQn, I2C1_IRQn }

/* ----------------------------------------------------------------------------
   -- I2C - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2C_Register_Accessor_Macros I2C - Register accessor macros
 * @{
 */


/* I2C - Register instance definitions */
/* I2C0 */
#define I2C0_A1                                  I2C_A1_REG(I2C0)
#define I2C0_F                                   I2C_F_REG(I2C0)
#define I2C0_C1                                  I2C_C1_REG(I2C0)
#define I2C0_S                                   I2C_S_REG(I2C0)
#define I2C0_D                                   I2C_D_REG(I2C0)
#define I2C0_C2                                  I2C_C2_REG(I2C0)
#define I2C0_FLT                                 I2C_FLT_REG(I2C0)
#define I2C0_RA                                  I2C_RA_REG(I2C0)
#define I2C0_SMB                                 I2C_SMB_REG(I2C0)
#define I2C0_A2                                  I2C_A2_REG(I2C0)
#define I2C0_SLTH                                I2C_SLTH_REG(I2C0)
#define I2C0_SLTL                                I2C_SLTL_REG(I2C0)
/* I2C1 */
#define I2C1_A1                                  I2C_A1_REG(I2C1)
#define I2C1_F                                   I2C_F_REG(I2C1)
#define I2C1_C1                                  I2C_C1_REG(I2C1)
#define I2C1_S                                   I2C_S_REG(I2C1)
#define I2C1_D                                   I2C_D_REG(I2C1)
#define I2C1_C2                                  I2C_C2_REG(I2C1)
#define I2C1_FLT                                 I2C_FLT_REG(I2C1)
#define I2C1_RA                                  I2C_RA_REG(I2C1)
#define I2C1_SMB                                 I2C_SMB_REG(I2C1)
#define I2C1_A2                                  I2C_A2_REG(I2C1)
#define I2C1_SLTH                                I2C_SLTH_REG(I2C1)
#define I2C1_SLTL                                I2C_SLTL_REG(I2C1)

/***********  Bits definition for I2Cx_A1 register  *************/
#define I2Cx_A1_AD                   ((uint8_t)0xFE)    /*!< Address [7:1] */

#define I2Cx_A1_AD_SHIT              1

/***********  Bits definition for I2Cx_F register  **************/
#define I2Cx_F_MULT                  ((uint8_t)0xC0)    /*!< Multiplier factor */
#define I2Cx_F_ICR                   ((uint8_t)0x3F)    /*!< Clock rate */

#define I2Cx_F_MULT_SHIFT            5

/***********  Bits definition for I2Cx_C1 register  *************/
#define I2Cx_C1_IICEN                ((uint8_t)0x80)    /*!< I2C Enable */
#define I2Cx_C1_IICIE                ((uint8_t)0x40)    /*!< I2C Interrupt Enable */
#define I2Cx_C1_MST                  ((uint8_t)0x20)    /*!< Master Mode Select */
#define I2Cx_C1_TX                   ((uint8_t)0x10)    /*!< Transmit Mode Select */
#define I2Cx_C1_TXAK                 ((uint8_t)0x08)    /*!< Transmit Acknowledge Enable */
#define I2Cx_C1_RSTA                 ((uint8_t)0x04)    /*!< Repeat START */
#define I2Cx_C1_WUEN                 ((uint8_t)0x02)    /*!< Wakeup Enable */
#define I2Cx_C1_DMAEN                ((uint8_t)0x01)    /*!< DMA Enable */

/***********  Bits definition for I2Cx_S register  **************/
#define I2Cx_S_TCF                   ((uint8_t)0x80)    /*!< Transfer Complete Flag */
#define I2Cx_S_IAAS                  ((uint8_t)0x40)    /*!< Addressed As A Slave */
#define I2Cx_S_BUSY                  ((uint8_t)0x20)    /*!< Bus Busy */
#define I2Cx_S_ARBL                  ((uint8_t)0x10)    /*!< Arbitration Lost */
#define I2Cx_S_RAM                   ((uint8_t)0x08)    /*!< Range Address Match */
#define I2Cx_S_SRW                   ((uint8_t)0x04)    /*!< Slave Read/Write */
#define I2Cx_S_IICIF                 ((uint8_t)0x02)    /*!< Interrupt Flag */
#define I2Cx_S_RXAK                  ((uint8_t)0x01)    /*!< Receive Acknowledge */

/***********  Bits definition for I2Cx_D register  **************/
#define I2Cx_D_DATA                  ((uint8_t)0xFF)    /*!< Data */

/***********  Bits definition for I2Cx_C2 register  *************/
#define I2Cx_C2_GCAEN                ((uint8_t)0x80)    /*!< General Call Address Enable */
#define I2Cx_C2_ADEXT                ((uint8_t)0x40)    /*!< Address Extension */
#define I2Cx_C2_HDRS                 ((uint8_t)0x20)    /*!< High Drive Select */
#define I2Cx_C2_SBRC                 ((uint8_t)0x10)    /*!< Slave Baud Rate Control */
#define I2Cx_C2_RMEN                 ((uint8_t)0x08)    /*!< Range Address Matching Enable */
#define I2Cx_C2_AD_10_8              ((uint8_t)0x03)    /*!< Slave Address [10:8] */

/***********  Bits definition for I2Cx_FLT register  ************/
#define I2Cx_FLT_SHEN                ((uint8_t)0x80)    /*!< Stop Hold Enable */
#define I2Cx_FLT_STOPF               ((uint8_t)0x40)    /*!< I2C Bus Stop Detect Flag */
#define I2Cx_FLT_STOPIE              ((uint8_t)0x20)    /*!< I2C Bus Stop Interrupt Enable */
#define I2Cx_FLT_FLT                 ((uint8_t)0x1F)    /*!< I2C Programmable Filter Factor */

/***********  Bits definition for I2Cx_RA register  *************/
#define I2Cx_RA_RAD                  ((uint8_t)0xFE)    /*!< Range Slave Address */

#define I2Cx_RA_RAD_SHIFT            1

/***********  Bits definition for I2Cx_SMB register  ************/
#define I2Cx_SMB_FACK                ((uint8_t)0x80)    /*!< Fast NACK/ACK Enable */
#define I2Cx_SMB_ALERTEN             ((uint8_t)0x40)    /*!< SMBus Alert Response Address Enable */
#define I2Cx_SMB_SIICAEN             ((uint8_t)0x20)    /*!< Second I2C Address Enable */
#define I2Cx_SMB_TCKSEL              ((uint8_t)0x10)    /*!< Timeout Counter Clock Select */
#define I2Cx_SMB_SLTF                ((uint8_t)0x08)    /*!< SCL Low Timeout Flag */
#define I2Cx_SMB_SHTF1               ((uint8_t)0x04)    /*!< SCL High Timeout Flag 1 */
#define I2Cx_SMB_SHTF2               ((uint8_t)0x02)    /*!< SCL High Timeout Flag 2 */
#define I2Cx_SMB_SHTF2IE             ((uint8_t)0x01)    /*!< SHTF2 Interrupt Enable */

/***********  Bits definition for I2Cx_A2 register  *************/
#define I2Cx_A2_SAD                  ((uint8_t)0xFE)    /*!< SMBus Address */

#define I2Cx_A2_SAD_SHIFT            1

/***********  Bits definition for I2Cx_SLTH register  ***********/
#define I2Cx_SLTH_SSLT               ((uint8_t)0xFF)    /*!< MSB of SCL low timeout value */

/***********  Bits definition for I2Cx_SLTL register  ***********/
#define I2Cx_SLTL_SSLT               ((uint8_t)0xFF)    /*!< LSB of SCL low timeout value */

/*!
 * @}
 */ /* end of group I2C_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group I2C_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- I2S Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Peripheral_Access_Layer I2S Peripheral Access Layer
 * @{
 */

/** I2S - Register Layout Typedef */
typedef struct {
  __IO uint32_t TCSR;                              /**< SAI Transmit Control Register, offset: 0x0 */
  __IO uint32_t TCR1;                              /**< SAI Transmit Configuration 1 Register, offset: 0x4 */
  __IO uint32_t TCR2;                              /**< SAI Transmit Configuration 2 Register, offset: 0x8 */
  __IO uint32_t TCR3;                              /**< SAI Transmit Configuration 3 Register, offset: 0xC */
  __IO uint32_t TCR4;                              /**< SAI Transmit Configuration 4 Register, offset: 0x10 */
  __IO uint32_t TCR5;                              /**< SAI Transmit Configuration 5 Register, offset: 0x14 */
       uint8_t RESERVED_0[8];
  __O  uint32_t TDR[1];                            /**< SAI Transmit Data Register, array offset: 0x20, array step: 0x4 */
       uint8_t RESERVED_1[28];
  __I  uint32_t TFR[1];                            /**< SAI Transmit FIFO Register, array offset: 0x40, array step: 0x4 */
       uint8_t RESERVED_2[28];
  __IO uint32_t TMR;                               /**< SAI Transmit Mask Register, offset: 0x60 */
       uint8_t RESERVED_3[28];
  __IO uint32_t RCSR;                              /**< SAI Receive Control Register, offset: 0x80 */
  __IO uint32_t RCR1;                              /**< SAI Receive Configuration 1 Register, offset: 0x84 */
  __IO uint32_t RCR2;                              /**< SAI Receive Configuration 2 Register, offset: 0x88 */
  __IO uint32_t RCR3;                              /**< SAI Receive Configuration 3 Register, offset: 0x8C */
  __IO uint32_t RCR4;                              /**< SAI Receive Configuration 4 Register, offset: 0x90 */
  __IO uint32_t RCR5;                              /**< SAI Receive Configuration 5 Register, offset: 0x94 */
       uint8_t RESERVED_4[8];
  __I  uint32_t RDR[1];                            /**< SAI Receive Data Register, array offset: 0xA0, array step: 0x4 */
       uint8_t RESERVED_5[28];
  __I  uint32_t RFR[1];                            /**< SAI Receive FIFO Register, array offset: 0xC0, array step: 0x4 */
       uint8_t RESERVED_6[28];
  __IO uint32_t RMR;                               /**< SAI Receive Mask Register, offset: 0xE0 */
       uint8_t RESERVED_7[28];
  __IO uint32_t MCR;                               /**< SAI MCLK Control Register, offset: 0x100 */
  __IO uint32_t MDR;                               /**< SAI MCLK Divide Register, offset: 0x104 */
} I2S_TypeDef, *I2S_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- I2S - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Register_Accessor_Macros I2S - Register accessor macros
 * @{
 */


/* I2S - Register accessors */
#define I2S_TCSR_REG(base)                       ((base)->TCSR)
#define I2S_TCR1_REG(base)                       ((base)->TCR1)
#define I2S_TCR2_REG(base)                       ((base)->TCR2)
#define I2S_TCR3_REG(base)                       ((base)->TCR3)
#define I2S_TCR4_REG(base)                       ((base)->TCR4)
#define I2S_TCR5_REG(base)                       ((base)->TCR5)
#define I2S_TDR_REG(base,index)                  ((base)->TDR[index])
#define I2S_TDR_COUNT                            1
#define I2S_TFR_REG(base,index)                  ((base)->TFR[index])
#define I2S_TFR_COUNT                            1
#define I2S_TMR_REG(base)                        ((base)->TMR)
#define I2S_RCSR_REG(base)                       ((base)->RCSR)
#define I2S_RCR1_REG(base)                       ((base)->RCR1)
#define I2S_RCR2_REG(base)                       ((base)->RCR2)
#define I2S_RCR3_REG(base)                       ((base)->RCR3)
#define I2S_RCR4_REG(base)                       ((base)->RCR4)
#define I2S_RCR5_REG(base)                       ((base)->RCR5)
#define I2S_RDR_REG(base,index)                  ((base)->RDR[index])
#define I2S_RDR_COUNT                            1
#define I2S_RFR_REG(base,index)                  ((base)->RFR[index])
#define I2S_RFR_COUNT                            1
#define I2S_RMR_REG(base)                        ((base)->RMR)
#define I2S_MCR_REG(base)                        ((base)->MCR)
#define I2S_MDR_REG(base)                        ((base)->MDR)

/*!
 * @}
 */ /* end of group I2S_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- I2S Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Register_Masks I2S Register Masks
 * @{
 */

/* TCSR Bit Fields */
#define I2S_TCSR_FRDE		((uint32_t)0x1u)
#define I2S_TCSR_FWDE		((uint32_t)0x2u)
#define I2S_TCSR_FRIE		((uint32_t)0x100u)
#define I2S_TCSR_FWIE		((uint32_t)0x200u)
#define I2S_TCSR_FEIE		((uint32_t)0x400u)
#define I2S_TCSR_SEIE		((uint32_t)0x800u)
#define I2S_TCSR_WSIE		((uint32_t)0x1000u)
#define I2S_TCSR_FRF		((uint32_t)0x10000u)
#define I2S_TCSR_FWF		((uint32_t)0x20000u)
#define I2S_TCSR_FEF		((uint32_t)0x40000u)
#define I2S_TCSR_SEF		((uint32_t)0x80000u)
#define I2S_TCSR_WSF		((uint32_t)0x100000u)
#define I2S_TCSR_SR		((uint32_t)0x1000000u)
#define I2S_TCSR_FR		((uint32_t)0x2000000u)
#define I2S_TCSR_BCE		((uint32_t)0x10000000u)
#define I2S_TCSR_DBGE		((uint32_t)0x20000000u)
#define I2S_TCSR_STOPE		((uint32_t)0x40000000u)
#define I2S_TCSR_TE		((uint32_t)0x80000000u)
/* TCR1 Bit Fields */
#define I2S_TCR1_TFW_MASK                        0x7u
#define I2S_TCR1_TFW_SHIFT                       0
#define I2S_TCR1_TFW_WIDTH                       3
#define I2S_TCR1_TFW(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_TCR1_TFW_SHIFT))&I2S_TCR1_TFW_MASK)
/* TCR2 Bit Fields */
#define I2S_TCR2_DIV_MASK                        0xFFu
#define I2S_TCR2_DIV_SHIFT                       0
#define I2S_TCR2_DIV_WIDTH                       8
#define I2S_TCR2_DIV(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_TCR2_DIV_SHIFT))&I2S_TCR2_DIV_MASK)
#define I2S_TCR2_BCD		((uint32_t)0x1000000u)
#define I2S_TCR2_BCP		((uint32_t)0x2000000u)
#define I2S_TCR2_MSEL_MASK                       0xC000000u
#define I2S_TCR2_MSEL_SHIFT                      26
#define I2S_TCR2_MSEL_WIDTH                      2
#define I2S_TCR2_MSEL(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_TCR2_MSEL_SHIFT))&I2S_TCR2_MSEL_MASK)
#define I2S_TCR2_BCI		((uint32_t)0x10000000u)
#define I2S_TCR2_BCS		((uint32_t)0x20000000u)
#define I2S_TCR2_SYNC_MASK                       0xC0000000u
#define I2S_TCR2_SYNC_SHIFT                      30
#define I2S_TCR2_SYNC_WIDTH                      2
#define I2S_TCR2_SYNC(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_TCR2_SYNC_SHIFT))&I2S_TCR2_SYNC_MASK)
/* TCR3 Bit Fields */
#define I2S_TCR3_WDFL_MASK                       0xFu
#define I2S_TCR3_WDFL_SHIFT                      0
#define I2S_TCR3_WDFL_WIDTH                      4
#define I2S_TCR3_WDFL(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_TCR3_WDFL_SHIFT))&I2S_TCR3_WDFL_MASK)
#define I2S_TCR3_TCE		((uint32_t)0x10000u)
/* TCR4 Bit Fields */
#define I2S_TCR4_FSD		((uint32_t)0x1u)
#define I2S_TCR4_FSP		((uint32_t)0x2u)
#define I2S_TCR4_ONDEM		((uint32_t)0x4u)
#define I2S_TCR4_FSE		((uint32_t)0x8u)
#define I2S_TCR4_MF		((uint32_t)0x10u)
#define I2S_TCR4_SYWD_MASK                       0x1F00u
#define I2S_TCR4_SYWD_SHIFT                      8
#define I2S_TCR4_SYWD_WIDTH                      5
#define I2S_TCR4_SYWD(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_TCR4_SYWD_SHIFT))&I2S_TCR4_SYWD_MASK)
#define I2S_TCR4_FRSZ_MASK                       0xF0000u
#define I2S_TCR4_FRSZ_SHIFT                      16
#define I2S_TCR4_FRSZ_WIDTH                      4
#define I2S_TCR4_FRSZ(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_TCR4_FRSZ_SHIFT))&I2S_TCR4_FRSZ_MASK)
#define I2S_TCR4_FPACK_MASK                      0x3000000u
#define I2S_TCR4_FPACK_SHIFT                     24
#define I2S_TCR4_FPACK_WIDTH                     2
#define I2S_TCR4_FPACK(x)                        (((uint32_t)(((uint32_t)(x))<<I2S_TCR4_FPACK_SHIFT))&I2S_TCR4_FPACK_MASK)
#define I2S_TCR4_FCONT		((uint32_t)0x10000000u)
/* TCR5 Bit Fields */
#define I2S_TCR5_FBT_MASK                        0x1F00u
#define I2S_TCR5_FBT_SHIFT                       8
#define I2S_TCR5_FBT_WIDTH                       5
#define I2S_TCR5_FBT(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_TCR5_FBT_SHIFT))&I2S_TCR5_FBT_MASK)
#define I2S_TCR5_W0W_MASK                        0x1F0000u
#define I2S_TCR5_W0W_SHIFT                       16
#define I2S_TCR5_W0W_WIDTH                       5
#define I2S_TCR5_W0W(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_TCR5_W0W_SHIFT))&I2S_TCR5_W0W_MASK)
#define I2S_TCR5_WNW_MASK                        0x1F000000u
#define I2S_TCR5_WNW_SHIFT                       24
#define I2S_TCR5_WNW_WIDTH                       5
#define I2S_TCR5_WNW(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_TCR5_WNW_SHIFT))&I2S_TCR5_WNW_MASK)
/* TDR Bit Fields */
#define I2S_TDR_TDR_MASK                         0xFFFFFFFFu
#define I2S_TDR_TDR_SHIFT                        0
#define I2S_TDR_TDR_WIDTH                        32
#define I2S_TDR_TDR(x)                           (((uint32_t)(((uint32_t)(x))<<I2S_TDR_TDR_SHIFT))&I2S_TDR_TDR_MASK)
/* TFR Bit Fields */
#define I2S_TFR_RFP_MASK                         0xFu
#define I2S_TFR_RFP_SHIFT                        0
#define I2S_TFR_RFP_WIDTH                        4
#define I2S_TFR_RFP(x)                           (((uint32_t)(((uint32_t)(x))<<I2S_TFR_RFP_SHIFT))&I2S_TFR_RFP_MASK)
#define I2S_TFR_WFP_MASK                         0xF0000u
#define I2S_TFR_WFP_SHIFT                        16
#define I2S_TFR_WFP_WIDTH                        4
#define I2S_TFR_WFP(x)                           (((uint32_t)(((uint32_t)(x))<<I2S_TFR_WFP_SHIFT))&I2S_TFR_WFP_MASK)
/* TMR Bit Fields */
#define I2S_TMR_TWM_MASK                         0xFFFFu
#define I2S_TMR_TWM_SHIFT                        0
#define I2S_TMR_TWM_WIDTH                        16
#define I2S_TMR_TWM(x)                           (((uint32_t)(((uint32_t)(x))<<I2S_TMR_TWM_SHIFT))&I2S_TMR_TWM_MASK)
/* RCSR Bit Fields */
#define I2S_RCSR_FRDE		((uint32_t)0x1u)
#define I2S_RCSR_FWDE		((uint32_t)0x2u)
#define I2S_RCSR_FRIE		((uint32_t)0x100u)
#define I2S_RCSR_FWIE		((uint32_t)0x200u)
#define I2S_RCSR_FEIE		((uint32_t)0x400u)
#define I2S_RCSR_SEIE		((uint32_t)0x800u)
#define I2S_RCSR_WSIE		((uint32_t)0x1000u)
#define I2S_RCSR_FRF		((uint32_t)0x10000u)
#define I2S_RCSR_FWF		((uint32_t)0x20000u)
#define I2S_RCSR_FEF		((uint32_t)0x40000u)
#define I2S_RCSR_SEF		((uint32_t)0x80000u)
#define I2S_RCSR_WSF		((uint32_t)0x100000u)
#define I2S_RCSR_SR		((uint32_t)0x1000000u)
#define I2S_RCSR_FR		((uint32_t)0x2000000u)
#define I2S_RCSR_BCE		((uint32_t)0x10000000u)
#define I2S_RCSR_DBGE		((uint32_t)0x20000000u)
#define I2S_RCSR_STOPE		((uint32_t)0x40000000u)
#define I2S_RCSR_RE		((uint32_t)0x80000000u)
/* RCR1 Bit Fields */
#define I2S_RCR1_RFW_MASK                        0x7u
#define I2S_RCR1_RFW_SHIFT                       0
#define I2S_RCR1_RFW_WIDTH                       3
#define I2S_RCR1_RFW(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_RCR1_RFW_SHIFT))&I2S_RCR1_RFW_MASK)
/* RCR2 Bit Fields */
#define I2S_RCR2_DIV_MASK                        0xFFu
#define I2S_RCR2_DIV_SHIFT                       0
#define I2S_RCR2_DIV_WIDTH                       8
#define I2S_RCR2_DIV(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_RCR2_DIV_SHIFT))&I2S_RCR2_DIV_MASK)
#define I2S_RCR2_BCD		((uint32_t)0x1000000u)
#define I2S_RCR2_BCP		((uint32_t)0x2000000u)
#define I2S_RCR2_MSEL_MASK                       0xC000000u
#define I2S_RCR2_MSEL_SHIFT                      26
#define I2S_RCR2_MSEL_WIDTH                      2
#define I2S_RCR2_MSEL(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_RCR2_MSEL_SHIFT))&I2S_RCR2_MSEL_MASK)
#define I2S_RCR2_BCI		((uint32_t)0x10000000u)
#define I2S_RCR2_BCS		((uint32_t)0x20000000u)
#define I2S_RCR2_SYNC_MASK                       0xC0000000u
#define I2S_RCR2_SYNC_SHIFT                      30
#define I2S_RCR2_SYNC_WIDTH                      2
#define I2S_RCR2_SYNC(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_RCR2_SYNC_SHIFT))&I2S_RCR2_SYNC_MASK)
/* RCR3 Bit Fields */
#define I2S_RCR3_WDFL_MASK                       0xFu
#define I2S_RCR3_WDFL_SHIFT                      0
#define I2S_RCR3_WDFL_WIDTH                      4
#define I2S_RCR3_WDFL(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_RCR3_WDFL_SHIFT))&I2S_RCR3_WDFL_MASK)
#define I2S_RCR3_RCE		((uint32_t)0x10000u)
/* RCR4 Bit Fields */
#define I2S_RCR4_FSD		((uint32_t)0x1u)
#define I2S_RCR4_FSP		((uint32_t)0x2u)
#define I2S_RCR4_ONDEM		((uint32_t)0x4u)
#define I2S_RCR4_FSE		((uint32_t)0x8u)
#define I2S_RCR4_MF		((uint32_t)0x10u)
#define I2S_RCR4_SYWD_MASK                       0x1F00u
#define I2S_RCR4_SYWD_SHIFT                      8
#define I2S_RCR4_SYWD_WIDTH                      5
#define I2S_RCR4_SYWD(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_RCR4_SYWD_SHIFT))&I2S_RCR4_SYWD_MASK)
#define I2S_RCR4_FRSZ_MASK                       0xF0000u
#define I2S_RCR4_FRSZ_SHIFT                      16
#define I2S_RCR4_FRSZ_WIDTH                      4
#define I2S_RCR4_FRSZ(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_RCR4_FRSZ_SHIFT))&I2S_RCR4_FRSZ_MASK)
#define I2S_RCR4_FPACK_MASK                      0x3000000u
#define I2S_RCR4_FPACK_SHIFT                     24
#define I2S_RCR4_FPACK_WIDTH                     2
#define I2S_RCR4_FPACK(x)                        (((uint32_t)(((uint32_t)(x))<<I2S_RCR4_FPACK_SHIFT))&I2S_RCR4_FPACK_MASK)
#define I2S_RCR4_FCONT		((uint32_t)0x10000000u)
/* RCR5 Bit Fields */
#define I2S_RCR5_FBT_MASK                        0x1F00u
#define I2S_RCR5_FBT_SHIFT                       8
#define I2S_RCR5_FBT_WIDTH                       5
#define I2S_RCR5_FBT(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_RCR5_FBT_SHIFT))&I2S_RCR5_FBT_MASK)
#define I2S_RCR5_W0W_MASK                        0x1F0000u
#define I2S_RCR5_W0W_SHIFT                       16
#define I2S_RCR5_W0W_WIDTH                       5
#define I2S_RCR5_W0W(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_RCR5_W0W_SHIFT))&I2S_RCR5_W0W_MASK)
#define I2S_RCR5_WNW_MASK                        0x1F000000u
#define I2S_RCR5_WNW_SHIFT                       24
#define I2S_RCR5_WNW_WIDTH                       5
#define I2S_RCR5_WNW(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_RCR5_WNW_SHIFT))&I2S_RCR5_WNW_MASK)
/* RDR Bit Fields */
#define I2S_RDR_RDR_MASK                         0xFFFFFFFFu
#define I2S_RDR_RDR_SHIFT                        0
#define I2S_RDR_RDR_WIDTH                        32
#define I2S_RDR_RDR(x)                           (((uint32_t)(((uint32_t)(x))<<I2S_RDR_RDR_SHIFT))&I2S_RDR_RDR_MASK)
/* RFR Bit Fields */
#define I2S_RFR_RFP_MASK                         0xFu
#define I2S_RFR_RFP_SHIFT                        0
#define I2S_RFR_RFP_WIDTH                        4
#define I2S_RFR_RFP(x)                           (((uint32_t)(((uint32_t)(x))<<I2S_RFR_RFP_SHIFT))&I2S_RFR_RFP_MASK)
#define I2S_RFR_WFP_MASK                         0xF0000u
#define I2S_RFR_WFP_SHIFT                        16
#define I2S_RFR_WFP_WIDTH                        4
#define I2S_RFR_WFP(x)                           (((uint32_t)(((uint32_t)(x))<<I2S_RFR_WFP_SHIFT))&I2S_RFR_WFP_MASK)
/* RMR Bit Fields */
#define I2S_RMR_RWM_MASK                         0xFFFFu
#define I2S_RMR_RWM_SHIFT                        0
#define I2S_RMR_RWM_WIDTH                        16
#define I2S_RMR_RWM(x)                           (((uint32_t)(((uint32_t)(x))<<I2S_RMR_RWM_SHIFT))&I2S_RMR_RWM_MASK)
/* MCR Bit Fields */
#define I2S_MCR_MICS_MASK                        0x3000000u
#define I2S_MCR_MICS_SHIFT                       24
#define I2S_MCR_MICS_WIDTH                       2
#define I2S_MCR_MICS(x)                          (((uint32_t)(((uint32_t)(x))<<I2S_MCR_MICS_SHIFT))&I2S_MCR_MICS_MASK)
#define I2S_MCR_MOE		((uint32_t)0x40000000u)
#define I2S_MCR_DUF		((uint32_t)0x80000000u)
/* MDR Bit Fields */
#define I2S_MDR_DIVIDE_MASK                      0xFFFu
#define I2S_MDR_DIVIDE_SHIFT                     0
#define I2S_MDR_DIVIDE_WIDTH                     12
#define I2S_MDR_DIVIDE(x)                        (((uint32_t)(((uint32_t)(x))<<I2S_MDR_DIVIDE_SHIFT))&I2S_MDR_DIVIDE_MASK)
#define I2S_MDR_FRACT_MASK                       0xFF000u
#define I2S_MDR_FRACT_SHIFT                      12
#define I2S_MDR_FRACT_WIDTH                      8
#define I2S_MDR_FRACT(x)                         (((uint32_t)(((uint32_t)(x))<<I2S_MDR_FRACT_SHIFT))&I2S_MDR_FRACT_MASK)

/*!
 * @}
 */ /* end of group I2S_Register_Masks */


/* I2S - Peripheral instance base addresses */
/** Peripheral I2S0 base address */
#define I2S0_BASE                                (0x4002F000u)
/** Peripheral I2S0 base pointer */
#define I2S0                                     ((I2S_TypeDef *)I2S0_BASE)
#define I2S0_BASE_PTR                            (I2S0)
/** Array initializer of I2S peripheral base addresses */
#define I2S_BASE_ADDRS                           { I2S0_BASE }
/** Array initializer of I2S peripheral base pointers */
#define I2S_BASE_PTRS                            { I2S0 }
/** Interrupt vectors for the I2S peripheral type */
#define I2S_RX_IRQS                              { I2S0_Rx_IRQn }
#define I2S_TX_IRQS                              { I2S0_Tx_IRQn }

/* ----------------------------------------------------------------------------
   -- I2S - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup I2S_Register_Accessor_Macros I2S - Register accessor macros
 * @{
 */


/* I2S - Register instance definitions */
/* I2S0 */
#define I2S0_TCSR                                I2S_TCSR_REG(I2S0)
#define I2S0_TCR1                                I2S_TCR1_REG(I2S0)
#define I2S0_TCR2                                I2S_TCR2_REG(I2S0)
#define I2S0_TCR3                                I2S_TCR3_REG(I2S0)
#define I2S0_TCR4                                I2S_TCR4_REG(I2S0)
#define I2S0_TCR5                                I2S_TCR5_REG(I2S0)
#define I2S0_TDR0                                I2S_TDR_REG(I2S0,0)
#define I2S0_TFR0                                I2S_TFR_REG(I2S0,0)
#define I2S0_TMR                                 I2S_TMR_REG(I2S0)
#define I2S0_RCSR                                I2S_RCSR_REG(I2S0)
#define I2S0_RCR1                                I2S_RCR1_REG(I2S0)
#define I2S0_RCR2                                I2S_RCR2_REG(I2S0)
#define I2S0_RCR3                                I2S_RCR3_REG(I2S0)
#define I2S0_RCR4                                I2S_RCR4_REG(I2S0)
#define I2S0_RCR5                                I2S_RCR5_REG(I2S0)
#define I2S0_RDR0                                I2S_RDR_REG(I2S0,0)
#define I2S0_RFR0                                I2S_RFR_REG(I2S0,0)
#define I2S0_RMR                                 I2S_RMR_REG(I2S0)
#define I2S0_MCR                                 I2S_MCR_REG(I2S0)
#define I2S0_MDR                                 I2S_MDR_REG(I2S0)

/* I2S - Register array accessors */
#define I2S0_TDR(index)                          I2S_TDR_REG(I2S0,index)
#define I2S0_TFR(index)                          I2S_TFR_REG(I2S0,index)
#define I2S0_RDR(index)                          I2S_RDR_REG(I2S0,index)
#define I2S0_RFR(index)                          I2S_RFR_REG(I2S0,index)

/*!
 * @}
 */ /* end of group I2S_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group I2S_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LLWU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Peripheral_Access_Layer LLWU Peripheral Access Layer
 * @{
 */

/** LLWU - Register Layout Typedef */
typedef struct {
  __IO uint8_t PE1;                                /**< LLWU Pin Enable 1 register, offset: 0x0 */
  __IO uint8_t PE2;                                /**< LLWU Pin Enable 2 register, offset: 0x1 */
  __IO uint8_t PE3;                                /**< LLWU Pin Enable 3 register, offset: 0x2 */
  __IO uint8_t PE4;                                /**< LLWU Pin Enable 4 register, offset: 0x3 */
  __IO uint8_t ME;                                 /**< LLWU Module Enable register, offset: 0x4 */
  __IO uint8_t F1;                                 /**< LLWU Flag 1 register, offset: 0x5 */
  __IO uint8_t F2;                                 /**< LLWU Flag 2 register, offset: 0x6 */
  __I  uint8_t F3;                                 /**< LLWU Flag 3 register, offset: 0x7 */
  __IO uint8_t FILT1;                              /**< LLWU Pin Filter 1 register, offset: 0x8 */
  __IO uint8_t FILT2;                              /**< LLWU Pin Filter 2 register, offset: 0x9 */
} LLWU_TypeDef, *LLWU_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- LLWU - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Register_Accessor_Macros LLWU - Register accessor macros
 * @{
 */


/* LLWU - Register accessors */
#define LLWU_PE1_REG(base)                       ((base)->PE1)
#define LLWU_PE2_REG(base)                       ((base)->PE2)
#define LLWU_PE3_REG(base)                       ((base)->PE3)
#define LLWU_PE4_REG(base)                       ((base)->PE4)
#define LLWU_ME_REG(base)                        ((base)->ME)
#define LLWU_F1_REG(base)                        ((base)->F1)
#define LLWU_F2_REG(base)                        ((base)->F2)
#define LLWU_F3_REG(base)                        ((base)->F3)
#define LLWU_FILT1_REG(base)                     ((base)->FILT1)
#define LLWU_FILT2_REG(base)                     ((base)->FILT2)

/*!
 * @}
 */ /* end of group LLWU_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- LLWU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Register_Masks LLWU Register Masks
 * @{
 */

/* PE1 Bit Fields */
#define LLWU_PE1_WUPE0_MASK                      0x3u
#define LLWU_PE1_WUPE0_SHIFT                     0
#define LLWU_PE1_WUPE0_WIDTH                     2
#define LLWU_PE1_WUPE0(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE1_WUPE0_SHIFT))&LLWU_PE1_WUPE0_MASK)
#define LLWU_PE1_WUPE1_MASK                      0xCu
#define LLWU_PE1_WUPE1_SHIFT                     2
#define LLWU_PE1_WUPE1_WIDTH                     2
#define LLWU_PE1_WUPE1(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE1_WUPE1_SHIFT))&LLWU_PE1_WUPE1_MASK)
#define LLWU_PE1_WUPE2_MASK                      0x30u
#define LLWU_PE1_WUPE2_SHIFT                     4
#define LLWU_PE1_WUPE2_WIDTH                     2
#define LLWU_PE1_WUPE2(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE1_WUPE2_SHIFT))&LLWU_PE1_WUPE2_MASK)
#define LLWU_PE1_WUPE3_MASK                      0xC0u
#define LLWU_PE1_WUPE3_SHIFT                     6
#define LLWU_PE1_WUPE3_WIDTH                     2
#define LLWU_PE1_WUPE3(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE1_WUPE3_SHIFT))&LLWU_PE1_WUPE3_MASK)
/* PE2 Bit Fields */
#define LLWU_PE2_WUPE4_MASK                      0x3u
#define LLWU_PE2_WUPE4_SHIFT                     0
#define LLWU_PE2_WUPE4_WIDTH                     2
#define LLWU_PE2_WUPE4(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE2_WUPE4_SHIFT))&LLWU_PE2_WUPE4_MASK)
#define LLWU_PE2_WUPE5_MASK                      0xCu
#define LLWU_PE2_WUPE5_SHIFT                     2
#define LLWU_PE2_WUPE5_WIDTH                     2
#define LLWU_PE2_WUPE5(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE2_WUPE5_SHIFT))&LLWU_PE2_WUPE5_MASK)
#define LLWU_PE2_WUPE6_MASK                      0x30u
#define LLWU_PE2_WUPE6_SHIFT                     4
#define LLWU_PE2_WUPE6_WIDTH                     2
#define LLWU_PE2_WUPE6(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE2_WUPE6_SHIFT))&LLWU_PE2_WUPE6_MASK)
#define LLWU_PE2_WUPE7_MASK                      0xC0u
#define LLWU_PE2_WUPE7_SHIFT                     6
#define LLWU_PE2_WUPE7_WIDTH                     2
#define LLWU_PE2_WUPE7(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE2_WUPE7_SHIFT))&LLWU_PE2_WUPE7_MASK)
/* PE3 Bit Fields */
#define LLWU_PE3_WUPE8_MASK                      0x3u
#define LLWU_PE3_WUPE8_SHIFT                     0
#define LLWU_PE3_WUPE8_WIDTH                     2
#define LLWU_PE3_WUPE8(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE3_WUPE8_SHIFT))&LLWU_PE3_WUPE8_MASK)
#define LLWU_PE3_WUPE9_MASK                      0xCu
#define LLWU_PE3_WUPE9_SHIFT                     2
#define LLWU_PE3_WUPE9_WIDTH                     2
#define LLWU_PE3_WUPE9(x)                        (((uint8_t)(((uint8_t)(x))<<LLWU_PE3_WUPE9_SHIFT))&LLWU_PE3_WUPE9_MASK)
#define LLWU_PE3_WUPE10_MASK                     0x30u
#define LLWU_PE3_WUPE10_SHIFT                    4
#define LLWU_PE3_WUPE10_WIDTH                    2
#define LLWU_PE3_WUPE10(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE3_WUPE10_SHIFT))&LLWU_PE3_WUPE10_MASK)
#define LLWU_PE3_WUPE11_MASK                     0xC0u
#define LLWU_PE3_WUPE11_SHIFT                    6
#define LLWU_PE3_WUPE11_WIDTH                    2
#define LLWU_PE3_WUPE11(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE3_WUPE11_SHIFT))&LLWU_PE3_WUPE11_MASK)
/* PE4 Bit Fields */
#define LLWU_PE4_WUPE12_MASK                     0x3u
#define LLWU_PE4_WUPE12_SHIFT                    0
#define LLWU_PE4_WUPE12_WIDTH                    2
#define LLWU_PE4_WUPE12(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE4_WUPE12_SHIFT))&LLWU_PE4_WUPE12_MASK)
#define LLWU_PE4_WUPE13_MASK                     0xCu
#define LLWU_PE4_WUPE13_SHIFT                    2
#define LLWU_PE4_WUPE13_WIDTH                    2
#define LLWU_PE4_WUPE13(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE4_WUPE13_SHIFT))&LLWU_PE4_WUPE13_MASK)
#define LLWU_PE4_WUPE14_MASK                     0x30u
#define LLWU_PE4_WUPE14_SHIFT                    4
#define LLWU_PE4_WUPE14_WIDTH                    2
#define LLWU_PE4_WUPE14(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE4_WUPE14_SHIFT))&LLWU_PE4_WUPE14_MASK)
#define LLWU_PE4_WUPE15_MASK                     0xC0u
#define LLWU_PE4_WUPE15_SHIFT                    6
#define LLWU_PE4_WUPE15_WIDTH                    2
#define LLWU_PE4_WUPE15(x)                       (((uint8_t)(((uint8_t)(x))<<LLWU_PE4_WUPE15_SHIFT))&LLWU_PE4_WUPE15_MASK)
/* ME Bit Fields */
#define LLWU_ME_WUME0		((uint8_t)0x1u)
#define LLWU_ME_WUME1		((uint8_t)0x2u)
#define LLWU_ME_WUME2		((uint8_t)0x4u)
#define LLWU_ME_WUME3		((uint8_t)0x8u)
#define LLWU_ME_WUME4		((uint8_t)0x10u)
#define LLWU_ME_WUME5		((uint8_t)0x20u)
#define LLWU_ME_WUME6		((uint8_t)0x40u)
#define LLWU_ME_WUME7		((uint8_t)0x80u)
/* F1 Bit Fields */
#define LLWU_F1_WUF0		((uint8_t)0x1u)
#define LLWU_F1_WUF1		((uint8_t)0x2u)
#define LLWU_F1_WUF2		((uint8_t)0x4u)
#define LLWU_F1_WUF3		((uint8_t)0x8u)
#define LLWU_F1_WUF4		((uint8_t)0x10u)
#define LLWU_F1_WUF5		((uint8_t)0x20u)
#define LLWU_F1_WUF6		((uint8_t)0x40u)
#define LLWU_F1_WUF7		((uint8_t)0x80u)
/* F2 Bit Fields */
#define LLWU_F2_WUF8		((uint8_t)0x1u)
#define LLWU_F2_WUF9		((uint8_t)0x2u)
#define LLWU_F2_WUF10		((uint8_t)0x4u)
#define LLWU_F2_WUF11		((uint8_t)0x8u)
#define LLWU_F2_WUF12		((uint8_t)0x10u)
#define LLWU_F2_WUF13		((uint8_t)0x20u)
#define LLWU_F2_WUF14		((uint8_t)0x40u)
#define LLWU_F2_WUF15		((uint8_t)0x80u)
/* F3 Bit Fields */
#define LLWU_F3_MWUF0		((uint8_t)0x1u)
#define LLWU_F3_MWUF1		((uint8_t)0x2u)
#define LLWU_F3_MWUF2		((uint8_t)0x4u)
#define LLWU_F3_MWUF3		((uint8_t)0x8u)
#define LLWU_F3_MWUF4		((uint8_t)0x10u)
#define LLWU_F3_MWUF5		((uint8_t)0x20u)
#define LLWU_F3_MWUF6		((uint8_t)0x40u)
#define LLWU_F3_MWUF7		((uint8_t)0x80u)
/* FILT1 Bit Fields */
#define LLWU_FILT1_FILTSEL_MASK                  0xFu
#define LLWU_FILT1_FILTSEL_SHIFT                 0
#define LLWU_FILT1_FILTSEL_WIDTH                 4
#define LLWU_FILT1_FILTSEL(x)                    (((uint8_t)(((uint8_t)(x))<<LLWU_FILT1_FILTSEL_SHIFT))&LLWU_FILT1_FILTSEL_MASK)
#define LLWU_FILT1_FILTE_MASK                    0x60u
#define LLWU_FILT1_FILTE_SHIFT                   5
#define LLWU_FILT1_FILTE_WIDTH                   2
#define LLWU_FILT1_FILTE(x)                      (((uint8_t)(((uint8_t)(x))<<LLWU_FILT1_FILTE_SHIFT))&LLWU_FILT1_FILTE_MASK)
#define LLWU_FILT1_FILTF		((uint8_t)0x80u)
/* FILT2 Bit Fields */
#define LLWU_FILT2_FILTSEL_MASK                  0xFu
#define LLWU_FILT2_FILTSEL_SHIFT                 0
#define LLWU_FILT2_FILTSEL_WIDTH                 4
#define LLWU_FILT2_FILTSEL(x)                    (((uint8_t)(((uint8_t)(x))<<LLWU_FILT2_FILTSEL_SHIFT))&LLWU_FILT2_FILTSEL_MASK)
#define LLWU_FILT2_FILTE_MASK                    0x60u
#define LLWU_FILT2_FILTE_SHIFT                   5
#define LLWU_FILT2_FILTE_WIDTH                   2
#define LLWU_FILT2_FILTE(x)                      (((uint8_t)(((uint8_t)(x))<<LLWU_FILT2_FILTE_SHIFT))&LLWU_FILT2_FILTE_MASK)
#define LLWU_FILT2_FILTF		((uint8_t)0x80u)

/*!
 * @}
 */ /* end of group LLWU_Register_Masks */


/* LLWU - Peripheral instance base addresses */
/** Peripheral LLWU base address */
#define LLWU_BASE                                (0x4007C000u)
/** Peripheral LLWU base pointer */
#define LLWU                                     ((LLWU_TypeDef *)LLWU_BASE)
#define LLWU_BASE_PTR                            (LLWU)
/** Array initializer of LLWU peripheral base addresses */
#define LLWU_BASE_ADDRS                          { LLWU_BASE }
/** Array initializer of LLWU peripheral base pointers */
#define LLWU_BASE_PTRS                           { LLWU }
/** Interrupt vectors for the LLWU peripheral type */
#define LLWU_IRQS                                { LLWU_IRQn }

/* ----------------------------------------------------------------------------
   -- LLWU - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LLWU_Register_Accessor_Macros LLWU - Register accessor macros
 * @{
 */


/* LLWU - Register instance definitions */
/* LLWU */
#define LLWU_PE1                                 LLWU_PE1_REG(LLWU)
#define LLWU_PE2                                 LLWU_PE2_REG(LLWU)
#define LLWU_PE3                                 LLWU_PE3_REG(LLWU)
#define LLWU_PE4                                 LLWU_PE4_REG(LLWU)
#define LLWU_ME                                  LLWU_ME_REG(LLWU)
#define LLWU_F1                                  LLWU_F1_REG(LLWU)
#define LLWU_F2                                  LLWU_F2_REG(LLWU)
#define LLWU_F3                                  LLWU_F3_REG(LLWU)
#define LLWU_FILT1                               LLWU_FILT1_REG(LLWU)
#define LLWU_FILT2                               LLWU_FILT2_REG(LLWU)

/*!
 * @}
 */ /* end of group LLWU_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group LLWU_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPTMR Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Peripheral_Access_Layer LPTMR Peripheral Access Layer
 * @{
 */

/** LPTMR - Register Layout Typedef */
typedef struct {
  __IO uint32_t CSR;                               /**< Low Power Timer Control Status Register, offset: 0x0 */
  __IO uint32_t PSR;                               /**< Low Power Timer Prescale Register, offset: 0x4 */

  __IO uint32_t CMR;                               /**< Low Power Timer Compare Register, offset: 0x8 */
  __IO uint32_t CNR;                               /**< Low Power Timer Counter Register, offset: 0xC */
} LPTMR_TypeDef, *LPTMR_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- LPTMR - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Accessor_Macros LPTMR - Register accessor macros
 * @{
 */


/* LPTMR - Register accessors */
#define LPTMR_CSR_REG(base)                      ((base)->CSR)
#define LPTMR_PSR_REG(base)                      ((base)->PSR)
#define LPTMR_CMR_REG(base)                      ((base)->CMR)
#define LPTMR_CNR_REG(base)                      ((base)->CNR)

/*!
 * @}
 */ /* end of group LPTMR_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- LPTMR Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Masks LPTMR Register Masks
 * @{
 */

/* CSR Bit Fields */
#define LPTMR_CSR_TEN		((uint32_t)0x1u)
#define LPTMR_CSR_TMS		((uint32_t)0x2u)
#define LPTMR_CSR_TFC		((uint32_t)0x4u)
#define LPTMR_CSR_TPP		((uint32_t)0x8u)
#define LPTMR_CSR_TPS_MASK                       0x30u
#define LPTMR_CSR_TPS_SHIFT                      4
#define LPTMR_CSR_TPS_WIDTH                      2
#define LPTMR_CSR_TPS(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_CSR_TPS_SHIFT))&LPTMR_CSR_TPS_MASK)
#define LPTMR_CSR_TIE		((uint32_t)0x40u)
#define LPTMR_CSR_TCF		((uint32_t)0x80u)
/* PSR Bit Fields */
#define LPTMR_PSR_PCS_MASK                       0x3u
#define LPTMR_PSR_PCS_SHIFT                      0
#define LPTMR_PSR_PCS_WIDTH                      2
#define LPTMR_PSR_PCS(x)                         (((uint32_t)(((uint32_t)(x))<<LPTMR_PSR_PCS_SHIFT))&LPTMR_PSR_PCS_MASK)
#define LPTMR_PSR_PBYP		((uint32_t)0x4u)
#define LPTMR_PSR_PRESCALE_MASK                  0x78u
#define LPTMR_PSR_PRESCALE_SHIFT                 3
#define LPTMR_PSR_PRESCALE_WIDTH                 4
#define LPTMR_PSR_PRESCALE(x)                    (((uint32_t)(((uint32_t)(x))<<LPTMR_PSR_PRESCALE_SHIFT))&LPTMR_PSR_PRESCALE_MASK)
/* CMR Bit Fields */
#define LPTMR_CMR_COMPARE_MASK                   0xFFFFu
#define LPTMR_CMR_COMPARE_SHIFT                  0
#define LPTMR_CMR_COMPARE_WIDTH                  16
#define LPTMR_CMR_COMPARE(x)                     (((uint32_t)(((uint32_t)(x))<<LPTMR_CMR_COMPARE_SHIFT))&LPTMR_CMR_COMPARE_MASK)
/* CNR Bit Fields */
#define LPTMR_CNR_COUNTER_MASK                   0xFFFFu
#define LPTMR_CNR_COUNTER_SHIFT                  0
#define LPTMR_CNR_COUNTER_WIDTH                  16
#define LPTMR_CNR_COUNTER(x)                     (((uint32_t)(((uint32_t)(x))<<LPTMR_CNR_COUNTER_SHIFT))&LPTMR_CNR_COUNTER_MASK)

/*!
 * @}
 */ /* end of group LPTMR_Register_Masks */


/* LPTMR - Peripheral instance base addresses */
/** Peripheral LPTMR0 base address */
#define LPTMR0_BASE                              (0x40040000u)
/** Peripheral LPTMR0 base pointer */
#define LPTMR0                                   ((LPTMR_TypeDef *)LPTMR0_BASE)
#define LPTMR0_BASE_PTR                          (LPTMR0)
/** Array initializer of LPTMR peripheral base addresses */
#define LPTMR_BASE_ADDRS                         { LPTMR0_BASE }
/** Array initializer of LPTMR peripheral base pointers */
#define LPTMR_BASE_PTRS                          { LPTMR0 }
/** Interrupt vectors for the LPTMR peripheral type */
#define LPTMR_IRQS                               { LPTMR0_IRQn }

/* ----------------------------------------------------------------------------
   -- LPTMR - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPTMR_Register_Accessor_Macros LPTMR - Register accessor macros
 * @{
 */


/* LPTMR - Register instance definitions */
/* LPTMR0 */
#define LPTMR0_CSR                               LPTMR_CSR_REG(LPTMR0)
#define LPTMR0_PSR                               LPTMR_PSR_REG(LPTMR0)
#define LPTMR0_CMR                               LPTMR_CMR_REG(LPTMR0)
#define LPTMR0_CNR                               LPTMR_CNR_REG(LPTMR0)

/*!
 * @}
 */ /* end of group LPTMR_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group LPTMR_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- LPUART Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Peripheral_Access_Layer LPUART Peripheral Access Layer
 * @{
 */

/** LPUART - Register Layout Typedef */
typedef struct {
  __IO uint32_t BAUD;                              /**< LPUART Baud Rate Register, offset: 0x0 */
  __IO uint32_t STAT;                              /**< LPUART Status Register, offset: 0x4 */
  __IO uint32_t CTRL;                              /**< LPUART Control Register, offset: 0x8 */
  __IO uint32_t DATA;                              /**< LPUART Data Register, offset: 0xC */
  __IO uint32_t MATCH;                             /**< LPUART Match Address Register, offset: 0x10 */
  __IO uint32_t MODIR;                             /**< LPUART Modem IrDA Register, offset: 0x14 */
} LPUART_TypeDef, *LPUART_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- LPUART - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Register_Accessor_Macros LPUART - Register accessor macros
 * @{
 */


/* LPUART - Register accessors */
#define LPUART_BAUD_REG(base)                    ((base)->BAUD)
#define LPUART_STAT_REG(base)                    ((base)->STAT)
#define LPUART_CTRL_REG(base)                    ((base)->CTRL)
#define LPUART_DATA_REG(base)                    ((base)->DATA)
#define LPUART_MATCH_REG(base)                   ((base)->MATCH)
#define LPUART_MODIR_REG(base)                   ((base)->MODIR)

/*!
 * @}
 */ /* end of group LPUART_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- LPUART Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Register_Masks LPUART Register Masks
 * @{
 */

/* BAUD Bit Fields */
#define LPUART_BAUD_SBR_MASK                     0x1FFFu
#define LPUART_BAUD_SBR_SHIFT                    0
#define LPUART_BAUD_SBR_WIDTH                    13
#define LPUART_BAUD_SBR(x)                       (((uint32_t)(((uint32_t)(x))<<LPUART_BAUD_SBR_SHIFT))&LPUART_BAUD_SBR_MASK)
#define LPUART_BAUD_SBNS		((uint32_t)0x2000u)
#define LPUART_BAUD_RXEDGIE		((uint32_t)0x4000u)
#define LPUART_BAUD_LBKDIE		((uint32_t)0x8000u)
#define LPUART_BAUD_RESYNCDIS		((uint32_t)0x10000u)
#define LPUART_BAUD_BOTHEDGE		((uint32_t)0x20000u)
#define LPUART_BAUD_MATCFG_MASK                  0xC0000u
#define LPUART_BAUD_MATCFG_SHIFT                 18
#define LPUART_BAUD_MATCFG_WIDTH                 2
#define LPUART_BAUD_MATCFG(x)                    (((uint32_t)(((uint32_t)(x))<<LPUART_BAUD_MATCFG_SHIFT))&LPUART_BAUD_MATCFG_MASK)
#define LPUART_BAUD_RDMAE		((uint32_t)0x200000u)
#define LPUART_BAUD_TDMAE		((uint32_t)0x800000u)
#define LPUART_BAUD_OSR_MASK                     0x1F000000u
#define LPUART_BAUD_OSR_SHIFT                    24
#define LPUART_BAUD_OSR_WIDTH                    5
#define LPUART_BAUD_OSR(x)                       (((uint32_t)(((uint32_t)(x))<<LPUART_BAUD_OSR_SHIFT))&LPUART_BAUD_OSR_MASK)
#define LPUART_BAUD_M10		((uint32_t)0x20000000u)
#define LPUART_BAUD_MAEN2		((uint32_t)0x40000000u)
#define LPUART_BAUD_MAEN1		((uint32_t)0x80000000u)
/* STAT Bit Fields */
#define LPUART_STAT_MA2F		((uint32_t)0x4000u)
#define LPUART_STAT_MA1F		((uint32_t)0x8000u)
#define LPUART_STAT_PF		((uint32_t)0x10000u)
#define LPUART_STAT_FE		((uint32_t)0x20000u)
#define LPUART_STAT_NF		((uint32_t)0x40000u)
#define LPUART_STAT_OR		((uint32_t)0x80000u)
#define LPUART_STAT_IDLE		((uint32_t)0x100000u)
#define LPUART_STAT_RDRF		((uint32_t)0x200000u)
#define LPUART_STAT_TC		((uint32_t)0x400000u)
#define LPUART_STAT_TDRE		((uint32_t)0x800000u)
#define LPUART_STAT_RAF		((uint32_t)0x1000000u)
#define LPUART_STAT_LBKDE		((uint32_t)0x2000000u)
#define LPUART_STAT_BRK13		((uint32_t)0x4000000u)
#define LPUART_STAT_RWUID		((uint32_t)0x8000000u)
#define LPUART_STAT_RXINV		((uint32_t)0x10000000u)
#define LPUART_STAT_MSBF		((uint32_t)0x20000000u)
#define LPUART_STAT_RXEDGIF		((uint32_t)0x40000000u)
#define LPUART_STAT_LBKDIF		((uint32_t)0x80000000u)
/* CTRL Bit Fields */
#define LPUART_CTRL_PT		((uint32_t)0x1u)
#define LPUART_CTRL_PE		((uint32_t)0x2u)
#define LPUART_CTRL_ILT		((uint32_t)0x4u)
#define LPUART_CTRL_WAKE		((uint32_t)0x8u)
#define LPUART_CTRL_M		((uint32_t)0x10u)
#define LPUART_CTRL_RSRC		((uint32_t)0x20u)
#define LPUART_CTRL_DOZEEN		((uint32_t)0x40u)
#define LPUART_CTRL_LOOPS		((uint32_t)0x80u)
#define LPUART_CTRL_IDLECFG_MASK                 0x700u
#define LPUART_CTRL_IDLECFG_SHIFT                8
#define LPUART_CTRL_IDLECFG_WIDTH                3
#define LPUART_CTRL_IDLECFG(x)                   (((uint32_t)(((uint32_t)(x))<<LPUART_CTRL_IDLECFG_SHIFT))&LPUART_CTRL_IDLECFG_MASK)
#define LPUART_CTRL_MA2IE		((uint32_t)0x4000u)
#define LPUART_CTRL_MA1IE		((uint32_t)0x8000u)
#define LPUART_CTRL_SBK		((uint32_t)0x10000u)
#define LPUART_CTRL_RWU		((uint32_t)0x20000u)
#define LPUART_CTRL_RE		((uint32_t)0x40000u)
#define LPUART_CTRL_TE		((uint32_t)0x80000u)
#define LPUART_CTRL_ILIE		((uint32_t)0x100000u)
#define LPUART_CTRL_RIE		((uint32_t)0x200000u)
#define LPUART_CTRL_TCIE		((uint32_t)0x400000u)
#define LPUART_CTRL_TIE		((uint32_t)0x800000u)
#define LPUART_CTRL_PEIE		((uint32_t)0x1000000u)
#define LPUART_CTRL_FEIE		((uint32_t)0x2000000u)
#define LPUART_CTRL_NEIE		((uint32_t)0x4000000u)
#define LPUART_CTRL_ORIE		((uint32_t)0x8000000u)
#define LPUART_CTRL_TXINV		((uint32_t)0x10000000u)
#define LPUART_CTRL_TXDIR		((uint32_t)0x20000000u)
#define LPUART_CTRL_R9T8		((uint32_t)0x40000000u)
#define LPUART_CTRL_R8T9		((uint32_t)0x80000000u)
/* DATA Bit Fields */
#define LPUART_DATA_R0T0		((uint32_t)0x1u)
#define LPUART_DATA_R1T1		((uint32_t)0x2u)
#define LPUART_DATA_R2T2		((uint32_t)0x4u)
#define LPUART_DATA_R3T3		((uint32_t)0x8u)
#define LPUART_DATA_R4T4		((uint32_t)0x10u)
#define LPUART_DATA_R5T5		((uint32_t)0x20u)
#define LPUART_DATA_R6T6		((uint32_t)0x40u)
#define LPUART_DATA_R7T7		((uint32_t)0x80u)
#define LPUART_DATA_R8T8		((uint32_t)0x100u)
#define LPUART_DATA_R9T9		((uint32_t)0x200u)
#define LPUART_DATA_IDLINE		((uint32_t)0x800u)
#define LPUART_DATA_RXEMPT		((uint32_t)0x1000u)
#define LPUART_DATA_FRETSC		((uint32_t)0x2000u)
#define LPUART_DATA_PARITYE		((uint32_t)0x4000u)
#define LPUART_DATA_NOISY		((uint32_t)0x8000u)
/* MATCH Bit Fields */
#define LPUART_MATCH_MA1_MASK                    0x3FFu
#define LPUART_MATCH_MA1_SHIFT                   0
#define LPUART_MATCH_MA1_WIDTH                   10
#define LPUART_MATCH_MA1(x)                      (((uint32_t)(((uint32_t)(x))<<LPUART_MATCH_MA1_SHIFT))&LPUART_MATCH_MA1_MASK)
#define LPUART_MATCH_MA2_MASK                    0x3FF0000u
#define LPUART_MATCH_MA2_SHIFT                   16
#define LPUART_MATCH_MA2_WIDTH                   10
#define LPUART_MATCH_MA2(x)                      (((uint32_t)(((uint32_t)(x))<<LPUART_MATCH_MA2_SHIFT))&LPUART_MATCH_MA2_MASK)
/* MODIR Bit Fields */
#define LPUART_MODIR_TXCTSE		((uint32_t)0x1u)
#define LPUART_MODIR_TXRTSE		((uint32_t)0x2u)
#define LPUART_MODIR_TXRTSPOL		((uint32_t)0x4u)
#define LPUART_MODIR_RXRTSE		((uint32_t)0x8u)
#define LPUART_MODIR_TXCTSC		((uint32_t)0x10u)
#define LPUART_MODIR_TXCTSSRC		((uint32_t)0x20u)
#define LPUART_MODIR_TNP_MASK                    0x30000u
#define LPUART_MODIR_TNP_SHIFT                   16
#define LPUART_MODIR_TNP_WIDTH                   2
#define LPUART_MODIR_TNP(x)                      (((uint32_t)(((uint32_t)(x))<<LPUART_MODIR_TNP_SHIFT))&LPUART_MODIR_TNP_MASK)
#define LPUART_MODIR_IREN		((uint32_t)0x40000u)

/*!
 * @}
 */ /* end of group LPUART_Register_Masks */


/* LPUART - Peripheral instance base addresses */
/** Peripheral LPUART0 base address */
#define LPUART0_BASE                             (0x4002A000u)
/** Peripheral LPUART0 base pointer */
#define LPUART0                                  ((LPUART_TypeDef *)LPUART0_BASE)
#define LPUART0_BASE_PTR                         (LPUART0)
/** Array initializer of LPUART peripheral base addresses */
#define LPUART_BASE_ADDRS                        { LPUART0_BASE }
/** Array initializer of LPUART peripheral base pointers */
#define LPUART_BASE_PTRS                         { LPUART0 }
/** Interrupt vectors for the LPUART peripheral type */
#define LPUART_RX_TX_IRQS                        { LPUART0_IRQn }
#define LPUART_ERR_IRQS                          { LPUART0_IRQn }

/* ----------------------------------------------------------------------------
   -- LPUART - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup LPUART_Register_Accessor_Macros LPUART - Register accessor macros
 * @{
 */


/* LPUART - Register instance definitions */
/* LPUART0 */
#define LPUART0_BAUD                             LPUART_BAUD_REG(LPUART0)
#define LPUART0_STAT                             LPUART_STAT_REG(LPUART0)
#define LPUART0_CTRL                             LPUART_CTRL_REG(LPUART0)
#define LPUART0_DATA                             LPUART_DATA_REG(LPUART0)
#define LPUART0_MATCH                            LPUART_MATCH_REG(LPUART0)
#define LPUART0_MODIR                            LPUART_MODIR_REG(LPUART0)

/*!
 * @}
 */ /* end of group LPUART_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group LPUART_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MCG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Peripheral_Access_Layer MCG Peripheral Access Layer
 * @{
 */

/** MCG - Register Layout Typedef */
typedef struct {
  __IO uint8_t C1;                                 /**< MCG Control 1 Register, offset: 0x0 */
  __IO uint8_t C2;                                 /**< MCG Control 2 Register, offset: 0x1 */
  __IO uint8_t C3;                                 /**< MCG Control 3 Register, offset: 0x2 */
  __IO uint8_t C4;                                 /**< MCG Control 4 Register, offset: 0x3 */
  __IO uint8_t C5;                                 /**< MCG Control 5 Register, offset: 0x4 */
  __IO uint8_t C6;                                 /**< MCG Control 6 Register, offset: 0x5 */
  __IO uint8_t S;                                  /**< MCG Status Register, offset: 0x6 */
       uint8_t RESERVED_0[1];
  __IO uint8_t SC;                                 /**< MCG Status and Control Register, offset: 0x8 */
       uint8_t RESERVED_1[1];
  __IO uint8_t ATCVH;                              /**< MCG Auto Trim Compare Value High Register, offset: 0xA */
  __IO uint8_t ATCVL;                              /**< MCG Auto Trim Compare Value Low Register, offset: 0xB */
  __IO uint8_t C7;                                 /**< MCG Control 7 Register, offset: 0xC */
  __IO uint8_t C8;                                 /**< MCG Control 8 Register, offset: 0xD */
} MCG_TypeDef, *MCG_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- MCG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Register_Accessor_Macros MCG - Register accessor macros
 * @{
 */


/* MCG - Register accessors */
#define MCG_C1_REG(base)                         ((base)->C1)
#define MCG_C2_REG(base)                         ((base)->C2)
#define MCG_C3_REG(base)                         ((base)->C3)
#define MCG_C4_REG(base)                         ((base)->C4)
#define MCG_C5_REG(base)                         ((base)->C5)
#define MCG_C6_REG(base)                         ((base)->C6)
#define MCG_S_REG(base)                          ((base)->S)
#define MCG_SC_REG(base)                         ((base)->SC)
#define MCG_ATCVH_REG(base)                      ((base)->ATCVH)
#define MCG_ATCVL_REG(base)                      ((base)->ATCVL)
#define MCG_C7_REG(base)                         ((base)->C7)
#define MCG_C8_REG(base)                         ((base)->C8)

/*!
 * @}
 */ /* end of group MCG_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- MCG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Register_Masks MCG Register Masks
 * @{
 */

/* C1 Bit Fields */
#define MCG_C1_IREFSTEN		((uint8_t)0x1u)
#define MCG_C1_IRCLKEN		((uint8_t)0x2u)
#define MCG_C1_IREFS		((uint8_t)0x4u)
#define MCG_C1_FRDIV_MASK                        0x38u
#define MCG_C1_FRDIV_SHIFT                       3
#define MCG_C1_FRDIV_WIDTH                       3
#define MCG_C1_FRDIV(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_C1_FRDIV_SHIFT))&MCG_C1_FRDIV_MASK)
#define MCG_C1_CLKS_MASK                         0xC0u
#define MCG_C1_CLKS_SHIFT                        6
#define MCG_C1_CLKS_WIDTH                        2
#define MCG_C1_CLKS(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_C1_CLKS_SHIFT))&MCG_C1_CLKS_MASK)
/* C2 Bit Fields */
#define MCG_C2_IRCS		((uint8_t)0x1u)
#define MCG_C2_LP		((uint8_t)0x2u)
#define MCG_C2_EREFS0		((uint8_t)0x4u)
#define MCG_C2_HGO		((uint8_t)0x8u)
#define MCG_C2_RANGE_MASK                        0x30u
#define MCG_C2_RANGE_SHIFT                       4
#define MCG_C2_RANGE_WIDTH                       2
#define MCG_C2_RANGE(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_C2_RANGE_SHIFT))&MCG_C2_RANGE_MASK)
#define MCG_C2_FCFTRIM		((uint8_t)0x40u)
#define MCG_C2_LOCRE0		((uint8_t)0x80u)
/* C3 Bit Fields */
#define MCG_C3_SCTRIM_MASK                       0xFFu
#define MCG_C3_SCTRIM_SHIFT                      0
#define MCG_C3_SCTRIM_WIDTH                      8
#define MCG_C3_SCTRIM(x)                         (((uint8_t)(((uint8_t)(x))<<MCG_C3_SCTRIM_SHIFT))&MCG_C3_SCTRIM_MASK)
/* C4 Bit Fields */
#define MCG_C4_SCFTRIM		((uint8_t)0x1u)
#define MCG_C4_FCTRIM_MASK                       0x1Eu
#define MCG_C4_FCTRIM_SHIFT                      1
#define MCG_C4_FCTRIM_WIDTH                      4
#define MCG_C4_FCTRIM(x)                         (((uint8_t)(((uint8_t)(x))<<MCG_C4_FCTRIM_SHIFT))&MCG_C4_FCTRIM_MASK)
#define MCG_C4_DRST_DRS_MASK                     0x60u
#define MCG_C4_DRST_DRS_SHIFT                    5
#define MCG_C4_DRST_DRS_WIDTH                    2
#define MCG_C4_DRST_DRS(x)                       (((uint8_t)(((uint8_t)(x))<<MCG_C4_DRST_DRS_SHIFT))&MCG_C4_DRST_DRS_MASK)
#define MCG_C4_DMX32		((uint8_t)0x80u)
/* C5 Bit Fields */
#define MCG_C5_PRDIV0_MASK                       0x1Fu
#define MCG_C5_PRDIV0_SHIFT                      0
#define MCG_C5_PRDIV0_WIDTH                      5
#define MCG_C5_PRDIV0(x)                         (((uint8_t)(((uint8_t)(x))<<MCG_C5_PRDIV0_SHIFT))&MCG_C5_PRDIV0_MASK)
#define MCG_C5_PLLSTEN0		((uint8_t)0x20u)
#define MCG_C5_PLLCLKEN0		((uint8_t)0x40u)
/* C6 Bit Fields */
#define MCG_C6_VDIV0_MASK                        0x1Fu
#define MCG_C6_VDIV0_SHIFT                       0
#define MCG_C6_VDIV0_WIDTH                       5
#define MCG_C6_VDIV0(x)                          (((uint8_t)(((uint8_t)(x))<<MCG_C6_VDIV0_SHIFT))&MCG_C6_VDIV0_MASK)
#define MCG_C6_CME0		((uint8_t)0x20u)
#define MCG_C6_PLLS		((uint8_t)0x40u)
#define MCG_C6_LOLIE0		((uint8_t)0x80u)
/* S Bit Fields */
#define MCG_S_IRCST		((uint8_t)0x1u)
#define MCG_S_OSCINIT0		((uint8_t)0x2u)
#define MCG_S_CLKST_MASK                         0xCu
#define MCG_S_CLKST_SHIFT                        2
#define MCG_S_CLKST_WIDTH                        2
#define MCG_S_CLKST(x)                           (((uint8_t)(((uint8_t)(x))<<MCG_S_CLKST_SHIFT))&MCG_S_CLKST_MASK)
#define MCG_S_CLKST_FLL             MCG_S_CLKST(0)   /*!< Output of the FLL is selected */
#define MCG_S_CLKST_IRCLK           MCG_S_CLKST(1)   /*!< Internal reference clock is selected */
#define MCG_S_CLKST_ERCLK           MCG_S_CLKST(2)   /*!< External reference clock is selected */
#define MCG_S_CLKST_PLL             MCG_S_CLKST(3)   /*!< Output of the PLL is selected */
#define MCG_S_IREFST		((uint8_t)0x10u)
#define MCG_S_PLLST		((uint8_t)0x20u)
#define MCG_S_LOCK0		((uint8_t)0x40u)
#define MCG_S_LOLS0		((uint8_t)0x80u)
/* SC Bit Fields */
#define MCG_SC_LOCS0		((uint8_t)0x1u)
#define MCG_SC_FCRDIV_MASK                       0xEu
#define MCG_SC_FCRDIV_SHIFT                      1
#define MCG_SC_FCRDIV_WIDTH                      3
#define MCG_SC_FCRDIV(x)                         (((uint8_t)(((uint8_t)(x))<<MCG_SC_FCRDIV_SHIFT))&MCG_SC_FCRDIV_MASK)
#define MCG_SC_FLTPRSRV		((uint8_t)0x10u)
#define MCG_SC_ATMF		((uint8_t)0x20u)
#define MCG_SC_ATMS		((uint8_t)0x40u)
#define MCG_SC_ATME		((uint8_t)0x80u)
/* ATCVH Bit Fields */
#define MCG_ATCVH_ATCVH_MASK                     0xFFu
#define MCG_ATCVH_ATCVH_SHIFT                    0
#define MCG_ATCVH_ATCVH_WIDTH                    8
#define MCG_ATCVH_ATCVH(x)                       (((uint8_t)(((uint8_t)(x))<<MCG_ATCVH_ATCVH_SHIFT))&MCG_ATCVH_ATCVH_MASK)
/* ATCVL Bit Fields */
#define MCG_ATCVL_ATCVL_MASK                     0xFFu
#define MCG_ATCVL_ATCVL_SHIFT                    0
#define MCG_ATCVL_ATCVL_WIDTH                    8
#define MCG_ATCVL_ATCVL(x)                       (((uint8_t)(((uint8_t)(x))<<MCG_ATCVL_ATCVL_SHIFT))&MCG_ATCVL_ATCVL_MASK)
/* C7 Bit Fields */
#define MCG_C7_OSCSEL_MASK                       0x3u
#define MCG_C7_OSCSEL_SHIFT                      0
#define MCG_C7_OSCSEL_WIDTH                      2
#define MCG_C7_OSCSEL(x)                         (((uint8_t)(((uint8_t)(x))<<MCG_C7_OSCSEL_SHIFT))&MCG_C7_OSCSEL_MASK)
/* C8 Bit Fields */
#define MCG_C8_LOCS1		((uint8_t)0x1u)
#define MCG_C8_CME1		((uint8_t)0x20u)
#define MCG_C8_LOLRE		((uint8_t)0x40u)
#define MCG_C8_LOCRE1		((uint8_t)0x80u)

/*!
 * @}
 */ /* end of group MCG_Register_Masks */


/* MCG - Peripheral instance base addresses */
/** Peripheral MCG base address */
#define MCG_BASE                                 (0x40064000u)
/** Peripheral MCG base pointer */
#define MCG                                      ((MCG_TypeDef *)MCG_BASE)
#define MCG_BASE_PTR                             (MCG)
/** Array initializer of MCG peripheral base addresses */
#define MCG_BASE_ADDRS                           { MCG_BASE }
/** Array initializer of MCG peripheral base pointers */
#define MCG_BASE_PTRS                            { MCG }

/* ----------------------------------------------------------------------------
   -- MCG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCG_Register_Accessor_Macros MCG - Register accessor macros
 * @{
 */


/* MCG - Register instance definitions */
/* MCG */
#define MCG_C1                                   MCG_C1_REG(MCG)
#define MCG_C2                                   MCG_C2_REG(MCG)
#define MCG_C3                                   MCG_C3_REG(MCG)
#define MCG_C4                                   MCG_C4_REG(MCG)
#define MCG_C5                                   MCG_C5_REG(MCG)
#define MCG_C6                                   MCG_C6_REG(MCG)
#define MCG_S                                    MCG_S_REG(MCG)
#define MCG_SC                                   MCG_SC_REG(MCG)
#define MCG_ATCVH                                MCG_ATCVH_REG(MCG)
#define MCG_ATCVL                                MCG_ATCVL_REG(MCG)
#define MCG_C7                                   MCG_C7_REG(MCG)
#define MCG_C8                                   MCG_C8_REG(MCG)

/*!
 * @}
 */ /* end of group MCG_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group MCG_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- MCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Peripheral_Access_Layer MCM Peripheral Access Layer
 * @{
 */

/** MCM - Register Layout Typedef */
typedef struct {
       uint8_t RESERVED_0[8];
  __I  uint16_t PLASC;                             /**< Crossbar Switch (AXBS) Slave Configuration, offset: 0x8 */
  __I  uint16_t PLAMC;                             /**< Crossbar Switch (AXBS) Master Configuration, offset: 0xA */
  __IO uint32_t PLACR;                             /**< Crossbar Switch (AXBS) Control Register, offset: 0xC */
  __IO uint32_t ISCR;                              /**< Interrupt Status and Control Register, offset: 0x10 */
       uint8_t RESERVED_1[44];
  __IO uint32_t CPO;                               /**< Compute Operation Control Register, offset: 0x40 */
} MCM_TypeDef, *MCM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- MCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Accessor_Macros MCM - Register accessor macros
 * @{
 */


/* MCM - Register accessors */
#define MCM_PLASC_REG(base)                      ((base)->PLASC)
#define MCM_PLAMC_REG(base)                      ((base)->PLAMC)
#define MCM_PLACR_REG(base)                      ((base)->PLACR)
#define MCM_ISCR_REG(base)                       ((base)->ISCR)
#define MCM_CPO_REG(base)                        ((base)->CPO)

/*!
 * @}
 */ /* end of group MCM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- MCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Masks MCM Register Masks
 * @{
 */

/* PLASC Bit Fields */
#define MCM_PLASC_ASC_MASK                       0xFFu
#define MCM_PLASC_ASC_SHIFT                      0
#define MCM_PLASC_ASC_WIDTH                      8
#define MCM_PLASC_ASC(x)                         (((uint16_t)(((uint16_t)(x))<<MCM_PLASC_ASC_SHIFT))&MCM_PLASC_ASC_MASK)
/* PLAMC Bit Fields */
#define MCM_PLAMC_AMC_MASK                       0xFFu
#define MCM_PLAMC_AMC_SHIFT                      0
#define MCM_PLAMC_AMC_WIDTH                      8
#define MCM_PLAMC_AMC(x)                         (((uint16_t)(((uint16_t)(x))<<MCM_PLAMC_AMC_SHIFT))&MCM_PLAMC_AMC_MASK)
/* PLACR Bit Fields */
#define MCM_PLACR_ARB		((uint32_t)0x200u)
/* ISCR Bit Fields */
#define MCM_ISCR_FIOC		((uint32_t)0x100u)
#define MCM_ISCR_FDZC		((uint32_t)0x200u)
#define MCM_ISCR_FOFC		((uint32_t)0x400u)
#define MCM_ISCR_FUFC		((uint32_t)0x800u)
#define MCM_ISCR_FIXC		((uint32_t)0x1000u)
#define MCM_ISCR_FIDC		((uint32_t)0x8000u)
#define MCM_ISCR_FIOCE		((uint32_t)0x1000000u)
#define MCM_ISCR_FDZCE		((uint32_t)0x2000000u)
#define MCM_ISCR_FOFCE		((uint32_t)0x4000000u)
#define MCM_ISCR_FUFCE		((uint32_t)0x8000000u)
#define MCM_ISCR_FIXCE		((uint32_t)0x10000000u)
#define MCM_ISCR_FIDCE		((uint32_t)0x80000000u)
/* CPO Bit Fields */
#define MCM_CPO_CPOREQ		((uint32_t)0x1u)
#define MCM_CPO_CPOACK		((uint32_t)0x2u)
#define MCM_CPO_CPOWOI		((uint32_t)0x4u)

/*!
 * @}
 */ /* end of group MCM_Register_Masks */


/* MCM - Peripheral instance base addresses */
/** Peripheral MCM base address */
#define MCM_BASE                                 (0xE0080000u)
/** Peripheral MCM base pointer */
#define MCM                                      ((MCM_TypeDef *)MCM_BASE)
#define MCM_BASE_PTR                             (MCM)
/** Array initializer of MCM peripheral base addresses */
#define MCM_BASE_ADDRS                           { MCM_BASE }
/** Array initializer of MCM peripheral base pointers */
#define MCM_BASE_PTRS                            { MCM }
/** Interrupt vectors for the MCM peripheral type */
#define MCM_IRQS                                 { MCM_IRQn }

/* ----------------------------------------------------------------------------
   -- MCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MCM_Register_Accessor_Macros MCM - Register accessor macros
 * @{
 */


/* MCM - Register instance definitions */
/* MCM */
#define MCM_PLASC                                MCM_PLASC_REG(MCM)
#define MCM_PLAMC                                MCM_PLAMC_REG(MCM)
#define MCM_PLACR                                MCM_PLACR_REG(MCM)
#define MCM_ISCR                                 MCM_ISCR_REG(MCM)
#define MCM_CPO                                  MCM_CPO_REG(MCM)

/*!
 * @}
 */ /* end of group MCM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group MCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- NV Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Peripheral_Access_Layer NV Peripheral Access Layer
 * @{
 */

/** NV - Register Layout Typedef */
typedef struct {
  __I  uint8_t BACKKEY3;                           /**< Backdoor Comparison Key 3., offset: 0x0 */
  __I  uint8_t BACKKEY2;                           /**< Backdoor Comparison Key 2., offset: 0x1 */
  __I  uint8_t BACKKEY1;                           /**< Backdoor Comparison Key 1., offset: 0x2 */
  __I  uint8_t BACKKEY0;                           /**< Backdoor Comparison Key 0., offset: 0x3 */
  __I  uint8_t BACKKEY7;                           /**< Backdoor Comparison Key 7., offset: 0x4 */
  __I  uint8_t BACKKEY6;                           /**< Backdoor Comparison Key 6., offset: 0x5 */
  __I  uint8_t BACKKEY5;                           /**< Backdoor Comparison Key 5., offset: 0x6 */
  __I  uint8_t BACKKEY4;                           /**< Backdoor Comparison Key 4., offset: 0x7 */
  __I  uint8_t FPROT3;                             /**< Non-volatile P-Flash Protection 1 - Low Register, offset: 0x8 */
  __I  uint8_t FPROT2;                             /**< Non-volatile P-Flash Protection 1 - High Register, offset: 0x9 */
  __I  uint8_t FPROT1;                             /**< Non-volatile P-Flash Protection 0 - Low Register, offset: 0xA */
  __I  uint8_t FPROT0;                             /**< Non-volatile P-Flash Protection 0 - High Register, offset: 0xB */
  __I  uint8_t FSEC;                               /**< Non-volatile Flash Security Register, offset: 0xC */
  __I  uint8_t FOPT;                               /**< Non-volatile Flash Option Register, offset: 0xD */
} NV_TypeDef, *NV_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- NV - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Register_Accessor_Macros NV - Register accessor macros
 * @{
 */


/* NV - Register accessors */
#define NV_BACKKEY3_REG(base)                    ((base)->BACKKEY3)
#define NV_BACKKEY2_REG(base)                    ((base)->BACKKEY2)
#define NV_BACKKEY1_REG(base)                    ((base)->BACKKEY1)
#define NV_BACKKEY0_REG(base)                    ((base)->BACKKEY0)
#define NV_BACKKEY7_REG(base)                    ((base)->BACKKEY7)
#define NV_BACKKEY6_REG(base)                    ((base)->BACKKEY6)
#define NV_BACKKEY5_REG(base)                    ((base)->BACKKEY5)
#define NV_BACKKEY4_REG(base)                    ((base)->BACKKEY4)
#define NV_FPROT3_REG(base)                      ((base)->FPROT3)
#define NV_FPROT2_REG(base)                      ((base)->FPROT2)
#define NV_FPROT1_REG(base)                      ((base)->FPROT1)
#define NV_FPROT0_REG(base)                      ((base)->FPROT0)
#define NV_FSEC_REG(base)                        ((base)->FSEC)
#define NV_FOPT_REG(base)                        ((base)->FOPT)

/*!
 * @}
 */ /* end of group NV_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- NV Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Register_Masks NV Register Masks
 * @{
 */

/* BACKKEY3 Bit Fields */
#define NV_BACKKEY3_KEY_MASK                     0xFFu
#define NV_BACKKEY3_KEY_SHIFT                    0
#define NV_BACKKEY3_KEY_WIDTH                    8
#define NV_BACKKEY3_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY3_KEY_SHIFT))&NV_BACKKEY3_KEY_MASK)
/* BACKKEY2 Bit Fields */
#define NV_BACKKEY2_KEY_MASK                     0xFFu
#define NV_BACKKEY2_KEY_SHIFT                    0
#define NV_BACKKEY2_KEY_WIDTH                    8
#define NV_BACKKEY2_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY2_KEY_SHIFT))&NV_BACKKEY2_KEY_MASK)
/* BACKKEY1 Bit Fields */
#define NV_BACKKEY1_KEY_MASK                     0xFFu
#define NV_BACKKEY1_KEY_SHIFT                    0
#define NV_BACKKEY1_KEY_WIDTH                    8
#define NV_BACKKEY1_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY1_KEY_SHIFT))&NV_BACKKEY1_KEY_MASK)
/* BACKKEY0 Bit Fields */
#define NV_BACKKEY0_KEY_MASK                     0xFFu
#define NV_BACKKEY0_KEY_SHIFT                    0
#define NV_BACKKEY0_KEY_WIDTH                    8
#define NV_BACKKEY0_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY0_KEY_SHIFT))&NV_BACKKEY0_KEY_MASK)
/* BACKKEY7 Bit Fields */
#define NV_BACKKEY7_KEY_MASK                     0xFFu
#define NV_BACKKEY7_KEY_SHIFT                    0
#define NV_BACKKEY7_KEY_WIDTH                    8
#define NV_BACKKEY7_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY7_KEY_SHIFT))&NV_BACKKEY7_KEY_MASK)
/* BACKKEY6 Bit Fields */
#define NV_BACKKEY6_KEY_MASK                     0xFFu
#define NV_BACKKEY6_KEY_SHIFT                    0
#define NV_BACKKEY6_KEY_WIDTH                    8
#define NV_BACKKEY6_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY6_KEY_SHIFT))&NV_BACKKEY6_KEY_MASK)
/* BACKKEY5 Bit Fields */
#define NV_BACKKEY5_KEY_MASK                     0xFFu
#define NV_BACKKEY5_KEY_SHIFT                    0
#define NV_BACKKEY5_KEY_WIDTH                    8
#define NV_BACKKEY5_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY5_KEY_SHIFT))&NV_BACKKEY5_KEY_MASK)
/* BACKKEY4 Bit Fields */
#define NV_BACKKEY4_KEY_MASK                     0xFFu
#define NV_BACKKEY4_KEY_SHIFT                    0
#define NV_BACKKEY4_KEY_WIDTH                    8
#define NV_BACKKEY4_KEY(x)                       (((uint8_t)(((uint8_t)(x))<<NV_BACKKEY4_KEY_SHIFT))&NV_BACKKEY4_KEY_MASK)
/* FPROT3 Bit Fields */
#define NV_FPROT3_PROT_MASK                      0xFFu
#define NV_FPROT3_PROT_SHIFT                     0
#define NV_FPROT3_PROT_WIDTH                     8
#define NV_FPROT3_PROT(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FPROT3_PROT_SHIFT))&NV_FPROT3_PROT_MASK)
/* FPROT2 Bit Fields */
#define NV_FPROT2_PROT_MASK                      0xFFu
#define NV_FPROT2_PROT_SHIFT                     0
#define NV_FPROT2_PROT_WIDTH                     8
#define NV_FPROT2_PROT(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FPROT2_PROT_SHIFT))&NV_FPROT2_PROT_MASK)
/* FPROT1 Bit Fields */
#define NV_FPROT1_PROT_MASK                      0xFFu
#define NV_FPROT1_PROT_SHIFT                     0
#define NV_FPROT1_PROT_WIDTH                     8
#define NV_FPROT1_PROT(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FPROT1_PROT_SHIFT))&NV_FPROT1_PROT_MASK)
/* FPROT0 Bit Fields */
#define NV_FPROT0_PROT_MASK                      0xFFu
#define NV_FPROT0_PROT_SHIFT                     0
#define NV_FPROT0_PROT_WIDTH                     8
#define NV_FPROT0_PROT(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FPROT0_PROT_SHIFT))&NV_FPROT0_PROT_MASK)
/* FSEC Bit Fields */
#define NV_FSEC_SEC_MASK                         0x3u
#define NV_FSEC_SEC_SHIFT                        0
#define NV_FSEC_SEC_WIDTH                        2
#define NV_FSEC_SEC(x)                           (((uint8_t)(((uint8_t)(x))<<NV_FSEC_SEC_SHIFT))&NV_FSEC_SEC_MASK)
#define NV_FSEC_FSLACC_MASK                      0xCu
#define NV_FSEC_FSLACC_SHIFT                     2
#define NV_FSEC_FSLACC_WIDTH                     2
#define NV_FSEC_FSLACC(x)                        (((uint8_t)(((uint8_t)(x))<<NV_FSEC_FSLACC_SHIFT))&NV_FSEC_FSLACC_MASK)
#define NV_FSEC_MEEN_MASK                        0x30u
#define NV_FSEC_MEEN_SHIFT                       4
#define NV_FSEC_MEEN_WIDTH                       2
#define NV_FSEC_MEEN(x)                          (((uint8_t)(((uint8_t)(x))<<NV_FSEC_MEEN_SHIFT))&NV_FSEC_MEEN_MASK)
#define NV_FSEC_KEYEN_MASK                       0xC0u
#define NV_FSEC_KEYEN_SHIFT                      6
#define NV_FSEC_KEYEN_WIDTH                      2
#define NV_FSEC_KEYEN(x)                         (((uint8_t)(((uint8_t)(x))<<NV_FSEC_KEYEN_SHIFT))&NV_FSEC_KEYEN_MASK)
/* FOPT Bit Fields */
#define NV_FOPT_LPBOOT		((uint8_t)0x1u)
#define NV_FOPT_EZPORT_DIS		((uint8_t)0x2u)
#define NV_FOPT_NMI_DIS		((uint8_t)0x4u)
#define NV_FOPT_FAST_INIT		((uint8_t)0x20u)

/*!
 * @}
 */ /* end of group NV_Register_Masks */


/* NV - Peripheral instance base addresses */
/** Peripheral FTFA_FlashConfig base address */
#define FTFA_FlashConfig_BASE                    (0x400u)
/** Peripheral FTFA_FlashConfig base pointer */
#define FTFA_FlashConfig                         ((NV_TypeDef *)FTFA_FlashConfig_BASE)
#define FTFA_FlashConfig_BASE_PTR                (FTFA_FlashConfig)
/** Array initializer of NV peripheral base addresses */
#define NV_BASE_ADDRS                            { FTFA_FlashConfig_BASE }
/** Array initializer of NV peripheral base pointers */
#define NV_BASE_PTRS                             { FTFA_FlashConfig }

/* ----------------------------------------------------------------------------
   -- NV - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup NV_Register_Accessor_Macros NV - Register accessor macros
 * @{
 */


/* NV - Register instance definitions */
/* FTFA_FlashConfig */
#define NV_BACKKEY3                              NV_BACKKEY3_REG(FTFA_FlashConfig)
#define NV_BACKKEY2                              NV_BACKKEY2_REG(FTFA_FlashConfig)
#define NV_BACKKEY1                              NV_BACKKEY1_REG(FTFA_FlashConfig)
#define NV_BACKKEY0                              NV_BACKKEY0_REG(FTFA_FlashConfig)
#define NV_BACKKEY7                              NV_BACKKEY7_REG(FTFA_FlashConfig)
#define NV_BACKKEY6                              NV_BACKKEY6_REG(FTFA_FlashConfig)
#define NV_BACKKEY5                              NV_BACKKEY5_REG(FTFA_FlashConfig)
#define NV_BACKKEY4                              NV_BACKKEY4_REG(FTFA_FlashConfig)
#define NV_FPROT3                                NV_FPROT3_REG(FTFA_FlashConfig)
#define NV_FPROT2                                NV_FPROT2_REG(FTFA_FlashConfig)
#define NV_FPROT1                                NV_FPROT1_REG(FTFA_FlashConfig)
#define NV_FPROT0                                NV_FPROT0_REG(FTFA_FlashConfig)
#define NV_FSEC                                  NV_FSEC_REG(FTFA_FlashConfig)
#define NV_FOPT                                  NV_FOPT_REG(FTFA_FlashConfig)

/*!
 * @}
 */ /* end of group NV_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group NV_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- OSC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Peripheral_Access_Layer OSC Peripheral Access Layer
 * @{
 */

/** OSC - Register Layout Typedef */
typedef struct {
  __IO uint8_t CR;                                 /**< OSC Control Register, offset: 0x0 */
       uint8_t RESERVED_0[1];
  __IO uint8_t DIV;                                /**< OSC_DIV, offset: 0x2 */
} OSC_TypeDef, *OSC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- OSC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Register_Accessor_Macros OSC - Register accessor macros
 * @{
 */


/* OSC - Register accessors */
#define OSC_CR_REG(base)                         ((base)->CR)
#define OSC_DIV_REG(base)                        ((base)->DIV)

/*!
 * @}
 */ /* end of group OSC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- OSC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Register_Masks OSC Register Masks
 * @{
 */

/* CR Bit Fields */
#define OSC_CR_SC16P		((uint8_t)0x1u)
#define OSC_CR_SC8P		((uint8_t)0x2u)
#define OSC_CR_SC4P		((uint8_t)0x4u)
#define OSC_CR_SC2P		((uint8_t)0x8u)
#define OSC_CR_EREFSTEN		((uint8_t)0x20u)
#define OSC_CR_ERCLKEN		((uint8_t)0x80u)
/* DIV Bit Fields */
#define OSC_DIV_ERPS_MASK                        0xC0u
#define OSC_DIV_ERPS_SHIFT                       6
#define OSC_DIV_ERPS_WIDTH                       2
#define OSC_DIV_ERPS(x)                          (((uint8_t)(((uint8_t)(x))<<OSC_DIV_ERPS_SHIFT))&OSC_DIV_ERPS_MASK)

/*!
 * @}
 */ /* end of group OSC_Register_Masks */


/* OSC - Peripheral instance base addresses */
/** Peripheral OSC base address */
#define OSC_BASE                                 (0x40065000u)
/** Peripheral OSC base pointer */
#define OSC                                      ((OSC_TypeDef *)OSC_BASE)
#define OSC_BASE_PTR                             (OSC)
/** Array initializer of OSC peripheral base addresses */
#define OSC_BASE_ADDRS                           { OSC_BASE }
/** Array initializer of OSC peripheral base pointers */
#define OSC_BASE_PTRS                            { OSC }

/* ----------------------------------------------------------------------------
   -- OSC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup OSC_Register_Accessor_Macros OSC - Register accessor macros
 * @{
 */


/* OSC - Register instance definitions */
/* OSC */
#define OSC_CR                                   OSC_CR_REG(OSC)
#define OSC_DIV                                  OSC_DIV_REG(OSC)

/*!
 * @}
 */ /* end of group OSC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group OSC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PDB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Peripheral_Access_Layer PDB Peripheral Access Layer
 * @{
 */

/** PDB - Register Layout Typedef */
typedef struct {
  __IO uint32_t SC;                                /**< Status and Control register, offset: 0x0 */
  __IO uint32_t MOD;                               /**< Modulus register, offset: 0x4 */
  __I  uint32_t CNT;                               /**< Counter register, offset: 0x8 */
  __IO uint32_t IDLY;                              /**< Interrupt Delay register, offset: 0xC */
  struct {                                         /* offset: 0x10, array step: 0x28 */
    __IO uint32_t C1;                                /**< Channel n Control register 1, array offset: 0x10, array step: 0x28 */
    __IO uint32_t S;                                 /**< Channel n Status register, array offset: 0x14, array step: 0x28 */
    __IO uint32_t DLY[2];                            /**< Channel n Delay 0 register..Channel n Delay 1 register, array offset: 0x18, array step: index*0x28, index2*0x4 */
         uint8_t RESERVED_0[24];
  } CH[2];
       uint8_t RESERVED_0[240];
  struct {                                         /* offset: 0x150, array step: 0x8 */
    __IO uint32_t INTC;                              /**< DAC Interval Trigger n Control register, array offset: 0x150, array step: 0x8 */
    __IO uint32_t INT;                               /**< DAC Interval n register, array offset: 0x154, array step: 0x8 */
  } DAC[2];
       uint8_t RESERVED_1[48];
  __IO uint32_t POEN;                              /**< Pulse-Out n Enable register, offset: 0x190 */
  __IO uint32_t PODLY[2];                          /**< Pulse-Out n Delay register, array offset: 0x194, array step: 0x4 */
} PDB_TypeDef, *PDB_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PDB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Accessor_Macros PDB - Register accessor macros
 * @{
 */


/* PDB - Register accessors */
#define PDB_SC_REG(base)                         ((base)->SC)
#define PDB_MOD_REG(base)                        ((base)->MOD)
#define PDB_CNT_REG(base)                        ((base)->CNT)
#define PDB_IDLY_REG(base)                       ((base)->IDLY)
#define PDB_C1_REG(base,index)                   ((base)->CH[index].C1)
#define PDB_C1_COUNT                             2
#define PDB_S_REG(base,index)                    ((base)->CH[index].S)
#define PDB_S_COUNT                              2
#define PDB_DLY_REG(base,index,index2)           ((base)->CH[index].DLY[index2])
#define PDB_DLY_COUNT                            2
#define PDB_DLY_COUNT2                           2
#define PDB_INTC_REG(base,index)                 ((base)->DAC[index].INTC)
#define PDB_INTC_COUNT                           2
#define PDB_INT_REG(base,index)                  ((base)->DAC[index].INT)
#define PDB_INT_COUNT                            2
#define PDB_POEN_REG(base)                       ((base)->POEN)
#define PDB_PODLY_REG(base,index)                ((base)->PODLY[index])
#define PDB_PODLY_COUNT                          2

/*!
 * @}
 */ /* end of group PDB_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PDB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Masks PDB Register Masks
 * @{
 */

/* SC Bit Fields */
#define PDB_SC_LDOK		((uint32_t)0x1u)
#define PDB_SC_CONT		((uint32_t)0x2u)
#define PDB_SC_MULT_MASK                         0xCu
#define PDB_SC_MULT_SHIFT                        2
#define PDB_SC_MULT_WIDTH                        2
#define PDB_SC_MULT(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_SC_MULT_SHIFT))&PDB_SC_MULT_MASK)
#define PDB_SC_PDBIE		((uint32_t)0x20u)
#define PDB_SC_PDBIF		((uint32_t)0x40u)
#define PDB_SC_PDBEN		((uint32_t)0x80u)
#define PDB_SC_TRGSEL_MASK                       0xF00u
#define PDB_SC_TRGSEL_SHIFT                      8
#define PDB_SC_TRGSEL_WIDTH                      4
#define PDB_SC_TRGSEL(x)                         (((uint32_t)(((uint32_t)(x))<<PDB_SC_TRGSEL_SHIFT))&PDB_SC_TRGSEL_MASK)
#define PDB_SC_PRESCALER_MASK                    0x7000u
#define PDB_SC_PRESCALER_SHIFT                   12
#define PDB_SC_PRESCALER_WIDTH                   3
#define PDB_SC_PRESCALER(x)                      (((uint32_t)(((uint32_t)(x))<<PDB_SC_PRESCALER_SHIFT))&PDB_SC_PRESCALER_MASK)
#define PDB_SC_DMAEN		((uint32_t)0x8000u)
#define PDB_SC_SWTRIG		((uint32_t)0x10000u)
#define PDB_SC_PDBEIE		((uint32_t)0x20000u)
#define PDB_SC_LDMOD_MASK                        0xC0000u
#define PDB_SC_LDMOD_SHIFT                       18
#define PDB_SC_LDMOD_WIDTH                       2
#define PDB_SC_LDMOD(x)                          (((uint32_t)(((uint32_t)(x))<<PDB_SC_LDMOD_SHIFT))&PDB_SC_LDMOD_MASK)
/* MOD Bit Fields */
#define PDB_MOD_MOD_MASK                         0xFFFFu
#define PDB_MOD_MOD_SHIFT                        0
#define PDB_MOD_MOD_WIDTH                        16
#define PDB_MOD_MOD(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_MOD_MOD_SHIFT))&PDB_MOD_MOD_MASK)
/* CNT Bit Fields */
#define PDB_CNT_CNT_MASK                         0xFFFFu
#define PDB_CNT_CNT_SHIFT                        0
#define PDB_CNT_CNT_WIDTH                        16
#define PDB_CNT_CNT(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_CNT_CNT_SHIFT))&PDB_CNT_CNT_MASK)
/* IDLY Bit Fields */
#define PDB_IDLY_IDLY_MASK                       0xFFFFu
#define PDB_IDLY_IDLY_SHIFT                      0
#define PDB_IDLY_IDLY_WIDTH                      16
#define PDB_IDLY_IDLY(x)                         (((uint32_t)(((uint32_t)(x))<<PDB_IDLY_IDLY_SHIFT))&PDB_IDLY_IDLY_MASK)
/* C1 Bit Fields */
#define PDB_C1_EN_MASK                           0xFFu
#define PDB_C1_EN_SHIFT                          0
#define PDB_C1_EN_WIDTH                          8
#define PDB_C1_EN(x)                             (((uint32_t)(((uint32_t)(x))<<PDB_C1_EN_SHIFT))&PDB_C1_EN_MASK)
#define PDB_C1_TOS_MASK                          0xFF00u
#define PDB_C1_TOS_SHIFT                         8
#define PDB_C1_TOS_WIDTH                         8
#define PDB_C1_TOS(x)                            (((uint32_t)(((uint32_t)(x))<<PDB_C1_TOS_SHIFT))&PDB_C1_TOS_MASK)
#define PDB_C1_BB_MASK                           0xFF0000u
#define PDB_C1_BB_SHIFT                          16
#define PDB_C1_BB_WIDTH                          8
#define PDB_C1_BB(x)                             (((uint32_t)(((uint32_t)(x))<<PDB_C1_BB_SHIFT))&PDB_C1_BB_MASK)
/* S Bit Fields */
#define PDB_S_ERR_MASK                           0xFFu
#define PDB_S_ERR_SHIFT                          0
#define PDB_S_ERR_WIDTH                          8
#define PDB_S_ERR(x)                             (((uint32_t)(((uint32_t)(x))<<PDB_S_ERR_SHIFT))&PDB_S_ERR_MASK)
#define PDB_S_CF_MASK                            0xFF0000u
#define PDB_S_CF_SHIFT                           16
#define PDB_S_CF_WIDTH                           8
#define PDB_S_CF(x)                              (((uint32_t)(((uint32_t)(x))<<PDB_S_CF_SHIFT))&PDB_S_CF_MASK)
/* DLY Bit Fields */
#define PDB_DLY_DLY_MASK                         0xFFFFu
#define PDB_DLY_DLY_SHIFT                        0
#define PDB_DLY_DLY_WIDTH                        16
#define PDB_DLY_DLY(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_DLY_DLY_SHIFT))&PDB_DLY_DLY_MASK)
/* INTC Bit Fields */
#define PDB_INTC_TOE		((uint32_t)0x1u)
#define PDB_INTC_EXT		((uint32_t)0x2u)
/* INT Bit Fields */
#define PDB_INT_INT_MASK                         0xFFFFu
#define PDB_INT_INT_SHIFT                        0
#define PDB_INT_INT_WIDTH                        16
#define PDB_INT_INT(x)                           (((uint32_t)(((uint32_t)(x))<<PDB_INT_INT_SHIFT))&PDB_INT_INT_MASK)
/* POEN Bit Fields */
#define PDB_POEN_POEN_MASK                       0xFFu
#define PDB_POEN_POEN_SHIFT                      0
#define PDB_POEN_POEN_WIDTH                      8
#define PDB_POEN_POEN(x)                         (((uint32_t)(((uint32_t)(x))<<PDB_POEN_POEN_SHIFT))&PDB_POEN_POEN_MASK)
/* PODLY Bit Fields */
#define PDB_PODLY_DLY2_MASK                      0xFFFFu
#define PDB_PODLY_DLY2_SHIFT                     0
#define PDB_PODLY_DLY2_WIDTH                     16
#define PDB_PODLY_DLY2(x)                        (((uint32_t)(((uint32_t)(x))<<PDB_PODLY_DLY2_SHIFT))&PDB_PODLY_DLY2_MASK)
#define PDB_PODLY_DLY1_MASK                      0xFFFF0000u
#define PDB_PODLY_DLY1_SHIFT                     16
#define PDB_PODLY_DLY1_WIDTH                     16
#define PDB_PODLY_DLY1(x)                        (((uint32_t)(((uint32_t)(x))<<PDB_PODLY_DLY1_SHIFT))&PDB_PODLY_DLY1_MASK)

/*!
 * @}
 */ /* end of group PDB_Register_Masks */


/* PDB - Peripheral instance base addresses */
/** Peripheral PDB0 base address */
#define PDB0_BASE                                (0x40036000u)
/** Peripheral PDB0 base pointer */
#define PDB0                                     ((PDB_TypeDef *)PDB0_BASE)
#define PDB0_BASE_PTR                            (PDB0)
/** Array initializer of PDB peripheral base addresses */
#define PDB_BASE_ADDRS                           { PDB0_BASE }
/** Array initializer of PDB peripheral base pointers */
#define PDB_BASE_PTRS                            { PDB0 }
/** Interrupt vectors for the PDB peripheral type */
#define PDB_IRQS                                 { PDB0_IRQn }

/* ----------------------------------------------------------------------------
   -- PDB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PDB_Register_Accessor_Macros PDB - Register accessor macros
 * @{
 */


/* PDB - Register instance definitions */
/* PDB0 */
#define PDB0_SC                                  PDB_SC_REG(PDB0)
#define PDB0_MOD                                 PDB_MOD_REG(PDB0)
#define PDB0_CNT                                 PDB_CNT_REG(PDB0)
#define PDB0_IDLY                                PDB_IDLY_REG(PDB0)
#define PDB0_CH0C1                               PDB_C1_REG(PDB0,0)
#define PDB0_CH0S                                PDB_S_REG(PDB0,0)
#define PDB0_CH0DLY0                             PDB_DLY_REG(PDB0,0,0)
#define PDB0_CH0DLY1                             PDB_DLY_REG(PDB0,0,1)
#define PDB0_CH1C1                               PDB_C1_REG(PDB0,1)
#define PDB0_CH1S                                PDB_S_REG(PDB0,1)
#define PDB0_CH1DLY0                             PDB_DLY_REG(PDB0,1,0)
#define PDB0_CH1DLY1                             PDB_DLY_REG(PDB0,1,1)
#define PDB0_DACINTC0                            PDB_INTC_REG(PDB0,0)
#define PDB0_DACINT0                             PDB_INT_REG(PDB0,0)
#define PDB0_DACINTC1                            PDB_INTC_REG(PDB0,1)
#define PDB0_DACINT1                             PDB_INT_REG(PDB0,1)
#define PDB0_POEN                                PDB_POEN_REG(PDB0)
#define PDB0_PO0DLY                              PDB_PODLY_REG(PDB0,0)
#define PDB0_PO1DLY                              PDB_PODLY_REG(PDB0,1)

/* PDB - Register array accessors */
#define PDB0_C1(index)                           PDB_C1_REG(PDB0,index)
#define PDB0_S(index)                            PDB_S_REG(PDB0,index)
#define PDB0_DLY(index,index2)                   PDB_DLY_REG(PDB0,index,index2)
#define PDB0_INTC(index)                         PDB_INTC_REG(PDB0,index)
#define PDB0_INT(index)                          PDB_INT_REG(PDB0,index)
#define PDB0_PODLY(index)                        PDB_PODLY_REG(PDB0,index)

/*!
 * @}
 */ /* end of group PDB_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PDB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PIT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Peripheral_Access_Layer PIT Peripheral Access Layer
 * @{
 */

/** PIT - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< PIT Module Control Register, offset: 0x0 */
       uint8_t RESERVED_0[252];
  struct {                                         /* offset: 0x100, array step: 0x10 */
    __IO uint32_t LDVAL;                             /**< Timer Load Value Register, array offset: 0x100, array step: 0x10 */
    __I  uint32_t CVAL;                              /**< Current Timer Value Register, array offset: 0x104, array step: 0x10 */
    __IO uint32_t TCTRL;                             /**< Timer Control Register, array offset: 0x108, array step: 0x10 */
    __IO uint32_t TFLG;                              /**< Timer Flag Register, array offset: 0x10C, array step: 0x10 */
  } CHANNEL[4];
} PIT_TypeDef, *PIT_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PIT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Accessor_Macros PIT - Register accessor macros
 * @{
 */


/* PIT - Register accessors */
#define PIT_MCR_REG(base)                        ((base)->MCR)
#define PIT_LDVAL_REG(base,index)                ((base)->CHANNEL[index].LDVAL)
#define PIT_LDVAL_COUNT                          4
#define PIT_CVAL_REG(base,index)                 ((base)->CHANNEL[index].CVAL)
#define PIT_CVAL_COUNT                           4
#define PIT_TCTRL_REG(base,index)                ((base)->CHANNEL[index].TCTRL)
#define PIT_TCTRL_COUNT                          4
#define PIT_TFLG_REG(base,index)                 ((base)->CHANNEL[index].TFLG)
#define PIT_TFLG_COUNT                           4

/*!
 * @}
 */ /* end of group PIT_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PIT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Masks PIT Register Masks
 * @{
 */

/* MCR Bit Fields */
#define PIT_MCR_FRZ		((uint32_t)0x1u)
#define PIT_MCR_MDIS		((uint32_t)0x2u)
/* LDVAL Bit Fields */
#define PIT_LDVAL_TSV_MASK                       0xFFFFFFFFu
#define PIT_LDVAL_TSV_SHIFT                      0
#define PIT_LDVAL_TSV_WIDTH                      32
#define PIT_LDVAL_TSV(x)                         (((uint32_t)(((uint32_t)(x))<<PIT_LDVAL_TSV_SHIFT))&PIT_LDVAL_TSV_MASK)
/* CVAL Bit Fields */
#define PIT_CVAL_TVL_MASK                        0xFFFFFFFFu
#define PIT_CVAL_TVL_SHIFT                       0
#define PIT_CVAL_TVL_WIDTH                       32
#define PIT_CVAL_TVL(x)                          (((uint32_t)(((uint32_t)(x))<<PIT_CVAL_TVL_SHIFT))&PIT_CVAL_TVL_MASK)
/* TCTRL Bit Fields */
#define PIT_TCTRL_TEN		((uint32_t)0x1u)
#define PIT_TCTRL_TIE		((uint32_t)0x2u)
#define PIT_TCTRL_CHN		((uint32_t)0x4u)
/* TFLG Bit Fields */
#define PIT_TFLG_TIF		((uint32_t)0x1u)

/*!
 * @}
 */ /* end of group PIT_Register_Masks */


/* PIT - Peripheral instance base addresses */
/** Peripheral PIT base address */
#define PIT_BASE                                 (0x40037000u)
/** Peripheral PIT base pointer */
#define PIT                                      ((PIT_TypeDef *)PIT_BASE)
#define PIT_BASE_PTR                             (PIT)
/** Array initializer of PIT peripheral base addresses */
#define PIT_BASE_ADDRS                           { PIT_BASE }
/** Array initializer of PIT peripheral base pointers */
#define PIT_BASE_PTRS                            { PIT }
/** Interrupt vectors for the PIT peripheral type */
#define PIT_IRQS                                 { PIT0_IRQn, PIT1_IRQn, PIT2_IRQn, PIT3_IRQn }

/* ----------------------------------------------------------------------------
   -- PIT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PIT_Register_Accessor_Macros PIT - Register accessor macros
 * @{
 */


/* PIT - Register instance definitions */
/* PIT */
#define PIT_MCR                                  PIT_MCR_REG(PIT)
#define PIT_LDVAL0                               PIT_LDVAL_REG(PIT,0)
#define PIT_CVAL0                                PIT_CVAL_REG(PIT,0)
#define PIT_TCTRL0                               PIT_TCTRL_REG(PIT,0)
#define PIT_TFLG0                                PIT_TFLG_REG(PIT,0)
#define PIT_LDVAL1                               PIT_LDVAL_REG(PIT,1)
#define PIT_CVAL1                                PIT_CVAL_REG(PIT,1)
#define PIT_TCTRL1                               PIT_TCTRL_REG(PIT,1)
#define PIT_TFLG1                                PIT_TFLG_REG(PIT,1)
#define PIT_LDVAL2                               PIT_LDVAL_REG(PIT,2)
#define PIT_CVAL2                                PIT_CVAL_REG(PIT,2)
#define PIT_TCTRL2                               PIT_TCTRL_REG(PIT,2)
#define PIT_TFLG2                                PIT_TFLG_REG(PIT,2)
#define PIT_LDVAL3                               PIT_LDVAL_REG(PIT,3)
#define PIT_CVAL3                                PIT_CVAL_REG(PIT,3)
#define PIT_TCTRL3                               PIT_TCTRL_REG(PIT,3)
#define PIT_TFLG3                                PIT_TFLG_REG(PIT,3)

/* PIT - Register array accessors */
#define PIT_LDVAL(index)                         PIT_LDVAL_REG(PIT,index)
#define PIT_CVAL(index)                          PIT_CVAL_REG(PIT,index)
#define PIT_TCTRL(index)                         PIT_TCTRL_REG(PIT,index)
#define PIT_TFLG(index)                          PIT_TFLG_REG(PIT,index)

/*!
 * @}
 */ /* end of group PIT_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PIT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Peripheral_Access_Layer PMC Peripheral Access Layer
 * @{
 */

/** PMC - Register Layout Typedef */
typedef struct {
  __IO uint8_t LVDSC1;                             /**< Low Voltage Detect Status And Control 1 register, offset: 0x0 */
  __IO uint8_t LVDSC2;                             /**< Low Voltage Detect Status And Control 2 register, offset: 0x1 */
  __IO uint8_t REGSC;                              /**< Regulator Status And Control register, offset: 0x2 */
} PMC_TypeDef, *PMC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Accessor_Macros PMC - Register accessor macros
 * @{
 */


/* PMC - Register accessors */
#define PMC_LVDSC1_REG(base)                     ((base)->LVDSC1)
#define PMC_LVDSC2_REG(base)                     ((base)->LVDSC2)
#define PMC_REGSC_REG(base)                      ((base)->REGSC)

/*!
 * @}
 */ /* end of group PMC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Masks PMC Register Masks
 * @{
 */

/* LVDSC1 Bit Fields */
#define PMC_LVDSC1_LVDV_MASK                     0x3u
#define PMC_LVDSC1_LVDV_SHIFT                    0
#define PMC_LVDSC1_LVDV_WIDTH                    2
#define PMC_LVDSC1_LVDV(x)                       (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC1_LVDV_SHIFT))&PMC_LVDSC1_LVDV_MASK)
#define PMC_LVDSC1_LVDRE		((uint8_t)0x10u)
#define PMC_LVDSC1_LVDIE		((uint8_t)0x20u)
#define PMC_LVDSC1_LVDACK		((uint8_t)0x40u)
#define PMC_LVDSC1_LVDF		((uint8_t)0x80u)
/* LVDSC2 Bit Fields */
#define PMC_LVDSC2_LVWV_MASK                     0x3u
#define PMC_LVDSC2_LVWV_SHIFT                    0
#define PMC_LVDSC2_LVWV_WIDTH                    2
#define PMC_LVDSC2_LVWV(x)                       (((uint8_t)(((uint8_t)(x))<<PMC_LVDSC2_LVWV_SHIFT))&PMC_LVDSC2_LVWV_MASK)
#define PMC_LVDSC2_LVWIE		((uint8_t)0x20u)
#define PMC_LVDSC2_LVWACK		((uint8_t)0x40u)
#define PMC_LVDSC2_LVWF		((uint8_t)0x80u)
/* REGSC Bit Fields */
#define PMC_REGSC_BGBE		((uint8_t)0x1u)
#define PMC_REGSC_REGONS		((uint8_t)0x4u)
#define PMC_REGSC_ACKISO		((uint8_t)0x8u)
#define PMC_REGSC_BGEN		((uint8_t)0x10u)

/*!
 * @}
 */ /* end of group PMC_Register_Masks */


/* PMC - Peripheral instance base addresses */
/** Peripheral PMC base address */
#define PMC_BASE                                 (0x4007D000u)
/** Peripheral PMC base pointer */
#define PMC                                      ((PMC_TypeDef *)PMC_BASE)
#define PMC_BASE_PTR                             (PMC)
/** Array initializer of PMC peripheral base addresses */
#define PMC_BASE_ADDRS                           { PMC_BASE }
/** Array initializer of PMC peripheral base pointers */
#define PMC_BASE_PTRS                            { PMC }
/** Interrupt vectors for the PMC peripheral type */
#define PMC_IRQS                                 { LVD_LVW_IRQn }

/* ----------------------------------------------------------------------------
   -- PMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PMC_Register_Accessor_Macros PMC - Register accessor macros
 * @{
 */


/* PMC - Register instance definitions */
/* PMC */
#define PMC_LVDSC1                               PMC_LVDSC1_REG(PMC)
#define PMC_LVDSC2                               PMC_LVDSC2_REG(PMC)
#define PMC_REGSC                                PMC_REGSC_REG(PMC)

/*!
 * @}
 */ /* end of group PMC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- PORT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Peripheral_Access_Layer PORT Peripheral Access Layer
 * @{
 */

/** PORT - Register Layout Typedef */
typedef struct {
  __IO uint32_t PCR[32];                           /**< Pin Control Register n, array offset: 0x0, array step: 0x4 */
  __O  uint32_t GPCLR;                             /**< Global Pin Control Low Register, offset: 0x80 */
  __O  uint32_t GPCHR;                             /**< Global Pin Control High Register, offset: 0x84 */
       uint8_t RESERVED_0[24];
  __IO uint32_t ISFR;                              /**< Interrupt Status Flag Register, offset: 0xA0 */
       uint8_t RESERVED_1[28];
  __IO uint32_t DFER;                              /**< Digital Filter Enable Register, offset: 0xC0 */
  __IO uint32_t DFCR;                              /**< Digital Filter Clock Register, offset: 0xC4 */
  __IO uint32_t DFWR;                              /**< Digital Filter Width Register, offset: 0xC8 */
} PORT_TypeDef, *PORT_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- PORT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Accessor_Macros PORT - Register accessor macros
 * @{
 */


/* PORT - Register accessors */
#define PORT_PCR_REG(base,index)                 ((base)->PCR[index])
#define PORT_PCR_COUNT                           32
#define PORT_GPCLR_REG(base)                     ((base)->GPCLR)
#define PORT_GPCHR_REG(base)                     ((base)->GPCHR)
#define PORT_ISFR_REG(base)                      ((base)->ISFR)
#define PORT_DFER_REG(base)                      ((base)->DFER)
#define PORT_DFCR_REG(base)                      ((base)->DFCR)
#define PORT_DFWR_REG(base)                      ((base)->DFWR)

/*!
 * @}
 */ /* end of group PORT_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- PORT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Masks PORT Register Masks
 * @{
 */

/* PCR Bit Fields */
#define PORT_PCR_PS		((uint32_t)0x1u)
#define PORT_PCR_PE		((uint32_t)0x2u)
#define PORT_PCR_SRE		((uint32_t)0x4u)
#define PORT_PCR_PFE		((uint32_t)0x10u)
#define PORT_PCR_ODE		((uint32_t)0x20u)
#define PORT_PCR_DSE		((uint32_t)0x40u)
#define PORT_PCR_MUX_MASK                        0x700u
#define PORT_PCR_MUX_SHIFT                       8
#define PORT_PCR_MUX_WIDTH                       3
#define PORT_PCR_MUX(x)                          (((uint32_t)(((uint32_t)(x))<<PORT_PCR_MUX_SHIFT))&PORT_PCR_MUX_MASK)
#define PORT_PCR_LK		((uint32_t)0x8000u)
#define PORT_PCR_IRQC_MASK                       0xF0000u
#define PORT_PCR_IRQC_SHIFT                      16
#define PORT_PCR_IRQC_WIDTH                      4
#define PORT_PCR_IRQC(x)                         (((uint32_t)(((uint32_t)(x))<<PORT_PCR_IRQC_SHIFT))&PORT_PCR_IRQC_MASK)
#define PORT_PCR_ISF		((uint32_t)0x1000000u)
/* GPCLR Bit Fields */
#define PORT_GPCLR_GPWD_MASK                     0xFFFFu
#define PORT_GPCLR_GPWD_SHIFT                    0
#define PORT_GPCLR_GPWD_WIDTH                    16
#define PORT_GPCLR_GPWD(x)                       (((uint32_t)(((uint32_t)(x))<<PORT_GPCLR_GPWD_SHIFT))&PORT_GPCLR_GPWD_MASK)
#define PORT_GPCLR_GPWE_MASK                     0xFFFF0000u
#define PORT_GPCLR_GPWE_SHIFT                    16
#define PORT_GPCLR_GPWE_WIDTH                    16
#define PORT_GPCLR_GPWE(x)                       (((uint32_t)(((uint32_t)(x))<<PORT_GPCLR_GPWE_SHIFT))&PORT_GPCLR_GPWE_MASK)
/* GPCHR Bit Fields */
#define PORT_GPCHR_GPWD_MASK                     0xFFFFu
#define PORT_GPCHR_GPWD_SHIFT                    0
#define PORT_GPCHR_GPWD_WIDTH                    16
#define PORT_GPCHR_GPWD(x)                       (((uint32_t)(((uint32_t)(x))<<PORT_GPCHR_GPWD_SHIFT))&PORT_GPCHR_GPWD_MASK)
#define PORT_GPCHR_GPWE_MASK                     0xFFFF0000u
#define PORT_GPCHR_GPWE_SHIFT                    16
#define PORT_GPCHR_GPWE_WIDTH                    16
#define PORT_GPCHR_GPWE(x)                       (((uint32_t)(((uint32_t)(x))<<PORT_GPCHR_GPWE_SHIFT))&PORT_GPCHR_GPWE_MASK)
/* ISFR Bit Fields */
#define PORT_ISFR_ISF_MASK                       0xFFFFFFFFu
#define PORT_ISFR_ISF_SHIFT                      0
#define PORT_ISFR_ISF_WIDTH                      32
#define PORT_ISFR_ISF(x)                         (((uint32_t)(((uint32_t)(x))<<PORT_ISFR_ISF_SHIFT))&PORT_ISFR_ISF_MASK)
/* DFER Bit Fields */
#define PORT_DFER_DFE_MASK                       0xFFFFFFFFu
#define PORT_DFER_DFE_SHIFT                      0
#define PORT_DFER_DFE_WIDTH                      32
#define PORT_DFER_DFE(x)                         (((uint32_t)(((uint32_t)(x))<<PORT_DFER_DFE_SHIFT))&PORT_DFER_DFE_MASK)
/* DFCR Bit Fields */
#define PORT_DFCR_CS		((uint32_t)0x1u)
/* DFWR Bit Fields */
#define PORT_DFWR_FILT_MASK                      0x1Fu
#define PORT_DFWR_FILT_SHIFT                     0
#define PORT_DFWR_FILT_WIDTH                     5
#define PORT_DFWR_FILT(x)                        (((uint32_t)(((uint32_t)(x))<<PORT_DFWR_FILT_SHIFT))&PORT_DFWR_FILT_MASK)

/*!
 * @}
 */ /* end of group PORT_Register_Masks */


/* PORT - Peripheral instance base addresses */
/** Peripheral PORTA base address */
#define PORTA_BASE                               (0x40049000u)
/** Peripheral PORTA base pointer */
#define PORTA                                    ((PORT_TypeDef *)PORTA_BASE)
#define PORTA_BASE_PTR                           (PORTA)
/** Peripheral PORTB base address */
#define PORTB_BASE                               (0x4004A000u)
/** Peripheral PORTB base pointer */
#define PORTB                                    ((PORT_TypeDef *)PORTB_BASE)
#define PORTB_BASE_PTR                           (PORTB)
/** Peripheral PORTC base address */
#define PORTC_BASE                               (0x4004B000u)
/** Peripheral PORTC base pointer */
#define PORTC                                    ((PORT_TypeDef *)PORTC_BASE)
#define PORTC_BASE_PTR                           (PORTC)
/** Peripheral PORTD base address */
#define PORTD_BASE                               (0x4004C000u)
/** Peripheral PORTD base pointer */
#define PORTD                                    ((PORT_TypeDef *)PORTD_BASE)
#define PORTD_BASE_PTR                           (PORTD)
/** Peripheral PORTE base address */
#define PORTE_BASE                               (0x4004D000u)
/** Peripheral PORTE base pointer */
#define PORTE                                    ((PORT_TypeDef *)PORTE_BASE)
#define PORTE_BASE_PTR                           (PORTE)
/** Array initializer of PORT peripheral base addresses */
#define PORT_BASE_ADDRS                          { PORTA_BASE, PORTB_BASE, PORTC_BASE, PORTD_BASE, PORTE_BASE }
/** Array initializer of PORT peripheral base pointers */
#define PORT_BASE_PTRS                           { PORTA, PORTB, PORTC, PORTD, PORTE }
/** Interrupt vectors for the PORT peripheral type */
#define PORT_IRQS                                { PORTA_IRQn, PORTB_IRQn, PORTC_IRQn, PORTD_IRQn, PORTE_IRQn }

/* ----------------------------------------------------------------------------
   -- PORT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup PORT_Register_Accessor_Macros PORT - Register accessor macros
 * @{
 */


/* PORT - Register instance definitions */
/* PORTA */
#define PORTA_PCR0                               PORT_PCR_REG(PORTA,0)
#define PORTA_PCR1                               PORT_PCR_REG(PORTA,1)
#define PORTA_PCR2                               PORT_PCR_REG(PORTA,2)
#define PORTA_PCR3                               PORT_PCR_REG(PORTA,3)
#define PORTA_PCR4                               PORT_PCR_REG(PORTA,4)
#define PORTA_PCR5                               PORT_PCR_REG(PORTA,5)
#define PORTA_PCR6                               PORT_PCR_REG(PORTA,6)
#define PORTA_PCR7                               PORT_PCR_REG(PORTA,7)
#define PORTA_PCR8                               PORT_PCR_REG(PORTA,8)
#define PORTA_PCR9                               PORT_PCR_REG(PORTA,9)
#define PORTA_PCR10                              PORT_PCR_REG(PORTA,10)
#define PORTA_PCR11                              PORT_PCR_REG(PORTA,11)
#define PORTA_PCR12                              PORT_PCR_REG(PORTA,12)
#define PORTA_PCR13                              PORT_PCR_REG(PORTA,13)
#define PORTA_PCR14                              PORT_PCR_REG(PORTA,14)
#define PORTA_PCR15                              PORT_PCR_REG(PORTA,15)
#define PORTA_PCR16                              PORT_PCR_REG(PORTA,16)
#define PORTA_PCR17                              PORT_PCR_REG(PORTA,17)
#define PORTA_PCR18                              PORT_PCR_REG(PORTA,18)
#define PORTA_PCR19                              PORT_PCR_REG(PORTA,19)
#define PORTA_PCR20                              PORT_PCR_REG(PORTA,20)
#define PORTA_PCR21                              PORT_PCR_REG(PORTA,21)
#define PORTA_PCR22                              PORT_PCR_REG(PORTA,22)
#define PORTA_PCR23                              PORT_PCR_REG(PORTA,23)
#define PORTA_PCR24                              PORT_PCR_REG(PORTA,24)
#define PORTA_PCR25                              PORT_PCR_REG(PORTA,25)
#define PORTA_PCR26                              PORT_PCR_REG(PORTA,26)
#define PORTA_PCR27                              PORT_PCR_REG(PORTA,27)
#define PORTA_PCR28                              PORT_PCR_REG(PORTA,28)
#define PORTA_PCR29                              PORT_PCR_REG(PORTA,29)
#define PORTA_PCR30                              PORT_PCR_REG(PORTA,30)
#define PORTA_PCR31                              PORT_PCR_REG(PORTA,31)
#define PORTA_GPCLR                              PORT_GPCLR_REG(PORTA)
#define PORTA_GPCHR                              PORT_GPCHR_REG(PORTA)
#define PORTA_ISFR                               PORT_ISFR_REG(PORTA)
/* PORTB */
#define PORTB_PCR0                               PORT_PCR_REG(PORTB,0)
#define PORTB_PCR1                               PORT_PCR_REG(PORTB,1)
#define PORTB_PCR2                               PORT_PCR_REG(PORTB,2)
#define PORTB_PCR3                               PORT_PCR_REG(PORTB,3)
#define PORTB_PCR4                               PORT_PCR_REG(PORTB,4)
#define PORTB_PCR5                               PORT_PCR_REG(PORTB,5)
#define PORTB_PCR6                               PORT_PCR_REG(PORTB,6)
#define PORTB_PCR7                               PORT_PCR_REG(PORTB,7)
#define PORTB_PCR8                               PORT_PCR_REG(PORTB,8)
#define PORTB_PCR9                               PORT_PCR_REG(PORTB,9)
#define PORTB_PCR10                              PORT_PCR_REG(PORTB,10)
#define PORTB_PCR11                              PORT_PCR_REG(PORTB,11)
#define PORTB_PCR12                              PORT_PCR_REG(PORTB,12)
#define PORTB_PCR13                              PORT_PCR_REG(PORTB,13)
#define PORTB_PCR14                              PORT_PCR_REG(PORTB,14)
#define PORTB_PCR15                              PORT_PCR_REG(PORTB,15)
#define PORTB_PCR16                              PORT_PCR_REG(PORTB,16)
#define PORTB_PCR17                              PORT_PCR_REG(PORTB,17)
#define PORTB_PCR18                              PORT_PCR_REG(PORTB,18)
#define PORTB_PCR19                              PORT_PCR_REG(PORTB,19)
#define PORTB_PCR20                              PORT_PCR_REG(PORTB,20)
#define PORTB_PCR21                              PORT_PCR_REG(PORTB,21)
#define PORTB_PCR22                              PORT_PCR_REG(PORTB,22)
#define PORTB_PCR23                              PORT_PCR_REG(PORTB,23)
#define PORTB_PCR24                              PORT_PCR_REG(PORTB,24)
#define PORTB_PCR25                              PORT_PCR_REG(PORTB,25)
#define PORTB_PCR26                              PORT_PCR_REG(PORTB,26)
#define PORTB_PCR27                              PORT_PCR_REG(PORTB,27)
#define PORTB_PCR28                              PORT_PCR_REG(PORTB,28)
#define PORTB_PCR29                              PORT_PCR_REG(PORTB,29)
#define PORTB_PCR30                              PORT_PCR_REG(PORTB,30)
#define PORTB_PCR31                              PORT_PCR_REG(PORTB,31)
#define PORTB_GPCLR                              PORT_GPCLR_REG(PORTB)
#define PORTB_GPCHR                              PORT_GPCHR_REG(PORTB)
#define PORTB_ISFR                               PORT_ISFR_REG(PORTB)
/* PORTC */
#define PORTC_PCR0                               PORT_PCR_REG(PORTC,0)
#define PORTC_PCR1                               PORT_PCR_REG(PORTC,1)
#define PORTC_PCR2                               PORT_PCR_REG(PORTC,2)
#define PORTC_PCR3                               PORT_PCR_REG(PORTC,3)
#define PORTC_PCR4                               PORT_PCR_REG(PORTC,4)
#define PORTC_PCR5                               PORT_PCR_REG(PORTC,5)
#define PORTC_PCR6                               PORT_PCR_REG(PORTC,6)
#define PORTC_PCR7                               PORT_PCR_REG(PORTC,7)
#define PORTC_PCR8                               PORT_PCR_REG(PORTC,8)
#define PORTC_PCR9                               PORT_PCR_REG(PORTC,9)
#define PORTC_PCR10                              PORT_PCR_REG(PORTC,10)
#define PORTC_PCR11                              PORT_PCR_REG(PORTC,11)
#define PORTC_PCR12                              PORT_PCR_REG(PORTC,12)
#define PORTC_PCR13                              PORT_PCR_REG(PORTC,13)
#define PORTC_PCR14                              PORT_PCR_REG(PORTC,14)
#define PORTC_PCR15                              PORT_PCR_REG(PORTC,15)
#define PORTC_PCR16                              PORT_PCR_REG(PORTC,16)
#define PORTC_PCR17                              PORT_PCR_REG(PORTC,17)
#define PORTC_PCR18                              PORT_PCR_REG(PORTC,18)
#define PORTC_PCR19                              PORT_PCR_REG(PORTC,19)
#define PORTC_PCR20                              PORT_PCR_REG(PORTC,20)
#define PORTC_PCR21                              PORT_PCR_REG(PORTC,21)
#define PORTC_PCR22                              PORT_PCR_REG(PORTC,22)
#define PORTC_PCR23                              PORT_PCR_REG(PORTC,23)
#define PORTC_PCR24                              PORT_PCR_REG(PORTC,24)
#define PORTC_PCR25                              PORT_PCR_REG(PORTC,25)
#define PORTC_PCR26                              PORT_PCR_REG(PORTC,26)
#define PORTC_PCR27                              PORT_PCR_REG(PORTC,27)
#define PORTC_PCR28                              PORT_PCR_REG(PORTC,28)
#define PORTC_PCR29                              PORT_PCR_REG(PORTC,29)
#define PORTC_PCR30                              PORT_PCR_REG(PORTC,30)
#define PORTC_PCR31                              PORT_PCR_REG(PORTC,31)
#define PORTC_GPCLR                              PORT_GPCLR_REG(PORTC)
#define PORTC_GPCHR                              PORT_GPCHR_REG(PORTC)
#define PORTC_ISFR                               PORT_ISFR_REG(PORTC)
/* PORTD */
#define PORTD_PCR0                               PORT_PCR_REG(PORTD,0)
#define PORTD_PCR1                               PORT_PCR_REG(PORTD,1)
#define PORTD_PCR2                               PORT_PCR_REG(PORTD,2)
#define PORTD_PCR3                               PORT_PCR_REG(PORTD,3)
#define PORTD_PCR4                               PORT_PCR_REG(PORTD,4)
#define PORTD_PCR5                               PORT_PCR_REG(PORTD,5)
#define PORTD_PCR6                               PORT_PCR_REG(PORTD,6)
#define PORTD_PCR7                               PORT_PCR_REG(PORTD,7)
#define PORTD_PCR8                               PORT_PCR_REG(PORTD,8)
#define PORTD_PCR9                               PORT_PCR_REG(PORTD,9)
#define PORTD_PCR10                              PORT_PCR_REG(PORTD,10)
#define PORTD_PCR11                              PORT_PCR_REG(PORTD,11)
#define PORTD_PCR12                              PORT_PCR_REG(PORTD,12)
#define PORTD_PCR13                              PORT_PCR_REG(PORTD,13)
#define PORTD_PCR14                              PORT_PCR_REG(PORTD,14)
#define PORTD_PCR15                              PORT_PCR_REG(PORTD,15)
#define PORTD_PCR16                              PORT_PCR_REG(PORTD,16)
#define PORTD_PCR17                              PORT_PCR_REG(PORTD,17)
#define PORTD_PCR18                              PORT_PCR_REG(PORTD,18)
#define PORTD_PCR19                              PORT_PCR_REG(PORTD,19)
#define PORTD_PCR20                              PORT_PCR_REG(PORTD,20)
#define PORTD_PCR21                              PORT_PCR_REG(PORTD,21)
#define PORTD_PCR22                              PORT_PCR_REG(PORTD,22)
#define PORTD_PCR23                              PORT_PCR_REG(PORTD,23)
#define PORTD_PCR24                              PORT_PCR_REG(PORTD,24)
#define PORTD_PCR25                              PORT_PCR_REG(PORTD,25)
#define PORTD_PCR26                              PORT_PCR_REG(PORTD,26)
#define PORTD_PCR27                              PORT_PCR_REG(PORTD,27)
#define PORTD_PCR28                              PORT_PCR_REG(PORTD,28)
#define PORTD_PCR29                              PORT_PCR_REG(PORTD,29)
#define PORTD_PCR30                              PORT_PCR_REG(PORTD,30)
#define PORTD_PCR31                              PORT_PCR_REG(PORTD,31)
#define PORTD_GPCLR                              PORT_GPCLR_REG(PORTD)
#define PORTD_GPCHR                              PORT_GPCHR_REG(PORTD)
#define PORTD_ISFR                               PORT_ISFR_REG(PORTD)
#define PORTD_DFER                               PORT_DFER_REG(PORTD)
#define PORTD_DFCR                               PORT_DFCR_REG(PORTD)
#define PORTD_DFWR                               PORT_DFWR_REG(PORTD)
/* PORTE */
#define PORTE_PCR0                               PORT_PCR_REG(PORTE,0)
#define PORTE_PCR1                               PORT_PCR_REG(PORTE,1)
#define PORTE_PCR2                               PORT_PCR_REG(PORTE,2)
#define PORTE_PCR3                               PORT_PCR_REG(PORTE,3)
#define PORTE_PCR4                               PORT_PCR_REG(PORTE,4)
#define PORTE_PCR5                               PORT_PCR_REG(PORTE,5)
#define PORTE_PCR6                               PORT_PCR_REG(PORTE,6)
#define PORTE_PCR7                               PORT_PCR_REG(PORTE,7)
#define PORTE_PCR8                               PORT_PCR_REG(PORTE,8)
#define PORTE_PCR9                               PORT_PCR_REG(PORTE,9)
#define PORTE_PCR10                              PORT_PCR_REG(PORTE,10)
#define PORTE_PCR11                              PORT_PCR_REG(PORTE,11)
#define PORTE_PCR12                              PORT_PCR_REG(PORTE,12)
#define PORTE_PCR13                              PORT_PCR_REG(PORTE,13)
#define PORTE_PCR14                              PORT_PCR_REG(PORTE,14)
#define PORTE_PCR15                              PORT_PCR_REG(PORTE,15)
#define PORTE_PCR16                              PORT_PCR_REG(PORTE,16)
#define PORTE_PCR17                              PORT_PCR_REG(PORTE,17)
#define PORTE_PCR18                              PORT_PCR_REG(PORTE,18)
#define PORTE_PCR19                              PORT_PCR_REG(PORTE,19)
#define PORTE_PCR20                              PORT_PCR_REG(PORTE,20)
#define PORTE_PCR21                              PORT_PCR_REG(PORTE,21)
#define PORTE_PCR22                              PORT_PCR_REG(PORTE,22)
#define PORTE_PCR23                              PORT_PCR_REG(PORTE,23)
#define PORTE_PCR24                              PORT_PCR_REG(PORTE,24)
#define PORTE_PCR25                              PORT_PCR_REG(PORTE,25)
#define PORTE_PCR26                              PORT_PCR_REG(PORTE,26)
#define PORTE_PCR27                              PORT_PCR_REG(PORTE,27)
#define PORTE_PCR28                              PORT_PCR_REG(PORTE,28)
#define PORTE_PCR29                              PORT_PCR_REG(PORTE,29)
#define PORTE_PCR30                              PORT_PCR_REG(PORTE,30)
#define PORTE_PCR31                              PORT_PCR_REG(PORTE,31)
#define PORTE_GPCLR                              PORT_GPCLR_REG(PORTE)
#define PORTE_GPCHR                              PORT_GPCHR_REG(PORTE)
#define PORTE_ISFR                               PORT_ISFR_REG(PORTE)

/* PORT - Register array accessors */
#define PORTA_PCR(index)                         PORT_PCR_REG(PORTA,index)
#define PORTB_PCR(index)                         PORT_PCR_REG(PORTB,index)
#define PORTC_PCR(index)                         PORT_PCR_REG(PORTC,index)
#define PORTD_PCR(index)                         PORT_PCR_REG(PORTD,index)
#define PORTE_PCR(index)                         PORT_PCR_REG(PORTE,index)

/********  Bits definition for PORTx_PCRn register  *************/
#define PORTx_PCRn_ISF               ((uint32_t)0x01000000)    /*!< Interrupt Status Flag */
#define PORTx_PCRn_IRQC_SHIFT        16
#define PORTx_PCRn_IRQC_MASK         ((uint32_t)((uint32_t)0xF << PORTx_PCRn_IRQC_SHIFT))
#define PORTx_PCRn_IRQC(x)           ((uint32_t)(((uint32_t)(x) << PORTx_PCRn_IRQC_SHIFT) & PORTx_PCRn_IRQC_MASK))
#define PORTx_PCRn_LK                ((uint32_t)0x00008000)    /*!< Lock Register */
#define PORTx_PCRn_MUX_SHIFT         8                         /*!< Pin Mux Control (shift) */
#define PORTx_PCRn_MUX_MASK          ((uint32_t)((uint32_t)0x7 << PORTx_PCRn_MUX_SHIFT))   /*!< Pin Mux Control (mask) */
#define PORTx_PCRn_MUX(x)            ((uint32_t)(((uint32_t)(x) << PORTx_PCRn_MUX_SHIFT) & PORTx_PCRn_MUX_MASK))  /*!< Pin Mux Control */
#define PORTx_PCRn_DSE               ((uint32_t)0x00000040)    /*!< Drive Strength Enable */
#define PORTx_PCRn_ODE               ((uint32_t)0x00000020)    /*!< Open Drain Enable */
#define PORTx_PCRn_PFE               ((uint32_t)0x00000010)    /*!< Passive Filter Enable */
#define PORTx_PCRn_SRE               ((uint32_t)0x00000004)    /*!< Slew Rate Enable */
#define PORTx_PCRn_PE                ((uint32_t)0x00000002)    /*!< Pull Enable */
#define PORTx_PCRn_PS                ((uint32_t)0x00000001)    /*!< Pull Select */

/*!
 * @}
 */ /* end of group PORT_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group PORT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RCM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Peripheral_Access_Layer RCM Peripheral Access Layer
 * @{
 */

/** RCM - Register Layout Typedef */
typedef struct {
  __I  uint8_t SRS0;                               /**< System Reset Status Register 0, offset: 0x0 */
  __I  uint8_t SRS1;                               /**< System Reset Status Register 1, offset: 0x1 */
       uint8_t RESERVED_0[2];
  __IO uint8_t RPFC;                               /**< Reset Pin Filter Control register, offset: 0x4 */
  __IO uint8_t RPFW;                               /**< Reset Pin Filter Width register, offset: 0x5 */
       uint8_t RESERVED_1[1];
  __I  uint8_t MR;                                 /**< Mode Register, offset: 0x7 */
  __IO uint8_t SSRS0;                              /**< Sticky System Reset Status Register 0, offset: 0x8 */
  __IO uint8_t SSRS1;                              /**< Sticky System Reset Status Register 1, offset: 0x9 */
} RCM_TypeDef, *RCM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- RCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Register_Accessor_Macros RCM - Register accessor macros
 * @{
 */


/* RCM - Register accessors */
#define RCM_SRS0_REG(base)                       ((base)->SRS0)
#define RCM_SRS1_REG(base)                       ((base)->SRS1)
#define RCM_RPFC_REG(base)                       ((base)->RPFC)
#define RCM_RPFW_REG(base)                       ((base)->RPFW)
#define RCM_MR_REG(base)                         ((base)->MR)
#define RCM_SSRS0_REG(base)                      ((base)->SSRS0)
#define RCM_SSRS1_REG(base)                      ((base)->SSRS1)

/*!
 * @}
 */ /* end of group RCM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- RCM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Register_Masks RCM Register Masks
 * @{
 */

/* SRS0 Bit Fields */
#define RCM_SRS0_WAKEUP		((uint8_t)0x1u)
#define RCM_SRS0_LVD		((uint8_t)0x2u)
#define RCM_SRS0_LOC		((uint8_t)0x4u)
#define RCM_SRS0_LOL		((uint8_t)0x8u)
#define RCM_SRS0_WDOG		((uint8_t)0x20u)
#define RCM_SRS0_PIN		((uint8_t)0x40u)
#define RCM_SRS0_POR		((uint8_t)0x80u)
/* SRS1 Bit Fields */
#define RCM_SRS1_JTAG		((uint8_t)0x1u)
#define RCM_SRS1_LOCKUP		((uint8_t)0x2u)
#define RCM_SRS1_SW		((uint8_t)0x4u)
#define RCM_SRS1_MDM_AP		((uint8_t)0x8u)
#define RCM_SRS1_EZPT		((uint8_t)0x10u)
#define RCM_SRS1_SACKERR		((uint8_t)0x20u)
/* RPFC Bit Fields */
#define RCM_RPFC_RSTFLTSRW_MASK                  0x3u
#define RCM_RPFC_RSTFLTSRW_SHIFT                 0
#define RCM_RPFC_RSTFLTSRW_WIDTH                 2
#define RCM_RPFC_RSTFLTSRW(x)                    (((uint8_t)(((uint8_t)(x))<<RCM_RPFC_RSTFLTSRW_SHIFT))&RCM_RPFC_RSTFLTSRW_MASK)
#define RCM_RPFC_RSTFLTSS		((uint8_t)0x4u)
/* RPFW Bit Fields */
#define RCM_RPFW_RSTFLTSEL_MASK                  0x1Fu
#define RCM_RPFW_RSTFLTSEL_SHIFT                 0
#define RCM_RPFW_RSTFLTSEL_WIDTH                 5
#define RCM_RPFW_RSTFLTSEL(x)                    (((uint8_t)(((uint8_t)(x))<<RCM_RPFW_RSTFLTSEL_SHIFT))&RCM_RPFW_RSTFLTSEL_MASK)
/* MR Bit Fields */
#define RCM_MR_EZP_MS		((uint8_t)0x2u)
/* SSRS0 Bit Fields */
#define RCM_SSRS0_SWAKEUP		((uint8_t)0x1u)
#define RCM_SSRS0_SLVD		((uint8_t)0x2u)
#define RCM_SSRS0_SLOC		((uint8_t)0x4u)
#define RCM_SSRS0_SLOL		((uint8_t)0x8u)
#define RCM_SSRS0_SWDOG		((uint8_t)0x20u)
#define RCM_SSRS0_SPIN		((uint8_t)0x40u)
#define RCM_SSRS0_SPOR		((uint8_t)0x80u)
/* SSRS1 Bit Fields */
#define RCM_SSRS1_SJTAG		((uint8_t)0x1u)
#define RCM_SSRS1_SLOCKUP		((uint8_t)0x2u)
#define RCM_SSRS1_SSW		((uint8_t)0x4u)
#define RCM_SSRS1_SMDM_AP		((uint8_t)0x8u)
#define RCM_SSRS1_SEZPT		((uint8_t)0x10u)
#define RCM_SSRS1_SSACKERR		((uint8_t)0x20u)

/*!
 * @}
 */ /* end of group RCM_Register_Masks */


/* RCM - Peripheral instance base addresses */
/** Peripheral RCM base address */
#define RCM_BASE                                 (0x4007F000u)
/** Peripheral RCM base pointer */
#define RCM                                      ((RCM_TypeDef *)RCM_BASE)
#define RCM_BASE_PTR                             (RCM)
/** Array initializer of RCM peripheral base addresses */
#define RCM_BASE_ADDRS                           { RCM_BASE }
/** Array initializer of RCM peripheral base pointers */
#define RCM_BASE_PTRS                            { RCM }

/* ----------------------------------------------------------------------------
   -- RCM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RCM_Register_Accessor_Macros RCM - Register accessor macros
 * @{
 */


/* RCM - Register instance definitions */
/* RCM */
#define RCM_SRS0                                 RCM_SRS0_REG(RCM)
#define RCM_SRS1                                 RCM_SRS1_REG(RCM)
#define RCM_RPFC                                 RCM_RPFC_REG(RCM)
#define RCM_RPFW                                 RCM_RPFW_REG(RCM)
#define RCM_MR                                   RCM_MR_REG(RCM)
#define RCM_SSRS0                                RCM_SSRS0_REG(RCM)
#define RCM_SSRS1                                RCM_SSRS1_REG(RCM)

/*!
 * @}
 */ /* end of group RCM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group RCM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RFSYS Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFSYS_Peripheral_Access_Layer RFSYS Peripheral Access Layer
 * @{
 */

/** RFSYS - Register Layout Typedef */
typedef struct {
  __IO uint32_t REG[8];                            /**< Register file register, array offset: 0x0, array step: 0x4 */
} RFSYS_TypeDef, *RFSYS_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- RFSYS - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFSYS_Register_Accessor_Macros RFSYS - Register accessor macros
 * @{
 */


/* RFSYS - Register accessors */
#define RFSYS_REG_REG(base,index)                ((base)->REG[index])
#define RFSYS_REG_COUNT                          8

/*!
 * @}
 */ /* end of group RFSYS_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- RFSYS Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFSYS_Register_Masks RFSYS Register Masks
 * @{
 */

/* REG Bit Fields */
#define RFSYS_REG_LL_MASK                        0xFFu
#define RFSYS_REG_LL_SHIFT                       0
#define RFSYS_REG_LL_WIDTH                       8
#define RFSYS_REG_LL(x)                          (((uint32_t)(((uint32_t)(x))<<RFSYS_REG_LL_SHIFT))&RFSYS_REG_LL_MASK)
#define RFSYS_REG_LH_MASK                        0xFF00u
#define RFSYS_REG_LH_SHIFT                       8
#define RFSYS_REG_LH_WIDTH                       8
#define RFSYS_REG_LH(x)                          (((uint32_t)(((uint32_t)(x))<<RFSYS_REG_LH_SHIFT))&RFSYS_REG_LH_MASK)
#define RFSYS_REG_HL_MASK                        0xFF0000u
#define RFSYS_REG_HL_SHIFT                       16
#define RFSYS_REG_HL_WIDTH                       8
#define RFSYS_REG_HL(x)                          (((uint32_t)(((uint32_t)(x))<<RFSYS_REG_HL_SHIFT))&RFSYS_REG_HL_MASK)
#define RFSYS_REG_HH_MASK                        0xFF000000u
#define RFSYS_REG_HH_SHIFT                       24
#define RFSYS_REG_HH_WIDTH                       8
#define RFSYS_REG_HH(x)                          (((uint32_t)(((uint32_t)(x))<<RFSYS_REG_HH_SHIFT))&RFSYS_REG_HH_MASK)

/*!
 * @}
 */ /* end of group RFSYS_Register_Masks */


/* RFSYS - Peripheral instance base addresses */
/** Peripheral RFSYS base address */
#define RFSYS_BASE                               (0x40041000u)
/** Peripheral RFSYS base pointer */
#define RFSYS                                    ((RFSYS_TypeDef *)RFSYS_BASE)
#define RFSYS_BASE_PTR                           (RFSYS)
/** Array initializer of RFSYS peripheral base addresses */
#define RFSYS_BASE_ADDRS                         { RFSYS_BASE }
/** Array initializer of RFSYS peripheral base pointers */
#define RFSYS_BASE_PTRS                          { RFSYS }

/* ----------------------------------------------------------------------------
   -- RFSYS - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFSYS_Register_Accessor_Macros RFSYS - Register accessor macros
 * @{
 */


/* RFSYS - Register instance definitions */
/* RFSYS */
#define RFSYS_REG0                               RFSYS_REG_REG(RFSYS,0)
#define RFSYS_REG1                               RFSYS_REG_REG(RFSYS,1)
#define RFSYS_REG2                               RFSYS_REG_REG(RFSYS,2)
#define RFSYS_REG3                               RFSYS_REG_REG(RFSYS,3)
#define RFSYS_REG4                               RFSYS_REG_REG(RFSYS,4)
#define RFSYS_REG5                               RFSYS_REG_REG(RFSYS,5)
#define RFSYS_REG6                               RFSYS_REG_REG(RFSYS,6)
#define RFSYS_REG7                               RFSYS_REG_REG(RFSYS,7)

/* RFSYS - Register array accessors */
#define RFSYS_REG(index)                         RFSYS_REG_REG(RFSYS,index)

/*!
 * @}
 */ /* end of group RFSYS_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group RFSYS_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RFVBAT Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFVBAT_Peripheral_Access_Layer RFVBAT Peripheral Access Layer
 * @{
 */

/** RFVBAT - Register Layout Typedef */
typedef struct {
  __IO uint32_t REG[8];                            /**< VBAT register file register, array offset: 0x0, array step: 0x4 */
} RFVBAT_TypeDef, *RFVBAT_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- RFVBAT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFVBAT_Register_Accessor_Macros RFVBAT - Register accessor macros
 * @{
 */


/* RFVBAT - Register accessors */
#define RFVBAT_REG_REG(base,index)               ((base)->REG[index])
#define RFVBAT_REG_COUNT                         8

/*!
 * @}
 */ /* end of group RFVBAT_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- RFVBAT Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFVBAT_Register_Masks RFVBAT Register Masks
 * @{
 */

/* REG Bit Fields */
#define RFVBAT_REG_LL_MASK                       0xFFu
#define RFVBAT_REG_LL_SHIFT                      0
#define RFVBAT_REG_LL_WIDTH                      8
#define RFVBAT_REG_LL(x)                         (((uint32_t)(((uint32_t)(x))<<RFVBAT_REG_LL_SHIFT))&RFVBAT_REG_LL_MASK)
#define RFVBAT_REG_LH_MASK                       0xFF00u
#define RFVBAT_REG_LH_SHIFT                      8
#define RFVBAT_REG_LH_WIDTH                      8
#define RFVBAT_REG_LH(x)                         (((uint32_t)(((uint32_t)(x))<<RFVBAT_REG_LH_SHIFT))&RFVBAT_REG_LH_MASK)
#define RFVBAT_REG_HL_MASK                       0xFF0000u
#define RFVBAT_REG_HL_SHIFT                      16
#define RFVBAT_REG_HL_WIDTH                      8
#define RFVBAT_REG_HL(x)                         (((uint32_t)(((uint32_t)(x))<<RFVBAT_REG_HL_SHIFT))&RFVBAT_REG_HL_MASK)
#define RFVBAT_REG_HH_MASK                       0xFF000000u
#define RFVBAT_REG_HH_SHIFT                      24
#define RFVBAT_REG_HH_WIDTH                      8
#define RFVBAT_REG_HH(x)                         (((uint32_t)(((uint32_t)(x))<<RFVBAT_REG_HH_SHIFT))&RFVBAT_REG_HH_MASK)

/*!
 * @}
 */ /* end of group RFVBAT_Register_Masks */


/* RFVBAT - Peripheral instance base addresses */
/** Peripheral RFVBAT base address */
#define RFVBAT_BASE                              (0x4003E000u)
/** Peripheral RFVBAT base pointer */
#define RFVBAT                                   ((RFVBAT_TypeDef *)RFVBAT_BASE)
#define RFVBAT_BASE_PTR                          (RFVBAT)
/** Array initializer of RFVBAT peripheral base addresses */
#define RFVBAT_BASE_ADDRS                        { RFVBAT_BASE }
/** Array initializer of RFVBAT peripheral base pointers */
#define RFVBAT_BASE_PTRS                         { RFVBAT }

/* ----------------------------------------------------------------------------
   -- RFVBAT - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RFVBAT_Register_Accessor_Macros RFVBAT - Register accessor macros
 * @{
 */


/* RFVBAT - Register instance definitions */
/* RFVBAT */
#define RFVBAT_REG0                              RFVBAT_REG_REG(RFVBAT,0)
#define RFVBAT_REG1                              RFVBAT_REG_REG(RFVBAT,1)
#define RFVBAT_REG2                              RFVBAT_REG_REG(RFVBAT,2)
#define RFVBAT_REG3                              RFVBAT_REG_REG(RFVBAT,3)
#define RFVBAT_REG4                              RFVBAT_REG_REG(RFVBAT,4)
#define RFVBAT_REG5                              RFVBAT_REG_REG(RFVBAT,5)
#define RFVBAT_REG6                              RFVBAT_REG_REG(RFVBAT,6)
#define RFVBAT_REG7                              RFVBAT_REG_REG(RFVBAT,7)

/* RFVBAT - Register array accessors */
#define RFVBAT_REG(index)                        RFVBAT_REG_REG(RFVBAT,index)

/*!
 * @}
 */ /* end of group RFVBAT_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group RFVBAT_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RNG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RNG_Peripheral_Access_Layer RNG Peripheral Access Layer
 * @{
 */

/** RNG - Register Layout Typedef */
typedef struct {
  __IO uint32_t CR;                                /**< RNGA Control Register, offset: 0x0 */
  __I  uint32_t SR;                                /**< RNGA Status Register, offset: 0x4 */
  __O  uint32_t ER;                                /**< RNGA Entropy Register, offset: 0x8 */
  __I  uint32_t OR;                                /**< RNGA Output Register, offset: 0xC */
} RNG_TypeDef, *RNG_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- RNG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RNG_Register_Accessor_Macros RNG - Register accessor macros
 * @{
 */


/* RNG - Register accessors */
#define RNG_CR_REG(base)                         ((base)->CR)
#define RNG_SR_REG(base)                         ((base)->SR)
#define RNG_ER_REG(base)                         ((base)->ER)
#define RNG_OR_REG(base)                         ((base)->OR)

/*!
 * @}
 */ /* end of group RNG_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- RNG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RNG_Register_Masks RNG Register Masks
 * @{
 */

/* CR Bit Fields */
#define RNG_CR_GO		((uint32_t)0x1u)
#define RNG_CR_HA		((uint32_t)0x2u)
#define RNG_CR_INTM		((uint32_t)0x4u)
#define RNG_CR_CLRI		((uint32_t)0x8u)
#define RNG_CR_SLP		((uint32_t)0x10u)
/* SR Bit Fields */
#define RNG_SR_SECV		((uint32_t)0x1u)
#define RNG_SR_LRS		((uint32_t)0x2u)
#define RNG_SR_ORU		((uint32_t)0x4u)
#define RNG_SR_ERRI		((uint32_t)0x8u)
#define RNG_SR_SLP		((uint32_t)0x10u)
#define RNG_SR_OREG_LVL_MASK                     0xFF00u
#define RNG_SR_OREG_LVL_SHIFT                    8
#define RNG_SR_OREG_LVL_WIDTH                    8
#define RNG_SR_OREG_LVL(x)                       (((uint32_t)(((uint32_t)(x))<<RNG_SR_OREG_LVL_SHIFT))&RNG_SR_OREG_LVL_MASK)
#define RNG_SR_OREG_SIZE_MASK                    0xFF0000u
#define RNG_SR_OREG_SIZE_SHIFT                   16
#define RNG_SR_OREG_SIZE_WIDTH                   8
#define RNG_SR_OREG_SIZE(x)                      (((uint32_t)(((uint32_t)(x))<<RNG_SR_OREG_SIZE_SHIFT))&RNG_SR_OREG_SIZE_MASK)
/* ER Bit Fields */
#define RNG_ER_EXT_ENT_MASK                      0xFFFFFFFFu
#define RNG_ER_EXT_ENT_SHIFT                     0
#define RNG_ER_EXT_ENT_WIDTH                     32
#define RNG_ER_EXT_ENT(x)                        (((uint32_t)(((uint32_t)(x))<<RNG_ER_EXT_ENT_SHIFT))&RNG_ER_EXT_ENT_MASK)
/* OR Bit Fields */
#define RNG_OR_RANDOUT_MASK                      0xFFFFFFFFu
#define RNG_OR_RANDOUT_SHIFT                     0
#define RNG_OR_RANDOUT_WIDTH                     32
#define RNG_OR_RANDOUT(x)                        (((uint32_t)(((uint32_t)(x))<<RNG_OR_RANDOUT_SHIFT))&RNG_OR_RANDOUT_MASK)

/*!
 * @}
 */ /* end of group RNG_Register_Masks */


/* RNG - Peripheral instance base addresses */
/** Peripheral RNG base address */
#define RNG_BASE                                 (0x40029000u)
/** Peripheral RNG base pointer */
#define RNG                                      ((RNG_TypeDef *)RNG_BASE)
#define RNG_BASE_PTR                             (RNG)
/** Array initializer of RNG peripheral base addresses */
#define RNG_BASE_ADDRS                           { RNG_BASE }
/** Array initializer of RNG peripheral base pointers */
#define RNG_BASE_PTRS                            { RNG }
/** Interrupt vectors for the RNG peripheral type */
#define RNG_IRQS                                 { RNG_IRQn }

/* ----------------------------------------------------------------------------
   -- RNG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RNG_Register_Accessor_Macros RNG - Register accessor macros
 * @{
 */


/* RNG - Register instance definitions */
/* RNG */
#define RNG_CR                                   RNG_CR_REG(RNG)
#define RNG_SR                                   RNG_SR_REG(RNG)
#define RNG_ER                                   RNG_ER_REG(RNG)
#define RNG_OR                                   RNG_OR_REG(RNG)

/*!
 * @}
 */ /* end of group RNG_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group RNG_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- RTC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Peripheral_Access_Layer RTC Peripheral Access Layer
 * @{
 */

/** RTC - Register Layout Typedef */
typedef struct {
  __IO uint32_t TSR;                               /**< RTC Time Seconds Register, offset: 0x0 */
  __IO uint32_t TPR;                               /**< RTC Time Prescaler Register, offset: 0x4 */
  __IO uint32_t TAR;                               /**< RTC Time Alarm Register, offset: 0x8 */
  __IO uint32_t TCR;                               /**< RTC Time Compensation Register, offset: 0xC */
  __IO uint32_t CR;                                /**< RTC Control Register, offset: 0x10 */
  __IO uint32_t SR;                                /**< RTC Status Register, offset: 0x14 */
  __IO uint32_t LR;                                /**< RTC Lock Register, offset: 0x18 */
  __IO uint32_t IER;                               /**< RTC Interrupt Enable Register, offset: 0x1C */
       uint8_t RESERVED_0[2016];
  __IO uint32_t WAR;                               /**< RTC Write Access Register, offset: 0x800 */
  __IO uint32_t RAR;                               /**< RTC Read Access Register, offset: 0x804 */
} RTC_TypeDef, *RTC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- RTC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Register_Accessor_Macros RTC - Register accessor macros
 * @{
 */


/* RTC - Register accessors */
#define RTC_TSR_REG(base)                        ((base)->TSR)
#define RTC_TPR_REG(base)                        ((base)->TPR)
#define RTC_TAR_REG(base)                        ((base)->TAR)
#define RTC_TCR_REG(base)                        ((base)->TCR)
#define RTC_CR_REG(base)                         ((base)->CR)
#define RTC_SR_REG(base)                         ((base)->SR)
#define RTC_LR_REG(base)                         ((base)->LR)
#define RTC_IER_REG(base)                        ((base)->IER)
#define RTC_WAR_REG(base)                        ((base)->WAR)
#define RTC_RAR_REG(base)                        ((base)->RAR)

/*!
 * @}
 */ /* end of group RTC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- RTC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Register_Masks RTC Register Masks
 * @{
 */

/* TSR Bit Fields */
#define RTC_TSR_TSR_MASK                         0xFFFFFFFFu
#define RTC_TSR_TSR_SHIFT                        0
#define RTC_TSR_TSR_WIDTH                        32
#define RTC_TSR_TSR(x)                           (((uint32_t)(((uint32_t)(x))<<RTC_TSR_TSR_SHIFT))&RTC_TSR_TSR_MASK)
/* TPR Bit Fields */
#define RTC_TPR_TPR_MASK                         0xFFFFu
#define RTC_TPR_TPR_SHIFT                        0
#define RTC_TPR_TPR_WIDTH                        16
#define RTC_TPR_TPR(x)                           (((uint32_t)(((uint32_t)(x))<<RTC_TPR_TPR_SHIFT))&RTC_TPR_TPR_MASK)
/* TAR Bit Fields */
#define RTC_TAR_TAR_MASK                         0xFFFFFFFFu
#define RTC_TAR_TAR_SHIFT                        0
#define RTC_TAR_TAR_WIDTH                        32
#define RTC_TAR_TAR(x)                           (((uint32_t)(((uint32_t)(x))<<RTC_TAR_TAR_SHIFT))&RTC_TAR_TAR_MASK)
/* TCR Bit Fields */
#define RTC_TCR_TCR_MASK                         0xFFu
#define RTC_TCR_TCR_SHIFT                        0
#define RTC_TCR_TCR_WIDTH                        8
#define RTC_TCR_TCR(x)                           (((uint32_t)(((uint32_t)(x))<<RTC_TCR_TCR_SHIFT))&RTC_TCR_TCR_MASK)
#define RTC_TCR_CIR_MASK                         0xFF00u
#define RTC_TCR_CIR_SHIFT                        8
#define RTC_TCR_CIR_WIDTH                        8
#define RTC_TCR_CIR(x)                           (((uint32_t)(((uint32_t)(x))<<RTC_TCR_CIR_SHIFT))&RTC_TCR_CIR_MASK)
#define RTC_TCR_TCV_MASK                         0xFF0000u
#define RTC_TCR_TCV_SHIFT                        16
#define RTC_TCR_TCV_WIDTH                        8
#define RTC_TCR_TCV(x)                           (((uint32_t)(((uint32_t)(x))<<RTC_TCR_TCV_SHIFT))&RTC_TCR_TCV_MASK)
#define RTC_TCR_CIC_MASK                         0xFF000000u
#define RTC_TCR_CIC_SHIFT                        24
#define RTC_TCR_CIC_WIDTH                        8
#define RTC_TCR_CIC(x)                           (((uint32_t)(((uint32_t)(x))<<RTC_TCR_CIC_SHIFT))&RTC_TCR_CIC_MASK)
/* CR Bit Fields */
#define RTC_CR_SWR		((uint32_t)0x1u)
#define RTC_CR_WPE		((uint32_t)0x2u)
#define RTC_CR_SUP		((uint32_t)0x4u)
#define RTC_CR_UM		((uint32_t)0x8u)
#define RTC_CR_WPS		((uint32_t)0x10u)
#define RTC_CR_OSCE		((uint32_t)0x100u)
#define RTC_CR_CLKO		((uint32_t)0x200u)
#define RTC_CR_SC16P		((uint32_t)0x400u)
#define RTC_CR_SC8P		((uint32_t)0x800u)
#define RTC_CR_SC4P		((uint32_t)0x1000u)
#define RTC_CR_SC2P		((uint32_t)0x2000u)
/* SR Bit Fields */
#define RTC_SR_TIF		((uint32_t)0x1u)
#define RTC_SR_TOF		((uint32_t)0x2u)
#define RTC_SR_TAF		((uint32_t)0x4u)
#define RTC_SR_TCE		((uint32_t)0x10u)
/* LR Bit Fields */
#define RTC_LR_TCL		((uint32_t)0x8u)
#define RTC_LR_CRL		((uint32_t)0x10u)
#define RTC_LR_SRL		((uint32_t)0x20u)
#define RTC_LR_LRL		((uint32_t)0x40u)
/* IER Bit Fields */
#define RTC_IER_TIIE		((uint32_t)0x1u)
#define RTC_IER_TOIE		((uint32_t)0x2u)
#define RTC_IER_TAIE		((uint32_t)0x4u)
#define RTC_IER_TSIE		((uint32_t)0x10u)
#define RTC_IER_WPON		((uint32_t)0x80u)
/* WAR Bit Fields */
#define RTC_WAR_TSRW		((uint32_t)0x1u)
#define RTC_WAR_TPRW		((uint32_t)0x2u)
#define RTC_WAR_TARW		((uint32_t)0x4u)
#define RTC_WAR_TCRW		((uint32_t)0x8u)
#define RTC_WAR_CRW		((uint32_t)0x10u)
#define RTC_WAR_SRW		((uint32_t)0x20u)
#define RTC_WAR_LRW		((uint32_t)0x40u)
#define RTC_WAR_IERW		((uint32_t)0x80u)
/* RAR Bit Fields */
#define RTC_RAR_TSRR		((uint32_t)0x1u)
#define RTC_RAR_TPRR		((uint32_t)0x2u)
#define RTC_RAR_TARR		((uint32_t)0x4u)
#define RTC_RAR_TCRR		((uint32_t)0x8u)
#define RTC_RAR_CRR		((uint32_t)0x10u)
#define RTC_RAR_SRR		((uint32_t)0x20u)
#define RTC_RAR_LRR		((uint32_t)0x40u)
#define RTC_RAR_IERR		((uint32_t)0x80u)

/*!
 * @}
 */ /* end of group RTC_Register_Masks */


/* RTC - Peripheral instance base addresses */
/** Peripheral RTC base address */
#define RTC_BASE                                 (0x4003D000u)
/** Peripheral RTC base pointer */
#define RTC                                      ((RTC_TypeDef *)RTC_BASE)
#define RTC_BASE_PTR                             (RTC)
/** Array initializer of RTC peripheral base addresses */
#define RTC_BASE_ADDRS                           { RTC_BASE }
/** Array initializer of RTC peripheral base pointers */
#define RTC_BASE_PTRS                            { RTC }
/** Interrupt vectors for the RTC peripheral type */
#define RTC_IRQS                                 { RTC_IRQn }
#define RTC_SECONDS_IRQS                         { RTC_Seconds_IRQn }

/* ----------------------------------------------------------------------------
   -- RTC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup RTC_Register_Accessor_Macros RTC - Register accessor macros
 * @{
 */


/* RTC - Register instance definitions */
/* RTC */
#define RTC_TSR                                  RTC_TSR_REG(RTC)
#define RTC_TPR                                  RTC_TPR_REG(RTC)
#define RTC_TAR                                  RTC_TAR_REG(RTC)
#define RTC_TCR                                  RTC_TCR_REG(RTC)
#define RTC_CR                                   RTC_CR_REG(RTC)
#define RTC_SR                                   RTC_SR_REG(RTC)
#define RTC_LR                                   RTC_LR_REG(RTC)
#define RTC_IER                                  RTC_IER_REG(RTC)
#define RTC_WAR                                  RTC_WAR_REG(RTC)
#define RTC_RAR                                  RTC_RAR_REG(RTC)

/*!
 * @}
 */ /* end of group RTC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group RTC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SIM Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Peripheral_Access_Layer SIM Peripheral Access Layer
 * @{
 */

/** SIM - Register Layout Typedef */
typedef struct {
  __IO uint32_t SOPT1;                             /**< System Options Register 1, offset: 0x0 */
  __IO uint32_t SOPT1CFG;                          /**< SOPT1 Configuration Register, offset: 0x4 */
       uint8_t RESERVED_0[4092];
  __IO uint32_t SOPT2;                             /**< System Options Register 2, offset: 0x1004 */
       uint8_t RESERVED_1[4];
  __IO uint32_t SOPT4;                             /**< System Options Register 4, offset: 0x100C */
  __IO uint32_t SOPT5;                             /**< System Options Register 5, offset: 0x1010 */
       uint8_t RESERVED_2[4];
  __IO uint32_t SOPT7;                             /**< System Options Register 7, offset: 0x1018 */
  __IO uint32_t SOPT8;                             /**< System Options Register 8, offset: 0x101C */
       uint8_t RESERVED_3[4];
  __I  uint32_t SDID;                              /**< System Device Identification Register, offset: 0x1024 */
       uint8_t RESERVED_4[12];
  __IO uint32_t SCGC4;                             /**< System Clock Gating Control Register 4, offset: 0x1034 */
  __IO uint32_t SCGC5;                             /**< System Clock Gating Control Register 5, offset: 0x1038 */
  __IO uint32_t SCGC6;                             /**< System Clock Gating Control Register 6, offset: 0x103C */
  __IO uint32_t SCGC7;                             /**< System Clock Gating Control Register 7, offset: 0x1040 */
  __IO uint32_t CLKDIV1;                           /**< System Clock Divider Register 1, offset: 0x1044 */
  __IO uint32_t CLKDIV2;                           /**< System Clock Divider Register 2, offset: 0x1048 */
  __IO uint32_t FCFG1;                             /**< Flash Configuration Register 1, offset: 0x104C */
  __I  uint32_t FCFG2;                             /**< Flash Configuration Register 2, offset: 0x1050 */
  __I  uint32_t UIDH;                              /**< Unique Identification Register High, offset: 0x1054 */
  __I  uint32_t UIDMH;                             /**< Unique Identification Register Mid-High, offset: 0x1058 */
  __I  uint32_t UIDML;                             /**< Unique Identification Register Mid Low, offset: 0x105C */
  __I  uint32_t UIDL;                              /**< Unique Identification Register Low, offset: 0x1060 */
} SIM_TypeDef, *SIM_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SIM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Accessor_Macros SIM - Register accessor macros
 * @{
 */


/* SIM - Register accessors */
#define SIM_SOPT1_REG(base)                      ((base)->SOPT1)
#define SIM_SOPT1CFG_REG(base)                   ((base)->SOPT1CFG)
#define SIM_SOPT2_REG(base)                      ((base)->SOPT2)
#define SIM_SOPT4_REG(base)                      ((base)->SOPT4)
#define SIM_SOPT5_REG(base)                      ((base)->SOPT5)
#define SIM_SOPT7_REG(base)                      ((base)->SOPT7)
#define SIM_SOPT8_REG(base)                      ((base)->SOPT8)
#define SIM_SDID_REG(base)                       ((base)->SDID)
#define SIM_SCGC4_REG(base)                      ((base)->SCGC4)
#define SIM_SCGC5_REG(base)                      ((base)->SCGC5)
#define SIM_SCGC6_REG(base)                      ((base)->SCGC6)
#define SIM_SCGC7_REG(base)                      ((base)->SCGC7)
#define SIM_CLKDIV1_REG(base)                    ((base)->CLKDIV1)
#define SIM_CLKDIV2_REG(base)                    ((base)->CLKDIV2)
#define SIM_FCFG1_REG(base)                      ((base)->FCFG1)
#define SIM_FCFG2_REG(base)                      ((base)->FCFG2)
#define SIM_UIDH_REG(base)                       ((base)->UIDH)
#define SIM_UIDMH_REG(base)                      ((base)->UIDMH)
#define SIM_UIDML_REG(base)                      ((base)->UIDML)
#define SIM_UIDL_REG(base)                       ((base)->UIDL)

/*!
 * @}
 */ /* end of group SIM_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- SIM Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Masks SIM Register Masks
 * @{
 */

/* SOPT1 Bit Fields */
#define SIM_SOPT1_RAMSIZE_MASK                   0xF000u
#define SIM_SOPT1_RAMSIZE_SHIFT                  12
#define SIM_SOPT1_RAMSIZE_WIDTH                  4
#define SIM_SOPT1_RAMSIZE(x)                     (((uint32_t)(((uint32_t)(x))<<SIM_SOPT1_RAMSIZE_SHIFT))&SIM_SOPT1_RAMSIZE_MASK)
#define SIM_SOPT1_OSC32KOUT_MASK                 0x30000u
#define SIM_SOPT1_OSC32KOUT_SHIFT                16
#define SIM_SOPT1_OSC32KOUT_WIDTH                2
#define SIM_SOPT1_OSC32KOUT(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SOPT1_OSC32KOUT_SHIFT))&SIM_SOPT1_OSC32KOUT_MASK)
#define SIM_SOPT1_OSC32KSEL_MASK                 0xC0000u
#define SIM_SOPT1_OSC32KSEL_SHIFT                18
#define SIM_SOPT1_OSC32KSEL_WIDTH                2
#define SIM_SOPT1_OSC32KSEL(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SOPT1_OSC32KSEL_SHIFT))&SIM_SOPT1_OSC32KSEL_MASK)
#define SIM_SOPT1_USBVSTBY		((uint32_t)0x20000000u)
#define SIM_SOPT1_USBSSTBY		((uint32_t)0x40000000u)
#define SIM_SOPT1_USBREGEN		((uint32_t)0x80000000u)
/* SOPT1CFG Bit Fields */
#define SIM_SOPT1CFG_URWE		((uint32_t)0x1000000u)
#define SIM_SOPT1CFG_UVSWE		((uint32_t)0x2000000u)
#define SIM_SOPT1CFG_USSWE		((uint32_t)0x4000000u)
/* SOPT2 Bit Fields */
#define SIM_SOPT2_RTCCLKOUTSEL		((uint32_t)0x10u)
#define SIM_SOPT2_CLKOUTSEL_MASK                 0xE0u
#define SIM_SOPT2_CLKOUTSEL_SHIFT                5
#define SIM_SOPT2_CLKOUTSEL_WIDTH                3
#define SIM_SOPT2_CLKOUTSEL(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SOPT2_CLKOUTSEL_SHIFT))&SIM_SOPT2_CLKOUTSEL_MASK)
#define SIM_SOPT2_FBSL_MASK                      0x300u
#define SIM_SOPT2_FBSL_SHIFT                     8
#define SIM_SOPT2_FBSL_WIDTH                     2
#define SIM_SOPT2_FBSL(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SOPT2_FBSL_SHIFT))&SIM_SOPT2_FBSL_MASK)
#define SIM_SOPT2_TRACECLKSEL		((uint32_t)0x1000u)
#define SIM_SOPT2_PLLFLLSEL_MASK                 0x30000u
#define SIM_SOPT2_PLLFLLSEL_SHIFT                16
#define SIM_SOPT2_PLLFLLSEL_WIDTH                2
#define SIM_SOPT2_PLLFLLSEL(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SOPT2_PLLFLLSEL_SHIFT))&SIM_SOPT2_PLLFLLSEL_MASK)
#define SIM_SOPT2_USBSRC		((uint32_t)0x40000u)
#define SIM_SOPT2_LPUARTSRC_MASK                 0xC000000u
#define SIM_SOPT2_LPUARTSRC_SHIFT                26
#define SIM_SOPT2_LPUARTSRC_WIDTH                2
#define SIM_SOPT2_LPUARTSRC(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_SOPT2_LPUARTSRC_SHIFT))&SIM_SOPT2_LPUARTSRC_MASK)
/* SOPT4 Bit Fields */
#define SIM_SOPT4_FTM0FLT0		((uint32_t)0x1u)
#define SIM_SOPT4_FTM0FLT1		((uint32_t)0x2u)
#define SIM_SOPT4_FTM1FLT0		((uint32_t)0x10u)
#define SIM_SOPT4_FTM2FLT0		((uint32_t)0x100u)
#define SIM_SOPT4_FTM3FLT0		((uint32_t)0x1000u)
#define SIM_SOPT4_FTM1CH0SRC_MASK                0xC0000u
#define SIM_SOPT4_FTM1CH0SRC_SHIFT               18
#define SIM_SOPT4_FTM1CH0SRC_WIDTH               2
#define SIM_SOPT4_FTM1CH0SRC(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM1CH0SRC_SHIFT))&SIM_SOPT4_FTM1CH0SRC_MASK)
#define SIM_SOPT4_FTM2CH0SRC_MASK                0x300000u
#define SIM_SOPT4_FTM2CH0SRC_SHIFT               20
#define SIM_SOPT4_FTM2CH0SRC_WIDTH               2
#define SIM_SOPT4_FTM2CH0SRC(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT4_FTM2CH0SRC_SHIFT))&SIM_SOPT4_FTM2CH0SRC_MASK)
#define SIM_SOPT4_FTM2CH1SRC		((uint32_t)0x400000u)
#define SIM_SOPT4_FTM0CLKSEL		((uint32_t)0x1000000u)
#define SIM_SOPT4_FTM1CLKSEL		((uint32_t)0x2000000u)
#define SIM_SOPT4_FTM2CLKSEL		((uint32_t)0x4000000u)
#define SIM_SOPT4_FTM3CLKSEL		((uint32_t)0x8000000u)
#define SIM_SOPT4_FTM0TRG0SRC		((uint32_t)0x10000000u)
#define SIM_SOPT4_FTM0TRG1SRC		((uint32_t)0x20000000u)
#define SIM_SOPT4_FTM3TRG0SRC		((uint32_t)0x40000000u)
#define SIM_SOPT4_FTM3TRG1SRC		((uint32_t)0x80000000u)
/* SOPT5 Bit Fields */
#define SIM_SOPT5_UART0TXSRC_MASK                0x3u
#define SIM_SOPT5_UART0TXSRC_SHIFT               0
#define SIM_SOPT5_UART0TXSRC_WIDTH               2
#define SIM_SOPT5_UART0TXSRC(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT5_UART0TXSRC_SHIFT))&SIM_SOPT5_UART0TXSRC_MASK)
#define SIM_SOPT5_UART0RXSRC_MASK                0xCu
#define SIM_SOPT5_UART0RXSRC_SHIFT               2
#define SIM_SOPT5_UART0RXSRC_WIDTH               2
#define SIM_SOPT5_UART0RXSRC(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT5_UART0RXSRC_SHIFT))&SIM_SOPT5_UART0RXSRC_MASK)
#define SIM_SOPT5_UART1TXSRC_MASK                0x30u
#define SIM_SOPT5_UART1TXSRC_SHIFT               4
#define SIM_SOPT5_UART1TXSRC_WIDTH               2
#define SIM_SOPT5_UART1TXSRC(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT5_UART1TXSRC_SHIFT))&SIM_SOPT5_UART1TXSRC_MASK)
#define SIM_SOPT5_UART1RXSRC_MASK                0xC0u
#define SIM_SOPT5_UART1RXSRC_SHIFT               6
#define SIM_SOPT5_UART1RXSRC_WIDTH               2
#define SIM_SOPT5_UART1RXSRC(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT5_UART1RXSRC_SHIFT))&SIM_SOPT5_UART1RXSRC_MASK)
#define SIM_SOPT5_LPUART0RXSRC_MASK              0xC0000u
#define SIM_SOPT5_LPUART0RXSRC_SHIFT             18
#define SIM_SOPT5_LPUART0RXSRC_WIDTH             2
#define SIM_SOPT5_LPUART0RXSRC(x)                (((uint32_t)(((uint32_t)(x))<<SIM_SOPT5_LPUART0RXSRC_SHIFT))&SIM_SOPT5_LPUART0RXSRC_MASK)
/* SOPT7 Bit Fields */
#define SIM_SOPT7_ADC0TRGSEL_MASK                0xFu
#define SIM_SOPT7_ADC0TRGSEL_SHIFT               0
#define SIM_SOPT7_ADC0TRGSEL_WIDTH               4
#define SIM_SOPT7_ADC0TRGSEL(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT7_ADC0TRGSEL_SHIFT))&SIM_SOPT7_ADC0TRGSEL_MASK)
#define SIM_SOPT7_ADC0PRETRGSEL		((uint32_t)0x10u)
#define SIM_SOPT7_ADC0ALTTRGEN		((uint32_t)0x80u)
#define SIM_SOPT7_ADC1TRGSEL_MASK                0xF00u
#define SIM_SOPT7_ADC1TRGSEL_SHIFT               8
#define SIM_SOPT7_ADC1TRGSEL_WIDTH               4
#define SIM_SOPT7_ADC1TRGSEL(x)                  (((uint32_t)(((uint32_t)(x))<<SIM_SOPT7_ADC1TRGSEL_SHIFT))&SIM_SOPT7_ADC1TRGSEL_MASK)
#define SIM_SOPT7_ADC1PRETRGSEL		((uint32_t)0x1000u)
#define SIM_SOPT7_ADC1ALTTRGEN		((uint32_t)0x8000u)
/* SOPT8 Bit Fields */
#define SIM_SOPT8_FTM0SYNCBIT		((uint32_t)0x1u)
#define SIM_SOPT8_FTM1SYNCBIT		((uint32_t)0x2u)
#define SIM_SOPT8_FTM2SYNCBIT		((uint32_t)0x4u)
#define SIM_SOPT8_FTM3SYNCBIT		((uint32_t)0x8u)
#define SIM_SOPT8_FTM0OCH0SRC		((uint32_t)0x10000u)
#define SIM_SOPT8_FTM0OCH1SRC		((uint32_t)0x20000u)
#define SIM_SOPT8_FTM0OCH2SRC		((uint32_t)0x40000u)
#define SIM_SOPT8_FTM0OCH3SRC		((uint32_t)0x80000u)
#define SIM_SOPT8_FTM0OCH4SRC		((uint32_t)0x100000u)
#define SIM_SOPT8_FTM0OCH5SRC		((uint32_t)0x200000u)
#define SIM_SOPT8_FTM0OCH6SRC		((uint32_t)0x400000u)
#define SIM_SOPT8_FTM0OCH7SRC		((uint32_t)0x800000u)
#define SIM_SOPT8_FTM3OCH0SRC		((uint32_t)0x1000000u)
#define SIM_SOPT8_FTM3OCH1SRC		((uint32_t)0x2000000u)
#define SIM_SOPT8_FTM3OCH2SRC		((uint32_t)0x4000000u)
#define SIM_SOPT8_FTM3OCH3SRC		((uint32_t)0x8000000u)
#define SIM_SOPT8_FTM3OCH4SRC		((uint32_t)0x10000000u)
#define SIM_SOPT8_FTM3OCH5SRC		((uint32_t)0x20000000u)
#define SIM_SOPT8_FTM3OCH6SRC		((uint32_t)0x40000000u)
#define SIM_SOPT8_FTM3OCH7SRC		((uint32_t)0x80000000u)
/* SDID Bit Fields */
#define SIM_SDID_PINID_MASK                      0xFu
#define SIM_SDID_PINID_SHIFT                     0
#define SIM_SDID_PINID_WIDTH                     4
#define SIM_SDID_PINID(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SDID_PINID_SHIFT))&SIM_SDID_PINID_MASK)
#define SIM_SDID_FAMID_MASK                      0x70u
#define SIM_SDID_FAMID_SHIFT                     4
#define SIM_SDID_FAMID_WIDTH                     3
#define SIM_SDID_FAMID(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SDID_FAMID_SHIFT))&SIM_SDID_FAMID_MASK)
#define SIM_SDID_DIEID_MASK                      0xF80u
#define SIM_SDID_DIEID_SHIFT                     7
#define SIM_SDID_DIEID_WIDTH                     5
#define SIM_SDID_DIEID(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SDID_DIEID_SHIFT))&SIM_SDID_DIEID_MASK)
#define SIM_SDID_REVID_MASK                      0xF000u
#define SIM_SDID_REVID_SHIFT                     12
#define SIM_SDID_REVID_WIDTH                     4
#define SIM_SDID_REVID(x)                        (((uint32_t)(((uint32_t)(x))<<SIM_SDID_REVID_SHIFT))&SIM_SDID_REVID_MASK)
#define SIM_SDID_SERIESID_MASK                   0xF00000u
#define SIM_SDID_SERIESID_SHIFT                  20
#define SIM_SDID_SERIESID_WIDTH                  4
#define SIM_SDID_SERIESID(x)                     (((uint32_t)(((uint32_t)(x))<<SIM_SDID_SERIESID_SHIFT))&SIM_SDID_SERIESID_MASK)
#define SIM_SDID_SUBFAMID_MASK                   0xF000000u
#define SIM_SDID_SUBFAMID_SHIFT                  24
#define SIM_SDID_SUBFAMID_WIDTH                  4
#define SIM_SDID_SUBFAMID(x)                     (((uint32_t)(((uint32_t)(x))<<SIM_SDID_SUBFAMID_SHIFT))&SIM_SDID_SUBFAMID_MASK)
#define SIM_SDID_FAMILYID_MASK                   0xF0000000u
#define SIM_SDID_FAMILYID_SHIFT                  28
#define SIM_SDID_FAMILYID_WIDTH                  4
#define SIM_SDID_FAMILYID(x)                     (((uint32_t)(((uint32_t)(x))<<SIM_SDID_FAMILYID_SHIFT))&SIM_SDID_FAMILYID_MASK)
/* SCGC4 Bit Fields */
#define SIM_SCGC4_EWM		((uint32_t)0x2u)
#define SIM_SCGC4_I2C0		((uint32_t)0x40u)
#define SIM_SCGC4_I2C1		((uint32_t)0x80u)
#define SIM_SCGC4_UART0		((uint32_t)0x400u)
#define SIM_SCGC4_UART1		((uint32_t)0x800u)
#define SIM_SCGC4_UART2		((uint32_t)0x1000u)
#define SIM_SCGC4_USBOTG		((uint32_t)0x40000u)
#define SIM_SCGC4_CMP		((uint32_t)0x80000u)
#define SIM_SCGC4_VREF		((uint32_t)0x100000u)
/* SCGC5 Bit Fields */
#define SIM_SCGC5_LPTMR		((uint32_t)0x1u)
#define SIM_SCGC5_PORTA		((uint32_t)0x200u)
#define SIM_SCGC5_PORTB		((uint32_t)0x400u)
#define SIM_SCGC5_PORTC		((uint32_t)0x800u)
#define SIM_SCGC5_PORTD		((uint32_t)0x1000u)
#define SIM_SCGC5_PORTE		((uint32_t)0x2000u)
/* SCGC6 Bit Fields */
#define SIM_SCGC6_FTF		((uint32_t)0x1u)
#define SIM_SCGC6_DMAMUX		((uint32_t)0x2u)
#define SIM_SCGC6_FTM3		((uint32_t)0x40u)
#define SIM_SCGC6_ADC1		((uint32_t)0x80u)
#define SIM_SCGC6_DAC1		((uint32_t)0x100u)
#define SIM_SCGC6_RNGA		((uint32_t)0x200u)
#define SIM_SCGC6_LPUART0		((uint32_t)0x400u)
#define SIM_SCGC6_SPI0		((uint32_t)0x1000u)
#define SIM_SCGC6_SPI1		((uint32_t)0x2000u)
#define SIM_SCGC6_I2S		((uint32_t)0x8000u)
#define SIM_SCGC6_CRC		((uint32_t)0x40000u)
#define SIM_SCGC6_PDB		((uint32_t)0x400000u)
#define SIM_SCGC6_PIT		((uint32_t)0x800000u)
#define SIM_SCGC6_FTM0		((uint32_t)0x1000000u)
#define SIM_SCGC6_FTM1		((uint32_t)0x2000000u)
#define SIM_SCGC6_FTM2		((uint32_t)0x4000000u)
#define SIM_SCGC6_ADC0		((uint32_t)0x8000000u)
#define SIM_SCGC6_RTC		((uint32_t)0x20000000u)
#define SIM_SCGC6_DAC0		((uint32_t)0x80000000u)
/* SCGC7 Bit Fields */
#define SIM_SCGC7_FLEXBUS		((uint32_t)0x1u)
#define SIM_SCGC7_DMA		((uint32_t)0x2u)
/* CLKDIV1 Bit Fields */
#define SIM_CLKDIV1_OUTDIV4_MASK                 0xF0000u
#define SIM_CLKDIV1_OUTDIV4_SHIFT                16
#define SIM_CLKDIV1_OUTDIV4_WIDTH                4
#define SIM_CLKDIV1_OUTDIV4(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV1_OUTDIV4_SHIFT))&SIM_CLKDIV1_OUTDIV4_MASK)
#define SIM_CLKDIV1_OUTDIV3_MASK                 0xF00000u
#define SIM_CLKDIV1_OUTDIV3_SHIFT                20
#define SIM_CLKDIV1_OUTDIV3_WIDTH                4
#define SIM_CLKDIV1_OUTDIV3(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV1_OUTDIV3_SHIFT))&SIM_CLKDIV1_OUTDIV3_MASK)
#define SIM_CLKDIV1_OUTDIV2_MASK                 0xF000000u
#define SIM_CLKDIV1_OUTDIV2_SHIFT                24
#define SIM_CLKDIV1_OUTDIV2_WIDTH                4
#define SIM_CLKDIV1_OUTDIV2(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV1_OUTDIV2_SHIFT))&SIM_CLKDIV1_OUTDIV2_MASK)
#define SIM_CLKDIV1_OUTDIV1_MASK                 0xF0000000u
#define SIM_CLKDIV1_OUTDIV1_SHIFT                28
#define SIM_CLKDIV1_OUTDIV1_WIDTH                4
#define SIM_CLKDIV1_OUTDIV1(x)                   (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV1_OUTDIV1_SHIFT))&SIM_CLKDIV1_OUTDIV1_MASK)
/* CLKDIV2 Bit Fields */
#define SIM_CLKDIV2_USBFRAC		((uint32_t)0x1u)
#define SIM_CLKDIV2_USBDIV_MASK                  0xEu
#define SIM_CLKDIV2_USBDIV_SHIFT                 1
#define SIM_CLKDIV2_USBDIV_WIDTH                 3
#define SIM_CLKDIV2_USBDIV(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_CLKDIV2_USBDIV_SHIFT))&SIM_CLKDIV2_USBDIV_MASK)
/* FCFG1 Bit Fields */
#define SIM_FCFG1_FLASHDIS		((uint32_t)0x1u)
#define SIM_FCFG1_FLASHDOZE		((uint32_t)0x2u)
#define SIM_FCFG1_PFSIZE_MASK                    0xF000000u
#define SIM_FCFG1_PFSIZE_SHIFT                   24
#define SIM_FCFG1_PFSIZE_WIDTH                   4
#define SIM_FCFG1_PFSIZE(x)                      (((uint32_t)(((uint32_t)(x))<<SIM_FCFG1_PFSIZE_SHIFT))&SIM_FCFG1_PFSIZE_MASK)
/* FCFG2 Bit Fields */
#define SIM_FCFG2_MAXADDR1_MASK                  0x7F0000u
#define SIM_FCFG2_MAXADDR1_SHIFT                 16
#define SIM_FCFG2_MAXADDR1_WIDTH                 7
#define SIM_FCFG2_MAXADDR1(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_FCFG2_MAXADDR1_SHIFT))&SIM_FCFG2_MAXADDR1_MASK)
#define SIM_FCFG2_MAXADDR0_MASK                  0x7F000000u
#define SIM_FCFG2_MAXADDR0_SHIFT                 24
#define SIM_FCFG2_MAXADDR0_WIDTH                 7
#define SIM_FCFG2_MAXADDR0(x)                    (((uint32_t)(((uint32_t)(x))<<SIM_FCFG2_MAXADDR0_SHIFT))&SIM_FCFG2_MAXADDR0_MASK)
/* UIDH Bit Fields */
#define SIM_UIDH_UID_MASK                        0xFFFFFFFFu
#define SIM_UIDH_UID_SHIFT                       0
#define SIM_UIDH_UID_WIDTH                       32
#define SIM_UIDH_UID(x)                          (((uint32_t)(((uint32_t)(x))<<SIM_UIDH_UID_SHIFT))&SIM_UIDH_UID_MASK)
/* UIDMH Bit Fields */
#define SIM_UIDMH_UID_MASK                       0xFFFFFFFFu
#define SIM_UIDMH_UID_SHIFT                      0
#define SIM_UIDMH_UID_WIDTH                      32
#define SIM_UIDMH_UID(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_UIDMH_UID_SHIFT))&SIM_UIDMH_UID_MASK)
/* UIDML Bit Fields */
#define SIM_UIDML_UID_MASK                       0xFFFFFFFFu
#define SIM_UIDML_UID_SHIFT                      0
#define SIM_UIDML_UID_WIDTH                      32
#define SIM_UIDML_UID(x)                         (((uint32_t)(((uint32_t)(x))<<SIM_UIDML_UID_SHIFT))&SIM_UIDML_UID_MASK)
/* UIDL Bit Fields */
#define SIM_UIDL_UID_MASK                        0xFFFFFFFFu
#define SIM_UIDL_UID_SHIFT                       0
#define SIM_UIDL_UID_WIDTH                       32
#define SIM_UIDL_UID(x)                          (((uint32_t)(((uint32_t)(x))<<SIM_UIDL_UID_SHIFT))&SIM_UIDL_UID_MASK)

/*!
 * @}
 */ /* end of group SIM_Register_Masks */


/* SIM - Peripheral instance base addresses */
/** Peripheral SIM base address */
#define SIM_BASE                                 (0x40047000u)
/** Peripheral SIM base pointer */
#define SIM                                      ((SIM_TypeDef *)SIM_BASE)
#define SIM_BASE_PTR                             (SIM)
/** Array initializer of SIM peripheral base addresses */
#define SIM_BASE_ADDRS                           { SIM_BASE }
/** Array initializer of SIM peripheral base pointers */
#define SIM_BASE_PTRS                            { SIM }

/* ----------------------------------------------------------------------------
   -- SIM - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SIM_Register_Accessor_Macros SIM - Register accessor macros
 * @{
 */


/* SIM - Register instance definitions */
/* SIM */
#define SIM_SOPT1                                SIM_SOPT1_REG(SIM)
#define SIM_SOPT1CFG                             SIM_SOPT1CFG_REG(SIM)
#define SIM_SOPT2                                SIM_SOPT2_REG(SIM)
#define SIM_SOPT4                                SIM_SOPT4_REG(SIM)
#define SIM_SOPT5                                SIM_SOPT5_REG(SIM)
#define SIM_SOPT7                                SIM_SOPT7_REG(SIM)
#define SIM_SOPT8                                SIM_SOPT8_REG(SIM)
#define SIM_SDID                                 SIM_SDID_REG(SIM)
#define SIM_SCGC4                                SIM_SCGC4_REG(SIM)
#define SIM_SCGC5                                SIM_SCGC5_REG(SIM)
#define SIM_SCGC6                                SIM_SCGC6_REG(SIM)
#define SIM_SCGC7                                SIM_SCGC7_REG(SIM)
#define SIM_CLKDIV1                              SIM_CLKDIV1_REG(SIM)
#define SIM_CLKDIV2                              SIM_CLKDIV2_REG(SIM)
#define SIM_FCFG1                                SIM_FCFG1_REG(SIM)
#define SIM_FCFG2                                SIM_FCFG2_REG(SIM)
#define SIM_UIDH                                 SIM_UIDH_REG(SIM)
#define SIM_UIDMH                                SIM_UIDMH_REG(SIM)
#define SIM_UIDML                                SIM_UIDML_REG(SIM)
#define SIM_UIDL                                 SIM_UIDL_REG(SIM)

/*!
 * @}
 */ /* end of group SIM_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group SIM_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SMC Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Peripheral_Access_Layer SMC Peripheral Access Layer
 * @{
 */

/** SMC - Register Layout Typedef */
typedef struct {
  __IO uint8_t PMPROT;                             /**< Power Mode Protection register, offset: 0x0 */
  __IO uint8_t PMCTRL;                             /**< Power Mode Control register, offset: 0x1 */
  __IO uint8_t STOPCTRL;                           /**< Stop Control Register, offset: 0x2 */
  __I  uint8_t PMSTAT;                             /**< Power Mode Status register, offset: 0x3 */
} SMC_TypeDef, *SMC_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Register_Accessor_Macros SMC - Register accessor macros
 * @{
 */


/* SMC - Register accessors */
#define SMC_PMPROT_REG(base)                     ((base)->PMPROT)
#define SMC_PMCTRL_REG(base)                     ((base)->PMCTRL)
#define SMC_STOPCTRL_REG(base)                   ((base)->STOPCTRL)
#define SMC_PMSTAT_REG(base)                     ((base)->PMSTAT)

/*!
 * @}
 */ /* end of group SMC_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- SMC Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Register_Masks SMC Register Masks
 * @{
 */

/* PMPROT Bit Fields */
#define SMC_PMPROT_AVLLS		((uint8_t)0x2u)
#define SMC_PMPROT_ALLS		((uint8_t)0x8u)
#define SMC_PMPROT_AVLP		((uint8_t)0x20u)
#define SMC_PMPROT_AHSRUN		((uint8_t)0x80u)
/* PMCTRL Bit Fields */
#define SMC_PMCTRL_STOPM_MASK                    0x7u
#define SMC_PMCTRL_STOPM_SHIFT                   0
#define SMC_PMCTRL_STOPM_WIDTH                   3
#define SMC_PMCTRL_STOPM(x)                      (((uint8_t)(((uint8_t)(x))<<SMC_PMCTRL_STOPM_SHIFT))&SMC_PMCTRL_STOPM_MASK)
#define SMC_PMCTRL_STOPA		((uint8_t)0x8u)
#define SMC_PMCTRL_RUNM_MASK                     0x60u
#define SMC_PMCTRL_RUNM_SHIFT                    5
#define SMC_PMCTRL_RUNM_WIDTH                    2
#define SMC_PMCTRL_RUNM(x)                       (((uint8_t)(((uint8_t)(x))<<SMC_PMCTRL_RUNM_SHIFT))&SMC_PMCTRL_RUNM_MASK)
/* STOPCTRL Bit Fields */
#define SMC_STOPCTRL_LLSM_MASK                   0x7u
#define SMC_STOPCTRL_LLSM_SHIFT                  0
#define SMC_STOPCTRL_LLSM_WIDTH                  3
#define SMC_STOPCTRL_LLSM(x)                     (((uint8_t)(((uint8_t)(x))<<SMC_STOPCTRL_LLSM_SHIFT))&SMC_STOPCTRL_LLSM_MASK)
#define SMC_STOPCTRL_PORPO		((uint8_t)0x20u)
#define SMC_STOPCTRL_PSTOPO_MASK                 0xC0u
#define SMC_STOPCTRL_PSTOPO_SHIFT                6
#define SMC_STOPCTRL_PSTOPO_WIDTH                2
#define SMC_STOPCTRL_PSTOPO(x)                   (((uint8_t)(((uint8_t)(x))<<SMC_STOPCTRL_PSTOPO_SHIFT))&SMC_STOPCTRL_PSTOPO_MASK)
/* PMSTAT Bit Fields */
#define SMC_PMSTAT_PMSTAT_MASK                   0xFFu
#define SMC_PMSTAT_PMSTAT_SHIFT                  0
#define SMC_PMSTAT_PMSTAT_WIDTH                  8
#define SMC_PMSTAT_PMSTAT(x)                     (((uint8_t)(((uint8_t)(x))<<SMC_PMSTAT_PMSTAT_SHIFT))&SMC_PMSTAT_PMSTAT_MASK)

/*!
 * @}
 */ /* end of group SMC_Register_Masks */


/* SMC - Peripheral instance base addresses */
/** Peripheral SMC base address */
#define SMC_BASE                                 (0x4007E000u)
/** Peripheral SMC base pointer */
#define SMC                                      ((SMC_TypeDef *)SMC_BASE)
#define SMC_BASE_PTR                             (SMC)
/** Array initializer of SMC peripheral base addresses */
#define SMC_BASE_ADDRS                           { SMC_BASE }
/** Array initializer of SMC peripheral base pointers */
#define SMC_BASE_PTRS                            { SMC }

/* ----------------------------------------------------------------------------
   -- SMC - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SMC_Register_Accessor_Macros SMC - Register accessor macros
 * @{
 */


/* SMC - Register instance definitions */
/* SMC */
#define SMC_PMPROT                               SMC_PMPROT_REG(SMC)
#define SMC_PMCTRL                               SMC_PMCTRL_REG(SMC)
#define SMC_STOPCTRL                             SMC_STOPCTRL_REG(SMC)
#define SMC_PMSTAT                               SMC_PMSTAT_REG(SMC)

/*!
 * @}
 */ /* end of group SMC_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group SMC_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- SPI Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Peripheral_Access_Layer SPI Peripheral Access Layer
 * @{
 */

/** SPI - Register Layout Typedef */
typedef struct {
  __IO uint32_t MCR;                               /**< Module Configuration Register, offset: 0x0 */
       uint8_t RESERVED_0[4];
  __IO uint32_t TCR;                               /**< Transfer Count Register, offset: 0x8 */
  union {                                          /* offset: 0xC */
    __IO uint32_t CTAR[2];                           /**< Clock and Transfer Attributes Register (In Master Mode), array offset: 0xC, array step: 0x4 */
    __IO uint32_t CTAR_SLAVE[1];                     /**< Clock and Transfer Attributes Register (In Slave Mode), array offset: 0xC, array step: 0x4 */
  };
       uint8_t RESERVED_1[24];
  __IO uint32_t SR;                                /**< Status Register, offset: 0x2C */
  __IO uint32_t RSER;                              /**< DMA/Interrupt Request Select and Enable Register, offset: 0x30 */
  union {                                          /* offset: 0x34 */
    __IO uint32_t PUSHR;                             /**< PUSH TX FIFO Register In Master Mode, offset: 0x34 */
    __IO uint32_t PUSHR_SLAVE;                       /**< PUSH TX FIFO Register In Slave Mode, offset: 0x34 */
  };
  __I  uint32_t POPR;                              /**< POP RX FIFO Register, offset: 0x38 */
  __I  uint32_t TXFR0;                             /**< Transmit FIFO Registers, offset: 0x3C */
  __I  uint32_t TXFR1;                             /**< Transmit FIFO Registers, offset: 0x40 */
  __I  uint32_t TXFR2;                             /**< Transmit FIFO Registers, offset: 0x44 */
  __I  uint32_t TXFR3;                             /**< Transmit FIFO Registers, offset: 0x48 */
       uint8_t RESERVED_2[48];
  __I  uint32_t RXFR0;                             /**< Receive FIFO Registers, offset: 0x7C */
  __I  uint32_t RXFR1;                             /**< Receive FIFO Registers, offset: 0x80 */
  __I  uint32_t RXFR2;                             /**< Receive FIFO Registers, offset: 0x84 */
  __I  uint32_t RXFR3;                             /**< Receive FIFO Registers, offset: 0x88 */
} SPI_TypeDef, *SPI_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- SPI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Accessor_Macros SPI - Register accessor macros
 * @{
 */


/* SPI - Register accessors */
#define SPI_MCR_REG(base)                        ((base)->MCR)
#define SPI_TCR_REG(base)                        ((base)->TCR)
#define SPI_CTAR_REG(base,index2)                ((base)->CTAR[index2])
#define SPI_CTAR_COUNT                           2
#define SPI_CTAR_SLAVE_REG(base,index2)          ((base)->CTAR_SLAVE[index2])
#define SPI_CTAR_SLAVE_COUNT                     1
#define SPI_SR_REG(base)                         ((base)->SR)
#define SPI_RSER_REG(base)                       ((base)->RSER)
#define SPI_PUSHR_REG(base)                      ((base)->PUSHR)
#define SPI_PUSHR_SLAVE_REG(base)                ((base)->PUSHR_SLAVE)
#define SPI_POPR_REG(base)                       ((base)->POPR)
#define SPI_TXFR0_REG(base)                      ((base)->TXFR0)
#define SPI_TXFR1_REG(base)                      ((base)->TXFR1)
#define SPI_TXFR2_REG(base)                      ((base)->TXFR2)
#define SPI_TXFR3_REG(base)                      ((base)->TXFR3)
#define SPI_RXFR0_REG(base)                      ((base)->RXFR0)
#define SPI_RXFR1_REG(base)                      ((base)->RXFR1)
#define SPI_RXFR2_REG(base)                      ((base)->RXFR2)
#define SPI_RXFR3_REG(base)                      ((base)->RXFR3)

/*!
 * @}
 */ /* end of group SPI_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- SPI Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Masks SPI Register Masks
 * @{
 */

/* MCR Bit Fields */
#define SPI_MCR_HALT		((uint32_t)0x1u)
#define SPI_MCR_SMPL_PT_MASK                     0x300u
#define SPI_MCR_SMPL_PT_SHIFT                    8
#define SPI_MCR_SMPL_PT_WIDTH                    2
#define SPI_MCR_SMPL_PT(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_MCR_SMPL_PT_SHIFT))&SPI_MCR_SMPL_PT_MASK)
#define SPI_MCR_CLR_RXF		((uint32_t)0x400u)
#define SPI_MCR_CLR_TXF		((uint32_t)0x800u)
#define SPI_MCR_DIS_RXF		((uint32_t)0x1000u)
#define SPI_MCR_DIS_TXF		((uint32_t)0x2000u)
#define SPI_MCR_MDIS		((uint32_t)0x4000u)
#define SPI_MCR_DOZE		((uint32_t)0x8000u)
#define SPI_MCR_PCSIS_MASK                       0x3F0000u
#define SPI_MCR_PCSIS_SHIFT                      16
#define SPI_MCR_PCSIS_WIDTH                      6
#define SPI_MCR_PCSIS(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_MCR_PCSIS_SHIFT))&SPI_MCR_PCSIS_MASK)
#define SPI_MCR_ROOE		((uint32_t)0x1000000u)
#define SPI_MCR_PCSSE		((uint32_t)0x2000000u)
#define SPI_MCR_MTFE		((uint32_t)0x4000000u)
#define SPI_MCR_FRZ		((uint32_t)0x8000000u)
#define SPI_MCR_DCONF_MASK                       0x30000000u
#define SPI_MCR_DCONF_SHIFT                      28
#define SPI_MCR_DCONF_WIDTH                      2
#define SPI_MCR_DCONF(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_MCR_DCONF_SHIFT))&SPI_MCR_DCONF_MASK)
#define SPI_MCR_CONT_SCKE		((uint32_t)0x40000000u)
#define SPI_MCR_MSTR		((uint32_t)0x80000000u)
/* TCR Bit Fields */
#define SPI_TCR_SPI_TCNT_MASK                    0xFFFF0000u
#define SPI_TCR_SPI_TCNT_SHIFT                   16
#define SPI_TCR_SPI_TCNT_WIDTH                   16
#define SPI_TCR_SPI_TCNT(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TCR_SPI_TCNT_SHIFT))&SPI_TCR_SPI_TCNT_MASK)
/* CTAR Bit Fields */
#define SPI_CTAR_BR_MASK                         0xFu
#define SPI_CTAR_BR_SHIFT                        0
#define SPI_CTAR_BR_WIDTH                        4
#define SPI_CTAR_BR(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_BR_SHIFT))&SPI_CTAR_BR_MASK)
#define SPI_CTAR_DT_MASK                         0xF0u
#define SPI_CTAR_DT_SHIFT                        4
#define SPI_CTAR_DT_WIDTH                        4
#define SPI_CTAR_DT(x)                           (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_DT_SHIFT))&SPI_CTAR_DT_MASK)
#define SPI_CTAR_ASC_MASK                        0xF00u
#define SPI_CTAR_ASC_SHIFT                       8
#define SPI_CTAR_ASC_WIDTH                       4
#define SPI_CTAR_ASC(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_ASC_SHIFT))&SPI_CTAR_ASC_MASK)
#define SPI_CTAR_CSSCK_MASK                      0xF000u
#define SPI_CTAR_CSSCK_SHIFT                     12
#define SPI_CTAR_CSSCK_WIDTH                     4
#define SPI_CTAR_CSSCK(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_CSSCK_SHIFT))&SPI_CTAR_CSSCK_MASK)
#define SPI_CTAR_PBR_MASK                        0x30000u
#define SPI_CTAR_PBR_SHIFT                       16
#define SPI_CTAR_PBR_WIDTH                       2
#define SPI_CTAR_PBR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PBR_SHIFT))&SPI_CTAR_PBR_MASK)
#define SPI_CTAR_PDT_MASK                        0xC0000u
#define SPI_CTAR_PDT_SHIFT                       18
#define SPI_CTAR_PDT_WIDTH                       2
#define SPI_CTAR_PDT(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PDT_SHIFT))&SPI_CTAR_PDT_MASK)
#define SPI_CTAR_PASC_MASK                       0x300000u
#define SPI_CTAR_PASC_SHIFT                      20
#define SPI_CTAR_PASC_WIDTH                      2
#define SPI_CTAR_PASC(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PASC_SHIFT))&SPI_CTAR_PASC_MASK)
#define SPI_CTAR_PCSSCK_MASK                     0xC00000u
#define SPI_CTAR_PCSSCK_SHIFT                    22
#define SPI_CTAR_PCSSCK_WIDTH                    2
#define SPI_CTAR_PCSSCK(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_PCSSCK_SHIFT))&SPI_CTAR_PCSSCK_MASK)
#define SPI_CTAR_LSBFE		((uint32_t)0x1000000u)
#define SPI_CTAR_CPHA		((uint32_t)0x2000000u)
#define SPI_CTAR_CPOL		((uint32_t)0x4000000u)
#define SPI_CTAR_FMSZ_MASK                       0x78000000u
#define SPI_CTAR_FMSZ_SHIFT                      27
#define SPI_CTAR_FMSZ_WIDTH                      4
#define SPI_CTAR_FMSZ(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_FMSZ_SHIFT))&SPI_CTAR_FMSZ_MASK)
#define SPI_CTAR_DBR		((uint32_t)0x80000000u)
/* CTAR_SLAVE Bit Fields */
#define SPI_CTAR_SLAVE_CPHA		((uint32_t)0x2000000u)
#define SPI_CTAR_SLAVE_CPOL		((uint32_t)0x4000000u)
#define SPI_CTAR_SLAVE_FMSZ_MASK                 0xF8000000u
#define SPI_CTAR_SLAVE_FMSZ_SHIFT                27
#define SPI_CTAR_SLAVE_FMSZ_WIDTH                5
#define SPI_CTAR_SLAVE_FMSZ(x)                   (((uint32_t)(((uint32_t)(x))<<SPI_CTAR_SLAVE_FMSZ_SHIFT))&SPI_CTAR_SLAVE_FMSZ_MASK)
/* SR Bit Fields */
#define SPI_SR_POPNXTPTR_MASK                    0xFu
#define SPI_SR_POPNXTPTR_SHIFT                   0
#define SPI_SR_POPNXTPTR_WIDTH                   4
#define SPI_SR_POPNXTPTR(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_SR_POPNXTPTR_SHIFT))&SPI_SR_POPNXTPTR_MASK)
#define SPI_SR_RXCTR_MASK                        0xF0u
#define SPI_SR_RXCTR_SHIFT                       4
#define SPI_SR_RXCTR_WIDTH                       4
#define SPI_SR_RXCTR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_SR_RXCTR_SHIFT))&SPI_SR_RXCTR_MASK)
#define SPI_SR_TXNXTPTR_MASK                     0xF00u
#define SPI_SR_TXNXTPTR_SHIFT                    8
#define SPI_SR_TXNXTPTR_WIDTH                    4
#define SPI_SR_TXNXTPTR(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_SR_TXNXTPTR_SHIFT))&SPI_SR_TXNXTPTR_MASK)
#define SPI_SR_TXCTR_MASK                        0xF000u
#define SPI_SR_TXCTR_SHIFT                       12
#define SPI_SR_TXCTR_WIDTH                       4
#define SPI_SR_TXCTR(x)                          (((uint32_t)(((uint32_t)(x))<<SPI_SR_TXCTR_SHIFT))&SPI_SR_TXCTR_MASK)
#define SPI_SR_RFDF		((uint32_t)0x20000u)
#define SPI_SR_RFOF		((uint32_t)0x80000u)
#define SPI_SR_TFFF		((uint32_t)0x2000000u)
#define SPI_SR_TFUF		((uint32_t)0x8000000u)
#define SPI_SR_EOQF		((uint32_t)0x10000000u)
#define SPI_SR_TXRXS		((uint32_t)0x40000000u)
#define SPI_SR_TCF		((uint32_t)0x80000000u)
/* RSER Bit Fields */
#define SPI_RSER_RFDF_DIRS		((uint32_t)0x10000u)
#define SPI_RSER_RFDF_RE		((uint32_t)0x20000u)
#define SPI_RSER_RFOF_RE		((uint32_t)0x80000u)
#define SPI_RSER_TFFF_DIRS		((uint32_t)0x1000000u)
#define SPI_RSER_TFFF_RE		((uint32_t)0x2000000u)
#define SPI_RSER_TFUF_RE		((uint32_t)0x8000000u)
#define SPI_RSER_EOQF_RE		((uint32_t)0x10000000u)
#define SPI_RSER_TCF_RE		((uint32_t)0x80000000u)
/* PUSHR Bit Fields */
#define SPI_PUSHR_TXDATA_MASK                    0xFFFFu
#define SPI_PUSHR_TXDATA_SHIFT                   0
#define SPI_PUSHR_TXDATA_WIDTH                   16
#define SPI_PUSHR_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_TXDATA_SHIFT))&SPI_PUSHR_TXDATA_MASK)
#define SPI_PUSHR_PCS_MASK                       0x3F0000u
#define SPI_PUSHR_PCS_SHIFT                      16
#define SPI_PUSHR_PCS_WIDTH                      6
#define SPI_PUSHR_PCS(x)                         (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_PCS_SHIFT))&SPI_PUSHR_PCS_MASK)
#define SPI_PUSHR_CTCNT		((uint32_t)0x4000000u)
#define SPI_PUSHR_EOQ		((uint32_t)0x8000000u)
#define SPI_PUSHR_CTAS_MASK                      0x70000000u
#define SPI_PUSHR_CTAS_SHIFT                     28
#define SPI_PUSHR_CTAS_WIDTH                     3
#define SPI_PUSHR_CTAS(x)                        (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_CTAS_SHIFT))&SPI_PUSHR_CTAS_MASK)
#define SPI_PUSHR_CONT		((uint32_t)0x80000000u)
/* PUSHR_SLAVE Bit Fields */
#define SPI_PUSHR_SLAVE_TXDATA_MASK              0xFFFFFFFFu
#define SPI_PUSHR_SLAVE_TXDATA_SHIFT             0
#define SPI_PUSHR_SLAVE_TXDATA_WIDTH             32
#define SPI_PUSHR_SLAVE_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_PUSHR_SLAVE_TXDATA_SHIFT))&SPI_PUSHR_SLAVE_TXDATA_MASK)
/* POPR Bit Fields */
#define SPI_POPR_RXDATA_MASK                     0xFFFFFFFFu
#define SPI_POPR_RXDATA_SHIFT                    0
#define SPI_POPR_RXDATA_WIDTH                    32
#define SPI_POPR_RXDATA(x)                       (((uint32_t)(((uint32_t)(x))<<SPI_POPR_RXDATA_SHIFT))&SPI_POPR_RXDATA_MASK)
/* TXFR0 Bit Fields */
#define SPI_TXFR0_TXDATA_MASK                    0xFFFFu
#define SPI_TXFR0_TXDATA_SHIFT                   0
#define SPI_TXFR0_TXDATA_WIDTH                   16
#define SPI_TXFR0_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR0_TXDATA_SHIFT))&SPI_TXFR0_TXDATA_MASK)
#define SPI_TXFR0_TXCMD_TXDATA_MASK              0xFFFF0000u
#define SPI_TXFR0_TXCMD_TXDATA_SHIFT             16
#define SPI_TXFR0_TXCMD_TXDATA_WIDTH             16
#define SPI_TXFR0_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR0_TXCMD_TXDATA_SHIFT))&SPI_TXFR0_TXCMD_TXDATA_MASK)
/* TXFR1 Bit Fields */
#define SPI_TXFR1_TXDATA_MASK                    0xFFFFu
#define SPI_TXFR1_TXDATA_SHIFT                   0
#define SPI_TXFR1_TXDATA_WIDTH                   16
#define SPI_TXFR1_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR1_TXDATA_SHIFT))&SPI_TXFR1_TXDATA_MASK)
#define SPI_TXFR1_TXCMD_TXDATA_MASK              0xFFFF0000u
#define SPI_TXFR1_TXCMD_TXDATA_SHIFT             16
#define SPI_TXFR1_TXCMD_TXDATA_WIDTH             16
#define SPI_TXFR1_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR1_TXCMD_TXDATA_SHIFT))&SPI_TXFR1_TXCMD_TXDATA_MASK)
/* TXFR2 Bit Fields */
#define SPI_TXFR2_TXDATA_MASK                    0xFFFFu
#define SPI_TXFR2_TXDATA_SHIFT                   0
#define SPI_TXFR2_TXDATA_WIDTH                   16
#define SPI_TXFR2_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR2_TXDATA_SHIFT))&SPI_TXFR2_TXDATA_MASK)
#define SPI_TXFR2_TXCMD_TXDATA_MASK              0xFFFF0000u
#define SPI_TXFR2_TXCMD_TXDATA_SHIFT             16
#define SPI_TXFR2_TXCMD_TXDATA_WIDTH             16
#define SPI_TXFR2_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR2_TXCMD_TXDATA_SHIFT))&SPI_TXFR2_TXCMD_TXDATA_MASK)
/* TXFR3 Bit Fields */
#define SPI_TXFR3_TXDATA_MASK                    0xFFFFu
#define SPI_TXFR3_TXDATA_SHIFT                   0
#define SPI_TXFR3_TXDATA_WIDTH                   16
#define SPI_TXFR3_TXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_TXFR3_TXDATA_SHIFT))&SPI_TXFR3_TXDATA_MASK)
#define SPI_TXFR3_TXCMD_TXDATA_MASK              0xFFFF0000u
#define SPI_TXFR3_TXCMD_TXDATA_SHIFT             16
#define SPI_TXFR3_TXCMD_TXDATA_WIDTH             16
#define SPI_TXFR3_TXCMD_TXDATA(x)                (((uint32_t)(((uint32_t)(x))<<SPI_TXFR3_TXCMD_TXDATA_SHIFT))&SPI_TXFR3_TXCMD_TXDATA_MASK)
/* RXFR0 Bit Fields */
#define SPI_RXFR0_RXDATA_MASK                    0xFFFFFFFFu
#define SPI_RXFR0_RXDATA_SHIFT                   0
#define SPI_RXFR0_RXDATA_WIDTH                   32
#define SPI_RXFR0_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR0_RXDATA_SHIFT))&SPI_RXFR0_RXDATA_MASK)
/* RXFR1 Bit Fields */
#define SPI_RXFR1_RXDATA_MASK                    0xFFFFFFFFu
#define SPI_RXFR1_RXDATA_SHIFT                   0
#define SPI_RXFR1_RXDATA_WIDTH                   32
#define SPI_RXFR1_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR1_RXDATA_SHIFT))&SPI_RXFR1_RXDATA_MASK)
/* RXFR2 Bit Fields */
#define SPI_RXFR2_RXDATA_MASK                    0xFFFFFFFFu
#define SPI_RXFR2_RXDATA_SHIFT                   0
#define SPI_RXFR2_RXDATA_WIDTH                   32
#define SPI_RXFR2_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR2_RXDATA_SHIFT))&SPI_RXFR2_RXDATA_MASK)
/* RXFR3 Bit Fields */
#define SPI_RXFR3_RXDATA_MASK                    0xFFFFFFFFu
#define SPI_RXFR3_RXDATA_SHIFT                   0
#define SPI_RXFR3_RXDATA_WIDTH                   32
#define SPI_RXFR3_RXDATA(x)                      (((uint32_t)(((uint32_t)(x))<<SPI_RXFR3_RXDATA_SHIFT))&SPI_RXFR3_RXDATA_MASK)

/*!
 * @}
 */ /* end of group SPI_Register_Masks */


/* SPI - Peripheral instance base addresses */
/** Peripheral SPI0 base address */
#define SPI0_BASE                                (0x4002C000u)
/** Peripheral SPI0 base pointer */
#define SPI0                                     ((SPI_TypeDef *)SPI0_BASE)
#define SPI0_BASE_PTR                            (SPI0)
/** Peripheral SPI1 base address */
#define SPI1_BASE                                (0x4002D000u)
/** Peripheral SPI1 base pointer */
#define SPI1                                     ((SPI_TypeDef *)SPI1_BASE)
#define SPI1_BASE_PTR                            (SPI1)
/** Array initializer of SPI peripheral base addresses */
#define SPI_BASE_ADDRS                           { SPI0_BASE, SPI1_BASE }
/** Array initializer of SPI peripheral base pointers */
#define SPI_BASE_PTRS                            { SPI0, SPI1 }
/** Interrupt vectors for the SPI peripheral type */
#define SPI_IRQS                                 { SPI0_IRQn, SPI1_IRQn }

/* ----------------------------------------------------------------------------
   -- SPI - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup SPI_Register_Accessor_Macros SPI - Register accessor macros
 * @{
 */


/* SPI - Register instance definitions */
/* SPI0 */
#define SPI0_MCR                                 SPI_MCR_REG(SPI0)
#define SPI0_TCR                                 SPI_TCR_REG(SPI0)
#define SPI0_CTAR0                               SPI_CTAR_REG(SPI0,0)
#define SPI0_CTAR0_SLAVE                         SPI_CTAR_SLAVE_REG(SPI0,0)
#define SPI0_CTAR1                               SPI_CTAR_REG(SPI0,1)
#define SPI0_SR                                  SPI_SR_REG(SPI0)
#define SPI0_RSER                                SPI_RSER_REG(SPI0)
#define SPI0_PUSHR                               SPI_PUSHR_REG(SPI0)
#define SPI0_PUSHR_SLAVE                         SPI_PUSHR_SLAVE_REG(SPI0)
#define SPI0_POPR                                SPI_POPR_REG(SPI0)
#define SPI0_TXFR0                               SPI_TXFR0_REG(SPI0)
#define SPI0_TXFR1                               SPI_TXFR1_REG(SPI0)
#define SPI0_TXFR2                               SPI_TXFR2_REG(SPI0)
#define SPI0_TXFR3                               SPI_TXFR3_REG(SPI0)
#define SPI0_RXFR0                               SPI_RXFR0_REG(SPI0)
#define SPI0_RXFR1                               SPI_RXFR1_REG(SPI0)
#define SPI0_RXFR2                               SPI_RXFR2_REG(SPI0)
#define SPI0_RXFR3                               SPI_RXFR3_REG(SPI0)
/* SPI1 */
#define SPI1_MCR                                 SPI_MCR_REG(SPI1)
#define SPI1_TCR                                 SPI_TCR_REG(SPI1)
#define SPI1_CTAR0                               SPI_CTAR_REG(SPI1,0)
#define SPI1_CTAR0_SLAVE                         SPI_CTAR_SLAVE_REG(SPI1,0)
#define SPI1_CTAR1                               SPI_CTAR_REG(SPI1,1)
#define SPI1_SR                                  SPI_SR_REG(SPI1)
#define SPI1_RSER                                SPI_RSER_REG(SPI1)
#define SPI1_PUSHR                               SPI_PUSHR_REG(SPI1)
#define SPI1_PUSHR_SLAVE                         SPI_PUSHR_SLAVE_REG(SPI1)
#define SPI1_POPR                                SPI_POPR_REG(SPI1)
#define SPI1_TXFR0                               SPI_TXFR0_REG(SPI1)
#define SPI1_TXFR1                               SPI_TXFR1_REG(SPI1)
#define SPI1_TXFR2                               SPI_TXFR2_REG(SPI1)
#define SPI1_TXFR3                               SPI_TXFR3_REG(SPI1)
#define SPI1_RXFR0                               SPI_RXFR0_REG(SPI1)
#define SPI1_RXFR1                               SPI_RXFR1_REG(SPI1)
#define SPI1_RXFR2                               SPI_RXFR2_REG(SPI1)
#define SPI1_RXFR3                               SPI_RXFR3_REG(SPI1)

/* SPI - Register array accessors */
#define SPI0_CTAR(index2)                        SPI_CTAR_REG(SPI0,index2)
#define SPI1_CTAR(index2)                        SPI_CTAR_REG(SPI1,index2)
#define SPI0_CTAR_SLAVE(index2)                  SPI_CTAR_SLAVE_REG(SPI0,index2)
#define SPI1_CTAR_SLAVE(index2)                  SPI_CTAR_SLAVE_REG(SPI1,index2)

/***********  Bits definition for SPIx_MCR register  *************/
#define SPIx_MCR_MSTR            ((uint32_t)0x80000000)      // Master/Slave Mode Select
#define SPIx_MCR_CONT_SCKE       ((uint32_t)0x40000000)      // Continuous SCK Enable
#define SPIx_MCR_DCONF(n)        (((n) & 3) << 28)           // DSPI Configuration
#define SPIx_MCR_FRZ             ((uint32_t)0x08000000)      // Freeze
#define SPIx_MCR_MTFE            ((uint32_t)0x04000000)      // Modified Timing Format Enable
#define SPIx_MCR_ROOE            ((uint32_t)0x01000000)      // Receive FIFO Overflow Overwrite Enable
#define SPIx_MCR_PCSIS(n)        (((n) & 0x1F) << 16)        // Peripheral Chip Select x Inactive State
#define SPIx_MCR_DOZE            ((uint32_t)0x00008000)      // Doze Enable
#define SPIx_MCR_MDIS            ((uint32_t)0x00004000)      // Module Disable
#define SPIx_MCR_DIS_TXF         ((uint32_t)0x00002000)      // Disable Transmit FIFO
#define SPIx_MCR_DIS_RXF         ((uint32_t)0x00001000)      // Disable Receive FIFO
#define SPIx_MCR_CLR_TXF         ((uint32_t)0x00000800)      // Clear the TX FIFO and counter
#define SPIx_MCR_CLR_RXF         ((uint32_t)0x00000400)      // Clear the RX FIFO and counter
#define SPIx_MCR_SMPL_PT(n)      (((n) & 3) << 8)            // Sample Point
#define SPIx_MCR_HALT            ((uint32_t)0x00000001)      // Halt

/***********  Bits definition for SPIx_TCR register  *************/
#define SPIx_TCR_TCNT(n)         (((n) & 0xffff) << 16)      // DSPI Transfer Count Register

/***********  Bits definition for SPIx_CTARn register  *************/
#define SPIx_CTARn_DBR            ((uint32_t)0x80000000)     // Double Baud Rate
#define SPIx_CTARn_FMSZ_SHIFT     27                         // Frame Size Shift
#define SPIx_CTARn_FMSZ_MASK      0xF                        // Frame Size Mask
#define SPIx_CTARn_FMSZ(n)        (((n) & 15) << 27)         // Frame Size (+1)
#define SPIx_CTARn_CPOL           ((uint32_t)0x04000000)     // Clock Polarity
#define SPIx_CTARn_CPHA           ((uint32_t)0x02000000)     // Clock Phase
#define SPIx_CTARn_LSBFE          ((uint32_t)0x01000000)     // LSB First
#define SPIx_CTARn_PCSSCK(n)      (((n) & 3) << 22)          // PCS to SCK Delay Prescaler
#define SPIx_CTARn_PASC(n)        (((n) & 3) << 20)          // After SCK Delay Prescaler
#define SPIx_CTARn_PDT(n)         (((n) & 3) << 18)          // Delay after Transfer Prescaler
#define SPIx_CTARn_PBR(n)         (((n) & 3) << 16)          // Baud Rate Prescaler
#define SPIx_CTARn_CSSCK(n)       (((n) & 15) << 12)         // PCS to SCK Delay Scaler
#define SPIx_CTARn_ASC(n)         (((n) & 15) << 8)          // After SCK Delay Scaler
#define SPIx_CTARn_DT(n)          (((n) & 15) << 4)          // Delay After Transfer Scaler
#define SPIx_CTARn_BR(n)          (((n) & 15) << 0)          // Baud Rate Scaler


/***********  Bits definition for SPIx_CTARn_SLAVE register  *************/
#define SPIx_CTARn_SLAVE_FMSZ(n)  (((n) & 15) << 27)         // Frame Size (+1)
#define SPIx_CTARn_SLAVE_CPOL     ((uint32_t)0x04000000)     // Clock Polarity
#define SPIx_CTARn_SLAVE_CPHA     ((uint32_t)0x02000000)     // Clock Phase

/***********  Bits definition for SPIx_SR register  *************/
#define SPIx_SR_TCF               ((uint32_t)0x80000000)     // Transfer Complete Flag
#define SPIx_SR_TXRXS             ((uint32_t)0x40000000)     // TX and RX Status
#define SPIx_SR_EOQF              ((uint32_t)0x10000000)     // End of Queue Flag
#define SPIx_SR_TFUF              ((uint32_t)0x08000000)     // Transmit FIFO Underflow Flag
#define SPIx_SR_TFFF              ((uint32_t)0x02000000)     // Transmit FIFO Fill Flag
#define SPIx_SR_RFOF              ((uint32_t)0x00080000)     // Receive FIFO Overflow Flag
#define SPIx_SR_RFDF              ((uint32_t)0x00020000)     // Receive FIFO Drain Flag
#define SPIx_SR_TXCTR             (((n) & 15) << 12)         // TX FIFO Counter
#define SPIx_SR_TXNXPTR           (((n) & 15) << 8)          // Transmit Next Pointer
#define SPIx_SR_RXCTR             (((n) & 15) << 4)          // RX FIFO Counter
#define SPIx_SR_POPNXTPTR         ((n) & 15)                 // POP Next Pointer

/***********  Bits definition for SPIx_SR register  *************/
#define SPIx_RSER_TCF_RE         ((uint32_t)0x80000000)      // Transmission Complete Request Enable
#define SPIx_RSER_EOQF_RE        ((uint32_t)0x10000000)      // DSPI Finished Request Request Enable
#define SPIx_RSER_TFUF_RE        ((uint32_t)0x08000000)      // Transmit FIFO Underflow Request Enable
#define SPIx_RSER_TFFF_RE        ((uint32_t)0x02000000)      // Transmit FIFO Fill Request Enable
#define SPIx_RSER_TFFF_DIRS      ((uint32_t)0x01000000)      // Transmit FIFO FIll Dma or Interrupt Request Select
#define SPIx_RSER_RFOF_RE        ((uint32_t)0x00080000)      // Receive FIFO Overflow Request Enable
#define SPIx_RSER_RFDF_RE        ((uint32_t)0x00020000)      // Receive FIFO Drain Request Enable
#define SPIx_RSER_RFDF_DIRS      ((uint32_t)0x00010000)      // Receive FIFO Drain DMA or Interrupt Request Select

/***********  Bits definition for SPIx_PUSHR register  *************/
#define SPIx_PUSHR_CONT          ((uint32_t)0x80000000)      // Continuous Peripheral Chip Select Enable
#define SPIx_PUSHR_CTAS(n)       (((n) & 7) << 28)           // Clock and Transfer Attributes Select
#define SPIx_PUSHR_EOQ           ((uint32_t)0x08000000)      // End Of Queue
#define SPIx_PUSHR_CTCNT         ((uint32_t)0x04000000)      // Clear Transfer Counter
#define SPIx_PUSHR_PCS(n)        (((n) & 31) << 16)          // Peripheral Chip Select
#define SPIx_PUSHR_TXDATA(n)     ((n) & 0xffff)              // Transmit Data

/***********  Bits definition for SPIx_PUSHR_SLAVE register  *************/
#define SPIx_PUSHR_SLAVE_TXDATA(n) (((n) & 0xffff) << 0)     // Transmit Data in slave mode

/***********  Bits definition for SPIx_POPR register  *************/
#define SPIx_POPR_RXDATA(n)      (((n) & 0xffff) << 16)      // Received Data

/***********  Bits definition for SPIx_TXFRn register  *************/
#define SPIx_TXFRn_TXCMD_TXDATA  (((n) & 0xffff) << 16)      // Transmit Command (in master mode)
#define SPIx_TXFRn_TXDATA(n)     (((n) & 0xffff) << 0)       // Transmit Data

/***********  Bits definition for SPIx_RXFRn register  *************/
#define SPIx_RXFRn_RXDATA(n)     (((n) & 0xffff) << 0)       // Receive Data

/*!
 * @}
 */ /* end of group SPI_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group SPI_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- UART Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UART_Peripheral_Access_Layer UART Peripheral Access Layer
 * @{
 */

/** UART - Register Layout Typedef */
typedef struct {
  __IO uint8_t BDH;                                /**< UART Baud Rate Registers: High, offset: 0x0 */
  __IO uint8_t BDL;                                /**< UART Baud Rate Registers: Low, offset: 0x1 */
  __IO uint8_t C1;                                 /**< UART Control Register 1, offset: 0x2 */
  __IO uint8_t C2;                                 /**< UART Control Register 2, offset: 0x3 */
  __I  uint8_t S1;                                 /**< UART Status Register 1, offset: 0x4 */
  __IO uint8_t S2;                                 /**< UART Status Register 2, offset: 0x5 */
  __IO uint8_t C3;                                 /**< UART Control Register 3, offset: 0x6 */
  __IO uint8_t D;                                  /**< UART Data Register, offset: 0x7 */
  __IO uint8_t MA1;                                /**< UART Match Address Registers 1, offset: 0x8 */
  __IO uint8_t MA2;                                /**< UART Match Address Registers 2, offset: 0x9 */
  __IO uint8_t C4;                                 /**< UART Control Register 4, offset: 0xA */
  __IO uint8_t C5;                                 /**< UART Control Register 5, offset: 0xB */
  __I  uint8_t ED;                                 /**< UART Extended Data Register, offset: 0xC */
  __IO uint8_t MODEM;                              /**< UART Modem Register, offset: 0xD */
  __IO uint8_t IR;                                 /**< UART Infrared Register, offset: 0xE */
       uint8_t RESERVED_0[1];
  __IO uint8_t PFIFO;                              /**< UART FIFO Parameters, offset: 0x10 */
  __IO uint8_t CFIFO;                              /**< UART FIFO Control Register, offset: 0x11 */
  __IO uint8_t SFIFO;                              /**< UART FIFO Status Register, offset: 0x12 */
  __IO uint8_t TWFIFO;                             /**< UART FIFO Transmit Watermark, offset: 0x13 */
  __I  uint8_t TCFIFO;                             /**< UART FIFO Transmit Count, offset: 0x14 */
  __IO uint8_t RWFIFO;                             /**< UART FIFO Receive Watermark, offset: 0x15 */
  __I  uint8_t RCFIFO;                             /**< UART FIFO Receive Count, offset: 0x16 */
       uint8_t RESERVED_1[1];
  __IO uint8_t C7816;                              /**< UART 7816 Control Register, offset: 0x18 */
  __IO uint8_t IE7816;                             /**< UART 7816 Interrupt Enable Register, offset: 0x19 */
  __IO uint8_t IS7816;                             /**< UART 7816 Interrupt Status Register, offset: 0x1A */
  __IO uint8_t WP7816;                             /**< UART 7816 Wait Parameter Register, offset: 0x1B */
  __IO uint8_t WN7816;                             /**< UART 7816 Wait N Register, offset: 0x1C */
  __IO uint8_t WF7816;                             /**< UART 7816 Wait FD Register, offset: 0x1D */
  __IO uint8_t ET7816;                             /**< UART 7816 Error Threshold Register, offset: 0x1E */
  __IO uint8_t TL7816;                             /**< UART 7816 Transmit Length Register, offset: 0x1F */
       uint8_t RESERVED_2[26];
  __IO uint8_t AP7816A_T0;                         /**< UART 7816 ATR Duration Timer Register A, offset: 0x3A */
  __IO uint8_t AP7816B_T0;                         /**< UART 7816 ATR Duration Timer Register B, offset: 0x3B */
  union {                                          /* offset: 0x3C */
    struct {                                         /* offset: 0x3C */
      __IO uint8_t WP7816A_T0;                         /**< UART 7816 Wait Parameter Register A, offset: 0x3C */
      __IO uint8_t WP7816B_T0;                         /**< UART 7816 Wait Parameter Register B, offset: 0x3D */
    } TYPE0;
    struct {                                         /* offset: 0x3C */
      __IO uint8_t WP7816A_T1;                         /**< UART 7816 Wait Parameter Register A, offset: 0x3C */
      __IO uint8_t WP7816B_T1;                         /**< UART 7816 Wait Parameter Register B, offset: 0x3D */
    } TYPE1;
  };
  __IO uint8_t WGP7816_T1;                         /**< UART 7816 Wait and Guard Parameter Register, offset: 0x3E */
  __IO uint8_t WP7816C_T1;                         /**< UART 7816 Wait Parameter Register C, offset: 0x3F */
} UART_TypeDef, *UART_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- UART - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UART_Register_Accessor_Macros UART - Register accessor macros
 * @{
 */


/* UART - Register accessors */
#define UART_BDH_REG(base)                       ((base)->BDH)
#define UART_BDL_REG(base)                       ((base)->BDL)
#define UART_C1_REG(base)                        ((base)->C1)
#define UART_C2_REG(base)                        ((base)->C2)
#define UART_S1_REG(base)                        ((base)->S1)
#define UART_S2_REG(base)                        ((base)->S2)
#define UART_C3_REG(base)                        ((base)->C3)
#define UART_D_REG(base)                         ((base)->D)
#define UART_MA1_REG(base)                       ((base)->MA1)
#define UART_MA2_REG(base)                       ((base)->MA2)
#define UART_C4_REG(base)                        ((base)->C4)
#define UART_C5_REG(base)                        ((base)->C5)
#define UART_ED_REG(base)                        ((base)->ED)
#define UART_MODEM_REG(base)                     ((base)->MODEM)
#define UART_IR_REG(base)                        ((base)->IR)
#define UART_PFIFO_REG(base)                     ((base)->PFIFO)
#define UART_CFIFO_REG(base)                     ((base)->CFIFO)
#define UART_SFIFO_REG(base)                     ((base)->SFIFO)
#define UART_TWFIFO_REG(base)                    ((base)->TWFIFO)
#define UART_TCFIFO_REG(base)                    ((base)->TCFIFO)
#define UART_RWFIFO_REG(base)                    ((base)->RWFIFO)
#define UART_RCFIFO_REG(base)                    ((base)->RCFIFO)
#define UART_C7816_REG(base)                     ((base)->C7816)
#define UART_IE7816_REG(base)                    ((base)->IE7816)
#define UART_IS7816_REG(base)                    ((base)->IS7816)
#define UART_WP7816_REG(base)                    ((base)->WP7816)
#define UART_WN7816_REG(base)                    ((base)->WN7816)
#define UART_WF7816_REG(base)                    ((base)->WF7816)
#define UART_ET7816_REG(base)                    ((base)->ET7816)
#define UART_TL7816_REG(base)                    ((base)->TL7816)
#define UART_AP7816A_T0_REG(base)                ((base)->AP7816A_T0)
#define UART_AP7816B_T0_REG(base)                ((base)->AP7816B_T0)
#define UART_WP7816A_T0_REG(base)                ((base)->TYPE0.WP7816A_T0)
#define UART_WP7816B_T0_REG(base)                ((base)->TYPE0.WP7816B_T0)
#define UART_WP7816A_T1_REG(base)                ((base)->TYPE1.WP7816A_T1)
#define UART_WP7816B_T1_REG(base)                ((base)->TYPE1.WP7816B_T1)
#define UART_WGP7816_T1_REG(base)                ((base)->WGP7816_T1)
#define UART_WP7816C_T1_REG(base)                ((base)->WP7816C_T1)

/*!
 * @}
 */ /* end of group UART_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- UART Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UART_Register_Masks UART Register Masks
 * @{
 */

/* BDH Bit Fields */
#define UART_BDH_SBR_MASK                        0x1Fu
#define UART_BDH_SBR_SHIFT                       0
#define UART_BDH_SBR_WIDTH                       5
#define UART_BDH_SBR(x)                          (((uint8_t)(((uint8_t)(x))<<UART_BDH_SBR_SHIFT))&UART_BDH_SBR_MASK)
#define UART_BDH_RXEDGIE		((uint8_t)0x40u)
#define UART_BDH_LBKDIE		((uint8_t)0x80u)
/* BDL Bit Fields */
#define UART_BDL_SBR_MASK                        0xFFu
#define UART_BDL_SBR_SHIFT                       0
#define UART_BDL_SBR_WIDTH                       8
#define UART_BDL_SBR(x)                          (((uint8_t)(((uint8_t)(x))<<UART_BDL_SBR_SHIFT))&UART_BDL_SBR_MASK)
/* C1 Bit Fields */
#define UART_C1_PT		((uint8_t)0x1u)
#define UART_C1_PE		((uint8_t)0x2u)
#define UART_C1_ILT		((uint8_t)0x4u)
#define UART_C1_WAKE		((uint8_t)0x8u)
#define UART_C1_M		((uint8_t)0x10u)
#define UART_C1_RSRC		((uint8_t)0x20u)
#define UART_C1_UARTSWAI		((uint8_t)0x40u)
#define UART_C1_LOOPS		((uint8_t)0x80u)
/* C2 Bit Fields */
#define UART_C2_SBK		((uint8_t)0x1u)
#define UART_C2_RWU		((uint8_t)0x2u)
#define UART_C2_RE		((uint8_t)0x4u)
#define UART_C2_TE		((uint8_t)0x8u)
#define UART_C2_ILIE		((uint8_t)0x10u)
#define UART_C2_RIE		((uint8_t)0x20u)
#define UART_C2_TCIE		((uint8_t)0x40u)
#define UART_C2_TIE		((uint8_t)0x80u)
/* S1 Bit Fields */
#define UART_S1_PF		((uint8_t)0x1u)
#define UART_S1_FE		((uint8_t)0x2u)
#define UART_S1_NF		((uint8_t)0x4u)
#define UART_S1_OR		((uint8_t)0x8u)
#define UART_S1_IDLE		((uint8_t)0x10u)
#define UART_S1_RDRF		((uint8_t)0x20u)
#define UART_S1_TC		((uint8_t)0x40u)
#define UART_S1_TDRE		((uint8_t)0x80u)
/* S2 Bit Fields */
#define UART_S2_RAF		((uint8_t)0x1u)
#define UART_S2_LBKDE		((uint8_t)0x2u)
#define UART_S2_BRK13		((uint8_t)0x4u)
#define UART_S2_RWUID		((uint8_t)0x8u)
#define UART_S2_RXINV		((uint8_t)0x10u)
#define UART_S2_MSBF		((uint8_t)0x20u)
#define UART_S2_RXEDGIF		((uint8_t)0x40u)
#define UART_S2_LBKDIF		((uint8_t)0x80u)
/* C3 Bit Fields */
#define UART_C3_PEIE		((uint8_t)0x1u)
#define UART_C3_FEIE		((uint8_t)0x2u)
#define UART_C3_NEIE		((uint8_t)0x4u)
#define UART_C3_ORIE		((uint8_t)0x8u)
#define UART_C3_TXINV		((uint8_t)0x10u)
#define UART_C3_TXDIR		((uint8_t)0x20u)
#define UART_C3_T8		((uint8_t)0x40u)
#define UART_C3_R8		((uint8_t)0x80u)
/* D Bit Fields */
#define UART_D_RT_MASK                           0xFFu
#define UART_D_RT_SHIFT                          0
#define UART_D_RT_WIDTH                          8
#define UART_D_RT(x)                             (((uint8_t)(((uint8_t)(x))<<UART_D_RT_SHIFT))&UART_D_RT_MASK)
/* MA1 Bit Fields */
#define UART_MA1_MA_MASK                         0xFFu
#define UART_MA1_MA_SHIFT                        0
#define UART_MA1_MA_WIDTH                        8
#define UART_MA1_MA(x)                           (((uint8_t)(((uint8_t)(x))<<UART_MA1_MA_SHIFT))&UART_MA1_MA_MASK)
/* MA2 Bit Fields */
#define UART_MA2_MA_MASK                         0xFFu
#define UART_MA2_MA_SHIFT                        0
#define UART_MA2_MA_WIDTH                        8
#define UART_MA2_MA(x)                           (((uint8_t)(((uint8_t)(x))<<UART_MA2_MA_SHIFT))&UART_MA2_MA_MASK)
/* C4 Bit Fields */
#define UART_C4_BRFA_MASK                        0x1Fu
#define UART_C4_BRFA_SHIFT                       0
#define UART_C4_BRFA_WIDTH                       5
#define UART_C4_BRFA(x)                          (((uint8_t)(((uint8_t)(x))<<UART_C4_BRFA_SHIFT))&UART_C4_BRFA_MASK)
#define UART_C4_M10		((uint8_t)0x20u)
#define UART_C4_MAEN2		((uint8_t)0x40u)
#define UART_C4_MAEN1		((uint8_t)0x80u)
/* C5 Bit Fields */
#define UART_C5_RDMAS		((uint8_t)0x20u)
#define UART_C5_TDMAS		((uint8_t)0x80u)
/* ED Bit Fields */
#define UART_ED_PARITYE		((uint8_t)0x40u)
#define UART_ED_NOISY		((uint8_t)0x80u)
/* MODEM Bit Fields */
#define UART_MODEM_TXCTSE		((uint8_t)0x1u)
#define UART_MODEM_TXRTSE		((uint8_t)0x2u)
#define UART_MODEM_TXRTSPOL		((uint8_t)0x4u)
#define UART_MODEM_RXRTSE		((uint8_t)0x8u)
/* IR Bit Fields */
#define UART_IR_TNP_MASK                         0x3u
#define UART_IR_TNP_SHIFT                        0
#define UART_IR_TNP_WIDTH                        2
#define UART_IR_TNP(x)                           (((uint8_t)(((uint8_t)(x))<<UART_IR_TNP_SHIFT))&UART_IR_TNP_MASK)
#define UART_IR_IREN		((uint8_t)0x4u)
/* PFIFO Bit Fields */
#define UART_PFIFO_RXFIFOSIZE_MASK               0x7u
#define UART_PFIFO_RXFIFOSIZE_SHIFT              0
#define UART_PFIFO_RXFIFOSIZE_WIDTH              3
#define UART_PFIFO_RXFIFOSIZE(x)                 (((uint8_t)(((uint8_t)(x))<<UART_PFIFO_RXFIFOSIZE_SHIFT))&UART_PFIFO_RXFIFOSIZE_MASK)
#define UART_PFIFO_RXFE		((uint8_t)0x8u)
#define UART_PFIFO_TXFIFOSIZE_MASK               0x70u
#define UART_PFIFO_TXFIFOSIZE_SHIFT              4
#define UART_PFIFO_TXFIFOSIZE_WIDTH              3
#define UART_PFIFO_TXFIFOSIZE(x)                 (((uint8_t)(((uint8_t)(x))<<UART_PFIFO_TXFIFOSIZE_SHIFT))&UART_PFIFO_TXFIFOSIZE_MASK)
#define UART_PFIFO_TXFE		((uint8_t)0x80u)
/* CFIFO Bit Fields */
#define UART_CFIFO_RXUFE		((uint8_t)0x1u)
#define UART_CFIFO_TXOFE		((uint8_t)0x2u)
#define UART_CFIFO_RXOFE		((uint8_t)0x4u)
#define UART_CFIFO_RXFLUSH		((uint8_t)0x40u)
#define UART_CFIFO_TXFLUSH		((uint8_t)0x80u)
/* SFIFO Bit Fields */
#define UART_SFIFO_RXUF		((uint8_t)0x1u)
#define UART_SFIFO_TXOF		((uint8_t)0x2u)
#define UART_SFIFO_RXOF		((uint8_t)0x4u)
#define UART_SFIFO_RXEMPT		((uint8_t)0x40u)
#define UART_SFIFO_TXEMPT		((uint8_t)0x80u)
/* TWFIFO Bit Fields */
#define UART_TWFIFO_TXWATER_MASK                 0xFFu
#define UART_TWFIFO_TXWATER_SHIFT                0
#define UART_TWFIFO_TXWATER_WIDTH                8
#define UART_TWFIFO_TXWATER(x)                   (((uint8_t)(((uint8_t)(x))<<UART_TWFIFO_TXWATER_SHIFT))&UART_TWFIFO_TXWATER_MASK)
/* TCFIFO Bit Fields */
#define UART_TCFIFO_TXCOUNT_MASK                 0xFFu
#define UART_TCFIFO_TXCOUNT_SHIFT                0
#define UART_TCFIFO_TXCOUNT_WIDTH                8
#define UART_TCFIFO_TXCOUNT(x)                   (((uint8_t)(((uint8_t)(x))<<UART_TCFIFO_TXCOUNT_SHIFT))&UART_TCFIFO_TXCOUNT_MASK)
/* RWFIFO Bit Fields */
#define UART_RWFIFO_RXWATER_MASK                 0xFFu
#define UART_RWFIFO_RXWATER_SHIFT                0
#define UART_RWFIFO_RXWATER_WIDTH                8
#define UART_RWFIFO_RXWATER(x)                   (((uint8_t)(((uint8_t)(x))<<UART_RWFIFO_RXWATER_SHIFT))&UART_RWFIFO_RXWATER_MASK)
/* RCFIFO Bit Fields */
#define UART_RCFIFO_RXCOUNT_MASK                 0xFFu
#define UART_RCFIFO_RXCOUNT_SHIFT                0
#define UART_RCFIFO_RXCOUNT_WIDTH                8
#define UART_RCFIFO_RXCOUNT(x)                   (((uint8_t)(((uint8_t)(x))<<UART_RCFIFO_RXCOUNT_SHIFT))&UART_RCFIFO_RXCOUNT_MASK)
/* C7816 Bit Fields */
#define UART_C7816_ISO_7816E		((uint8_t)0x1u)
#define UART_C7816_TTYPE		((uint8_t)0x2u)
#define UART_C7816_INIT		((uint8_t)0x4u)
#define UART_C7816_ANACK		((uint8_t)0x8u)
#define UART_C7816_ONACK		((uint8_t)0x10u)
/* IE7816 Bit Fields */
#define UART_IE7816_RXTE		((uint8_t)0x1u)
#define UART_IE7816_TXTE		((uint8_t)0x2u)
#define UART_IE7816_GTVE		((uint8_t)0x4u)
#define UART_IE7816_ADTE		((uint8_t)0x8u)
#define UART_IE7816_INITDE		((uint8_t)0x10u)
#define UART_IE7816_BWTE		((uint8_t)0x20u)
#define UART_IE7816_CWTE		((uint8_t)0x40u)
#define UART_IE7816_WTE		((uint8_t)0x80u)
/* IS7816 Bit Fields */
#define UART_IS7816_RXT		((uint8_t)0x1u)
#define UART_IS7816_TXT		((uint8_t)0x2u)
#define UART_IS7816_GTV		((uint8_t)0x4u)
#define UART_IS7816_ADT		((uint8_t)0x8u)
#define UART_IS7816_INITD		((uint8_t)0x10u)
#define UART_IS7816_BWT		((uint8_t)0x20u)
#define UART_IS7816_CWT		((uint8_t)0x40u)
#define UART_IS7816_WT		((uint8_t)0x80u)
/* WP7816 Bit Fields */
#define UART_WP7816_WTX_MASK                     0xFFu
#define UART_WP7816_WTX_SHIFT                    0
#define UART_WP7816_WTX_WIDTH                    8
#define UART_WP7816_WTX(x)                       (((uint8_t)(((uint8_t)(x))<<UART_WP7816_WTX_SHIFT))&UART_WP7816_WTX_MASK)
/* WN7816 Bit Fields */
#define UART_WN7816_GTN_MASK                     0xFFu
#define UART_WN7816_GTN_SHIFT                    0
#define UART_WN7816_GTN_WIDTH                    8
#define UART_WN7816_GTN(x)                       (((uint8_t)(((uint8_t)(x))<<UART_WN7816_GTN_SHIFT))&UART_WN7816_GTN_MASK)
/* WF7816 Bit Fields */
#define UART_WF7816_GTFD_MASK                    0xFFu
#define UART_WF7816_GTFD_SHIFT                   0
#define UART_WF7816_GTFD_WIDTH                   8
#define UART_WF7816_GTFD(x)                      (((uint8_t)(((uint8_t)(x))<<UART_WF7816_GTFD_SHIFT))&UART_WF7816_GTFD_MASK)
/* ET7816 Bit Fields */
#define UART_ET7816_RXTHRESHOLD_MASK             0xFu
#define UART_ET7816_RXTHRESHOLD_SHIFT            0
#define UART_ET7816_RXTHRESHOLD_WIDTH            4
#define UART_ET7816_RXTHRESHOLD(x)               (((uint8_t)(((uint8_t)(x))<<UART_ET7816_RXTHRESHOLD_SHIFT))&UART_ET7816_RXTHRESHOLD_MASK)
#define UART_ET7816_TXTHRESHOLD_MASK             0xF0u
#define UART_ET7816_TXTHRESHOLD_SHIFT            4
#define UART_ET7816_TXTHRESHOLD_WIDTH            4
#define UART_ET7816_TXTHRESHOLD(x)               (((uint8_t)(((uint8_t)(x))<<UART_ET7816_TXTHRESHOLD_SHIFT))&UART_ET7816_TXTHRESHOLD_MASK)
/* TL7816 Bit Fields */
#define UART_TL7816_TLEN_MASK                    0xFFu
#define UART_TL7816_TLEN_SHIFT                   0
#define UART_TL7816_TLEN_WIDTH                   8
#define UART_TL7816_TLEN(x)                      (((uint8_t)(((uint8_t)(x))<<UART_TL7816_TLEN_SHIFT))&UART_TL7816_TLEN_MASK)
/* AP7816A_T0 Bit Fields */
#define UART_AP7816A_T0_ADTI_H_MASK              0xFFu
#define UART_AP7816A_T0_ADTI_H_SHIFT             0
#define UART_AP7816A_T0_ADTI_H_WIDTH             8
#define UART_AP7816A_T0_ADTI_H(x)                (((uint8_t)(((uint8_t)(x))<<UART_AP7816A_T0_ADTI_H_SHIFT))&UART_AP7816A_T0_ADTI_H_MASK)
/* AP7816B_T0 Bit Fields */
#define UART_AP7816B_T0_ADTI_L_MASK              0xFFu
#define UART_AP7816B_T0_ADTI_L_SHIFT             0
#define UART_AP7816B_T0_ADTI_L_WIDTH             8
#define UART_AP7816B_T0_ADTI_L(x)                (((uint8_t)(((uint8_t)(x))<<UART_AP7816B_T0_ADTI_L_SHIFT))&UART_AP7816B_T0_ADTI_L_MASK)
/* WP7816A_T0 Bit Fields */
#define UART_WP7816A_T0_WI_H_MASK                0xFFu
#define UART_WP7816A_T0_WI_H_SHIFT               0
#define UART_WP7816A_T0_WI_H_WIDTH               8
#define UART_WP7816A_T0_WI_H(x)                  (((uint8_t)(((uint8_t)(x))<<UART_WP7816A_T0_WI_H_SHIFT))&UART_WP7816A_T0_WI_H_MASK)
/* WP7816B_T0 Bit Fields */
#define UART_WP7816B_T0_WI_L_MASK                0xFFu
#define UART_WP7816B_T0_WI_L_SHIFT               0
#define UART_WP7816B_T0_WI_L_WIDTH               8
#define UART_WP7816B_T0_WI_L(x)                  (((uint8_t)(((uint8_t)(x))<<UART_WP7816B_T0_WI_L_SHIFT))&UART_WP7816B_T0_WI_L_MASK)
/* WP7816A_T1 Bit Fields */
#define UART_WP7816A_T1_BWI_H_MASK               0xFFu
#define UART_WP7816A_T1_BWI_H_SHIFT              0
#define UART_WP7816A_T1_BWI_H_WIDTH              8
#define UART_WP7816A_T1_BWI_H(x)                 (((uint8_t)(((uint8_t)(x))<<UART_WP7816A_T1_BWI_H_SHIFT))&UART_WP7816A_T1_BWI_H_MASK)
/* WP7816B_T1 Bit Fields */
#define UART_WP7816B_T1_BWI_L_MASK               0xFFu
#define UART_WP7816B_T1_BWI_L_SHIFT              0
#define UART_WP7816B_T1_BWI_L_WIDTH              8
#define UART_WP7816B_T1_BWI_L(x)                 (((uint8_t)(((uint8_t)(x))<<UART_WP7816B_T1_BWI_L_SHIFT))&UART_WP7816B_T1_BWI_L_MASK)
/* WGP7816_T1 Bit Fields */
#define UART_WGP7816_T1_BGI_MASK                 0xFu
#define UART_WGP7816_T1_BGI_SHIFT                0
#define UART_WGP7816_T1_BGI_WIDTH                4
#define UART_WGP7816_T1_BGI(x)                   (((uint8_t)(((uint8_t)(x))<<UART_WGP7816_T1_BGI_SHIFT))&UART_WGP7816_T1_BGI_MASK)
#define UART_WGP7816_T1_CWI1_MASK                0xF0u
#define UART_WGP7816_T1_CWI1_SHIFT               4
#define UART_WGP7816_T1_CWI1_WIDTH               4
#define UART_WGP7816_T1_CWI1(x)                  (((uint8_t)(((uint8_t)(x))<<UART_WGP7816_T1_CWI1_SHIFT))&UART_WGP7816_T1_CWI1_MASK)
/* WP7816C_T1 Bit Fields */
#define UART_WP7816C_T1_CWI2_MASK                0x1Fu
#define UART_WP7816C_T1_CWI2_SHIFT               0
#define UART_WP7816C_T1_CWI2_WIDTH               5
#define UART_WP7816C_T1_CWI2(x)                  (((uint8_t)(((uint8_t)(x))<<UART_WP7816C_T1_CWI2_SHIFT))&UART_WP7816C_T1_CWI2_MASK)

/*!
 * @}
 */ /* end of group UART_Register_Masks */


/* UART - Peripheral instance base addresses */
/** Peripheral UART0 base address */
#define UART0_BASE                               (0x4006A000u)
/** Peripheral UART0 base pointer */
#define UART0                                    ((UART_TypeDef *)UART0_BASE)
#define UART0_BASE_PTR                           (UART0)
/** Peripheral UART1 base address */
#define UART1_BASE                               (0x4006B000u)
/** Peripheral UART1 base pointer */
#define UART1                                    ((UART_TypeDef *)UART1_BASE)
#define UART1_BASE_PTR                           (UART1)
/** Peripheral UART2 base address */
#define UART2_BASE                               (0x4006C000u)
/** Peripheral UART2 base pointer */
#define UART2                                    ((UART_TypeDef *)UART2_BASE)
#define UART2_BASE_PTR                           (UART2)
/** Array initializer of UART peripheral base addresses */
#define UART_BASE_ADDRS                          { UART0_BASE, UART1_BASE, UART2_BASE }
/** Array initializer of UART peripheral base pointers */
#define UART_BASE_PTRS                           { UART0, UART1, UART2 }
/** Interrupt vectors for the UART peripheral type */
#define UART_RX_TX_IRQS                          { UART0_RX_TX_IRQn, UART1_RX_TX_IRQn, UART2_RX_TX_IRQn }
#define UART_ERR_IRQS                            { UART0_ERR_IRQn, UART1_ERR_IRQn, UART2_ERR_IRQn }

/* ----------------------------------------------------------------------------
   -- UART - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup UART_Register_Accessor_Macros UART - Register accessor macros
 * @{
 */


/* UART - Register instance definitions */
/* UART0 */
#define UART0_BDH                                UART_BDH_REG(UART0)
#define UART0_BDL                                UART_BDL_REG(UART0)
#define UART0_C1                                 UART_C1_REG(UART0)
#define UART0_C2                                 UART_C2_REG(UART0)
#define UART0_S1                                 UART_S1_REG(UART0)
#define UART0_S2                                 UART_S2_REG(UART0)
#define UART0_C3                                 UART_C3_REG(UART0)
#define UART0_D                                  UART_D_REG(UART0)
#define UART0_MA1                                UART_MA1_REG(UART0)
#define UART0_MA2                                UART_MA2_REG(UART0)
#define UART0_C4                                 UART_C4_REG(UART0)
#define UART0_C5                                 UART_C5_REG(UART0)
#define UART0_ED                                 UART_ED_REG(UART0)
#define UART0_MODEM                              UART_MODEM_REG(UART0)
#define UART0_IR                                 UART_IR_REG(UART0)
#define UART0_PFIFO                              UART_PFIFO_REG(UART0)
#define UART0_CFIFO                              UART_CFIFO_REG(UART0)
#define UART0_SFIFO                              UART_SFIFO_REG(UART0)
#define UART0_TWFIFO                             UART_TWFIFO_REG(UART0)
#define UART0_TCFIFO                             UART_TCFIFO_REG(UART0)
#define UART0_RWFIFO                             UART_RWFIFO_REG(UART0)
#define UART0_RCFIFO                             UART_RCFIFO_REG(UART0)
#define UART0_C7816                              UART_C7816_REG(UART0)
#define UART0_IE7816                             UART_IE7816_REG(UART0)
#define UART0_IS7816                             UART_IS7816_REG(UART0)
#define UART0_WP7816                             UART_WP7816_REG(UART0)
#define UART0_WN7816                             UART_WN7816_REG(UART0)
#define UART0_WF7816                             UART_WF7816_REG(UART0)
#define UART0_ET7816                             UART_ET7816_REG(UART0)
#define UART0_TL7816                             UART_TL7816_REG(UART0)
#define UART0_AP7816A_T0                         UART_AP7816A_T0_REG(UART0)
#define UART0_AP7816B_T0                         UART_AP7816B_T0_REG(UART0)
#define UART0_WP7816A_T0                         UART_WP7816A_T0_REG(UART0)
#define UART0_WP7816A_T1                         UART_WP7816A_T1_REG(UART0)
#define UART0_WP7816B_T0                         UART_WP7816B_T0_REG(UART0)
#define UART0_WP7816B_T1                         UART_WP7816B_T1_REG(UART0)
#define UART0_WGP7816_T1                         UART_WGP7816_T1_REG(UART0)
#define UART0_WP7816C_T1                         UART_WP7816C_T1_REG(UART0)
/* UART1 */
#define UART1_BDH                                UART_BDH_REG(UART1)
#define UART1_BDL                                UART_BDL_REG(UART1)
#define UART1_C1                                 UART_C1_REG(UART1)
#define UART1_C2                                 UART_C2_REG(UART1)
#define UART1_S1                                 UART_S1_REG(UART1)
#define UART1_S2                                 UART_S2_REG(UART1)
#define UART1_C3                                 UART_C3_REG(UART1)
#define UART1_D                                  UART_D_REG(UART1)
#define UART1_MA1                                UART_MA1_REG(UART1)
#define UART1_MA2                                UART_MA2_REG(UART1)
#define UART1_C4                                 UART_C4_REG(UART1)
#define UART1_C5                                 UART_C5_REG(UART1)
#define UART1_ED                                 UART_ED_REG(UART1)
#define UART1_MODEM                              UART_MODEM_REG(UART1)
#define UART1_IR                                 UART_IR_REG(UART1)
#define UART1_PFIFO                              UART_PFIFO_REG(UART1)
#define UART1_CFIFO                              UART_CFIFO_REG(UART1)
#define UART1_SFIFO                              UART_SFIFO_REG(UART1)
#define UART1_TWFIFO                             UART_TWFIFO_REG(UART1)
#define UART1_TCFIFO                             UART_TCFIFO_REG(UART1)
#define UART1_RWFIFO                             UART_RWFIFO_REG(UART1)
#define UART1_RCFIFO                             UART_RCFIFO_REG(UART1)
/* UART2 */
#define UART2_BDH                                UART_BDH_REG(UART2)
#define UART2_BDL                                UART_BDL_REG(UART2)
#define UART2_C1                                 UART_C1_REG(UART2)
#define UART2_C2                                 UART_C2_REG(UART2)
#define UART2_S1                                 UART_S1_REG(UART2)
#define UART2_S2                                 UART_S2_REG(UART2)
#define UART2_C3                                 UART_C3_REG(UART2)
#define UART2_D                                  UART_D_REG(UART2)
#define UART2_MA1                                UART_MA1_REG(UART2)
#define UART2_MA2                                UART_MA2_REG(UART2)
#define UART2_C4                                 UART_C4_REG(UART2)
#define UART2_C5                                 UART_C5_REG(UART2)
#define UART2_ED                                 UART_ED_REG(UART2)
#define UART2_MODEM                              UART_MODEM_REG(UART2)
#define UART2_IR                                 UART_IR_REG(UART2)
#define UART2_PFIFO                              UART_PFIFO_REG(UART2)
#define UART2_CFIFO                              UART_CFIFO_REG(UART2)
#define UART2_SFIFO                              UART_SFIFO_REG(UART2)
#define UART2_TWFIFO                             UART_TWFIFO_REG(UART2)
#define UART2_TCFIFO                             UART_TCFIFO_REG(UART2)
#define UART2_RWFIFO                             UART_RWFIFO_REG(UART2)
#define UART2_RCFIFO                             UART_RCFIFO_REG(UART2)


/****************************************************************/
/*                                                              */
/*     Universal Asynchronous Receiver/Transmitter (UART)       */
/*                                                              */
/****************************************************************/
/*********  Bits definition for UARTx_BDH register  *************/
#define UARTx_BDH_LBKDIE             ((uint8_t)0x80)    /*!< LIN Break Detect Interrupt Enable */
#define UARTx_BDH_RXEDGIE            ((uint8_t)0x40)    /*!< RxD Input Active Edge Interrupt Enable */
#define UARTx_BDH_SBR_MASK           ((uint8_t)0x1F)
#define UARTx_BDH_SBR(x)             ((uint8_t)((uint8_t)(x) & UARTx_BDH_SBR_MASK))  /*!< Baud Rate Modulo Divisor */

/*********  Bits definition for UARTx_BDL register  *************/
#define UARTx_BDL_SBR_MASK           ((uint8_t)0xFF)    /*!< Baud Rate Modulo Divisor */

/*********  Bits definition for UARTx_C1 register  **************/
#define UARTx_C1_LOOPS               ((uint8_t)0x80)    /*!< Loop Mode Select */
#define UARTx_C1_DOZEEN              ((uint8_t)0x40)    /*!< Doze Enable */
#define UARTx_C1_UARTSWAI            ((uint8_t)0x40)    /*!< UART Stops in Wait Mode */
#define UARTx_C1_RSRC                ((uint8_t)0x20)    /*!< Receiver Source Select */
#define UARTx_C1_M                   ((uint8_t)0x10)    /*!< 9-Bit or 8-Bit Mode Select */
#define UARTx_C1_WAKE                ((uint8_t)0x08)    /*!< Receiver Wakeup Method Select */
#define UARTx_C1_ILT                 ((uint8_t)0x04)    /*!< Idle Line Type Select */
#define UARTx_C1_PE                  ((uint8_t)0x02)    /*!< Parity Enable */
#define UARTx_C1_PT                  ((uint8_t)0x01)    /*!< Parity Type */

/*********  Bits definition for UARTx_C2 register  **************/
#define UARTx_C2_TIE                 ((uint8_t)0x80)    /*!< Transmit Interrupt Enable for TDRE */
#define UARTx_C2_TCIE                ((uint8_t)0x40)    /*!< Transmission Complete Interrupt Enable for TC */
#define UARTx_C2_RIE                 ((uint8_t)0x20)    /*!< Receiver Interrupt Enable for RDRF */
#define UARTx_C2_ILIE                ((uint8_t)0x10)    /*!< Idle Line Interrupt Enable for IDLE */
#define UARTx_C2_TE                  ((uint8_t)0x08)    /*!< Transmitter Enable */
#define UARTx_C2_RE                  ((uint8_t)0x04)    /*!< Receiver Enable */
#define UARTx_C2_RWU                 ((uint8_t)0x02)    /*!< Receiver Wakeup Control */
#define UARTx_C2_SBK                 ((uint8_t)0x01)    /*!< Send Break */

/*********  Bits definition for UARTx_S1 register  **************/
#define UARTx_S1_TDRE                ((uint8_t)0x80)    /*!< Transmit Data Register Empty Flag */
#define UARTx_S1_TC                  ((uint8_t)0x40)    /*!< Transmission Complete Flag */
#define UARTx_S1_RDRF                ((uint8_t)0x20)    /*!< Receiver Data Register Full Flag */
#define UARTx_S1_IDLE                ((uint8_t)0x10)    /*!< Idle Line Flag */
#define UARTx_S1_OR                  ((uint8_t)0x08)    /*!< Receiver Overrun Flag */
#define UARTx_S1_NF                  ((uint8_t)0x04)    /*!< Noise Flag */
#define UARTx_S1_FE                  ((uint8_t)0x02)    /*!< Framing Error Flag */
#define UARTx_S1_PF                  ((uint8_t)0x01)    /*!< Parity Error Flag */

/*********  Bits definition for UARTx_S2 register  **************/
#define UARTx_S2_LBKDIF              ((uint8_t)0x80)    /*!< LIN Break Detect Interrupt Flag */
#define UARTx_S2_RXEDGIF             ((uint8_t)0x40)    /*!< UART_RX Pin Active Edge Interrupt Flag */
#define UARTx_S2_MSBF                ((uint8_t)0x20)    /*!< MSB First */
#define UARTx_S2_RXINV               ((uint8_t)0x10)    /*!< Receive Data Inversion */
#define UARTx_S2_RWUID               ((uint8_t)0x08)    /*!< Receive Wake Up Idle Detect */
#define UARTx_S2_BRK13               ((uint8_t)0x04)    /*!< Break Character Generation Length */
#define UARTx_S2_LBKDE               ((uint8_t)0x02)    /*!< LIN Break Detect Enable */
#define UARTx_S2_RAF                 ((uint8_t)0x01)    /*!< Receiver Active Flag */

/*********  Bits definition for UARTx_C3 register  **************/
#define UARTx_C3_R8                  ((uint8_t)0x80)    /*!< Ninth Data Bit for Receiver */
#define UARTx_C3_T8                  ((uint8_t)0x40)    /*!< Ninth Data Bit for Transmitter */
#define UARTx_C3_TXDIR               ((uint8_t)0x20)    /*!< UART_TX Pin Direction in Single-Wire Mode */
#define UARTx_C3_TXINV               ((uint8_t)0x10)    /*!< Transmit Data Inversion */
#define UARTx_C3_ORIE                ((uint8_t)0x08)    /*!< Overrun Interrupt Enable */
#define UARTx_C3_NEIE                ((uint8_t)0x04)    /*!< Noise Error Interrupt Enable */
#define UARTx_C3_FEIE                ((uint8_t)0x02)    /*!< Framing Error Interrupt Enable */
#define UARTx_C3_PEIE                ((uint8_t)0x01)    /*!< Parity Error Interrupt Enable */

/*********  Bits definition for UARTx_D register  ***************/
#define UARTx_D_R7T7                 ((uint8_t)0x80)    /*!< Read receive data buffer 7 or write transmit data buffer 7 */
#define UARTx_D_R6T6                 ((uint8_t)0x40)    /*!< Read receive data buffer 6 or write transmit data buffer 6 */
#define UARTx_D_R5T5                 ((uint8_t)0x20)    /*!< Read receive data buffer 5 or write transmit data buffer 5 */
#define UARTx_D_R4T4                 ((uint8_t)0x10)    /*!< Read receive data buffer 4 or write transmit data buffer 4 */
#define UARTx_D_R3T3                 ((uint8_t)0x08)    /*!< Read receive data buffer 3 or write transmit data buffer 3 */
#define UARTx_D_R2T2                 ((uint8_t)0x04)    /*!< Read receive data buffer 2 or write transmit data buffer 2 */
#define UARTx_D_R1T1                 ((uint8_t)0x02)    /*!< Read receive data buffer 1 or write transmit data buffer 1 */
#define UARTx_D_R0T0                 ((uint8_t)0x01)    /*!< Read receive data buffer 0 or write transmit data buffer 0 */

/*********  Bits definition for UARTx_MA1 register  *************/
#define UARTx_MA1_MA                 ((uint8_t)0xFF)    /*!< Match Address */

/*********  Bits definition for UARTx_MA2 register  *************/
#define UARTx_MA2_MA                 ((uint8_t)0xFF)    /*!< Match Address */

/*********  Bits definition for UARTx_C4 register  **************/
#define UARTx_C4_MAEN1               ((uint8_t)0x80)    /*!< Match Address Mode Enable 1 */
#define UARTx_C4_MAEN2               ((uint8_t)0x40)    /*!< Match Address Mode Enable 2 */
#define UARTx_C4_M10                 ((uint8_t)0x20)    /*!< 10-bit Mode Select */
#define UARTx_C4_BRFA_MASK           ((uint8_t)0x1F)
#define UARTx_C4_BRFA(x)             ((uint8_t)((uint8_t)(x) & UARTx_C4_BRFA_MASK))  /*!< Baud Rate Fine Adjust */

/*********  Bits definition for UARTx_C5 register  **************/
#define UARTx_C5_TDMAE               ((uint8_t)0x80)    /*!< Transmitter DMA Enable */
#define UARTx_C5_RDMAE               ((uint8_t)0x20)    /*!< Receiver Full DMA Enable */
#define UARTx_C5_BOTHEDGE            ((uint8_t)0x02)    /*!< Both Edge Sampling */
#define UARTx_C5_RESYNCDIS           ((uint8_t)0x01)    /*!< Resynchronization Disable */

/*******  Bits definition for UARTx_CFIFO register  ************/
#define UARTx_CFIFO_TXFLUSH          ((uint8_t)0x80)    /*!< Transmit FIFO/Buffer Flush */
#define UARTx_CFIFO_RXFLUSH          ((uint8_t)0x40)    /*!< Receive FIFO/Buffer Flush */
#define UARTx_CFIFO_RXOFE            ((uint8_t)0x04)    /*!< Receive FIFO Overflow Interrupt Enable */
#define UARTx_CFIFO_TXOFE            ((uint8_t)0x02)    /*!< Transmit FIFO Overflow Interrupt Enable */
#define UARTx_CFIFO_RXUFE            ((uint8_t)0x01)    /*!< Receive FIFO Underflow Interrupt Enable */

/*******  Bits definition for UARTx_PFIFO register  ************/
#define UARTx_PFIFO_TXFE             ((uint8_t)0x80)    /*!< Transmit FIFO Enable */
#define UARTx_PFIFO_TXFIFOSIZE_SHIFT 4
#define UARTx_PFIFO_TXFIFOSIZE_MASK  ((uint8_t)((uint8_t)0x7 << UARTx_PFIFO_TXFIFOSIZE_SHIFT))
#define UARTx_PFIFO_TXFIFOSIZE(x)    ((uint8_t)(((uint8_t)(x) << UARTx_PFIFO_TXFIFOSIZE_SHIFT) & UARTx_PFIFO_TXFIFOSIZE_MASK))  /*!< Transmit FIFO Buffer depth */
#define UARTx_PFIFO_RXFE             ((uint8_t)0x08)    /*!< Receive FIFOh */
#define UARTx_PFIFO_RXFIFOSIZE_SHIFT 0
#define UARTx_PFIFO_RXFIFOSIZE_MASK  ((uint8_t)((uint8_t)0x7 << UARTx_PFIFO_RXFIFOSIZE_SHIFT))
#define UARTx_PFIFO_RXFIFOSIZE(x)    ((uint8_t)(((uint8_t)(x) << UARTx_PFIFO_RXFIFOSIZE_SHIFT) & UARTx_PFIFO_RXFIFOSIZE_MASK))  /*!< Receive FIFO Buffer depth */


/*!
 * @}
 */ /* end of group UART_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group UART_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- USB Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Peripheral_Access_Layer USB Peripheral Access Layer
 * @{
 */

/** USB - Register Layout Typedef */
typedef struct {
  __I  uint8_t PERID;                              /**< Peripheral ID register, offset: 0x0 */
       uint8_t RESERVED_0[3];
  __I  uint8_t IDCOMP;                             /**< Peripheral ID Complement register, offset: 0x4 */
       uint8_t RESERVED_1[3];
  __I  uint8_t REV;                                /**< Peripheral Revision register, offset: 0x8 */
       uint8_t RESERVED_2[3];
  __I  uint8_t ADDINFO;                            /**< Peripheral Additional Info register, offset: 0xC */
       uint8_t RESERVED_3[3];
  __IO uint8_t OTGISTAT;                           /**< OTG Interrupt Status register, offset: 0x10 */
       uint8_t RESERVED_4[3];
  __IO uint8_t OTGICR;                             /**< OTG Interrupt Control register, offset: 0x14 */
       uint8_t RESERVED_5[3];
  __IO uint8_t OTGSTAT;                            /**< OTG Status register, offset: 0x18 */
       uint8_t RESERVED_6[3];
  __IO uint8_t OTGCTL;                             /**< OTG Control register, offset: 0x1C */
       uint8_t RESERVED_7[99];
  __IO uint8_t ISTAT;                              /**< Interrupt Status register, offset: 0x80 */
       uint8_t RESERVED_8[3];
  __IO uint8_t INTEN;                              /**< Interrupt Enable register, offset: 0x84 */
       uint8_t RESERVED_9[3];
  __IO uint8_t ERRSTAT;                            /**< Error Interrupt Status register, offset: 0x88 */
       uint8_t RESERVED_10[3];
  __IO uint8_t ERREN;                              /**< Error Interrupt Enable register, offset: 0x8C */
       uint8_t RESERVED_11[3];
  __I  uint8_t STAT;                               /**< Status register, offset: 0x90 */
       uint8_t RESERVED_12[3];
  __IO uint8_t CTL;                                /**< Control register, offset: 0x94 */
       uint8_t RESERVED_13[3];
  __IO uint8_t ADDR;                               /**< Address register, offset: 0x98 */
       uint8_t RESERVED_14[3];
  __IO uint8_t BDTPAGE1;                           /**< BDT Page register 1, offset: 0x9C */
       uint8_t RESERVED_15[3];
  __IO uint8_t FRMNUML;                            /**< Frame Number register Low, offset: 0xA0 */
       uint8_t RESERVED_16[3];
  __IO uint8_t FRMNUMH;                            /**< Frame Number register High, offset: 0xA4 */
       uint8_t RESERVED_17[3];
  __IO uint8_t TOKEN;                              /**< Token register, offset: 0xA8 */
       uint8_t RESERVED_18[3];
  __IO uint8_t SOFTHLD;                            /**< SOF Threshold register, offset: 0xAC */
       uint8_t RESERVED_19[3];
  __IO uint8_t BDTPAGE2;                           /**< BDT Page Register 2, offset: 0xB0 */
       uint8_t RESERVED_20[3];
  __IO uint8_t BDTPAGE3;                           /**< BDT Page Register 3, offset: 0xB4 */
       uint8_t RESERVED_21[11];
  struct {                                         /* offset: 0xC0, array step: 0x4 */
    __IO uint8_t ENDPT;                              /**< Endpoint Control register, array offset: 0xC0, array step: 0x4 */
         uint8_t RESERVED_0[3];
  } ENDPOINT[16];
  __IO uint8_t USBCTRL;                            /**< USB Control register, offset: 0x100 */
       uint8_t RESERVED_22[3];
  __I  uint8_t OBSERVE;                            /**< USB OTG Observe register, offset: 0x104 */
       uint8_t RESERVED_23[3];
  __IO uint8_t CONTROL;                            /**< USB OTG Control register, offset: 0x108 */
       uint8_t RESERVED_24[3];
  __IO uint8_t USBTRC0;                            /**< USB Transceiver Control register 0, offset: 0x10C */
       uint8_t RESERVED_25[7];
  __IO uint8_t USBFRMADJUST;                       /**< Frame Adjust Register, offset: 0x114 */
       uint8_t RESERVED_26[43];
  __IO uint8_t CLK_RECOVER_CTRL;                   /**< USB Clock recovery control, offset: 0x140 */
       uint8_t RESERVED_27[3];
  __IO uint8_t CLK_RECOVER_IRC_EN;                 /**< IRC48M oscillator enable register, offset: 0x144 */
       uint8_t RESERVED_28[23];
  __IO uint8_t CLK_RECOVER_INT_STATUS;             /**< Clock recovery separated interrupt status, offset: 0x15C */
} USB_TypeDef, *USB_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- USB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Register_Accessor_Macros USB - Register accessor macros
 * @{
 */


/* USB - Register accessors */
#define USB_PERID_REG(base)                      ((base)->PERID)
#define USB_IDCOMP_REG(base)                     ((base)->IDCOMP)
#define USB_REV_REG(base)                        ((base)->REV)
#define USB_ADDINFO_REG(base)                    ((base)->ADDINFO)
#define USB_OTGISTAT_REG(base)                   ((base)->OTGISTAT)
#define USB_OTGICR_REG(base)                     ((base)->OTGICR)
#define USB_OTGSTAT_REG(base)                    ((base)->OTGSTAT)
#define USB_OTGCTL_REG(base)                     ((base)->OTGCTL)
#define USB_ISTAT_REG(base)                      ((base)->ISTAT)
#define USB_INTEN_REG(base)                      ((base)->INTEN)
#define USB_ERRSTAT_REG(base)                    ((base)->ERRSTAT)
#define USB_ERREN_REG(base)                      ((base)->ERREN)
#define USB_STAT_REG(base)                       ((base)->STAT)
#define USB_CTL_REG(base)                        ((base)->CTL)
#define USB_ADDR_REG(base)                       ((base)->ADDR)
#define USB_BDTPAGE1_REG(base)                   ((base)->BDTPAGE1)
#define USB_FRMNUML_REG(base)                    ((base)->FRMNUML)
#define USB_FRMNUMH_REG(base)                    ((base)->FRMNUMH)
#define USB_TOKEN_REG(base)                      ((base)->TOKEN)
#define USB_SOFTHLD_REG(base)                    ((base)->SOFTHLD)
#define USB_BDTPAGE2_REG(base)                   ((base)->BDTPAGE2)
#define USB_BDTPAGE3_REG(base)                   ((base)->BDTPAGE3)
#define USB_ENDPT_REG(base,index)                ((base)->ENDPOINT[index].ENDPT)
#define USB_ENDPT_COUNT                          16
#define USB_USBCTRL_REG(base)                    ((base)->USBCTRL)
#define USB_OBSERVE_REG(base)                    ((base)->OBSERVE)
#define USB_CONTROL_REG(base)                    ((base)->CONTROL)
#define USB_USBTRC0_REG(base)                    ((base)->USBTRC0)
#define USB_USBFRMADJUST_REG(base)               ((base)->USBFRMADJUST)
#define USB_CLK_RECOVER_CTRL_REG(base)           ((base)->CLK_RECOVER_CTRL)
#define USB_CLK_RECOVER_IRC_EN_REG(base)         ((base)->CLK_RECOVER_IRC_EN)
#define USB_CLK_RECOVER_INT_STATUS_REG(base)     ((base)->CLK_RECOVER_INT_STATUS)

/*!
 * @}
 */ /* end of group USB_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- USB Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Register_Masks USB Register Masks
 * @{
 */

/* PERID Bit Fields */
#define USB_PERID_ID_MASK                        0x3Fu
#define USB_PERID_ID_SHIFT                       0
#define USB_PERID_ID_WIDTH                       6
#define USB_PERID_ID(x)                          (((uint8_t)(((uint8_t)(x))<<USB_PERID_ID_SHIFT))&USB_PERID_ID_MASK)
/* IDCOMP Bit Fields */
#define USB_IDCOMP_NID_MASK                      0x3Fu
#define USB_IDCOMP_NID_SHIFT                     0
#define USB_IDCOMP_NID_WIDTH                     6
#define USB_IDCOMP_NID(x)                        (((uint8_t)(((uint8_t)(x))<<USB_IDCOMP_NID_SHIFT))&USB_IDCOMP_NID_MASK)
/* REV Bit Fields */
#define USB_REV_REV_MASK                         0xFFu
#define USB_REV_REV_SHIFT                        0
#define USB_REV_REV_WIDTH                        8
#define USB_REV_REV(x)                           (((uint8_t)(((uint8_t)(x))<<USB_REV_REV_SHIFT))&USB_REV_REV_MASK)
/* ADDINFO Bit Fields */
#define USB_ADDINFO_IEHOST		((uint8_t)0x1u)
/* OTGISTAT Bit Fields */
#define USB_OTGISTAT_AVBUSCHG		((uint8_t)0x1u)
#define USB_OTGISTAT_B_SESS_CHG		((uint8_t)0x4u)
#define USB_OTGISTAT_SESSVLDCHG		((uint8_t)0x8u)
#define USB_OTGISTAT_LINE_STATE_CHG		((uint8_t)0x20u)
#define USB_OTGISTAT_ONEMSEC		((uint8_t)0x40u)
#define USB_OTGISTAT_IDCHG		((uint8_t)0x80u)
/* OTGICR Bit Fields */
#define USB_OTGICR_AVBUSEN		((uint8_t)0x1u)
#define USB_OTGICR_BSESSEN		((uint8_t)0x4u)
#define USB_OTGICR_SESSVLDEN		((uint8_t)0x8u)
#define USB_OTGICR_LINESTATEEN		((uint8_t)0x20u)
#define USB_OTGICR_ONEMSECEN		((uint8_t)0x40u)
#define USB_OTGICR_IDEN		((uint8_t)0x80u)
/* OTGSTAT Bit Fields */
#define USB_OTGSTAT_AVBUSVLD		((uint8_t)0x1u)
#define USB_OTGSTAT_BSESSEND		((uint8_t)0x4u)
#define USB_OTGSTAT_SESS_VLD		((uint8_t)0x8u)
#define USB_OTGSTAT_LINESTATESTABLE		((uint8_t)0x20u)
#define USB_OTGSTAT_ONEMSECEN		((uint8_t)0x40u)
#define USB_OTGSTAT_ID		((uint8_t)0x80u)
/* OTGCTL Bit Fields */
#define USB_OTGCTL_OTGEN		((uint8_t)0x4u)
#define USB_OTGCTL_DMLOW		((uint8_t)0x10u)
#define USB_OTGCTL_DPLOW		((uint8_t)0x20u)
#define USB_OTGCTL_DPHIGH		((uint8_t)0x80u)
/* ISTAT Bit Fields */
#define USB_ISTAT_USBRST		((uint8_t)0x1u)
#define USB_ISTAT_ERROR		((uint8_t)0x2u)
#define USB_ISTAT_SOFTOK		((uint8_t)0x4u)
#define USB_ISTAT_TOKDNE		((uint8_t)0x8u)
#define USB_ISTAT_SLEEP		((uint8_t)0x10u)
#define USB_ISTAT_RESUME		((uint8_t)0x20u)
#define USB_ISTAT_ATTACH		((uint8_t)0x40u)
#define USB_ISTAT_STALL		((uint8_t)0x80u)
/* INTEN Bit Fields */
#define USB_INTEN_USBRSTEN		((uint8_t)0x1u)
#define USB_INTEN_ERROREN		((uint8_t)0x2u)
#define USB_INTEN_SOFTOKEN		((uint8_t)0x4u)
#define USB_INTEN_TOKDNEEN		((uint8_t)0x8u)
#define USB_INTEN_SLEEPEN		((uint8_t)0x10u)
#define USB_INTEN_RESUMEEN		((uint8_t)0x20u)
#define USB_INTEN_ATTACHEN		((uint8_t)0x40u)
#define USB_INTEN_STALLEN		((uint8_t)0x80u)
/* ERRSTAT Bit Fields */
#define USB_ERRSTAT_PIDERR		((uint8_t)0x1u)
#define USB_ERRSTAT_CRC5EOF		((uint8_t)0x2u)
#define USB_ERRSTAT_CRC16		((uint8_t)0x4u)
#define USB_ERRSTAT_DFN8		((uint8_t)0x8u)
#define USB_ERRSTAT_BTOERR		((uint8_t)0x10u)
#define USB_ERRSTAT_DMAERR		((uint8_t)0x20u)
#define USB_ERRSTAT_BTSERR		((uint8_t)0x80u)
/* ERREN Bit Fields */
#define USB_ERREN_PIDERREN		((uint8_t)0x1u)
#define USB_ERREN_CRC5EOFEN		((uint8_t)0x2u)
#define USB_ERREN_CRC16EN		((uint8_t)0x4u)
#define USB_ERREN_DFN8EN		((uint8_t)0x8u)
#define USB_ERREN_BTOERREN		((uint8_t)0x10u)
#define USB_ERREN_DMAERREN		((uint8_t)0x20u)
#define USB_ERREN_BTSERREN		((uint8_t)0x80u)
/* STAT Bit Fields */
#define USB_STAT_ODD		((uint8_t)0x4u)
#define USB_STAT_TX		((uint8_t)0x8u)
#define USB_STAT_ENDP_MASK                       0xF0u
#define USB_STAT_ENDP_SHIFT                      4
#define USB_STAT_ENDP_WIDTH                      4
#define USB_STAT_ENDP(x)                         (((uint8_t)(((uint8_t)(x))<<USB_STAT_ENDP_SHIFT))&USB_STAT_ENDP_MASK)
/* CTL Bit Fields */
#define USB_CTL_USBENSOFEN		((uint8_t)0x1u)
#define USB_CTL_ODDRST		((uint8_t)0x2u)
#define USB_CTL_RESUME		((uint8_t)0x4u)
#define USB_CTL_HOSTMODEEN		((uint8_t)0x8u)
#define USB_CTL_RESET		((uint8_t)0x10u)
#define USB_CTL_TXSUSPENDTOKENBUSY		((uint8_t)0x20u)
#define USB_CTL_SE0		((uint8_t)0x40u)
#define USB_CTL_JSTATE		((uint8_t)0x80u)
/* ADDR Bit Fields */
#define USB_ADDR_ADDR_MASK                       0x7Fu
#define USB_ADDR_ADDR_SHIFT                      0
#define USB_ADDR_ADDR_WIDTH                      7
#define USB_ADDR_ADDR(x)                         (((uint8_t)(((uint8_t)(x))<<USB_ADDR_ADDR_SHIFT))&USB_ADDR_ADDR_MASK)
#define USB_ADDR_LSEN		((uint8_t)0x80u)
/* BDTPAGE1 Bit Fields */
#define USB_BDTPAGE1_BDTBA_MASK                  0xFEu
#define USB_BDTPAGE1_BDTBA_SHIFT                 1
#define USB_BDTPAGE1_BDTBA_WIDTH                 7
#define USB_BDTPAGE1_BDTBA(x)                    (((uint8_t)(((uint8_t)(x))<<USB_BDTPAGE1_BDTBA_SHIFT))&USB_BDTPAGE1_BDTBA_MASK)
/* FRMNUML Bit Fields */
#define USB_FRMNUML_FRM_MASK                     0xFFu
#define USB_FRMNUML_FRM_SHIFT                    0
#define USB_FRMNUML_FRM_WIDTH                    8
#define USB_FRMNUML_FRM(x)                       (((uint8_t)(((uint8_t)(x))<<USB_FRMNUML_FRM_SHIFT))&USB_FRMNUML_FRM_MASK)
/* FRMNUMH Bit Fields */
#define USB_FRMNUMH_FRM_MASK                     0x7u
#define USB_FRMNUMH_FRM_SHIFT                    0
#define USB_FRMNUMH_FRM_WIDTH                    3
#define USB_FRMNUMH_FRM(x)                       (((uint8_t)(((uint8_t)(x))<<USB_FRMNUMH_FRM_SHIFT))&USB_FRMNUMH_FRM_MASK)
/* TOKEN Bit Fields */
#define USB_TOKEN_TOKENENDPT_MASK                0xFu
#define USB_TOKEN_TOKENENDPT_SHIFT               0
#define USB_TOKEN_TOKENENDPT_WIDTH               4
#define USB_TOKEN_TOKENENDPT(x)                  (((uint8_t)(((uint8_t)(x))<<USB_TOKEN_TOKENENDPT_SHIFT))&USB_TOKEN_TOKENENDPT_MASK)
#define USB_TOKEN_TOKENPID_MASK                  0xF0u
#define USB_TOKEN_TOKENPID_SHIFT                 4
#define USB_TOKEN_TOKENPID_WIDTH                 4
#define USB_TOKEN_TOKENPID(x)                    (((uint8_t)(((uint8_t)(x))<<USB_TOKEN_TOKENPID_SHIFT))&USB_TOKEN_TOKENPID_MASK)
/* SOFTHLD Bit Fields */
#define USB_SOFTHLD_CNT_MASK                     0xFFu
#define USB_SOFTHLD_CNT_SHIFT                    0
#define USB_SOFTHLD_CNT_WIDTH                    8
#define USB_SOFTHLD_CNT(x)                       (((uint8_t)(((uint8_t)(x))<<USB_SOFTHLD_CNT_SHIFT))&USB_SOFTHLD_CNT_MASK)
/* BDTPAGE2 Bit Fields */
#define USB_BDTPAGE2_BDTBA_MASK                  0xFFu
#define USB_BDTPAGE2_BDTBA_SHIFT                 0
#define USB_BDTPAGE2_BDTBA_WIDTH                 8
#define USB_BDTPAGE2_BDTBA(x)                    (((uint8_t)(((uint8_t)(x))<<USB_BDTPAGE2_BDTBA_SHIFT))&USB_BDTPAGE2_BDTBA_MASK)
/* BDTPAGE3 Bit Fields */
#define USB_BDTPAGE3_BDTBA_MASK                  0xFFu
#define USB_BDTPAGE3_BDTBA_SHIFT                 0
#define USB_BDTPAGE3_BDTBA_WIDTH                 8
#define USB_BDTPAGE3_BDTBA(x)                    (((uint8_t)(((uint8_t)(x))<<USB_BDTPAGE3_BDTBA_SHIFT))&USB_BDTPAGE3_BDTBA_MASK)
/* ENDPT Bit Fields */
#define USB_ENDPT_EPHSHK		((uint8_t)0x1u)
#define USB_ENDPT_EPSTALL		((uint8_t)0x2u)
#define USB_ENDPT_EPTXEN		((uint8_t)0x4u)
#define USB_ENDPT_EPRXEN		((uint8_t)0x8u)
#define USB_ENDPT_EPCTLDIS		((uint8_t)0x10u)
#define USB_ENDPT_RETRYDIS		((uint8_t)0x40u)
#define USB_ENDPT_HOSTWOHUB		((uint8_t)0x80u)
/* USBCTRL Bit Fields */
#define USB_USBCTRL_PDE		((uint8_t)0x40u)
#define USB_USBCTRL_SUSP		((uint8_t)0x80u)
/* OBSERVE Bit Fields */
#define USB_OBSERVE_DMPD		((uint8_t)0x10u)
#define USB_OBSERVE_DPPD		((uint8_t)0x40u)
#define USB_OBSERVE_DPPU		((uint8_t)0x80u)
/* CONTROL Bit Fields */
#define USB_CONTROL_DPPULLUPNONOTG		((uint8_t)0x10u)
/* USBTRC0 Bit Fields */
#define USB_USBTRC0_USB_RESUME_INT		((uint8_t)0x1u)
#define USB_USBTRC0_SYNC_DET		((uint8_t)0x2u)
#define USB_USBTRC0_USB_CLK_RECOVERY_INT		((uint8_t)0x4u)
#define USB_USBTRC0_USBRESMEN		((uint8_t)0x20u)
#define USB_USBTRC0_USBRESET		((uint8_t)0x80u)
/* USBFRMADJUST Bit Fields */
#define USB_USBFRMADJUST_ADJ_MASK                0xFFu
#define USB_USBFRMADJUST_ADJ_SHIFT               0
#define USB_USBFRMADJUST_ADJ_WIDTH               8
#define USB_USBFRMADJUST_ADJ(x)                  (((uint8_t)(((uint8_t)(x))<<USB_USBFRMADJUST_ADJ_SHIFT))&USB_USBFRMADJUST_ADJ_MASK)
/* CLK_RECOVER_CTRL Bit Fields */
#define USB_CLK_RECOVER_CTRL_RESTART_IFRTRIM_EN		((uint8_t)0x20u)
#define USB_CLK_RECOVER_CTRL_RESET_RESUME_ROUGH_EN		((uint8_t)0x40u)
#define USB_CLK_RECOVER_CTRL_CLOCK_RECOVER_EN		((uint8_t)0x80u)
/* CLK_RECOVER_IRC_EN Bit Fields */
#define USB_CLK_RECOVER_IRC_EN_REG_EN		((uint8_t)0x1u)
#define USB_CLK_RECOVER_IRC_EN_IRC_EN		((uint8_t)0x2u)
/* CLK_RECOVER_INT_STATUS Bit Fields */
#define USB_CLK_RECOVER_INT_STATUS_OVF_ERROR		((uint8_t)0x10u)

/*!
 * @}
 */ /* end of group USB_Register_Masks */


/* USB - Peripheral instance base addresses */
/** Peripheral USB0 base address */
#define USB0_BASE                                (0x40072000u)
/** Peripheral USB0 base pointer */
#define USB0                                     ((USB_TypeDef *)USB0_BASE)
#define USB0_BASE_PTR                            (USB0)
/** Array initializer of USB peripheral base addresses */
#define USB_BASE_ADDRS                           { USB0_BASE }
/** Array initializer of USB peripheral base pointers */
#define USB_BASE_PTRS                            { USB0 }
/** Interrupt vectors for the USB peripheral type */
#define USB_IRQS                                 { USB0_IRQn }

/* ----------------------------------------------------------------------------
   -- USB - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup USB_Register_Accessor_Macros USB - Register accessor macros
 * @{
 */


/* USB - Register instance definitions */
/* USB0 */
#define USB0_PERID                               USB_PERID_REG(USB0)
#define USB0_IDCOMP                              USB_IDCOMP_REG(USB0)
#define USB0_REV                                 USB_REV_REG(USB0)
#define USB0_ADDINFO                             USB_ADDINFO_REG(USB0)
#define USB0_OTGISTAT                            USB_OTGISTAT_REG(USB0)
#define USB0_OTGICR                              USB_OTGICR_REG(USB0)
#define USB0_OTGSTAT                             USB_OTGSTAT_REG(USB0)
#define USB0_OTGCTL                              USB_OTGCTL_REG(USB0)
#define USB0_ISTAT                               USB_ISTAT_REG(USB0)
#define USB0_INTEN                               USB_INTEN_REG(USB0)
#define USB0_ERRSTAT                             USB_ERRSTAT_REG(USB0)
#define USB0_ERREN                               USB_ERREN_REG(USB0)
#define USB0_STAT                                USB_STAT_REG(USB0)
#define USB0_CTL                                 USB_CTL_REG(USB0)
#define USB0_ADDR                                USB_ADDR_REG(USB0)
#define USB0_BDTPAGE1                            USB_BDTPAGE1_REG(USB0)
#define USB0_FRMNUML                             USB_FRMNUML_REG(USB0)
#define USB0_FRMNUMH                             USB_FRMNUMH_REG(USB0)
#define USB0_TOKEN                               USB_TOKEN_REG(USB0)
#define USB0_SOFTHLD                             USB_SOFTHLD_REG(USB0)
#define USB0_BDTPAGE2                            USB_BDTPAGE2_REG(USB0)
#define USB0_BDTPAGE3                            USB_BDTPAGE3_REG(USB0)
#define USB0_ENDPT0                              USB_ENDPT_REG(USB0,0)
#define USB0_ENDPT1                              USB_ENDPT_REG(USB0,1)
#define USB0_ENDPT2                              USB_ENDPT_REG(USB0,2)
#define USB0_ENDPT3                              USB_ENDPT_REG(USB0,3)
#define USB0_ENDPT4                              USB_ENDPT_REG(USB0,4)
#define USB0_ENDPT5                              USB_ENDPT_REG(USB0,5)
#define USB0_ENDPT6                              USB_ENDPT_REG(USB0,6)
#define USB0_ENDPT7                              USB_ENDPT_REG(USB0,7)
#define USB0_ENDPT8                              USB_ENDPT_REG(USB0,8)
#define USB0_ENDPT9                              USB_ENDPT_REG(USB0,9)
#define USB0_ENDPT10                             USB_ENDPT_REG(USB0,10)
#define USB0_ENDPT11                             USB_ENDPT_REG(USB0,11)
#define USB0_ENDPT12                             USB_ENDPT_REG(USB0,12)
#define USB0_ENDPT13                             USB_ENDPT_REG(USB0,13)
#define USB0_ENDPT14                             USB_ENDPT_REG(USB0,14)
#define USB0_ENDPT15                             USB_ENDPT_REG(USB0,15)
#define USB0_USBCTRL                             USB_USBCTRL_REG(USB0)
#define USB0_OBSERVE                             USB_OBSERVE_REG(USB0)
#define USB0_CONTROL                             USB_CONTROL_REG(USB0)
#define USB0_USBTRC0                             USB_USBTRC0_REG(USB0)
#define USB0_USBFRMADJUST                        USB_USBFRMADJUST_REG(USB0)
#define USB0_CLK_RECOVER_CTRL                    USB_CLK_RECOVER_CTRL_REG(USB0)
#define USB0_CLK_RECOVER_IRC_EN                  USB_CLK_RECOVER_IRC_EN_REG(USB0)
#define USB0_CLK_RECOVER_INT_STATUS              USB_CLK_RECOVER_INT_STATUS_REG(USB0)

/* USB - Register array accessors */
#define USB0_ENDPT(index)                        USB_ENDPT_REG(USB0,index)

/*!
 * @}
 */ /* end of group USB_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group USB_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- VREF Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup VREF_Peripheral_Access_Layer VREF Peripheral Access Layer
 * @{
 */

/** VREF - Register Layout Typedef */
typedef struct {
  __IO uint8_t TRM;                                /**< VREF Trim Register, offset: 0x0 */
  __IO uint8_t SC;                                 /**< VREF Status and Control Register, offset: 0x1 */
} VREF_TypeDef, *VREF_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- VREF - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup VREF_Register_Accessor_Macros VREF - Register accessor macros
 * @{
 */


/* VREF - Register accessors */
#define VREF_TRM_REG(base)                       ((base)->TRM)
#define VREF_SC_REG(base)                        ((base)->SC)

/*!
 * @}
 */ /* end of group VREF_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- VREF Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup VREF_Register_Masks VREF Register Masks
 * @{
 */

/* TRM Bit Fields */
#define VREF_TRM_TRIM_MASK                       0x3Fu
#define VREF_TRM_TRIM_SHIFT                      0
#define VREF_TRM_TRIM_WIDTH                      6
#define VREF_TRM_TRIM(x)                         (((uint8_t)(((uint8_t)(x))<<VREF_TRM_TRIM_SHIFT))&VREF_TRM_TRIM_MASK)
#define VREF_TRM_CHOPEN		((uint8_t)0x40u)
/* SC Bit Fields */
#define VREF_SC_MODE_LV_MASK                     0x3u
#define VREF_SC_MODE_LV_SHIFT                    0
#define VREF_SC_MODE_LV_WIDTH                    2
#define VREF_SC_MODE_LV(x)                       (((uint8_t)(((uint8_t)(x))<<VREF_SC_MODE_LV_SHIFT))&VREF_SC_MODE_LV_MASK)
#define VREF_SC_VREFST		((uint8_t)0x4u)
#define VREF_SC_ICOMPEN		((uint8_t)0x20u)
#define VREF_SC_REGEN		((uint8_t)0x40u)
#define VREF_SC_VREFEN		((uint8_t)0x80u)

/*!
 * @}
 */ /* end of group VREF_Register_Masks */


/* VREF - Peripheral instance base addresses */
/** Peripheral VREF base address */
#define VREF_BASE                                (0x40074000u)
/** Peripheral VREF base pointer */
#define VREF                                     ((VREF_TypeDef *)VREF_BASE)
#define VREF_BASE_PTR                            (VREF)
/** Array initializer of VREF peripheral base addresses */
#define VREF_BASE_ADDRS                          { VREF_BASE }
/** Array initializer of VREF peripheral base pointers */
#define VREF_BASE_PTRS                           { VREF }

/* ----------------------------------------------------------------------------
   -- VREF - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup VREF_Register_Accessor_Macros VREF - Register accessor macros
 * @{
 */


/* VREF - Register instance definitions */
/* VREF */
#define VREF_TRM                                 VREF_TRM_REG(VREF)
#define VREF_SC                                  VREF_SC_REG(VREF)

/*!
 * @}
 */ /* end of group VREF_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group VREF_Peripheral_Access_Layer */


/* ----------------------------------------------------------------------------
   -- WDOG Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Peripheral_Access_Layer WDOG Peripheral Access Layer
 * @{
 */

/** WDOG - Register Layout Typedef */
typedef struct {
  __IO uint16_t STCTRLH;                           /**< Watchdog Status and Control Register High, offset: 0x0 */
  __IO uint16_t STCTRLL;                           /**< Watchdog Status and Control Register Low, offset: 0x2 */
  __IO uint16_t TOVALH;                            /**< Watchdog Time-out Value Register High, offset: 0x4 */
  __IO uint16_t TOVALL;                            /**< Watchdog Time-out Value Register Low, offset: 0x6 */
  __IO uint16_t WINH;                              /**< Watchdog Window Register High, offset: 0x8 */
  __IO uint16_t WINL;                              /**< Watchdog Window Register Low, offset: 0xA */
  __IO uint16_t REFRESH;                           /**< Watchdog Refresh register, offset: 0xC */
  __IO uint16_t UNLOCK;                            /**< Watchdog Unlock register, offset: 0xE */
  __IO uint16_t TMROUTH;                           /**< Watchdog Timer Output Register High, offset: 0x10 */
  __IO uint16_t TMROUTL;                           /**< Watchdog Timer Output Register Low, offset: 0x12 */
  __IO uint16_t RSTCNT;                            /**< Watchdog Reset Count register, offset: 0x14 */
  __IO uint16_t PRESC;                             /**< Watchdog Prescaler register, offset: 0x16 */
} WDOG_TypeDef, *WDOG_MemMapPtr;

/* ----------------------------------------------------------------------------
   -- WDOG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Accessor_Macros WDOG - Register accessor macros
 * @{
 */


/* WDOG - Register accessors */
#define WDOG_STCTRLH_REG(base)                   ((base)->STCTRLH)
#define WDOG_STCTRLL_REG(base)                   ((base)->STCTRLL)
#define WDOG_TOVALH_REG(base)                    ((base)->TOVALH)
#define WDOG_TOVALL_REG(base)                    ((base)->TOVALL)
#define WDOG_WINH_REG(base)                      ((base)->WINH)
#define WDOG_WINL_REG(base)                      ((base)->WINL)
#define WDOG_REFRESH_REG(base)                   ((base)->REFRESH)
#define WDOG_UNLOCK_REG(base)                    ((base)->UNLOCK)
#define WDOG_TMROUTH_REG(base)                   ((base)->TMROUTH)
#define WDOG_TMROUTL_REG(base)                   ((base)->TMROUTL)
#define WDOG_RSTCNT_REG(base)                    ((base)->RSTCNT)
#define WDOG_PRESC_REG(base)                     ((base)->PRESC)

/*!
 * @}
 */ /* end of group WDOG_Register_Accessor_Macros */


/* ----------------------------------------------------------------------------
   -- WDOG Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Masks WDOG Register Masks
 * @{
 */

/* STCTRLH Bit Fields */
/********  Bits definition for WDOG_STCTRLH register  ***********/
#define WDOG_STCTRLH_DISTESTWDOG     ((uint16_t)0x4000)
#define WDOG_STCTRLH_BYTESEL_1_0     ((uint16_t)0x3000)
#define WDOG_STCTRLH_TESTSEL         ((uint16_t)0x0800)
#define WDOG_STCTRLH_TESTWDOG        ((uint16_t)0x0400)
#define WDOG_STCTRLH_WAITEN          ((uint16_t)0x0080)
#define WDOG_STCTRLH_STOPEN          ((uint16_t)0x0040)
#define WDOG_STCTRLH_DBGEN           ((uint16_t)0x0020)
#define WDOG_STCTRLH_ALLOWUPDATE     ((uint16_t)0x0010)
#define WDOG_STCTRLH_WINEN           ((uint16_t)0x0008)
#define WDOG_STCTRLH_IRQRSTEN        ((uint16_t)0x0004)
#define WDOG_STCTRLH_CLKSRC          ((uint16_t)0x0002)
#define WDOG_STCTRLH_WDOGEN          ((uint16_t)0x0001)

/* STCTRLL Bit Fields */
#define WDOG_STCTRLL_INTFLG          ((uint16_t)0x8000)

/* TOVALH Bit Fields */
#define WDOG_TOVALH_TOVALHIGH_MASK               0xFFFFu
#define WDOG_TOVALH_TOVALHIGH_SHIFT              0
#define WDOG_TOVALH_TOVALHIGH_WIDTH              16
#define WDOG_TOVALH_TOVALHIGH(x)                 (((uint16_t)(((uint16_t)(x))<<WDOG_TOVALH_TOVALHIGH_SHIFT))&WDOG_TOVALH_TOVALHIGH_MASK)
/* TOVALL Bit Fields */
#define WDOG_TOVALL_TOVALLOW_MASK                0xFFFFu
#define WDOG_TOVALL_TOVALLOW_SHIFT               0
#define WDOG_TOVALL_TOVALLOW_WIDTH               16
#define WDOG_TOVALL_TOVALLOW(x)                  (((uint16_t)(((uint16_t)(x))<<WDOG_TOVALL_TOVALLOW_SHIFT))&WDOG_TOVALL_TOVALLOW_MASK)
/* WINH Bit Fields */
#define WDOG_WINH_WINHIGH_MASK                   0xFFFFu
#define WDOG_WINH_WINHIGH_SHIFT                  0
#define WDOG_WINH_WINHIGH_WIDTH                  16
#define WDOG_WINH_WINHIGH(x)                     (((uint16_t)(((uint16_t)(x))<<WDOG_WINH_WINHIGH_SHIFT))&WDOG_WINH_WINHIGH_MASK)
/* WINL Bit Fields */
#define WDOG_WINL_WINLOW_MASK                    0xFFFFu
#define WDOG_WINL_WINLOW_SHIFT                   0
#define WDOG_WINL_WINLOW_WIDTH                   16
#define WDOG_WINL_WINLOW(x)                      (((uint16_t)(((uint16_t)(x))<<WDOG_WINL_WINLOW_SHIFT))&WDOG_WINL_WINLOW_MASK)
/* REFRESH Bit Fields */
#define WDOG_REFRESH_WDOGREFRESH_MASK            0xFFFFu
#define WDOG_REFRESH_WDOGREFRESH_SHIFT           0
#define WDOG_REFRESH_WDOGREFRESH_WIDTH           16
#define WDOG_REFRESH_WDOGREFRESH(x)              (((uint16_t)(((uint16_t)(x))<<WDOG_REFRESH_WDOGREFRESH_SHIFT))&WDOG_REFRESH_WDOGREFRESH_MASK)
/* UNLOCK Bit Fields */
#define WDOG_UNLOCK_WDOGUNLOCK_MASK              0xFFFFu
#define WDOG_UNLOCK_WDOGUNLOCK_SHIFT             0
#define WDOG_UNLOCK_WDOGUNLOCK_WIDTH             16
#define WDOG_UNLOCK_WDOGUNLOCK(x)                (((uint16_t)(((uint16_t)(x))<<WDOG_UNLOCK_WDOGUNLOCK_SHIFT))&WDOG_UNLOCK_WDOGUNLOCK_MASK)
/* TMROUTH Bit Fields */
#define WDOG_TMROUTH_TIMEROUTHIGH_MASK           0xFFFFu
#define WDOG_TMROUTH_TIMEROUTHIGH_SHIFT          0
#define WDOG_TMROUTH_TIMEROUTHIGH_WIDTH          16
#define WDOG_TMROUTH_TIMEROUTHIGH(x)             (((uint16_t)(((uint16_t)(x))<<WDOG_TMROUTH_TIMEROUTHIGH_SHIFT))&WDOG_TMROUTH_TIMEROUTHIGH_MASK)
/* TMROUTL Bit Fields */
#define WDOG_TMROUTL_TIMEROUTLOW_MASK            0xFFFFu
#define WDOG_TMROUTL_TIMEROUTLOW_SHIFT           0
#define WDOG_TMROUTL_TIMEROUTLOW_WIDTH           16
#define WDOG_TMROUTL_TIMEROUTLOW(x)              (((uint16_t)(((uint16_t)(x))<<WDOG_TMROUTL_TIMEROUTLOW_SHIFT))&WDOG_TMROUTL_TIMEROUTLOW_MASK)
/* RSTCNT Bit Fields */
#define WDOG_RSTCNT_RSTCNT_MASK                  0xFFFFu
#define WDOG_RSTCNT_RSTCNT_SHIFT                 0
#define WDOG_RSTCNT_RSTCNT_WIDTH                 16
#define WDOG_RSTCNT_RSTCNT(x)                    (((uint16_t)(((uint16_t)(x))<<WDOG_RSTCNT_RSTCNT_SHIFT))&WDOG_RSTCNT_RSTCNT_MASK)
/*********  Bits definition for WDOG_PRESC register  ************/
#define WDOG_PRESC_PRESCVAL          ((uint16_t)0x0700)

/*!
 * @}
 */ /* end of group WDOG_Register_Masks */


/* WDOG - Peripheral instance base addresses */
/** Peripheral WDOG base address */
#define WDOG_BASE                                (0x40052000u)
/** Peripheral WDOG base pointer */
#define WDOG                                     ((WDOG_TypeDef *)WDOG_BASE)
#define WDOG_BASE_PTR                            (WDOG)
/** Array initializer of WDOG peripheral base addresses */
#define WDOG_BASE_ADDRS                          { WDOG_BASE }
/** Array initializer of WDOG peripheral base pointers */
#define WDOG_BASE_PTRS                           { WDOG }
/** Interrupt vectors for the WDOG peripheral type */
#define WDOG_IRQS                                { WDOG_EWM_IRQn }

/* ----------------------------------------------------------------------------
   -- WDOG - Register accessor macros
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup WDOG_Register_Accessor_Macros WDOG - Register accessor macros
 * @{
 */


/* WDOG - Register instance definitions */
/* WDOG */
#define WDOG_STCTRLH                             WDOG_STCTRLH_REG(WDOG)
#define WDOG_STCTRLL                             WDOG_STCTRLL_REG(WDOG)
#define WDOG_TOVALH                              WDOG_TOVALH_REG(WDOG)
#define WDOG_TOVALL                              WDOG_TOVALL_REG(WDOG)
#define WDOG_WINH                                WDOG_WINH_REG(WDOG)
#define WDOG_WINL                                WDOG_WINL_REG(WDOG)
#define WDOG_REFRESH                             WDOG_REFRESH_REG(WDOG)
#define WDOG_UNLOCK                              WDOG_UNLOCK_REG(WDOG)
#define WDOG_TMROUTH                             WDOG_TMROUTH_REG(WDOG)
#define WDOG_TMROUTL                             WDOG_TMROUTL_REG(WDOG)
#define WDOG_RSTCNT                              WDOG_RSTCNT_REG(WDOG)
#define WDOG_PRESC                               WDOG_PRESC_REG(WDOG)

/*!
 * @}
 */ /* end of group WDOG_Register_Accessor_Macros */


/*!
 * @}
 */ /* end of group WDOG_Peripheral_Access_Layer */


/*
** End of section using anonymous unions
*/

#if defined(__ARMCC_VERSION)
  #pragma pop
#elif defined(__CWCC__)
  #pragma pop
#elif defined(__GNUC__)
  /* leave anonymous unions enabled */
#elif defined(__IAR_SYSTEMS_ICC__)
  #pragma language=default
#else
  #error Not supported compiler type
#endif

/*!
 * @}
 */ /* end of group Peripheral_access_layer */


/* ----------------------------------------------------------------------------
   -- Backward Compatibility
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup Backward_Compatibility_Symbols Backward Compatibility
 * @{
 */

#define  MCG_C2_EREFS0_MASK          MCG_C2_EREFS_MASK
#define  MCG_C2_EREFS0_SHIFT         MCG_C2_EREFS_SHIFT
#define  MCG_C2_HGO0_MASK            MCG_C2_HGO_MASK
#define  MCG_C2_HGO0_SHIFT           MCG_C2_HGO_SHIFT
#define  MCG_C2_RANGE0_MASK          MCG_C2_RANGE_MASK
#define  MCG_C2_RANGE0_SHIFT         MCG_C2_RANGE_SHIFT
#define  MCG_C2_RANGE0(x)            MCG_C2_RANGE(x)
#define ADC_BASES                    ADC_BASE_PTRS
#define CMP_BASES                    CMP_BASE_PTRS
#define CRC_BASES                    CRC_BASE_PTRS
#define DAC_BASES                    DAC_BASE_PTRS
#define DMA_BASES                    DMA_BASE_PTRS
#define DMAMUX_BASES                 DMAMUX_BASE_PTRS
#define EWM_BASES                    EWM_BASE_PTRS
#define FB_BASES                     FB_BASE_PTRS
#define FMC_BASES                    FMC_BASE_PTRS
#define FTFA_BASES                   FTFA_BASE_PTRS
#define FTM_BASES                    FTM_BASE_PTRS
#define GPIO_BASES                   GPIO_BASE_PTRS
#define I2C_BASES                    I2C_BASE_PTRS
#define I2S_BASES                    I2S_BASE_PTRS
#define LLWU_BASES                   LLWU_BASE_PTRS
#define LPTMR_BASES                  LPTMR_BASE_PTRS
#define LPUART_BASES                 LPUART_BASE_PTRS
#define MCG_BASES                    MCG_BASE_PTRS
#define MCM_ISR_REG(base)            MCM_ISCR_REG(base)
#define MCM_ISR_FIOC_MASK            MCM_ISCR_FIOC_MASK
#define MCM_ISR_FIOC_SHIFT           MCM_ISCR_FIOC_SHIFT
#define MCM_ISR_FDZC_MASK            MCM_ISCR_FDZC_MASK
#define MCM_ISR_FDZC_SHIFT           MCM_ISCR_FDZC_SHIFT
#define MCM_ISR_FOFC_MASK            MCM_ISCR_FOFC_MASK
#define MCM_ISR_FOFC_SHIFT           MCM_ISCR_FOFC_SHIFT
#define MCM_ISR_FUFC_MASK            MCM_ISCR_FUFC_MASK
#define MCM_ISR_FUFC_SHIFT           MCM_ISCR_FUFC_SHIFT
#define MCM_ISR_FIXC_MASK            MCM_ISCR_FIXC_MASK
#define MCM_ISR_FIXC_SHIFT           MCM_ISCR_FIXC_SHIFT
#define MCM_ISR_FIDC_MASK            MCM_ISCR_FIDC_MASK
#define MCM_ISR_FIDC_SHIFT           MCM_ISCR_FIDC_SHIFT
#define MCM_ISR_FIOCE_MASK           MCM_ISCR_FIOCE_MASK
#define MCM_ISR_FIOCE_SHIFT          MCM_ISCR_FIOCE_SHIFT
#define MCM_ISR_FDZCE_MASK           MCM_ISCR_FDZCE_MASK
#define MCM_ISR_FDZCE_SHIFT          MCM_ISCR_FDZCE_SHIFT
#define MCM_ISR_FOFCE_MASK           MCM_ISCR_FOFCE_MASK
#define MCM_ISR_FOFCE_SHIFT          MCM_ISCR_FOFCE_SHIFT
#define MCM_ISR_FUFCE_MASK           MCM_ISCR_FUFCE_MASK
#define MCM_ISR_FUFCE_SHIFT          MCM_ISCR_FUFCE_SHIFT
#define MCM_ISR_FIXCE_MASK           MCM_ISCR_FIXCE_MASK
#define MCM_ISR_FIXCE_SHIFT          MCM_ISCR_FIXCE_SHIFT
#define MCM_ISR_FIDCE_MASK           MCM_ISCR_FIDCE_MASK
#define MCM_ISR_FIDCE_SHIFT          MCM_ISCR_FIDCE_SHIFT
#define MCM_BASES                    MCM_BASE_PTRS
#define NV_BASES                     NV_BASE_PTRS
#define OSC_BASES                    OSC_BASE_PTRS
#define PDB_BASES                    PDB_BASE_PTRS
#define PIT_BASES                    PIT_BASE_PTRS
#define PMC_BASES                    PMC_BASE_PTRS
#define PORT_BASES                   PORT_BASE_PTRS
#define RCM_BASES                    RCM_BASE_PTRS
#define RFSYS_BASES                  RFSYS_BASE_PTRS
#define RFVBAT_BASES                 RFVBAT_BASE_PTRS
#define RNG_BASES                    RNG_BASE_PTRS
#define RTC_BASES                    RTC_BASE_PTRS
#define SIM_BASES                    SIM_BASE_PTRS
#define SMC_BASES                    SMC_BASE_PTRS
#define SPI_BASES                    SPI_BASE_PTRS
#define UART_BASES                   UART_BASE_PTRS
#define USB_BASES                    USB_BASE_PTRS
#define VREF_BASES                   VREF_BASE_PTRS
#define WDOG_BASES                   WDOG_BASE_PTRS
#define USB_ADDINFO_IRQNUM_MASK      This_symbol_has_been_deprecated
#define USB_ADDINFO_IRQNUM_SHIFT     This_symbol_has_been_deprecated
#define USB_ADDINFO_IRQNUM(x)        This_symbol_has_been_deprecated
#define Watchdog_IRQn                WDOG_EWM_IRQn
#define Watchdog_IRQHandler          WDOG_EWM_IRQHandler
#define LPTimer_IRQn                 LPTMR0_IRQn
#define LPTimer_IRQHandler           LPTMR0_IRQHandler
#define LLW_IRQn                     LLWU_IRQn
#define LLW_IRQHandler               LLWU_IRQHandler

/*!
 * @}
 */ /* end of group Backward_Compatibility_Symbols */


#else /* #if !defined(MK22F51212_H_) */
  /* There is already included the same memory map. Check if it is compatible (has the same major version) */
  #if (MCU_MEM_MAP_VERSION != 0x0200u)
    #if (!defined(MCU_MEM_MAP_SUPPRESS_VERSION_WARNING))
      #warning There are included two not compatible versions of memory maps. Please check possible differences.
    #endif /* (!defined(MCU_MEM_MAP_SUPPRESS_VERSION_WARNING)) */
  #endif /* (MCU_MEM_MAP_VERSION != 0x0200u) */
#endif  /* #if !defined(MK22F51212_H_) */

/* MK22F51212.h, eof. */

