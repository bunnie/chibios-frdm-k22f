/*
    ChibiOS - Copyright (C) 2014 Derek Mulcahy

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
 * @file    KL2x/kinetis_registry.h
 * @brief   KL2x capabilities registry.
 *
 * @addtogroup HAL
 * @{
 */

#ifndef _KINETIS_REGISTRY_H_
#define _KINETIS_REGISTRY_H_

/*===========================================================================*/
/* Platform capabilities.                                                    */
/*===========================================================================*/

/**
 * @name    K22x capabilities - VectorN, where N is the physical address of the vector in hex
 * @{
 */
/* EXT attributes.*/

#define KINETIS_PORTA_IRQ_VECTOR    Vector12C
#define KINETIS_PORTB_IRQ_VECTOR    Vector130
#define KINETIS_PORTC_IRQ_VECTOR    Vector134
#define KINETIS_PORTD_IRQ_VECTOR    Vector138
#define KINETIS_PORTE_IRQ_VECTOR    Vector13C

/* ADC attributes.*/
#define KINETIS_HAS_ADC0            TRUE
#define KINETIS_ADC0_IRQ_VECTOR     VectorDC

/* I2C attributes.*/
#define KINETIS_I2C0_IRQ_VECTOR     VectorA0
#define KINETIS_I2C1_IRQ_VECTOR     VectorA4

/* USB attributes */
#define KINETIS_USB_IRQ_VECTOR      Vector114

/* SPI attributes */
#define KINETIS_SPI0_IRQ_VECTOR     VectorA8
#define KINETIS_SPI1_IRQ_VECTOR     VectorAC
#define KINETIS_DMA0_IRQ_VECTOR     Vector40  // using DMA0_IRQn

/* I2S attributes */
#define KINETIS_I2S0_TX_VECTOR      VectorB0
#define KINETIS_I2S0_RX_VECTOR      VectorB4

/* UART attributes */
#define KINETIS_UART0_STAT_VECTOR   VectorBC
#define KINETIS_UART0_ERR_VECTOR    VectorC0
#define KINETIS_UART1_STAT_VECTOR   VectorC4
#define KINETIS_UART1_ERR_VECTOR    VectorC8
#define KINETIS_UART2_STAT_VECTOR   VectorCC
#define KINETIS_UART2_ERR_VECTOR    VectorD0

/** @} */

#endif /* _KINETIS_REGISTRY_H_ */

/** @} */
