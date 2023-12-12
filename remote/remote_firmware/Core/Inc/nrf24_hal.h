#ifndef __NRF24_HAL_H
#define __NRF24_HAL_H


// Hardware abstraction layer for NRF24L01+ transceiver (hardware depended functions)
// GPIO pins definition
// GPIO pins initialization and control functions
// SPI transmit functions


// Peripheral libraries
#include <stm32f1xx.h>
#include <stm32f1xx_ll_gpio.h>
#include <spi.h>


// SPI port peripheral
#define nRF24_SPI_PORT             hspi1

// nRF24 GPIO peripherals
#define nRF24_GPIO_PERIPHERALS     RCC_APB2ENR_IOPBEN

// CE (chip enable) pin (PB11)
#define nRF24_CE_PORT              GPIOB
#define nRF24_CE_PIN               LL_GPIO_PIN_1
#define nRF24_CE_L                 LL_GPIO_ResetOutputPin(nRF24_CE_PORT, nRF24_CE_PIN)
#define nRF24_CE_H                 LL_GPIO_SetOutputPin(nRF24_CE_PORT, nRF24_CE_PIN)

// CSN (chip select negative) pin (PB12)
#define nRF24_CSN_PORT             GPIOA
#define nRF24_CSN_PIN              LL_GPIO_PIN_4
#define nRF24_CSN_L                LL_GPIO_ResetOutputPin(nRF24_CSN_PORT, nRF24_CSN_PIN)
#define nRF24_CSN_H                LL_GPIO_SetOutputPin(nRF24_CSN_PORT, nRF24_CSN_PIN)

// IRQ pin (PB10)
#define nRF24_IRQ_PORT             GPIOB
#define nRF24_IRQ_PIN              LL_GPIO_PIN_0


// Macros for the RX on/off
#define nRF24_RX_ON                nRF24_CE_H
#define nRF24_RX_OFF               nRF24_CE_L


// Function prototypes
void nRF24_GPIO_Init(void);
uint8_t nRF24_LL_RW(uint8_t data);

#endif // __NRF24_HAL_H
