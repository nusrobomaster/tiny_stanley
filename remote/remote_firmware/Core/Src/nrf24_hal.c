#include "nrf24_hal.h"


// Configure the GPIO lines of the nRF24L01 transceiver
// note: IRQ pin must be configured separately
void nRF24_GPIO_Init(void) {
}

// Low level SPI transmit/receive function (hardware depended)
// input:
//   data - value to transmit via SPI
// return: value received from SPI
uint8_t nRF24_LL_RW(uint8_t data) {
	 // Wait until TX buffer is empty
	while (nRF24_SPI_PORT.State != HAL_SPI_STATE_READY);
	uint8_t rx_buffer = 0;
	HAL_SPI_TransmitReceive(&nRF24_SPI_PORT, &data, &rx_buffer, 1, 100);
	// Return received byte
	return rx_buffer;
}
