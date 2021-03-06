#include <msp430.h>
#include <string.h>
#include "msprf24.h"
#include "nrf_userconfig.h"

volatile unsigned int user;

int main()
{
	uint8_t addr[5];
	uint8_t buf[32];

	WDTCTL = WDTHOLD | WDTPW;
	DCOCTL = CALDCO_16MHZ;
	BCSCTL1 = CALBC1_16MHZ;
	BCSCTL2 = DIVS_1;  // SMCLK = DCOCLK/4
	// SPI (USCI) uses SMCLK, prefer SMCLK < 10MHz (SPI speed limit for nRF24 = 10MHz)
	user = 0xFE;

	// Red LED will be our output
	P1DIR |= BIT0;
	P1OUT &= ~BIT0;

	/* Initial values for nRF24L01+ library config variables */
	rf_crc = RF24_EN_CRC; // CRC enabled, 8-bit
	rf_addr_width      = 5;
	rf_speed_power     = RF24_SPEED_MIN | RF24_POWER_MAX;
	rf_channel         = 120;
	msprf24_init();
	msprf24_set_pipe_packetsize(0, 0);
	msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst

	// Set our RX address
	memcpy(addr, "\xDE\xAD\xBE\xEF\x01", 5);
	w_rx_addr(0, addr);

	// Receive mode
	if (!(RF24_QUEUE_RXEMPTY & msprf24_queue_state())) {
		flush_rx();
	}
	msprf24_activate_rx();
	LPM4;

	while (1) {
		if (rf_irq & RF24_IRQ_FLAGGED) {
			msprf24_get_irq_reason();
		}
		if (rf_irq & RF24_IRQ_RX || msprf24_rx_pending()) {
			r_rx_payload(r_rx_peek_payload_size(), buf);
			msprf24_irq_clear(RF24_IRQ_RX);
			user = buf[0];

			if (buf[0] == '0')
				P1OUT &= ~BIT0;
			if (buf[0] == '1')
				P1OUT |= BIT0;

		} else {
			user = 0xFF;
		}
		LPM4;
	}
	return 0;
}
