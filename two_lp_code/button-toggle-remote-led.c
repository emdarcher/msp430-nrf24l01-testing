//toggles the remote led using a the button on the lp connected to P1.3

#include <msp430.h>
#include "msprf24.h"
#include "nrf_userconfig.h"
#include <string.h>

volatile unsigned int user;

void setup_button(void);

volatile int wdtsleep;

/* Sleep for <cycles> * 47ms */
void wdt_sleep(unsigned int cycles)
{
        wdtsleep = cycles;
        WDTCTL = WDTPW | WDTTMSEL | WDTCNTCL | WDTSSEL | WDTIS1;  // WDT interval = 512 VLOCLK cycles, about 47ms
        IFG1 &= ~WDTIFG;
        IE1 |= WDTIE;
        LPM3;
        WDTCTL = WDTPW | WDTHOLD;
}

// WDT overflow/STOP
//#pragma vector=WDT_VECTOR
//__interrupt void WDT_ISR(void)

__attribute__((interrupt(WDT_VECTOR)))
void WDT_ISR(void)
{
        if (wdtsleep) {
                wdtsleep--;
        } else {
                IFG1 &= ~WDTIFG;
                IE1 &= ~WDTIE;
                __bic_SR_register_on_exit(LPM3_bits);
        }
}


int main()
{
	uint8_t addr[5];
	uint8_t buf[32];

	WDTCTL = WDTHOLD | WDTPW;
	DCOCTL = CALDCO_16MHZ;
	BCSCTL1 = CALBC1_16MHZ;
	BCSCTL2 = DIVS_1;  // SMCLK = DCOCLK/2
	// SPI (USCI) uses SMCLK, prefer SMCLK < 10MHz (SPI speed limit for nRF24 = 10MHz)

	// Red, Green LED used for status
	//P1DIR |= 0x41;
	//P1OUT &= ~0x41;

        BCSCTL3 = LFXT1S_2;  // ACLK = VLOCLK/1
        BCSCTL3 &= ~(XT2OF|LFXT1OF);

	wdtsleep = 0;
    
    setup_button();
    
	user = 0xFE;

	/* Initial values for nRF24L01+ library config variables */
	rf_crc = RF24_EN_CRC; // CRC enabled, 8-bit
	rf_addr_width      = 5;
	rf_speed_power     = RF24_SPEED_MIN | RF24_POWER_MAX;
	rf_channel         = 120;
	msprf24_init();  // All RX pipes closed by default
	msprf24_set_pipe_packetsize(0, 0);//dynamic packet size
	msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst enabled for receiving Auto-ACKs
	// Note: Pipe#0 is hardcoded in the transceiver hardware as the designated "pipe" for a TX node to receive
	// auto-ACKs.  This does not have to match the pipe# used on the RX side.

	// Transmit to 'rad01' (0x72 0x61 0x64 0x30 0x31)
	msprf24_standby();
	user = msprf24_current_state();
	//addr[0] = 'r'; addr[1] = 'a'; addr[2] = 'd'; addr[3] = '0'; addr[4] = '1';
    memcpy(addr, "\xDE\xAD\xBE\xEF\x00", 5);
	w_tx_addr(addr);
	w_rx_addr(0, addr);  // Pipe 0 receives auto-ack's, autoacks are sent back to the TX addr so the PTX node
			     // needs to listen to the TX addr on pipe#0 to receive them.
	buf[0] = '1';
	buf[1] = '\0';
	w_tx_payload(32, buf);
	msprf24_activate_tx();
	//LPM4;
    //LPM3;
    while(1) {
		if (rf_irq & RF24_IRQ_FLAGGED) {  // Just acknowledging previous packet here
			msprf24_get_irq_reason();
			msprf24_irq_clear(RF24_IRQ_MASK);
			user = msprf24_get_last_retransmits();
		} else {  // WDT sleep completed, send a new packet
			/*if (buf[0] == '1')
				buf[0] = '0';
			else
				buf[0] = '1';*/
                
            if((P1IN & BIT3)==0){
                buf[0] = '1';
            }else{
                buf[0] = '0';
            }
			w_tx_payload(2, buf);
			msprf24_activate_tx();
		}

		wdt_sleep(1);  // ~500ms LPM3 sleep
	}
    
/*
	if (rf_irq & RF24_IRQ_FLAGGED) {
		msprf24_get_irq_reason();
		if (rf_irq & RF24_IRQ_TX)
			P1OUT |= 0x40;  // Green LED
		if (rf_irq & RF24_IRQ_TXFAILED)
			P1OUT |= 0x01;  // Red LED

		msprf24_irq_clear(rf_irq);
		user = msprf24_get_last_retransmits();
	}*/
    
	return 0;
}


void setup_button(void){
//setup button on P1.3 for irq
    
    P1DIR &= ~BIT3;
    P1REN |= BIT3;
    P1OUT |= BIT3;
    
}
