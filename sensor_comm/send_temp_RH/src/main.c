
//send value over radio to a reciever

#include <msp430.h>
#include <string.h>
#include "msprf24.h"
#include "nrf_userconfig.h"

volatile unsigned int user;
volatile int wdtsleep;

uint16_t ADC_read(void);
void ADC_init(void);
void Temp_ADC_init(void);

void halt_timerA(void); //use when not using timer

void freq_timerA_init(void);
volatile uint32_t input_frequency;
volatile uint32_t input_period;
volatile uint16_t previous_ccr;

#define TIMERA_DFLT_MODE_BITS MC_2 //continuous mode

uint32_t read_frequency(void);
uint32_t read_avg_freq(uint8_t avg_cnt);

uint32_t read_period(void);
uint32_t read_avg_period(uint8_t avg_cnt);

#define SMCLK_SPEED 1000000 //put the speed that SMCLK will be here

uint8_t period_to_RH(uint32_t in_per);
uint8_t freq_to_RH(uint16_t in_freq);

volatile uint16_t adc_val;
#define A4 BIT4



//clabration data aquisition code from this link:
// http://forum.43oh.com/topic/2027-how-to-use-temperature-calibration-data/
// helps quite a bit.

unsigned verify_info_chk(const unsigned * const begin, const unsigned * const end)
{
    const unsigned *p = begin + 1;                      // Begin at word after checksum
    unsigned chk = 0;                                   // Init checksum
    while(p < end) chk ^= *p++;                         // XOR all words in segment
    return chk + *begin;                                // Add checksum - result will be zero if checksum is valid
}

void const *find_tag(const unsigned * const begin, const unsigned * const end, const unsigned tag)
{
    const unsigned *p = begin + 1;                      // Begin at word after checksum
    do {                                                //
        const unsigned d = *p++;                        // Get a word
        if((d & 0xFF) == tag) return (void *)p;         // Return pointer if tag matches
        p += (d >> 9);                                  // Point to next tag
    } while(p < end);                                   // While still within segment
    return 0;                                           // Not found, return NULL pointer
}
typedef struct {
    unsigned adc_gain;                                  // ADC gain
    unsigned adc_offet;                                 // ADC offset
    unsigned ref15;                                     // ADC value of 1.5 volt input when using 1.5 volt reference
    unsigned t3015;                                     // ADC value of 30C when using 1.5 volt reference
    unsigned t8515;                                     // ADC value of 85C when using 1.5 volt reference
    unsigned ref25;                                     // ADC value of 2.5 volt input when using 2.5 volt reference
    unsigned t3025;                                     // ADC value of 30C when using 2.5 volt reference
    unsigned t8525;                                     // ADC value of 85C when using 2.5 volt reference
} TCAL;

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
    //DCOCTL = CALDCO_16MHZ;
    //BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_1MHZ;
    BCSCTL1 = CALBC1_1MHZ;
    BCSCTL2 = DIVS_0;  // SMCLK = DCOCLK/1
        BCSCTL3 = LFXT1S_2;  // ACLK = VLOCLK/1
        BCSCTL3 &= ~(XT2OF|LFXT1OF);


    //init the ADC
    //ADC_init();
    Temp_ADC_init();

//stuff for cal
const unsigned * const info_seg_a = (unsigned *)0x10C0;     // Address of info segement A
//const unsigned * const info_seg_a = (unsigned *)0x10DA;     // Address of info segement A
    const unsigned * const info_seg_a_end = info_seg_a + 32;    // 32 words in each segment
    

const TCAL * const cal = (TCAL *)(verify_info_chk(info_seg_a, info_seg_a_end) \
         ? 0 \
         : find_tag(info_seg_a, info_seg_a_end, 0x08));
    const long cc_scale  = cal ? 3604480L / (cal->t8515 - cal->t3015) : 0;
    const long cf_scale  = cal ? 6488064L / (cal->t8515 - cal->t3015) : 0;
    const long cc_offset = cal ? 1998848L - (cal->t3015 * cc_scale) : 0;
    const long cf_offset = cal ? 5668864L - (cal->t3015 * cf_scale) : 0;
    const long ck_offset = cc_offset + 17901158L;


    wdtsleep = 0;

    // SPI (USI) uses SMCLK, prefer SMCLK=DCO (no clock division)
    user = 0xFE;

    /* Initial values for nRF24L01+ library config variables */
    rf_crc = RF24_EN_CRC; // CRC enabled, 8-bit
    //| RF24_CRCO;
    rf_addr_width      = 5;
    //rf_speed_power     = RF24_SPEED_1MBPS | RF24_POWER_MAX;
    rf_speed_power      = RF24_SPEED_MIN | RF24_POWER_MAX;
    rf_channel         = 120;
    msprf24_init();  // All RX pipes closed by default
    msprf24_set_pipe_packetsize(0, 0);
    msprf24_open_pipe(0, 1);  // Open pipe#0 with Enhanced ShockBurst enabled for receiving Auto-ACKs
    // Note: Pipe#0 is hardcoded in the transceiver hardware as the designated "pipe" for a TX node to receive
    // auto-ACKs.  This does not have to match the pipe# used on the RX side.

    // Transmit to 0xDEADBEEF00
    msprf24_standby();
    user = msprf24_current_state();
    //addr[0] = 'r'; addr[1] = 'a'; addr[2] = 'd'; addr[3] = '0'; addr[4] = '1';
    memcpy(addr, "\xDE\xAD\xBE\xEF\x00", 5);
    w_tx_addr(addr);
    w_rx_addr(0, addr);  // Pipe 0 receives auto-ack's, autoacks are sent back to the TX addr so the PTX node
    // needs to listen to the TX addr on pipe#0 to receive them.
    buf[0] = '0';
    //buf[1]='\0';
    buf[2] = '\0';
    while(1) {
        if (rf_irq & RF24_IRQ_FLAGGED) {  // Just acknowledging previous packet here
            msprf24_get_irq_reason();
            msprf24_irq_clear(RF24_IRQ_MASK);
            user = msprf24_get_last_retransmits();
        } else {  // WDT sleep completed, send a new packet
            //if (buf[0] == '1')
            //	buf[0] = '0';
            //else
            //	buf[0] = '1';

            //buf[] = snprintf(buf,32,"%d",ADC_read());
            adc_val = ADC_read();
            
            //adc_val = 1023;
            //buf[0] = (uint8_t)(adc_val>>8);//get high byte
            //buf[1] = (uint8_t)(adc_val);//low byte
            //buf[0]=(uint8_t)(adc_val>>2);//8 bits?
            
            //F
            //buf[1]= ((48724L * adc_val) -  30634388L) >> 16;
            buf[0]=((cf_scale * adc_val) + cf_offset) >> 16; //calibrated
            
            //celsius
            //buf[1]=((27069L * adc_val) -  18169625L) >> 16; 
                //calibrated
                buf[1]= ((cc_scale * adc_val) + cc_offset) >> 16;
            
            w_tx_payload(32, buf);
            msprf24_activate_tx();
        }

        wdt_sleep(2);  //
    }
    return 0;
}

void halt_timerA( uint8_t halt_or_not ){
    //halts timerA if you want it to
    
    if(halt_or_not){
        //halt if "1"
        TACTL &= ~(MC0|MC1);
    } else {
        //undo halt and set to a default mode
        TACTL &= ~(MC0|MC1)
        TACTL |= TIMERA_DFLT_MODE_BITS;//set in define
    }
    
}

void freq_timerA_init(void){
    //initializes timerA for capturing frequency 
    
    //setup input pin gpio
    P1DIR &= ~BIT1; //set P1.1 for input
    P1SEL |= BIT1; //set for afio as T0.0
    
    //setup timer
    //TASSEL bit 1, for smclk input
    //no div
    //defined mode bits, should be continous
    //
    TACTL |= ( TASSEL1 | TIMERA_DFLT_MODE_BITS  );
    
    //init capture/compare 0 
    //CMx bit 0, for capture on rising edge
    //CAP, set for capture
    //CCIE, capture/compare interrupt enable
    TACCTL0 |= ( CM0 | CAP | CCIE );
    
    
}

uint8_t freq_to_RH(uint32_t in_freq){
    //takes a frequency in Hz and returns corresponding relative humidity %
    
    /*
     * from datasheet for HS1101LF, equations to get RH%
     * from the freq output of a 555 timer circuit
     * with the HS1101LF as a variable capacitor in astable circuit
     * 
     *  thigh = C@%RH*(R22+R4)*ln2
        tlow= C@%RH*R22*ln2
        F = 1/(thigh+tlow) = 1/(C@%RH*(R4+2*R22)*ln2)
        Output duty cycle = thigh*F = R22/(R4+2*R22)
    
    period
    */
    
    
    
    
}


uint8_t period_to_RH(uint32_t in_per){
    
    //takes a frequency in Hz and returns corresponding relative humidity %
    
    /*
     * from datasheet for HS1101LF, equations to get RH%
     * from the freq output of a 555 timer circuit
     * with the HS1101LF as a variable capacitor in astable circuit
     * 
     *  thigh = C@%RH*(R22+R4)*ln2
        tlow= C@%RH*R22*ln2
        F = 1/(thigh+tlow) = 1/(C@%RH*(R4+2*R22)*ln2)
        Output duty cycle = thigh*F = R22/(R4+2*R22)
    
    period= (thigh+tlow)
    * period = (C@%RH*(R4+2*R22)*ln(2))
    * R22=499k ohm resistor 0.5% tolerance
    * R4= 49.9k ohm resistor 1% tolerance
    * period = (C@%RH*(49900+2*499000)*ln(2))
    *  
    *
    */
    
    
    
}


void ADC_init(void) {
            // Use Vcc/Vss for Up/Low Refs, 16 x ADC10CLKs, turn on ADC
    ADC10CTL0 = SREF_0 + ADC10SHT_2 + ADC10ON;
            // A4 input, use ADC10CLK div 1, single channel mode  
    ADC10CTL1 =  INCH_4 + SHS_0 + ADC10SSEL_0 + ADC10DIV_0 + CONSEQ_0;
    ADC10AE0 = A4;      // Enable ADC input on P1.1
}

void Temp_ADC_init(void){
    
    ADC10CTL0 = 0;                                      // Configure ADC
    ADC10CTL1 = INCH_10 | ADC10DIV_3;                   //
    ADC10CTL0 = SREF_1 | ADC10SHT_3 | REFON | ADC10ON | ADC10IE;
    
    ADC10CTL0 |= ADC10IE;                               // Enable ADC conversion complete interrupt
}
uint32_t read_period(void){
    
    //un-halt the timer if halted
    halt_timerA(0);
    
    
    //go into LPM0 
    __bis_SR_register(LPM0_bits + GIE);//wait for capture 1
    __nop();
    //then capture 2
    __bis_SR_register(LPM0_bits + GIE);
    
    //uint32_t temp_period_store = input_period;
    halt_timerA(1); //halt the timer
    
    return input_period;
    
}
uint32_t read_avg_period(uint8_t avg_cnt){
    
    //avg_cnt is the number of samples to take in the avg
    
    uint32_t temp_period_store;
    
    //un-halt the timer if halted
    halt_timerA(0);
    
    //get capture 0 to initiate
    __bis_SR_register(LPM0_bits + GIE);
    
    while(avg_cnt--){
        //loop amount of avg_cnt to take multiple vals for avg
        
        __bis_SR_register(LPM0_bits + GIE);//wait for a capture
        
        temp_period_store += input_period; //add the value
    }
    
    temp_period_store /= avg_cnt; //devide sum by avg_cnt to get avg
    
    return temp_period_store;
}

uint32_t read_frequency(void){
    //reads a single frequency
    
    //un-halt the timer if halted
    halt_timerA(0);
    
    
    //go into LPM0 
    __bis_SR_register(LPM0_bits + GIE);//wait for capture 1
    __nop();
    //then capture 2
    __bis_SR_register(LPM0_bits + GIE);
    
    //uint32_t temp_period_store = input_period;
    halt_timerA(1); //halt the timer
    
    return (uint32_t)(SMCLK_SPEED/input_period); //return frequency in Hz

}

uint32_t read_avg_freq(uint8_t avg_cnt){
    //avg_cnt is the number of samples to take in the avg
    
    uint32_t temp_period_store;
    
    //un-halt the timer if halted
    halt_timerA(0);
    
    //get capture 0 to initiate
    __bis_SR_register(LPM0_bits + GIE);
    
    while(avg_cnt--){
        //loop amount of avg_cnt to take multiple vals for avg
        
        __bis_SR_register(LPM0_bits + GIE);//wait for a capture
        
        temp_period_store += input_period; //add the value
    }
    
    temp_period_store /= avg_cnt; //devide sum by avg_cnt to get avg
    
    return (uint32_t)(SMCLK_SPEED/temp_period_store);//return freq in Hz
}


uint16_t ADC_read(void){
	uint16_t val=0;
    ADC10CTL0 |= ENC+ ADC10SC;     // Enable conversions.
	//while ((ADC10CTL1 & ADC10BUSY) == 0x01){
	//}   // wait for conversion to end
    
    //LPM0; //will exit when ADC done
    __bis_SR_register(LPM0_bits + GIE);             // Sleep until conversion complete
    //change to different sleep for lower power?
	//a1_val=ADC10MEM;
    val=ADC10MEM;
	ADC10CTL0&=~ENC;                     //disable adc conv
    //add turning off adc for power saving?
    
	return val;
}



//#pragma vector = ADC10_VECTOR                           // ADC conversion complete interrupt
//__interrupt void ADC10_ISR(void)                        //
__attribute__((interrupt(ADC10_VECTOR)))
void ADC10_ISR(void)
{                                                       //
    __bic_SR_register_on_exit(LPM0_bits);               // Wakeup main code
}

//timerA ccr0 rising edge interrupt
__attribute__((interrupt(TIMER0_A0_VECTOR)))
void TACCR0_ISR(void)
{
    uint16_t now_ccr;
    now_ccr = TACCR0;
    
    if((TACTL & TAIFG)){
        //if timer has overflowed
        input_period = (uint32_t)((now_ccr + 0x10000)-previous_ccr);
        TACTL &= ~(TAIFG); //reset flag
    } else {
        //if timer has not overflowed
        input_period = (uint32_t)((now_ccr - previous_ccr));
    }
    
    previous_ccr = now_ccr;
    
    __bic_SR_register_on_exit(LPM0_bits);//wake from LPM0
}

