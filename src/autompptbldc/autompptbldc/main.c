/*
* autompptbldc.c
*
* Created: 8/3/2018 12:49:07 PM
* Author : juwi
*/
#define F_CPU 32000000UL
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/io.h>
#include "constants.h"

//#define DEBUG_SERIAL


volatile unsigned char COMMUTATION_INDEX_SENSED, COMMUTATION_INDEX;
volatile uint32_t RT_TIME[6];
volatile uint16_t ADC;
volatile unsigned int BREAK;
volatile uint16_t average_time;

volatile float COMMUTATION_TIMING_FACTOR;

volatile uint8_t MOTOR_TO_SLOW;

#define HYSMODE	AC_HYSMODE_SMALL_gc

// comparator functions
static inline void SET_COMP_RSG(){
	ACA.AC0CTRL=AC_INTMODE_RISING_gc | AC_INTLVL_HI_gc | (1<<AC_HSMODE_bp) | HYSMODE | (1<<AC_ENABLE_bp);
}

static inline void  SET_COMP_FNG(){
	ACA.AC0CTRL=AC_INTMODE_FALLING_gc | AC_INTLVL_HI_gc | (1<<AC_HSMODE_bp) | HYSMODE | (1<<AC_ENABLE_bp);
}

static inline void  SET_COMP_BTH(){
	ACA.AC0CTRL=AC_INTMODE_BOTHEDGES_gc | AC_INTLVL_HI_gc | (1<<AC_HSMODE_bp) | HYSMODE | (1<<AC_ENABLE_bp);
}

static inline void  SET_COMP_OFF(){
	ACA.AC0CTRL=AC_INTMODE_RISING_gc | AC_INTLVL_OFF_gc | (1<<AC_HSMODE_bp) | HYSMODE | (1<<AC_ENABLE_bp);
}



void set_ac_interrupt(){
	switch (COMMUTATION_INDEX_SENSED){
		case 0:
		ACA.AC0MUXCTRL = SENSE_0_FNG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		SET_COMP_FNG();
		break;
		case 1:
		ACA.AC0MUXCTRL = SENSE_1_RSG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		SET_COMP_RSG();
		break;
		case 2:
		ACA.AC0MUXCTRL = SENSE_2_FNG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		SET_COMP_FNG();
		break;
		case 3:
		ACA.AC0MUXCTRL = SENSE_3_RSG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		SET_COMP_RSG();
		break;
		case 4:
		ACA.AC0MUXCTRL = SENSE_4_FNG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		SET_COMP_FNG();
		break;
		case 5:
		ACA.AC0MUXCTRL = SENSE_5_RSG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		SET_COMP_RSG();
		break;
	}
}


uint16_t adc_read(void)
{
	ADC_CH_t *adc_channel=&ADCA.CH0;
	unsigned int value;

	// Wait for the AD conversion to complete
	while ((adc_channel->INTFLAGS & ADC_CH_CHIF_bm)==0);

	adc_channel->INTFLAGS=ADC_CH_CHIF_bm;
	
	((unsigned char *) &value)[0]=adc_channel->RESL;
	((unsigned char *) &value)[1]=adc_channel->RESH;
	uint16_t res = (((((float)value * 21.0) / 4096.0)) * 1000) - 900;
	return res;
}


int uart_getchar(FILE *stream)
{
	char data;
	unsigned char status;

	while (1)
	{
		while (((status=USARTD0.STATUS) & USART_RXCIF_bm) == 0);
		data=USARTD0.DATA;
		if ((status & (USART_FERR_bm | USART_PERR_bm | USART_BUFOVF_bm)) == 0) return data;
	}
}


int uart_putchar(char c, FILE *stream)
{
	while ((USARTD0.STATUS & USART_DREIF_bm) == 0);
	USARTD0.DATA=c;
	return 1;
}


//uart ->printf binding
FILE uart_output = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
FILE uart_input = FDEV_SETUP_STREAM(NULL, uart_getchar, _FDEV_SETUP_READ);


#define ELEM_SWAP(a,b) { register uint16_t t=(a);(a)=(b);(b)=t; }

uint16_t quick_select(volatile uint16_t arr[], uint8_t n)
{
	uint16_t low, high ;
	uint16_t median;
	uint16_t middle, ll, hh;

	low = 0 ; high = n-1 ; median = (low + high) / 2;
	for (;;) {
		if (high <= low) /* One element only */
		return arr[median] ;

		if (high == low + 1) {  /* Two elements only */
			if (arr[low] > arr[high])
			ELEM_SWAP(arr[low], arr[high]) ;
			return arr[median] ;
		}

		/* Find median of low, middle and high items; swap into position low */
		middle = (low + high) / 2;
		if (arr[middle] > arr[high])    ELEM_SWAP(arr[middle], arr[high]) ;
		if (arr[low] > arr[high])       ELEM_SWAP(arr[low], arr[high]) ;
		if (arr[middle] > arr[low])     ELEM_SWAP(arr[middle], arr[low]) ;

		/* Swap low item (now in position middle) into position (low+1) */
		ELEM_SWAP(arr[middle], arr[low+1]) ;

		/* Nibble from each end towards middle, swapping items when stuck */
		ll = low + 1;
		hh = high;
		for (;;) {
			do ll++; while (arr[low] > arr[ll]) ;
			do hh--; while (arr[hh]  > arr[low]) ;

			if (hh < ll)
			break;

			ELEM_SWAP(arr[ll], arr[hh]) ;
		}

		/* Swap middle item (in position low) back into correct position */
		ELEM_SWAP(arr[low], arr[hh]) ;

		/* Re-set active partition */
		if (hh <= median)
		low = ll;
		if (hh >= median)
		high = hh - 1;
	}
}

#undef ELEM_SWAP

// Analog Comparator 0 for PORTA interrupt service routine
ISR(ACA_AC0_vect)
{
	static uint16_t commutation_time = 0;
	static uint16_t comperator_sleep_time = 0;

	uint16_t last_rt_time = (RT_TIME[0] + RT_TIME[1] + RT_TIME[2] + RT_TIME[3] + RT_TIME[4] + RT_TIME[5]) / 6;
		
	RT_TIME[COMMUTATION_INDEX_SENSED] = TCD0.CNT;

	// tcd0 measures the commutation time, reset to zero
	// tcd0 is also used to delay the commutation (aka timing) and to wait for the next phase change comparator window
	TCD0.CNT = 0;
	TCD0.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_MED_gc;


	#ifdef DEBUG_SERIAL
	uart_putchar('0', NULL);
	uart_putchar('\r', NULL);
	uart_putchar('\n', NULL);
	#endif

	//Disable Comparator Interrupt - gets re enabled in Commutation Timer
	SET_COMP_OFF();

	//now we look, in startup we don't do any timing or waiting, all is fired up immediately
	if(MOTOR_TO_SLOW == 1){
		//both are not usable, so immediate fire by setting to 1 and resetting the timer
		commutation_time = 1;
		comperator_sleep_time = 1;

		//RT_TIME[COMMUTATION_INDEX_SENSED] = 65535; //max value
		MOTOR_TO_SLOW = 0; //reset here, perhaps we are now fast enough...
		
	}else{
		
		average_time = last_rt_time;
		
		commutation_time = average_time / 24;
		comperator_sleep_time = average_time / 32; // time ignore comparator changes


	}

	//printf("CT: %u | CST: %u\r\n", commutation_time, comperator_sleep_time);

	TCD0.CCA = comperator_sleep_time;
	TCD0.CCB = commutation_time;
	TCD0.INTCTRLB=TC_CCBINTLVL_HI_gc| TC_CCAINTLVL_HI_gc; // waiting timer is urgent...

	//increase COMMUTATION_INDEX_SENSED by one looping in the range 0-5 via the fabulous modulo operator %
	COMMUTATION_INDEX_SENSED = (COMMUTATION_INDEX_SENSED + 1) % 6;
}

// Timer/Counter TCD1 Overflow/Underflow interrupt service routine
//RPM Speed Counter, if it overflows the Motor is Spinning slower than 128ms per phase change
ISR(TCD0_OVF_vect)
{
	#ifdef DEBUG_SERIAL
	uart_putchar('s', NULL);
	uart_putchar('\r', NULL);
	uart_putchar('\n', NULL);
	#endif

	TCD0.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;
	MOTOR_TO_SLOW=1;
}


// Timer/Counter TCD0 Compare/Capture A interrupt service routine
// re enable the analog comparator for the next phase
ISR(TCD0_CCA_vect)
{
	#ifdef DEBUG_SERIAL
	uart_putchar('1', NULL);
	uart_putchar('\r', NULL);
	uart_putchar('\n', NULL);
	#endif

	if (TCD0.INTFLAGS & TC0_CCAIF_bm){
		TCD0.INTFLAGS|=TC0_CCAIF_bm;
	}
	// re enable the expected new analog comparator match
	set_ac_interrupt();
	//TCD0.INTCTRLB=TC_CCBINTLVL_HI_gc | TC_CCAINTLVL_HI_gc;
}

// Timer/Counter TCD0 Compare/Capture B interrupt service routine
// Timing delay routine
ISR(TCD0_CCB_vect)
{
	#ifdef DEBUG_SERIAL
	uart_putchar('2', NULL);
	uart_putchar('\r', NULL);
	uart_putchar('\n', NULL);
	#endif

	if (TCD0.INTFLAGS & TC0_CCBIF_bm){ TCD0.INTFLAGS|=TC0_CCBIF_bm;
	}
	// this is the timing delay timer, used to wait an arbitrary amount and delay the commutation till then
	// in here the index is updated and the timer is set to wait for the next window in which the analog
	// comparator can wait for the phase change
	
	//TCD0.INTCTRLB = TC_CCBINTLVL_HI_gc | TC_CCAINTLVL_HI_gc;
	
	COMMUTATION_INDEX = COMMUTATION_INDEX_SENSED;
	
	SET_PHASE_PWM(COMMUTATION_INDEX, BREAK);
}

static inline void my_delay_ms(int ms)
{
	while (0 < ms)
	{
		_delay_ms(1);
		--ms;
	}
}

void startup(uint16_t startup_pwm, uint8_t hard_commutations,uint8_t wait_first, int16_t phase_delay_start, uint8_t phase_delay_decrement){

	asm("cli");
	//Disable Comparator Interrupt - re enabled in Commutation Timer
	SET_COMP_OFF();

	BREAK = 0;
	
	SET_PHASE_PWM(COMMUTATION_INDEX_SENSED, BREAK);
	COMMUTATION_INDEX = COMMUTATION_INDEX_SENSED;
	
	TCC0.CCA = startup_pwm;
	my_delay_ms(wait_first);
	
	for(uint8_t i=0; i<hard_commutations; i++){
		COMMUTATION_INDEX_SENSED = (COMMUTATION_INDEX_SENSED + 1) % 6;
		COMMUTATION_INDEX = COMMUTATION_INDEX_SENSED;
		SET_PHASE_PWM(COMMUTATION_INDEX_SENSED, BREAK);
		my_delay_ms(phase_delay_start);

		//prevent phase_delay to get below zero (wrap around)

		phase_delay_start -= phase_delay_decrement;
		phase_delay_start = phase_delay_start  > 3 ? phase_delay_start : 3;
	}
	COMMUTATION_INDEX = COMMUTATION_INDEX_SENSED;
	set_ac_interrupt();
	asm("sei");
}

void beep(uint16_t beep_pwm, uint16_t length, uint8_t tone){
	asm("cli");
	TCC0.CCA = beep_pwm;

	for(uint16_t i=0; i<length; i++){
		SET_PHASE_PWM(0, 0);
		for(uint8_t delay=0; delay < tone; delay++){
			_delay_us(100);
		}
		SET_PHASE_PWM(1, 0);
		for(uint8_t delay=0; delay < tone; delay++){
			_delay_us(100);
		}
	}

	SET_PHASE_PWM(COMMUTATION_INDEX, BREAK);
	asm("sei");

}

uint16_t get_transmitter_power(){
	static uint16_t power = 1000;
	volatile uint16_t servopulse = 0;
	uint16_t normalized_power = 0;
	servopulse = TCE0_CCABUF;


	if(servopulse > TRANSMITTER_MIN_PULSE && servopulse < TRANSMITTER_MAX_PULSE){
		//seems to be in a reasonable window
		//infinite response filtering + normalizing (0-1000) applied in the next line
		float norm_divisor = ((float)((float)TRANSMITTER_MAX_PULSE - (float)TRANSMITTER_MIN_PULSE)/1000.0);
		
		power += (((servopulse - TRANSMITTER_MIN_PULSE) / norm_divisor) - power) / TRANSMITTER_FILTER_STRENGTH; //the division normalizes the value to 0-1000, the 
	}else{
		//wrong signal received, reset to zero (which is 1000 here)
		power = 1000;
	}
	
	normalized_power = 1000 - power; //invert because it is getting lower the more the trigger is pushed...
	
	//now add full and no throttle positions, mix them in
	if (normalized_power < NORMALIZED_MIN_THROTTLE){
		return 0;
	}

	if (normalized_power > NORMALIZED_MIN_THROTTLE && normalized_power < NORMALIZED_MAX_THROTTLE){
		return (normalized_power - NORMALIZED_MIN_THROTTLE) + (((NORMALIZED_MIN_THROTTLE * normalized_power) / NORMALIZED_MAX_THROTTLE) * 2);
	}

	if (normalized_power > NORMALIZED_MAX_THROTTLE){
		return 1000;
	}

	return normalized_power;
}


int main(void)
{
	
	//****************************************************************************************************
	// local variables
	unsigned char tmp;
	uint32_t transmitter_power = 0;
	uint16_t out_power = 0;
	//****************************************************************************************************
	// globals init
	BREAK = 1;
	average_time = 0;
	
	// from the Analog comparator interrupt this is set:
	COMMUTATION_INDEX_SENSED = 0;
	// from the timing interrupt this is set(and then immediately used by pwm):
	COMMUTATION_INDEX = 0;
	
	// Roundtript time counter, overflow (65535) @ ca 128ms per Phase(60deg)
	RT_TIME[0] = 0;
	RT_TIME[1] = 0;
	RT_TIME[2] = 0;
	RT_TIME[3] = 0;
	RT_TIME[4] = 0;
	RT_TIME[5] = 0;
	
	//ADC Value
	ADC = 0;
	
	// set if rpm counter overflows
	MOTOR_TO_SLOW = 0;
	
	//TIMING Setting
	COMMUTATION_TIMING_FACTOR = 0.2;
	
	//****************************************************************************************************
	//uc setup...

	//****************************************************************************************************
	asm("cli");


	//****************************************************************************************************
	//clocks
	// use 32MHZ internal as source for pll, multiply by 16 for 128MHz base clock
	// divide sysclock down to 32MHz.
	// we use hires timer with 128MHz for fast PWM

	OSC.CTRL|=OSC_RC32KEN_bm;
	// wait for  32 kHz RC oscillator to stabilize
	while ((OSC.STATUS & OSC_RC32KRDY_bm)==0)
	_delay_us(1);

	DFLLRC32M.CTRL=0<<DFLL_ENABLE_bp;

	// Enable the internal 32 MHz RC oscillator
	OSC.CTRL|=OSC_RC32MEN_bm;
	// Wait for the internal 32 MHz RC oscillator to stabilize
	while ((OSC.STATUS & OSC_RC32MRDY_bm)==0)
	_delay_us(1);

	// Use the Internal 32kHz RC oscillator as DFLL clock reference
	OSC.DFLLCTRL=(OSC.DFLLCTRL & (~OSC_RC32MCREF_gm)) | OSC_RC32MCREF_RC32K_gc;

	// Enable the auto-calibration of the internal 32 MHz RC oscillator
	DFLLRC32M.CTRL=1<<DFLL_ENABLE_bp;

	// PLL initialization
	// Ensure that the PLL is disabled before configuring it
	OSC.CTRL&= ~OSC_PLLEN_bm;

	tmp=OSC_PLLSRC_RC32M_gc | (0<<OSC_PLLDIV_bp) | (16<<OSC_PLLFAC_gp);
	// Enable the PLL
	CCP=CCP_IOREG_gc;
	OSC.PLLCTRL=tmp;
	OSC.CTRL|=OSC_PLLEN_bm;

	// Wait for the PLL to stabilize
	while ((OSC.STATUS & OSC_PLLRDY_bm)==0)
	_delay_us(1);

	// System Clock prescaler A division factor: 1
	// System Clock prescalers B & C division factors: B:2, C:2
	// ClkPer4: 128000.000 kHz
	// ClkPer2: 64000.000 kHz
	// ClkPer:  32000.000 kHz
	// ClkCPU:  32000.000 kHz
	tmp=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
	CLK_PSADIV_1_gc | CLK_PSBCDIV_2_2_gc;
	CCP=CCP_IOREG_gc;
	CLK.PSCTRL=tmp;

	// Select the system clock source: Phase Locked Loop
	tmp=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_PLL_gc;
	CCP=CCP_IOREG_gc;
	CLK.CTRL=tmp;

	// Disable the unused oscillators: 2 MHz, external clock/crystal oscillator
	OSC.CTRL&= ~(OSC_RC2MEN_bm | OSC_XOSCEN_bm);

	// Lock the CLK.CTRL and CLK.PSCTRL registers
	tmp=CLK.LOCK | CLK_LOCK_bm;
	CCP=CCP_IOREG_gc;
	CLK.LOCK=tmp;

	// ClkPer output disabled
	PORTCFG.CLKEVOUT&= ~(PORTCFG_CLKOUTSEL_gm | PORTCFG_CLKOUT_gm);
	// PLL fault detection: On
	// External clock source failure detection: Off
	tmp=(OSC.XOSCFAIL & (~(OSC_PLLFDEN_bm | OSC_XOSCFDEN_bm))) | OSC_PLLFDEN_bm;
	CCP=CCP_IOREG_gc;
	OSC.XOSCFAIL=tmp;


	//****************************************************************************************************
	//****************************************************************************************************
	//event system init
	//used for servo pulse measuring
	//****************************************************************************************************
	
	// Event System Channel 0 source: Port B, Pin0
	EVSYS.CH0MUX=EVSYS_CHMUX_PORTB_PIN0_gc;

	// Event System Channel 0 Digital Filter Coefficient: 1 Sample
	// Quadrature Decoder: Off
	EVSYS.CH0CTRL=(EVSYS.CH0CTRL & (~(EVSYS_QDIRM_gm | EVSYS_QDIEN_bm | EVSYS_QDEN_bm | EVSYS_DIGFILT_gm))) |
	EVSYS_DIGFILT_1SAMPLE_gc;
	// Event System Channel output: Disabled
	PORTCFG.CLKEVOUT&= ~PORTCFG_EVOUT_gm;
	PORTCFG.EVOUTSEL&= ~PORTCFG_EVOUTSEL_gm;


	//****************************************************************************************************
	//****************************************************************************************************
	//now the ports an pins to the world
	
	//****************************************************************************************************
	// PORTA holds analog stuff, so input and no nothing pulled up
	PORTA.OUT=0x00;
	PORTA.DIR=0x00;
	PORTA.PIN0CTRL=PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN1CTRL=PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN2CTRL=PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN3CTRL=PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN4CTRL=PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN5CTRL=PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN6CTRL=PORT_ISC_INPUT_DISABLE_gc;
	PORTA.PIN7CTRL=PORT_ISC_INPUT_DISABLE_gc;

	
	PORTA.INTCTRL=PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTA.INT0MASK=0x00;
	PORTA.INT1MASK=0x00;

	//****************************************************************************************************
	// PORTB has the rx pin on pinb0, so here a pullup is required.
	PORTB.OUT=0x00;
	PORTB.DIR=0x00;

	PORTB.PIN0CTRL=PORT_OPC_PULLUP_gc;

	PORTB.INTCTRL=PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTB.INT0MASK=0x00;
	PORTB.INT1MASK=0x00;


	//****************************************************************************************************
	//Portc has the mosfet drivers attached to it. lower 6 have to be low and output

	PORTC.OUT=0x00;
	PORTC.DIR=0x3F;
	PORTC.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;
	PORTC.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;
	PORTC.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;
	PORTC.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;
	PORTC.PIN4CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;
	PORTC.PIN5CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_INPUT_DISABLE_gc;

	PORTC.REMAP=(0<<PORT_SPI_bp) | (0<<PORT_USART0_bp) | (0<<PORT_TC0D_bp) | (0<<PORT_TC0C_bp) | (0<<PORT_TC0B_bp) | (0<<PORT_TC0A_bp);
	PORTC.INTCTRL=PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTC.INT0MASK=0x00;
	PORTC.INT1MASK=0x00;
	
	//****************************************************************************************************
	// PORTD, PORTE and PORTR are not used right now.
	
	PORTD.OUT=0x08;
	PORTD.DIR=0x08;
	PORTD.INTCTRL=PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTD.INT0MASK=0x00;
	PORTD.INT1MASK=0x00;

	PORTE.OUT=0x00;
	PORTE.DIR=0x00;
	PORTE.INTCTRL=PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTE.INT0MASK=0x00;
	PORTE.INT1MASK=0x00;

	PORTR.OUT=0x00;
	PORTR.DIR=0x00;
	PORTR.INTCTRL=PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTR.INT0MASK=0x00;
	PORTR.INT1MASK=0x00;
	
	//****************************************************************************************************
	//****************************************************************************************************
	// now the timers are getting configured
	
	
	//****************************************************************************************************
	//TCC0
	// single slope pwm with 200KHz
	// Clock source: ClkPer4/1
	TCC0.CTRLA=TC_CLKSEL_DIV1_gc;
	TCC0.CTRLB=(0<<TC0_CCDEN_bp) | (0<<TC0_CCCEN_bp) | (0<<TC0_CCBEN_bp) | (1<<TC0_CCAEN_bp) | TC_WGMODE_SS_gc;

	TCC0.CTRLD=TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
	TCC0.CTRLE=TC_BYTEM_NORMAL_gc;
	TCC0.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;
	TCC0.INTCTRLB=TC_CCDINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;

	// ENABLE high resolution extension
	HIRESC.CTRLA|=HIRES_HREN0_bm;

	// Advanced Waveform Extension initialization
	tmp=MCU.AWEXLOCK & (~MCU_AWEXCLOCK_bm);
	CCP=CCP_IOREG_gc;
	MCU.AWEXLOCK=tmp;

	// setting awex in common waveform mode ( CCA defines all outputs)
	// no pattern generation, dead time is 0 (our fet driver is taking care of that)
	
	AWEXC.CTRL=(0<<AWEX_PGM_bp) | (1<<AWEX_CWCM_bp) | (0<<AWEX_DTICCDEN_bp) | (1<<AWEX_DTICCCEN_bp) |
	(1<<AWEX_DTICCBEN_bp) | (1<<AWEX_DTICCAEN_bp);
	AWEXC.DTLS=0;
	AWEXC.DTHS=0;
	AWEXC.OUTOVEN=0b00000000;

	AWEXC.FDCTRL=(AWEXC.FDCTRL & (~(AWEX_FDDBD_bm | AWEX_FDMODE_bm | AWEX_FDACT_gm))) |
	(1<<AWEX_FDDBD_bp) | (0<<AWEX_FDMODE_bp) | AWEX_FDACT_NONE_gc;
	AWEXC.FDEMASK=0b00000000;
	AWEXC.STATUS|=AWEXC.STATUS & AWEX_FDF_bm;

	// Clear the interrupt flags
	TCC0.INTFLAGS=TCC0.INTFLAGS;
	// Set Counter register
	TCC0.CNT=0x0000;
	// Period Register is the max counting of this timer before it resets.
	// by setting this we can set the PWM Frequency
	TCC0.PER=0x027C;
	// CCA holds the PWM Value for the Motor, 0x0000 - 0x027f
	TCC0.CCA=0x0000;

	TCC0.CCB=0x0000;
	TCC0.CCC=0x0000;
	TCC0.CCD=0x0000;

	
	//****************************************************************************************************
	//TCD0 is used for timing commutation and phase sensing
	// this is set so 128ms can be measured before overflowing the timer.
	// up to this rotation speed the motor is in startup
	TCD0.CTRLA=TC_CLKSEL_DIV8_gc;
	
	// TCCA and TCCB are used to time analog comparator reactivation and timing delay
	TCD0.CTRLB=(1<<TC0_CCBEN_bp) | (1<<TC0_CCAEN_bp) | TC_WGMODE_NORMAL_gc;

	TCD0.CTRLD=TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;
	TCD0.CTRLE=TC_BYTEM_NORMAL_gc;
	TCD0.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_MED_gc;

	// both interrupts are critical to smooth motor rotation, so they are high level, but disabled now
	TCD0.INTCTRLB=TC_CCDINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;

	HIRESD.CTRLA&= ~HIRES_HREN0_bm;
	TCD0.INTFLAGS=TCD0.INTFLAGS;

	TCD0.CNT=0x0000;
	TCD0.PER=0xFFFF;
	TCD0.CCA=0x0000;
	TCD0.CCB=0x0000;
	TCD0.CCC=0x0000;
	TCD0.CCD=0x0000;
	
	
	//****************************************************************************************************
	// now the pulse width capture has to be done.
	// at half clock speed (16MHz) we need 4ms to count to 65535 :)
	// that means a servo pulse will show up between 16000 and 32000 units, being 1-2ms long :)
	// almost magic
	TCE0.CTRLA=TC_CLKSEL_DIV2_gc;

	TCE0.CTRLB=(0<<TC0_CCDEN_bp) | (0<<TC0_CCCEN_bp) | (0<<TC0_CCBEN_bp) | (1<<TC0_CCAEN_bp) |
	TC_WGMODE_NORMAL_gc;
	// event channel 0
	// pulse width capture
	TCE0.CTRLD=TC_EVACT_PW_gc | TC_EVSEL_CH0_gc;

	TCE0.CTRLE=TC_BYTEM_NORMAL_gc;
	TCE0.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;
	TCE0.INTCTRLB=TC_CCDINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;
	HIRESE.CTRLA&= ~HIRES_HREN0_bm;
	TCE0.INTFLAGS=TCE0.INTFLAGS;
	TCE0.CNT=0x0000;
	TCE0.PER=0xFFFF;
	TCE0.CCA=0x0000;
	TCE0.CCB=0x0000;
	TCE0.CCC=0x0000;
	TCE0.CCD=0x0000;
	
	
	//****************************************************************************************************
	// Analog 2 digital converter is getting prepped.
	// high impedance sources
	// no current limit
	// unsigned
	ADCA.CTRLB=(0<<ADC_IMPMODE_bp) | ADC_CURRLIMIT_HIGH_gc | (0<<ADC_CONMODE_bp) | ADC_RESOLUTION_12BIT_gc;

	// 62,500 kHz
	ADCA.PRESCALER=ADC_PRESCALER_DIV512_gc;

	// Ref 1V internal
	ADCA.REFCTRL=ADC_REFSEL_INT1V_gc | (0<<ADC_TEMPREF_bp) | (1<<ADC_BANDGAP_bp);

	ADCA.CMPL=0x00;
	ADCA.CMPH=0x00;

	// channel 0 gain: 1
	// channel 0 : single ended positive input signal
	ADCA.CH0.CTRL=(0<<ADC_CH_START_bp) | ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// channel 0 p in ADC7
	// channel 0 n in GND
	ADCA.CH0.MUXCTRL=ADC_CH_MUXPOS_PIN7_gc;

	// free run mode
	ADCA.CTRLB|=ADC_FREERUN_bm;
	// enable!
	ADCA.CTRLA|=ADC_ENABLE_bm;


	//****************************************************************************************************
	// Analog Comparator. used for back-emf measurement.

	// Window Mode enabled: Off
	ACA.WINCTRL=(0<<AC_WEN_bp) | AC_WINTMODE_ABOVE_gc | AC_WINTLVL_OFF_gc;
	// AC0 Positive Input: Pin 0
	// AC0 Negative Input: Pin 1
	ACA.AC0MUXCTRL=AC_MUXPOS_PIN0_gc | AC_MUXNEG_PIN1_gc;
	// AC0 is enabled
	// AC0 Interrupt/Event Mode: Both Edges
	// AC0 Interrupt Level: High-level
	// AC0 High Speed Mode: Off
	// AC0 Hysteresis: Large
	ACA.AC0CTRL=AC_INTMODE_BOTHEDGES_gc | AC_INTLVL_HI_gc | (1<<AC_HSMODE_bp) | AC_HYSMODE_SMALL_gc | (1<<AC_ENABLE_bp);
	// AC1 Positive Input: Pin 0
	// AC1 Negative Input: Pin 1
	ACA.AC1MUXCTRL=AC_MUXPOS_DAC_gc | AC_MUXNEG_PIN7_gc;
	// AC1 is enabled
	// AC1 Interrupt/Event Mode: Both Edges
	// AC1 Interrupt Level: High-level
	// AC1 High Speed Mode: Off
	// AC1 Hysteresis: Small
	ACA.AC1CTRL=AC_INTMODE_FALLING_gc | AC_INTLVL_OFF_gc | (1<<AC_HSMODE_bp) | AC_HYSMODE_SMALL_gc | (1<<AC_ENABLE_bp);
	// AC0 Output on PORTA: No
	// AC1 Output on PORTA: No
	ACA.CTRLA=(0<<AC_AC1OUT_bp) | (0<<AC_AC0OUT_bp);


	//****************************************************************************************************
	//uart setup
	// Baudrate 115200

	// Set TxD=1
	PORTD.OUT=0x08;

	// Communication mode: Asynchronous USART
	// Data bits: 8
	// Stop bits: 1
	// Parity: Disabled
	USARTD0.CTRLC=USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_CHSIZE_8BIT_gc;
	USARTD0.CTRLA=(USARTD0.CTRLA & (~(USART_RXCINTLVL_gm | USART_TXCINTLVL_gm | USART_DREINTLVL_gm))) |
	USART_RXCINTLVL_OFF_gc | USART_TXCINTLVL_OFF_gc | USART_DREINTLVL_OFF_gc;
	// Baudrate: 500000
	USARTD0.BAUDCTRLA=0x80;
	USARTD0.BAUDCTRLB=((0x09 << USART_BSCALE_gp) & USART_BSCALE_gm) | 0x01;

	// Receiver
	USARTD0.CTRLB=(USARTD0.CTRLB & (~(USART_RXEN_bm | USART_TXEN_bm | USART_CLK2X_bm | USART_MPCM_bm | USART_TXB8_bm))) |
	USART_RXEN_bm | USART_TXEN_bm;


	// now bind it ti stdout and in
	stdout = &uart_output;
	stdin  = &uart_input;
	
	//****************************************************************************************************
	//interrupts setup
	// Ll interrupt: On
	// Rr scheduling ll interrupt: On
	// Ml interrupt: On
	// Hl interrupt: On
	tmp=(PMIC.CTRL & (~(PMIC_RREN_bm | PMIC_IVSEL_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm))) |
	PMIC_LOLVLEN_bm | PMIC_RREN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	CCP=CCP_IOREG_gc;
	PMIC.CTRL=tmp;
	// default priority for round-robin scheduling
	PMIC.INTPRI=0x00;

	// DONE SETTING UP (BEING UPSET? ;) )
	
	
	// "and that's where the fun starts" (patrick jane)

	asm("sei");

	TCC0.CCA = 0x00; //no power applied
	BREAK = 1; //break applied
	beep(0x04f, 250, 3);
	_delay_ms(100);
	beep(0x04f, 400, 2);	
	// wait for no power from transmitter, a hundred times with no exception
	uint8_t no_power_count = 100;
	while(1){


		if(get_transmitter_power() == 0){
			no_power_count--;
			_delay_ms(5);
		}else{
			no_power_count = 100;
		}
		
		//break if we got no power from tx
		if(no_power_count < 1){
			beep(0x04f, 250, 3);
			_delay_ms(100);
			beep(0x04f, 1200, 2);
			break;
		}
	}
	
	
	
	while (1)
	{
		ADC=adc_read();

		transmitter_power = get_transmitter_power();
		
		//adc_filtered += (((double)ADC - (double)adc_filtered) / 32.0);
		
		//printf("%u mV\r\n", adc_filtered);	
		
		_delay_us(100);
	
		if(BREAK == 1){
			if(transmitter_power > 0){
				startup(0x01f, 3, 150, 50, 5);
			}
		}
		
		if(transmitter_power == 0){
			TCC0.CCA = 0x00; //shut down power
			BREAK = 1;
			continue;
		}
		
		//this makes sense, because we are getting here after startup from the previous if...
		//an else clause would mean that the power setting would be delayed for cycle time + a bit...
		if(BREAK == 0){
			//we are probably running
			uint16_t out_power_transmitter = (transmitter_power * PWM_TOP_PER) / 1000;
			
			out_power_transmitter = out_power_transmitter > PWM_TOP_PER ? PWM_TOP_PER : out_power_transmitter; //limit to PWM_TOP Value
			
			if(out_power < out_power_transmitter && ADC > MPPT_VOLTAGE){
				out_power++;
			}
			
			if(out_power > out_power_transmitter){
				out_power = out_power_transmitter;
			}
			
			if(out_power > 0 && ADC < MPPT_VOLTAGE){
				out_power--;
			}
			
			TCC0.CCA = out_power;
		}
		
		
	}
}

