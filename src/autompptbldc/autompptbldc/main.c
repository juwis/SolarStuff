/*
* autompptbldc.c
*
* Created: 8/3/2018 12:49:07 PM
* Author : juwi
*/

#include "includes.h"
#include "constants.h"
#include "setup.h"
#include "uart.h"

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
	
	uc_setup();
	uart_setup();
	
	
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

