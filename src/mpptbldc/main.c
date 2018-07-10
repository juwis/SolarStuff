/*
* mpptbldc.c
*
* Created: 6/24/2018 7:35:44 PM
* Author : juwi
*/
# define F_CPU 32000000UL

#include <avr/io.h>
#include "constants.h"
#include "staticfunc.h"
#include <avr/interrupt.h>
#include <util/delay.h>

#define PWM_TOP_PER		0x02FF

volatile unsigned char COMMUTATION_INDEX_SENSED, COMMUTATION_INDEX;
volatile unsigned int RT_TIME[6];
volatile unsigned int ADC;
volatile unsigned int BREAK;

unsigned int avrg(volatile unsigned int arr[], unsigned int n)
{
	
	unsigned long long int avarage = 0;
	unsigned int i;
	for (i=0; i<n; i++){
		avarage += arr[i];
	}
	
	return avarage / n;
}


volatile unsigned int COMMUTATION_DIVISOR=4; //5 -> about 12 degree

ISR(ACA_AC0_vect)
{
	//Disable Comparator Interrupt - gets re enabled in Commutation Timer
	SET_COMP_OFF();

	static unsigned int average_time = 0;
	static unsigned int commutation_time = 0;
	static unsigned int comperator_sleep_time = 0;

	average_time =  (RT_TIME[0] / 6) + (RT_TIME[1] / 6) + (RT_TIME[2] / 6) + (RT_TIME[3] / 6) + (RT_TIME[4] / 6) + (RT_TIME[5] / 6) ;
	commutation_time = average_time / COMMUTATION_DIVISOR;
	comperator_sleep_time = average_time / 2 + average_time / 3; // time ignore comparator changes

	RT_TIME[COMMUTATION_INDEX_SENSED] = TCE0.CNT;
	
	//tce0 measures the commutation time, reset to zero
	TCE0.CNT = 0;
	// tcc1 is used to delay the commutation (aka timing) and to wait for the next phase change comparator window
	TCC1.CNT = 0;
	TCC1.CCA = commutation_time;
	TCC1.CCB = comperator_sleep_time;
	TCC1.INTCTRLB=TC_CCBINTLVL_MED_gc | TC_CCAINTLVL_MED_gc;



	
	
	switch (COMMUTATION_INDEX_SENSED){
		case 0:
		ACA.AC0MUXCTRL = SENSE_1_RSG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		COMMUTATION_INDEX_SENSED = 1;
		break;
		case 1:
		ACA.AC0MUXCTRL = SENSE_2_FNG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		COMMUTATION_INDEX_SENSED = 2;
		break;
		case 2:
		ACA.AC0MUXCTRL = SENSE_3_RSG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		COMMUTATION_INDEX_SENSED = 3;
		break;
		case 3:
		ACA.AC0MUXCTRL = SENSE_4_FNG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		COMMUTATION_INDEX_SENSED = 4;
		break;
		case 4:
		ACA.AC0MUXCTRL = SENSE_5_RSG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		COMMUTATION_INDEX_SENSED = 5;
		break;
		case 5:
		ACA.AC0MUXCTRL = SENSE_0_FNG_AC_MUXPOS | AC_MUXNEG_PIN0_gc;
		COMMUTATION_INDEX_SENSED = 0;
		break;
	}

}


// pwm routine, output high
ISR(TCC0_OVF_vect)
{
	
	if(BREAK){
		PORTC.OUT = PHASES_LOW_BREAK;
		return;
	}

	switch (COMMUTATION_INDEX){
		case 0:
		PORTC.OUT = PHASE_0_ON;
		break;
		case 1:
		PORTC.OUT = PHASE_1_ON;
		break;
		case 2:
		PORTC.OUT = PHASE_2_ON;
		break;
		case 3:
		PORTC.OUT = PHASE_3_ON;
		break;
		case 4:
		PORTC.OUT = PHASE_4_ON;
		break;
		case 5:
		PORTC.OUT = PHASE_5_ON;
		break;
	}

}

// pwm routine, output off
ISR(TCC0_CCA_vect)
{
	// Ensure that the Compare/Capture A interrupt flag is cleared
	if (TCC0.INTFLAGS & TC0_CCAIF_bm) TCC0.INTFLAGS|=TC0_CCAIF_bm;

	if(BREAK){
		PORTC.OUT = PHASES_LOW_BREAK;
		return;
	}

	// if the headroom is to small to switch off the fets...
	if(TCC0_CCA > (PWM_TOP_PER-48)){
		//leave them switched on
		return;
	}


	//pwm off phase
	switch (COMMUTATION_INDEX){
		case 0:
		PORTC.OUT = PHASE_0_OFF;
		break;
		case 1:
		PORTC.OUT = PHASE_1_OFF;
		break;
		case 2:
		PORTC.OUT = PHASE_2_OFF;
		break;
		case 3:
		PORTC.OUT = PHASE_3_OFF;
		break;
		case 4:
		PORTC.OUT = PHASE_4_OFF;
		break;
		case 5:
		PORTC.OUT = PHASE_5_OFF;
		break;
	}
	
}

// Timer/Counter TCC1 Overflow/Underflow interrupt service routine
ISR(TCC1_OVF_vect)
{
	// Write your code here

}

unsigned int comm_table[] = {0,1,2,3,4,5};

// commutation delay timer
ISR(TCC1_CCA_vect)
{
	// Ensure that the Compare/Capture A interrupt flag is cleared
	if (TCC1.INTFLAGS & TC1_CCAIF_bm) TCC1.INTFLAGS|=TC1_CCAIF_bm;
	
	// this is the timing delay, used to wait an arbitrary amount and delay the commutation till then
	// in here the index is updated and the timer is set to wait for the next window in which the analog
	// comparator can wait for the phase change
	
	TCC1.INTCTRLB=TC_CCBINTLVL_MED_gc | TC_CCAINTLVL_OFF_gc;
	COMMUTATION_INDEX = comm_table[COMMUTATION_INDEX_SENSED];
}


//analog comparator window timer
ISR(TCC1_CCB_vect)
{
	// Ensure that the Compare/Capture B interrupt flag is cleared
	if (TCC1.INTFLAGS & TC1_CCBIF_bm) TCC1.INTFLAGS|=TC1_CCBIF_bm;
	// re enable the expected new analog comparator match
	switch (COMMUTATION_INDEX_SENSED){
		case 1:
		SET_COMP_RSG();
		break;
		case 2:
		SET_COMP_FNG();
		break;
		case 3:
		SET_COMP_RSG();
		break;
		case 4:
		SET_COMP_FNG();
		break;
		case 5:
		SET_COMP_RSG();
		break;
		case 0:
		SET_COMP_FNG();
		break;
	}

}



unsigned int adca_read(unsigned char channel)
{
	ADC_CH_t *pch=&ADCA.CH0+channel;
	unsigned int data;

	// Wait for the AD conversion to complete
	while ((pch->INTFLAGS & ADC_CH_CHIF_bm)==0);
	// Clear the interrupt flag
	pch->INTFLAGS=ADC_CH_CHIF_bm;
	// Read the AD conversion result
	((unsigned char *) &data)[0]=pch->RESL;
	((unsigned char *) &data)[1]=pch->RESH;
	return data;
}


void startup(){
	asm("cli");
	// disable timing/comparator timer
	TCC1.INTCTRLB=TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;
	_delay_ms(250);

	//Disable Comparator Interrupt - re enabled in Commutation Timer
	SET_COMP_OFF();

	asm("sei");

	TCC0_CCA = 0x009;
	if(COMMUTATION_INDEX < 5){
		COMMUTATION_INDEX ++;
		}else{
		COMMUTATION_INDEX = 0;
	}
	COMMUTATION_INDEX_SENSED = COMMUTATION_INDEX;

	TCC1.INTCTRLB=TC_CCBINTLVL_MED_gc | TC_CCAINTLVL_MED_gc;

}


int main(void)
{
	unsigned char n;
	unsigned int PWM = 0, TARGET_PWM = 0, INPUT_PWM = 0;

	int updown = -10;
	uint16_t servopuls = 55000;
	BREAK = 1;
	
	setup_system_clock();
	setup_io();
	setup_adc_a();
	setup_analog_comp_a();
	setup_event_system();
	setup_tcc0();
	setup_tcc1();
	setup_tcd0();
	setup_tce0();


	// Make sure the interrupts are disabled
	asm("cli");
	// Low level interrupt: On
	// Round-robin scheduling for low level interrupt: Off
	// Medium level interrupt: On
	// High level interrupt: On
	// The interrupt vectors will be placed at the start of the Application FLASH section
	n=(PMIC.CTRL & (~(PMIC_RREN_bm | PMIC_IVSEL_bm | PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm))) |
	PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	CCP=CCP_IOREG_gc;
	PMIC.CTRL=n;
	// Set the default priority for round-robin scheduling
	PMIC.INTPRI=0x00;


	// Globally enable interrupts
	asm("sei");
	
	
	//Motor PWM to zero

	TCC1.INTCTRLB=TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;
	_delay_ms(50);

	//Disable Comparator Interrupt - re enabled in Commutation Timer
	SET_COMP_OFF();
	BREAK = 0;
	COMMUTATION_INDEX = 0;
	COMMUTATION_INDEX_SENSED = 0;

	servopuls = 57000;
	BREAK = 1;
	while (1){
		servopuls = TCD0_CCA;
		//if(servopuls > 57000){
			//updown = -1;
		//}
//
		//if(servopuls < 30000){
			//updown = 1;
		//}
//
		//servopuls += updown;
		
		if(servopuls > 55000){
			PWM = 0;
			BREAK = 1;
			_delay_ms(1);
			continue;
	    }else{
			if(BREAK == 1){
				BREAK = 0;
				startup();
			}
			
			INPUT_PWM = (65535 - servopuls) / 48;
			if (INPUT_PWM > TARGET_PWM){
				TARGET_PWM ++;
				}else{
				TARGET_PWM --;
			}
			
		}
		ADC = adca_read(0);
		
		if(TARGET_PWM )
		
		if (PWM > TARGET_PWM){
			PWM = TARGET_PWM;
		}
		if(ADC > 2460){
			
			if(PWM < PWM_TOP_PER - 0x40 && PWM < TARGET_PWM){
				PWM ++;
				asm("cli");
				TCC0_CCA = PWM;
				asm("sei");
			}
			
			}else{
			
			if(PWM > 0x01){
				PWM --;
				PWM --;
				asm("cli");
				TCC0_CCA = PWM;
				asm("sei");
			}
			
		}
		
		_delay_us(100);
	}
}

//for(uint16_t i=TCC0_CCA; i > 0; i--){
//asm("cli");
//TCC0_CCA = i;
//asm("sei");
//_delay_us(100);
//}
//BREAK = 1;
//asm("cli");
//TCC0_CCA = 0x000;
//asm("sei");
//_delay_ms(4000);
//
//BREAK = 0;
//_delay_ms(50);
//
//startup();
//
//TCC0_CCA = 0x02F;
//for(uint16_t i=0x02F; i<(TCC0.PER - 48); i++){
//asm("cli");
//TCC0_CCA = i;
//asm("sei");
//_delay_ms(1);
//}
//
//_delay_ms(5000);
//}
//
//
//_delay_ms(100);
