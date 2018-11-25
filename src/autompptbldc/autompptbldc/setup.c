/*
* uc_config.c
*
* Created: 14.11.2018 16:41:26
*  Author: JWingert
*/
#include "includes.h"

void clock_setup(){
	//****************************************************************************************************
	//clocks
	// use 32MHZ internal as source for pll, multiply by 16 for 128MHz base clock
	// divide sysclock down to 32MHz.
	// we use hires timer with 128MHz for fast PWM

	unsigned char tmp;


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

}

void event_setup(){
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

}

void io_setup(){
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
}

void timer_setup(){
	
	//****************************************************************************************************
	//****************************************************************************************************
	// now the timers are getting configured
	
	
	//****************************************************************************************************
	
	unsigned char tmp;

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
}

void adc_setup(){
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
}

void ac_setup(){
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

}


void uc_setup(void){

	unsigned char tmp;
	asm("cli");

	clock_setup();
	event_setup();
	io_setup();
	timer_setup();
	adc_setup();

	
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

}