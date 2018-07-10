#ifndef STATICFUNC_H
#define STATICFUNC_H

static inline void SET_COMP_RSG(){
	ACA.AC0CTRL=AC_INTMODE_RISING_gc | AC_INTLVL_HI_gc | (1<<AC_HSMODE_bp) | AC_HYSMODE_LARGE_gc | (1<<AC_ENABLE_bp);
}

static inline void  SET_COMP_FNG(){
	ACA.AC0CTRL=AC_INTMODE_FALLING_gc | AC_INTLVL_HI_gc | (1<<AC_HSMODE_bp) | AC_HYSMODE_LARGE_gc | (1<<AC_ENABLE_bp);
}

static inline void  SET_COMP_BTH(){
	ACA.AC0CTRL=AC_INTMODE_BOTHEDGES_gc | AC_INTLVL_HI_gc | (1<<AC_HSMODE_bp) | AC_HYSMODE_LARGE_gc | (1<<AC_ENABLE_bp);
}

static inline void  SET_COMP_OFF(){
	ACA.AC0CTRL=AC_INTMODE_RISING_gc | AC_INTLVL_OFF_gc | (1<<AC_HSMODE_bp) | AC_HYSMODE_LARGE_gc | (1<<AC_ENABLE_bp);
}

static inline void setup_adc_a(void)
{
	// Gain stage impedance mode: Low-impedance sources
	// Current consumption: No limit
	// Conversion mode: Unsigned
	ADCA.CTRLB=(1<<ADC_IMPMODE_bp) | ADC_CURRLIMIT_NO_gc | (0<<ADC_CONMODE_bp) | ADC_RESOLUTION_12BIT_gc;

	// Clock frequency: 500,000 kHz
	ADCA.PRESCALER=ADC_PRESCALER_DIV64_gc;

	// Reference: Internal 1.00 V
	// Temperature reference: Off
	ADCA.REFCTRL=ADC_REFSEL_INT1V_gc | (0<<ADC_TEMPREF_bp) | (1<<ADC_BANDGAP_bp);

	// Initialize the ADC Compare register
	ADCA.CMPL=0x00;
	ADCA.CMPH=0x00;

	// ADC channel 0 gain: 1
	// ADC channel 0 input mode: Single-ended positive input signal
	ADCA.CH0.CTRL=(0<<ADC_CH_START_bp) | ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;

	// ADC channel 0 positive input: ADC7 pin
	// ADC channel 0 negative input: GND
	ADCA.CH0.MUXCTRL=ADC_CH_MUXPOS_PIN7_gc;

	// ADC is free-running, swiped channel(s): 0
	ADCA.EVCTRL=ADC_SWEEP_0_gc | ADC_EVACT_NONE_gc;

	// Channel 0 interrupt: Disabled
	ADCA.CH0.INTCTRL=ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
	// Channel 1 interrupt: Disabled
	ADCA.CH1.INTCTRL=ADC_CH_INTLVL_OFF_gc;
	// Channel 2 interrupt: Disabled
	ADCA.CH2.INTCTRL=ADC_CH_INTLVL_OFF_gc;
	// Channel 3 interrupt: Disabled
	ADCA.CH3.INTCTRL=ADC_CH_INTLVL_OFF_gc;

	// Free Running mode: On
	ADCA.CTRLB|=ADC_FREERUN_bm;

	// Enable the ADC
	ADCA.CTRLA|=ADC_ENABLE_bm;
}


static inline void setup_analog_comp_a(void)
{
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
	ACA.AC0CTRL=AC_INTMODE_BOTHEDGES_gc | AC_INTLVL_HI_gc | (1<<AC_HSMODE_bp) | AC_HYSMODE_LARGE_gc | (1<<AC_ENABLE_bp);
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


static inline void setup_io(){
	PORTA.OUT=0x00;
	PORTA.DIR=0x00;
	PORTA.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN4CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN5CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN6CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTA.PIN7CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTA.INTCTRL=(PORTA.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTA.INT0MASK=0x00;
	PORTA.INT1MASK=0x00;

	PORTB.OUT=0x00;
	PORTB.DIR=0x00;
	PORTB.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTB.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTB.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTB.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTB.INTCTRL=(PORTB.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTB.INT0MASK=0x00;
	PORTB.INT1MASK=0x00;

	PORTC.OUT=0x00;
	PORTC.DIR=0xFF;
	PORTC.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTC.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTC.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTC.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTC.PIN4CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTC.PIN5CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTC.PIN6CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTC.PIN7CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTC.REMAP=(0<<PORT_SPI_bp) | (0<<PORT_USART0_bp) | (0<<PORT_TC0D_bp) | (0<<PORT_TC0C_bp) | (0<<PORT_TC0B_bp) | (0<<PORT_TC0A_bp);
	PORTC.INTCTRL=(PORTC.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTC.INT0MASK=0x00;
	PORTC.INT1MASK=0x00;

	PORTD.OUT=0x00;
	PORTD.DIR=0x00;
	PORTD.INTCTRL=(PORTD.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTD.INT0MASK=0x00;
	PORTD.INT1MASK=0x00;

	PORTE.OUT=0x00;
	PORTE.DIR=0x00;
	PORTE.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTE.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTE.PIN2CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTE.PIN3CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTE.INTCTRL=(PORTE.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTE.INT0MASK=0x00;
	PORTE.INT1MASK=0x00;

	PORTR.OUT=0x00;
	PORTR.DIR=0x00;
	PORTR.PIN0CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTR.PIN1CTRL=PORT_OPC_TOTEM_gc | PORT_ISC_BOTHEDGES_gc;
	PORTR.INTCTRL=(PORTR.INTCTRL & (~(PORT_INT1LVL_gm | PORT_INT0LVL_gm))) |
	PORT_INT1LVL_OFF_gc | PORT_INT0LVL_OFF_gc;
	PORTR.INT0MASK=0x00;
	PORTR.INT1MASK=0x00;
}

// System Clocks initialization
static inline void setup_system_clock(void)
{
	unsigned char TMP,SREG_CACHE;

	// Save interrupts enabled/disabled state
	SREG_CACHE=SREG;
	// Disable interrupts
	asm("cli");

	// Internal 32 kHz RC oscillator initialization
	// Enable the internal 32 kHz RC oscillator
	OSC.CTRL|=OSC_RC32KEN_bm;
	// Wait for the internal 32 kHz RC oscillator to stabilize
	while ((OSC.STATUS & OSC_RC32KRDY_bm)==0);

	// Internal 32 MHz RC oscillator initialization
	// Internal 32 MHz RC osc. calibration reference clock source: Internal 32kHz RC oscillator

	// Oscillator calibration not enabled yet
	DFLLRC32M.CTRL=0<<DFLL_ENABLE_bp;

	// Enable the internal 32 MHz RC oscillator
	OSC.CTRL|=OSC_RC32MEN_bm;
	// Wait for the internal 32 MHz RC oscillator to stabilize
	while ((OSC.STATUS & OSC_RC32MRDY_bm)==0);

	// Use the Internal 32kHz RC oscillator as DFLL clock reference
	OSC.DFLLCTRL=(OSC.DFLLCTRL & (~OSC_RC32MCREF_gm)) | OSC_RC32MCREF_RC32K_gc;

	// Enable the auto-calibration of the internal 32 MHz RC oscillator
	DFLLRC32M.CTRL=1<<DFLL_ENABLE_bp;

	// System Clock prescaler A division factor: 1
	// System Clock prescalers B & C division factors: B:1, C:1
	// ClkPer4: 32000,000 kHz
	// ClkPer2: 32000,000 kHz
	// ClkPer:  32000,000 kHz
	// ClkCPU:  32000,000 kHz
	TMP=(CLK.PSCTRL & (~(CLK_PSADIV_gm | CLK_PSBCDIV1_bm | CLK_PSBCDIV0_bm))) |
	CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
	CCP=CCP_IOREG_gc;
	CLK.PSCTRL=TMP;

	// Select the system clock source: 32 MHz Internal RC Osc.
	TMP=(CLK.CTRL & (~CLK_SCLKSEL_gm)) | CLK_SCLKSEL_RC32M_gc;
	CCP=CCP_IOREG_gc;
	CLK.CTRL=TMP;

	// Internal 2 MHz RC oscillator initialization
	// Enable the internal 2 MHz RC oscillator
	OSC.CTRL|=OSC_RC2MEN_bm;
	// Wait for the internal 2 MHz RC oscillator to stabilize
	while ((OSC.STATUS & OSC_RC2MRDY_bm)==0);

	// Internal 2 MHz RC osc. calibration reference clock source: 32.768 kHz Internal Osc.
	OSC.DFLLCTRL&= ~OSC_RC2MCREF_bm;
	// Enable the auto-calibration of the internal 2 MHz RC oscillator
	DFLLRC2M.CTRL=1<<DFLL_ENABLE_bp;

	// PLL initialization
	// Ensure that the PLL is disabled before configuring it
	OSC.CTRL&= ~OSC_PLLEN_bm;
	// PLL clock source: 2 MHz RC Oscillator
	// PLL multiplication factor: 24
	// PLL output/2: Off
	// PLL frequency: 48 MHz
	// Set the PLL clock source and multiplication factor
	TMP=OSC_PLLSRC_RC2M_gc | (0<<OSC_PLLDIV_bp) | (24<<OSC_PLLFAC_gp);
	// Enable the PLL
	CCP=CCP_IOREG_gc;
	OSC.PLLCTRL=TMP;
	OSC.CTRL|=OSC_PLLEN_bm;

	// Wait for the PLL to stabilize
	while ((OSC.STATUS & OSC_PLLRDY_bm)==0);

	// Disable the unused oscillators: external clock/crystal oscillator
	OSC.CTRL&= ~(OSC_XOSCEN_bm);

	// ClkPer output disabled
	PORTCFG.CLKEVOUT&= ~(PORTCFG_CLKOUTSEL_gm | PORTCFG_CLKOUT_gm);
	// Restore interrupts enabled/disabled state
	SREG=SREG_CACHE;

}

static inline void setup_event_system(void)
{
	// Event System Channel 0 source: Port B, Pin0
	EVSYS.CH0MUX=EVSYS_CHMUX_PORTB_PIN0_gc;

	// Event System Channel 0 Digital Filter Coefficient: 4 Samples
	// Quadrature Decoder: Off
	EVSYS.CH0CTRL=(EVSYS.CH0CTRL & (~(EVSYS_QDIRM_gm | EVSYS_QDIEN_bm | EVSYS_QDEN_bm | EVSYS_DIGFILT_gm))) |
	EVSYS_DIGFILT_4SAMPLES_gc;

	// Event System Channel output: Disabled
	PORTCFG.CLKEVOUT&= ~PORTCFG_EVOUT_gm;
	PORTCFG.EVOUTSEL&= ~PORTCFG_EVOUTSEL_gm;
}

// Disable a Timer/Counter type TC0
static inline void tc0_disable(TC0_t *ptc)
{
	// Timer/Counter off
	ptc->CTRLA=TC_CLKSEL_OFF_gc;
	// Issue a reset command
	ptc->CTRLFSET=TC_CMD_RESET_gc;
}

// Disable a Timer/Counter type TC1
static inline void tc1_disable(TC1_t *ptc)
{
	// Timer/Counter off
	ptc->CTRLA=TC_CLKSEL_OFF_gc;
	// Issue a reset command
	ptc->CTRLFSET=TC_CMD_RESET_gc;
}

// Timer/Counter TCC0 initialization
static inline void setup_tcc0(void)
{
	unsigned char SREG_CACHE;
	unsigned char TMP;

	// Note: The correct PORTC direction for the Compare Channels
	// outputs is configured in the ports_init function.

	// Save interrupts enabled/disabled state
	SREG_CACHE=SREG;
	// Disable interrupts
	asm("cli");

	// Disable and reset the timer/counter just to be sure
	tc0_disable(&TCC0);
	// Clock source: ClkPer/1
	TCC0.CTRLA=TC_CLKSEL_DIV1_gc;
	// Mode: Normal Operation, Overflow Int./Event on TOP
	// Compare/Capture on channel A: Off
	// Compare/Capture on channel B: Off
	// Compare/Capture on channel C: Off
	// Compare/Capture on channel D: Off
	TCC0.CTRLB=(0<<TC0_CCDEN_bp) | (0<<TC0_CCCEN_bp) | (0<<TC0_CCBEN_bp) | (0<<TC0_CCAEN_bp) |
	TC_WGMODE_NORMAL_gc;
	// Capture event source: None
	// Capture event action: None
	TCC0.CTRLD=TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;

	// Set Timer/Counter in Normal mode
	TCC0.CTRLE=TC_BYTEM_NORMAL_gc;

	// Overflow interrupt: High Level
	// Error interrupt: Disabled
	TCC0.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_HI_gc;

	// Compare/Capture channel A interrupt: High Level
	// Compare/Capture channel B interrupt: Disabled
	// Compare/Capture channel C interrupt: Disabled
	// Compare/Capture channel D interrupt: Disabled
	TCC0.INTCTRLB=TC_CCDINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_MED_gc;

	// High resolution extension: Off
	HIRESC.CTRLA&= ~HIRES_HREN0_bm;

	// Advanced Waveform Extension initialization
	// Optimize for speed

	// Disable locking the AWEX configuration registers just to be sure
	TMP=MCU.AWEXLOCK & (~MCU_AWEXCLOCK_bm);
	CCP=CCP_IOREG_gc;
	MCU.AWEXLOCK=TMP;
	// Restore optimization for size if needed

	// Pattern generation: Off
	// Dead time insertion: Off
	AWEXC.CTRL=(0<<AWEX_PGM_bp) | (0<<AWEX_CWCM_bp) | (0<<AWEX_DTICCDEN_bp) | (0<<AWEX_DTICCCEN_bp) |
	(0<<AWEX_DTICCBEN_bp) | (0<<AWEX_DTICCAEN_bp);

	// Fault protection initialization
	// Fault detection on OCD Break detection: On
	// Fault detection restart mode: Latched Mode
	// Fault detection action: None (Fault protection disabled)
	AWEXC.FDCTRL=(AWEXC.FDCTRL & (~(AWEX_FDDBD_bm | AWEX_FDMODE_bm | AWEX_FDACT_gm))) |
	(0<<AWEX_FDDBD_bp) | (0<<AWEX_FDMODE_bp) | AWEX_FDACT_NONE_gc;
	// Fault detect events:
	// Event channel 0: Off
	// Event channel 1: Off
	// Event channel 2: Off
	// Event channel 3: Off
	// Event channel 4: Off
	// Event channel 5: Off
	// Event channel 6: Off
	// Event channel 7: Off
	AWEXC.FDEMASK=0b00000000;
	// Make sure the fault detect flag is cleared
	AWEXC.STATUS|=AWEXC.STATUS & AWEX_FDF_bm;

	// Clear the interrupt flags
	TCC0.INTFLAGS=TCC0.INTFLAGS;
	// Set Counter register
	TCC0.CNT=0x0000;
	// Set Period register
	TCC0.PER=0x02FF;
	// Set channel A Compare/Capture register
	TCC0.CCA=0x0000;
	// Set channel B Compare/Capture register
	TCC0.CCB=0x0000;
	// Set channel C Compare/Capture register
	TCC0.CCC=0x0000;
	// Set channel D Compare/Capture register
	TCC0.CCD=0x0000;

	// Restore interrupts enabled/disabled state
	SREG=SREG_CACHE;
}



// Timer/Counter TCC1 initialization
static inline void setup_tcc1(void)
{
	unsigned char SREG_CACHE;

	// Note: The correct PORTC direction for the Compare Channels
	// outputs is configured in the ports_init function.

	// Save interrupts enabled/disabled state
	SREG_CACHE=SREG;
	// Disable interrupts
	asm("cli");

	// Disable and reset the timer/counter just to be sure
	tc1_disable(&TCC1);
	// Clock source: ClkPer/64
	TCC1.CTRLA=TC_CLKSEL_DIV2_gc;
	// Mode: Normal Operation, Overflow Int./Event on TOP
	// Compare/Capture on channel A: Off
	// Compare/Capture on channel B: Off
	TCC1.CTRLB=(0<<TC1_CCBEN_bp) | (0<<TC1_CCAEN_bp) |
	TC_WGMODE_NORMAL_gc;
	// Capture event source: None
	// Capture event action: None
	TCC1.CTRLD=TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;

	// Set Timer/Counter in Normal mode
	TCC1.CTRLE=TC_BYTEM_NORMAL_gc;

	// Overflow interrupt: High Level
	// Error interrupt: Disabled
	TCC1.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;

	// Compare/Capture channel A interrupt: High Level
	// Compare/Capture channel B interrupt: Disabled
	TCC1.INTCTRLB=TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;

	// High resolution extension: Off
	HIRESC.CTRLA&= ~HIRES_HREN1_bm;

	// Clear the interrupt flags
	TCC1.INTFLAGS=TCC1.INTFLAGS;
	// Set Counter register
	TCC1.CNT=0x0000;
	// Set Period register
	TCC1.PER=0xFFFF;
	// Set channel A Compare/Capture register
	TCC1.CCA=0x0000;
	// Set channel B Compare/Capture register
	TCC1.CCB=0x0000;

	// Restore interrupts enabled/disabled state
	SREG=SREG_CACHE;
}


// Timer/Counter TCD0 initialization
static inline void setup_tcd0(void)
{
	unsigned char SREG_CACHE;

	// Note: The correct PORTD direction for the Compare Channels
	// outputs is configured in the ports_init function.

	// Save interrupts enabled/disabled state
	SREG_CACHE=SREG;
	// Disable interrupts
	asm("cli");

	// Disable and reset the timer/counter just to be sure
	tc0_disable(&TCD0);
	// Clock source: ClkPer/1
	TCD0.CTRLA=TC_CLKSEL_DIV1_gc;
	// Mode: Normal Operation, Overflow Int./Event on TOP
	// Compare/Capture on channel A: On
	// Compare/Capture on channel B: Off
	// Compare/Capture on channel C: Off
	// Compare/Capture on channel D: Off
	TCD0.CTRLB=(0<<TC0_CCDEN_bp) | (0<<TC0_CCCEN_bp) | (0<<TC0_CCBEN_bp) | (1<<TC0_CCAEN_bp) |
	TC_WGMODE_NORMAL_gc;
	// Capture event source: Event Channel 0
	// Capture event action: Pulse Width Capture
	TCD0.CTRLD=TC_EVACT_PW_gc | TC_EVSEL_CH0_gc;

	// Set Timer/Counter in Normal mode
	TCD0.CTRLE=TC_BYTEM_NORMAL_gc;

	// Overflow interrupt: Disabled
	// Error interrupt: Disabled
	TCD0.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;

	// Compare/Capture channel A interrupt: Disabled
	// Compare/Capture channel B interrupt: Disabled
	// Compare/Capture channel C interrupt: Disabled
	// Compare/Capture channel D interrupt: Disabled
	TCD0.INTCTRLB=TC_CCDINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;

	// High resolution extension: Off
	HIRESD.CTRLA&= ~HIRES_HREN0_bm;

	// Clear the interrupt flags
	TCD0.INTFLAGS=TCD0.INTFLAGS;
	// Set Counter register
	TCD0.CNT=0x0000;
	// Set Period register
	TCD0.PER=0xF9FF;
	// Set channel A Compare/Capture register
	TCD0.CCA=0x0000;
	// Set channel B Compare/Capture register
	TCD0.CCB=0x0000;
	// Set channel C Compare/Capture register
	TCD0.CCC=0x0000;
	// Set channel D Compare/Capture register
	TCD0.CCD=0x0000;

	// Restore interrupts enabled/disabled state
	SREG=SREG_CACHE;
}


// Timer/Counter TCE0 initialization
static inline void setup_tce0(void)
{
	unsigned char SREG_CACHE;

	// Note: The correct PORTE direction for the Compare Channels
	// outputs is configured in the ports_init function.

	// Save interrupts enabled/disabled state
	SREG_CACHE=SREG;
	// Disable interrupts
	asm("cli");

	// Disable and reset the timer/counter just to be sure
	tc0_disable(&TCE0);
	TCE0.CTRLA=TC_CLKSEL_DIV2_gc;

	TCE0.CTRLB=(0<<TC0_CCDEN_bp) | (0<<TC0_CCCEN_bp) | (0<<TC0_CCBEN_bp) | (0<<TC0_CCAEN_bp) |
	TC_WGMODE_NORMAL_gc;
	// Capture event source: Event Channel 0
	// Capture event action: Pulse Width Capture
	TCE0.CTRLD=TC_EVACT_OFF_gc | TC_EVSEL_OFF_gc;

	// Set Timer/Counter in Normal mode
	TCE0.CTRLE=TC_BYTEM_NORMAL_gc;

	TCE0.INTCTRLA=TC_ERRINTLVL_OFF_gc | TC_OVFINTLVL_OFF_gc;
	TCE0.INTCTRLB=TC_CCDINTLVL_OFF_gc | TC_CCCINTLVL_OFF_gc | TC_CCBINTLVL_OFF_gc | TC_CCAINTLVL_OFF_gc;

	HIRESE.CTRLA&= ~HIRES_HREN0_bm;

	// Clear the interrupt flags
	TCE0.INTFLAGS=TCE0.INTFLAGS;
	// Set Counter register
	TCE0.CNT=0x0000;
	// Set Period register
	TCE0.PER=0xFFFF;
	// Set channel A Compare/Capture register
	TCE0.CCA=0x0000;
	// Set channel B Compare/Capture register
	TCE0.CCB=0x0000;
	// Set channel C Compare/Capture register
	TCE0.CCC=0x0000;
	// Set channel D Compare/Capture register
	TCE0.CCD=0x0000;

	// Restore interrupts enabled/disabled state
	SREG=SREG_CACHE;
}


#endif