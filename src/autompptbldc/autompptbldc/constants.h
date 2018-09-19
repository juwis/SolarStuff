#ifndef CONSTANTS_H
#define CONSTANTS_H

#define MPPT_VOLTAGE	6350
#define MPPT_VOLTAGE_ADJUST_TOP	6550
#define MPPT_VOLTAGE_ADJUST_BOT	6150


#define TRANSMITTER_MIN_PULSE	17000
#define TRANSMITTER_MAX_PULSE	31000
#define TRANSMITTER_FILTER_STRENGTH	8

#define NORMALIZED_MIN_THROTTLE	50
#define NORMALIZED_MAX_THROTTLE	1000 - NORMALIZED_MIN_THROTTLE


#define PHASE_U_HIGH     0b00000011
#define PHASE_U_LOW      0b00000001
#define PHASE_U_PWM_PIN  0b00000010
#define PHASE_U_FLOAT    0b00000000
#define PHASE_V_HIGH     0b00001100
#define PHASE_V_LOW      0b00000100
#define PHASE_V_PWM_PIN  0b00001000
#define PHASE_V_FLOAT    0b00000000
#define PHASE_W_HIGH     0b00110000
#define PHASE_W_LOW      0b00010000
#define PHASE_W_PWM_PIN  0b00100000
#define PHASE_W_FLOAT    0b00000000

#define PHASE_0_ON                (PHASE_U_HIGH | PHASE_V_LOW | PHASE_W_FLOAT)
#define PHASE_1_ON                (PHASE_U_HIGH | PHASE_V_FLOAT | PHASE_W_LOW)
#define PHASE_2_ON                (PHASE_U_FLOAT | PHASE_V_HIGH | PHASE_W_LOW)
#define PHASE_3_ON                (PHASE_U_LOW | PHASE_V_HIGH | PHASE_W_FLOAT)
#define PHASE_4_ON                (PHASE_U_LOW | PHASE_V_FLOAT | PHASE_W_HIGH)
#define PHASE_5_ON                (PHASE_U_FLOAT | PHASE_V_LOW | PHASE_W_HIGH)

//make it indexable
uint8_t PHASE_ON[6] = {PHASE_0_ON, PHASE_1_ON, PHASE_2_ON, PHASE_3_ON, PHASE_4_ON, PHASE_5_ON};

#define PHASE_0_OFF				(PHASE_U_LOW | PHASE_V_LOW | PHASE_W_FLOAT)
#define PHASE_1_OFF				(PHASE_U_LOW | PHASE_V_FLOAT | PHASE_W_LOW)
#define PHASE_2_OFF				(PHASE_U_FLOAT | PHASE_V_LOW | PHASE_W_LOW)
#define PHASE_3_OFF				(PHASE_U_LOW | PHASE_V_LOW | PHASE_W_FLOAT)
#define PHASE_4_OFF				(PHASE_U_LOW | PHASE_V_FLOAT | PHASE_W_LOW)
#define PHASE_5_OFF				(PHASE_U_FLOAT | PHASE_V_LOW | PHASE_W_LOW)

//make it indexable
uint8_t PHASE_OFF[6] = {PHASE_0_OFF, PHASE_1_OFF, PHASE_2_OFF, PHASE_3_OFF, PHASE_4_OFF, PHASE_5_OFF};

#if 0
#define PHASE_0_PWM_PIN				(PHASE_U_PWM_PIN | PHASE_U_LOW)
#define PHASE_1_PWM_PIN				(PHASE_U_PWM_PIN | PHASE_U_LOW)
#define PHASE_2_PWM_PIN				(PHASE_V_PWM_PIN | PHASE_V_LOW)
#define PHASE_3_PWM_PIN				(PHASE_V_PWM_PIN | PHASE_V_LOW)
#define PHASE_4_PWM_PIN				(PHASE_W_PWM_PIN | PHASE_W_LOW)
#define PHASE_5_PWM_PIN				(PHASE_W_PWM_PIN | PHASE_W_LOW)
#else
#define PHASE_0_PWM_PIN			PHASE_U_PWM_PIN
#define PHASE_1_PWM_PIN			PHASE_U_PWM_PIN
#define PHASE_2_PWM_PIN			PHASE_V_PWM_PIN
#define PHASE_3_PWM_PIN			PHASE_V_PWM_PIN
#define PHASE_4_PWM_PIN			PHASE_W_PWM_PIN
#define PHASE_5_PWM_PIN			PHASE_W_PWM_PIN
#endif

//make it indexable
uint8_t PHASE_PWM_PIN[6] = {PHASE_0_PWM_PIN, PHASE_1_PWM_PIN, PHASE_2_PWM_PIN, PHASE_3_PWM_PIN, PHASE_4_PWM_PIN, PHASE_5_PWM_PIN};

#define PHASES_FLOAT			(PHASE_U_FLOAT | PHASE_V_FLOAT | PHASE_W_FLOAT)
#define PHASES_LOW_BREAK		(PHASE_U_LOW | PHASE_V_FLOAT | PHASE_W_LOW)
#define PHASES_HIGH_BREAK		(PHASE_U_HIGH | PHASE_V_HIGH | PHASE_W_HIGH)

#define SENSE_U_MUXPOS         AC_MUXPOS_PIN1_gc	//PortA Pin 1
#define SENSE_V_MUXPOS         AC_MUXPOS_PIN2_gc	//PortA Pin 2
#define SENSE_W_MUXPOS         AC_MUXPOS_PIN3_gc	//PortA Pin 3

// for PWM via the AWEX Unit we need to split the above information into three parts
// 
// - the actual PWMed pins
// - freewheeling enabled or not
// - Phase State
// and we will accomplish that by creating small inlined functions that set all up.

//the OUTput OVErride eNable register will take the pins that are switched
//PORTC is set to the static scheme for tha set phase.
static inline void  SET_PHASE_PWM(uint8_t phase, uint8_t breaking){
	if (breaking != 0){
		AWEXC.OUTOVEN = 0;
		PORTC.OUT = PHASES_LOW_BREAK;
	}else{
		//phase = (phase-1) %6;
		AWEXC.OUTOVEN = PHASE_PWM_PIN[phase];
		PORTC.OUT = PHASE_OFF[phase];
	}
}

#define SENSE_0_FNG_AC_MUXPOS	SENSE_W_MUXPOS
#define SENSE_1_RSG_AC_MUXPOS	SENSE_V_MUXPOS
#define SENSE_2_FNG_AC_MUXPOS	SENSE_U_MUXPOS
#define SENSE_3_RSG_AC_MUXPOS	SENSE_W_MUXPOS
#define SENSE_4_FNG_AC_MUXPOS	SENSE_V_MUXPOS
#define SENSE_5_RSG_AC_MUXPOS	SENSE_U_MUXPOS


#define PWM_TOP_PER		0x02FF

#endif