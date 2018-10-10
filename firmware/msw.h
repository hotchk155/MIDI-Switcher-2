
#include <system.h>

// MIDI message bytes
#define MIDI_MTC_QTR_FRAME 		0xf1
#define MIDI_SPP 				0xf2
#define MIDI_SONG_SELECT 		0xf3 
#define MIDI_SYNCH_TICK     	0xf8
#define MIDI_SYNCH_START    	0xfa
#define MIDI_SYNCH_CONTINUE 	0xfb
#define MIDI_SYNCH_STOP     	0xfc
#define MIDI_SYSEX_BEGIN     	0xf0
#define MIDI_SYSEX_END     		0xf7

#define MIDI_CC_NRPN_HI 		99
#define MIDI_CC_NRPN_LO 		98
#define MIDI_CC_DATA_HI 		6
#define MIDI_CC_DATA_LO 		38

// Sysex ID
#define MY_SYSEX_ID0	0x00
#define MY_SYSEX_ID1	0x7f
#define MY_SYSEX_ID2	0x19 // MIDI switcher patch

#define PARAMH_PORTA			1
#define PARAMH_PORTB			2
#define PARAMH_PORTC			3
#define PARAMH_PORTD			4
#define PARAMH_PORTE			5
#define PARAMH_PORTF			6
#define PARAMH_PORTG			7
#define PARAMH_PORTH			8

#define PARAML_TRIG_NONE		0
#define PARAML_TRIG_NOTE		1
#define PARAML_TRIG_CC			2
#define PARAML_TRIG_PGM			3
#define PARAML_TRIG2_VELOCITY	4
#define PARAML_TRIG2_VALUE		5

#define PARAML_ENV_SUSTAIN		50
#define PARAML_ENV_HOLD			51

#define PARAML_DUR_MOD_NONE		100
#define PARAML_DUR_MOD_VELOCITY	101
#define PARAML_DUR_MOD_CC		102

#define PARAML_PWM_MOD_NONE		110
#define PARAML_PWM_MOD_VELOCITY	111
#define PARAML_PWM_MOD_CC		112

#define PARAMH_PGM_ALL			10
#define PARAMH_PGM_SLOT1		11
#define PARAMH_PGM_SLOT2		12
#define PARAMH_PGM_SLOT3		13
#define PARAMH_PGM_SLOT4		14
#define PARAMH_PGM_SLOT5		15
#define PARAMH_PGM_SLOT6		16
#define PARAMH_PGM_SLOT7		17
#define PARAMH_PGM_SLOT8		18
#define PARAMH_PGM_SLOT9		19
#define PARAMH_PGM_SLOT10		20
#define PARAMH_PGM_SLOT11		21
#define PARAMH_PGM_SLOT12		22
#define PARAMH_PGM_SLOT13		23
#define PARAMH_PGM_SLOT14		24
#define PARAMH_PGM_SLOT15		25
#define PARAMH_PGM_SLOT16		26

#define PARAML_PGM_CLEAR		0
#define PARAML_PGM_MATCH		1
#define PARAML_PGM_PORT			2

typedef unsigned char byte;


/*

		VDD - VSS
H~		RA5	- RA0/PGD	TX(Not used)
LED		RA4 - RA1/PGC	RX
SW		VPP - RA2		E~
G~		RC5 - RC0		A
D		RC4 - RC1		F~
C		RC3 - RC2		B
*/
#define PGM_MAX 16
#define SWITCH_MAX 8


#define P_OUTA 		latc.0
#define P_OUTB 		latc.2
#define P_OUTC 		latc.3
#define P_OUTD 		latc.4
#define P_OUTE 		lata.2  //CCP3
#define P_OUTF 		latc.1	//CCP4
#define P_OUTG 		latc.5	//CCP1
#define P_OUTH 		lata.5	//CCP2
#define P_LED 		lata.4

#define LATC_BIT_OUTA	(1<<0)
#define LATC_BIT_OUTB	(1<<2)
#define LATC_BIT_OUTC	(1<<3)
#define LATC_BIT_OUTD	(1<<4)

/*
	port A	RC0
	port B	RC2
	port C	RC3
	port D 	RC4
	port E  RA2
	port F	RC1
	port G  RC5
	port H	RA5
	
	LED		RA4
*/

//////////////////////////////////////////////////////////////
void switch_init();
void switch_service();
void switch_reset();
void switch_tick();
void switch_on_note(byte chan, byte note, byte vel);
void switch_on_cc(byte chan, byte cc_no, byte value);
void switch_on_pgm(byte chan, byte pgm);
byte switch_cfg(byte param_hi, byte param_lo, byte value_hi, byte value_lo);
byte *switch_storage(int *len);
byte *switch_pgm_storage(int *len);

void storage_write_patch();
void storage_read_patch();
void pwm_set(byte which, byte duty);
