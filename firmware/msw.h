////////////////////////////////////////////////////////////
//
//   ///////  ///////  ////////  /////////    ////////
//   ////  ////  ////    ////    ////   ////    ////
//   ////  ////  ////    ////    ////   ////    ////
//   ////        ////    ////    ////   ////    ////
//   ////        ////  ////////  /////////    ////////
//
//   ////  //    // // ////// //// //   // ///// /////
//  //     //    // //   //  //    //   // //    //  //
//   ////  // // // //   //  //    /////// ////  /////
//      // // // // //   //  //    //   // //    //  //
//   ////  ///  /// //   //   //// //   // ///// //  //
//
////////////////////////////////////////////////////////////

#include <system.h>

typedef unsigned char byte;

/*
TRANSISTOR BOARD
		VDD - VSS
H~		RA5	- RA0/PGD	TX(Not used)
LED		RA4 - RA1/PGC	RX
SW		VPP - RA2		E~
G~		RC5 - RC0		A
D		RC4 - RC1		F~
C		RC3 - RC2		B
*/
#ifdef TRANSISTOR_SWITCHER

#define TRISA_MASK		0b11001011
#define TRISC_MASK		0b11000000
#define P_SWITCH		porta.3
#define P_WPU			wpua.3
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

#define APFCON0_MASK 	0b10000100 // RX is on RA1/TX is on RA0
#define APFCON1_MASK 	0b00000001 // CCP2 is on RA5
	
#endif

/*
Relays Board
		
		VDD - VSS
LED		RA5	- RA0/PGD	P7
SW		RA4 - RA1/PGC	P6
		VPP - RA2		P5
RX		RC5 - RC0		P4
P1		RC4 - RC1		P3
P0		RC3 - RC2		P2
*/	
#ifdef RELAY_SWITCHER

#define TRISA_MASK		0b11011111
#define TRISC_MASK		0b11111111
#define P_SWITCH		porta.4
#define P_WPU			wpua.4
#define P_LED 			lata.5

#define T_OUT0		trisc.3 
#define T_OUT1		trisc.4 
#define T_OUT2		trisc.2
#define T_OUT3		trisc.1 
#define T_OUT4		trisc.0
#define T_OUT5		trisa.2 
#define T_OUT6		trisa.1
#define T_OUT7		trisa.0	

#define APFCON0_MASK 	0b00000000
#define APFCON1_MASK 	0b00000000

#endif

#define PWM_HALF 		0x80
#define PWM_FULL 		0xfe

// Program defs
#define PGM_MAX 		16
#define SWITCH_MAX 		8

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

// SYSEX PARAMS
#define PARAMH_PORTA			1
#define PARAMH_PORTB			2
#define PARAMH_PORTC			3
#define PARAMH_PORTD			4
#define PARAMH_PORTE			5
#define PARAMH_PORTF			6
#define PARAMH_PORTG			7
#define PARAMH_PORTH			8
#define PARAMH_PORT_DEFAULT 	100

#define PARAML_TRIG_NONE		0
#define PARAML_TRIG_NOTE		1
#define PARAML_TRIG_CC			2
#define PARAML_TRIG_PGM			3
#define PARAML_TRIG_ALWAYS		4
#define PARAML_TRIG_NOTE_RANGE	5

#define PARAML_TRIG2_CHAN		10
#define PARAML_TRIG2_VALUE_MIN	11
#define PARAML_TRIG2_VALUE_MAX	12

#define PARAML_PGM_CHAN 		20
#define PARAML_ENV_SET 			50
#define PARAML_ENV_TYPE 		51
#define PARAML_ENV_HOLD 		52
#define PARAML_FLAGS 			60
#define PARAML_VEL_MOD_DEST 	70
#define PARAML_CC_MOD_DEST 		71
#define PARAML_CC_MOD_CHAN 		72
#define PARAML_CC_MOD_CC 		73


#define PARAMH_PGM_SLOT_BASE	10
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
#define PARAMH_PGM_SLOT_MAX		25


#define PARAML_PGM_MATCH		1
#define PARAML_PGM_PORT			2

// FUNCTION PROTOTYPES
void switch_init();
void switch_service();
void switch_reset();
void switch_tick();
void switch_on_note(byte chan, byte note, byte vel);
void switch_on_cc(byte chan, byte cc_no, byte value);
void switch_on_pgm(byte chan, byte pgm);
void switch_cfg(byte param_hi, byte param_lo, byte value_hi, byte value_lo);
void switch_storage_write(int* addr);
void switch_storage_read(int* addr);
void pwm_set(byte which, byte duty, byte gamma);
void storage_write(byte *data, int len, int* addr);
void storage_read(byte *data, int len, int* addr);
void storage_write_patch();
void storage_read_patch();
