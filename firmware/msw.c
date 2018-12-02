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
// 8-port MIDI-controlled low-side DC power switcher 
// 2018/hotchk155 - Sixty four pixels ltd
//
// VER 	DATE		
// 1	22oct18		initial version
// 1.1	02dec18		add latching mode 
//
////////////////////////////////////////////////////////////

//
// INCLUDE FILES
//
#include <system.h>
#include <memory.h>
#include "msw.h"


// CONFIG OPTIONS 
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

//
// LOCAL DATA
//

// States for sysex loading
enum {
	SYSEX_NONE,		// no sysex
	SYSEX_IGNORE,	// sysex in progress, but not for us
	SYSEX_ID0,		// expect first byte of id
	SYSEX_ID1,		// expect second byte of id
	SYSEX_ID2,		// expect third byte of id
	SYSEX_PARAMH,	// expect high byte of a param number
	SYSEX_PARAML,	// expect low byte of a param number
	SYSEX_VALUEH,	// expect high byte of a param value
	SYSEX_VALUEL	// expect low byte of a param value
};

// define the buffer used to receive MIDI input
#define SZ_RXBUFFER 			64				// size of MIDI receive buffer (power of 2)
#define SZ_RXBUFFER_MASK 		0x3F			// mask to keep an index within range of buffer
static volatile byte rx_buffer[SZ_RXBUFFER];	// the MIDI receive buffer
static volatile byte rx_head = 0;				// buffer data insertion index
static volatile byte rx_tail = 0;				// buffer data retrieval index

// State flags used while receiving MIDI data
static byte midi_lockout = 0;					// flag that says if MIDI is being ignored
static byte midi_status = 0;					// current MIDI message status (running status)
static byte midi_num_params = 0;				// number of parameters needed by current MIDI message
static byte midi_params[2];						// parameter values of current MIDI message
static byte midi_param = 0;						// number of params currently received
static byte midi_ticks = 0;						// number of MIDI clock ticks received
static byte sysex_state = SYSEX_NONE;			// whether we are currently inside a sysex block
static byte sysex_param_hi = 0;					// parameter number MSB for data received by Sysex
static byte sysex_param_lo = 0;					// parameter number LSB for data received by Sysex
static byte sysex_value_hi = 0;					// parameter value MSB for data received by Sysex

// Timer related stuff
#define TIMER_0_INIT_SCALAR		5				// Timer 0 initialiser to overlow at 1ms intervals
static volatile byte ms_tick = 0;				// once per millisecond tick flag used to synchronise stuff

// LED related stuff
#define LED_SHORT_BLINK		10
#define LED_LONG_BLINK		255
static volatile byte led_timeout = 0;

// number of ms to hold switch for MIDI lockout
#define LONG_PRESS_TIME		1000

// PWM duty for each output
static byte pwm_duty[8];

// Gamma correction table (from Adafruit example
// https://learn.adafruit.com/led-tricks-gamma-correction/the-quick-fix)
rom char *gamma = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,
    1,  1,  1,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,
    2,  3,  3,  3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,
    5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  9,  9,  9, 10,
   10, 10, 11, 11, 11, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16,
   17, 17, 18, 18, 19, 19, 20, 20, 21, 21, 22, 22, 23, 24, 24, 25,
   25, 26, 27, 27, 28, 29, 29, 30, 31, 32, 32, 33, 34, 35, 35, 36,
   37, 38, 39, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 50,
   51, 52, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 66, 67, 68,
   69, 70, 72, 73, 74, 75, 77, 78, 79, 81, 82, 83, 85, 86, 87, 89,
   90, 92, 93, 95, 96, 98, 99,101,102,104,105,107,109,110,112,114,
  115,117,119,120,122,124,126,127,129,131,133,135,137,138,140,142,
  144,146,148,150,152,154,156,158,160,162,164,167,169,171,173,175,
  177,180,182,184,186,189,191,193,196,198,200,203,205,208,210,213,
  215,218,220,223,225,228,231,233,236,239,241,244,247,249,252,255 
};

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
//
// PRIVATE FUNCTIONS
//
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINE
void interrupt( void )
{
	/////////////////////////////////////////////////////
	// TIMER 0 OVERFLOW
	if(intcon.2) {
		tmr0 = TIMER_0_INIT_SCALAR;		
		ms_tick = 1;
		intcon.2 = 0;		
	}		

	/////////////////////////////////////////////////////
	// UART RECEIVE
	if(pir1.5)
	{	
		byte b = rcreg;
		byte next_head = (rx_head + 1)&SZ_RXBUFFER_MASK;
		if(next_head != rx_tail) {
			rx_buffer[rx_head] = b;
			rx_head = next_head;
		}
		pir1.5 = 0;
	}
}

////////////////////////////////////////////////////////////
// INITIALISE SERIAL PORT FOR MIDI
static void uart_init()
{
	pir1.1 = 0;		//TXIF 		
	pir1.5 = 0;		//RCIF
	
	pie1.1 = 0;		//TXIE 		no interrupts
	pie1.5 = 1;		//RCIE 		enable
	
	baudcon.4 = 0;	// SCKP		synchronous bit polarity 
	baudcon.3 = 1;	// BRG16	enable 16 bit brg
	baudcon.1 = 0;	// WUE		wake up enable off
	baudcon.0 = 0;	// ABDEN	auto baud detect
		
	txsta.6 = 0;	// TX9		8 bit transmission
	txsta.5 = 0;	// TXEN		transmit enable
	txsta.4 = 0;	// SYNC		async mode
	txsta.3 = 0;	// SEDNB	break character
	txsta.2 = 0;	// BRGH		high baudrate 
	txsta.0 = 0;	// TX9D		bit 9

	rcsta.7 = 1;	// SPEN 	serial port enable
	rcsta.6 = 0;	// RX9 		8 bit operation
	rcsta.5 = 1;	// SREN 	enable receiver
	rcsta.4 = 1;	// CREN 	continuous receive enable
		
	spbrgh = 0;		// brg high byte
	spbrg = 31;		// brg low byte (31250)	
	
}

////////////////////////////////////////////////////////////
// INITIALISE TIMER 0 FOR MILLSECOND CLOCK
static void timer_init() {
	// Configure timer 0 (controls systemticks)
	// 	timer 0 runs at 4MHz
	// 	prescaled 1/16 = 250kHz
	// 	rollover at 250 = 1kHz
	// 	1ms per rollover	
	option_reg.5 = 0; // timer 0 driven from instruction cycle clock
	option_reg.3 = 0; // timer 0 is prescaled
	option_reg.2 = 0; // }
	option_reg.1 = 1; // } 1/16 prescaler
	option_reg.0 = 1; // }
	intcon.5 = 1; 	  // enabled timer 0 interrrupt
	intcon.2 = 0;     // clear interrupt fired flag
		
}

////////////////////////////////////////////////////////////
// INITIALISE PWM PERIPHERAL
static void pwm_init() {

	// ensure the output drivers for each
	// of the CCPx outputs are disabled
	trisa.2 = 1;
	trisa.5 = 1;
	trisc.1 = 1; 
	trisc.5 = 1; 	
		
	// Set CCPx to standard PWM mode
	ccp1con = 0b00001100; 
	ccp2con = 0b00001100; 
	ccp3con = 0b00001100; 
	ccp4con = 0b00001100; 

	// zero all duty cycles
	ccpr1l = 0; 
	ccpr2l = 0; 
	ccpr3l = 0; 
	ccpr4l = 0; 
	
	// set each CCP module to use timer 2
	ccptmrs = 0b00000000;
	
	// Configure timer 2 for x16 prescaler
	t2con = 0b00000010;

	// load timer 2 period register for 255 duty cycles	
	pr2 = 0xFE; 
	
	// clear Timer2 interrupt flag
	pir1.1 = 0;
	
	// start up the timer
	t2con.2 = 1;	
	
	// wait for Timer2 to overflow once
	while(!pir1.1); 

	// now we can enable the output drivers
	trisa.2 = 0;
	trisa.5 = 0;
	trisc.1 = 0; 
	trisc.5 = 0; 	
}

////////////////////////////////////////////////////////////
// HANDLE A PARAMETER (MSB-LSB) VALUE (MSB-LSB) PAIR 
static void sysex_param(byte param_hi, byte param_lo, byte value_hi, byte value_lo) {
	switch_cfg(param_hi, param_lo, value_hi, value_lo);
}

////////////////////////////////////////////////////////////
// RESET EVERYTHING
static void all_reset() {
	switch_reset();
}

////////////////////////////////////////////////////////////
// GET NEXT MESSAGE FROM MIDI INPUT
static byte midi_in()
{
	// loop until there is no more data or
	// we receive a full message
	for(;;)
	{
		// usart buffer overrun error?
		if(rcsta.1)
		{
			rcsta.4 = 0;
			rcsta.4 = 1;
		}
		
		// check for empty receive buffer
		if(rx_head == rx_tail)
			return 0;
		
		// read the character out of buffer
		byte ch = rx_buffer[rx_tail];
		++rx_tail;
		rx_tail&=SZ_RXBUFFER_MASK;

		// if MIDI lockout is active then the message will
		// simply be ignored
		if(midi_lockout) {
			midi_status = 0;
			return 0;
		}

		// flash the LED
		P_LED = 1;
		led_timeout = LED_SHORT_BLINK;		
				
		// SYSTEM MESSAGE
		if((ch & 0xf0) == 0xf0)
		{
			switch(ch)
			{
			// RELEVANT REALTIME MESSAGE 
			case MIDI_SYNCH_TICK:
			case MIDI_SYNCH_START:
			case MIDI_SYNCH_CONTINUE:
			case MIDI_SYNCH_STOP:
				return ch;		
			// SYSTEM COMMON MESSAGES WITH PARAMETERS
			case MIDI_MTC_QTR_FRAME:	// 1 param byte follows
			case MIDI_SONG_SELECT:		// 1 param byte follows			
			case MIDI_SPP:				// 2 param bytes follow
				midi_param = 0;
				midi_status = ch; 
				midi_num_params = (ch==MIDI_SPP)? 2:1;
				break;
			// START OF SYSEX	
			case MIDI_SYSEX_BEGIN:
				sysex_state = SYSEX_ID0; 
				break;
			// END OF SYSEX	
			case MIDI_SYSEX_END:
				switch(sysex_state) {
				case SYSEX_IGNORE: // we're ignoring a syex block
				case SYSEX_NONE: // we weren't even in sysex mode!					
					break;			
				case SYSEX_PARAMH:	// the state we'd expect to end in
					P_LED = 1; 
					storage_write_patch();
					delay_ms(250); 
					delay_ms(250); 
					delay_ms(250); 
					delay_ms(250); 
					P_LED = 0; 
					all_reset();
					break;
				default:	// any other state would imply bad sysex data
					for(char i=0; i<10; ++i) {
						P_LED = 1; 
						delay_ms(100);
						P_LED = 0; 
						delay_ms(100);
					}
					all_reset();
					break;
				}
				sysex_state = SYSEX_NONE; 
				break;
			}
			// Ignoring....			
			//  0xF4	RESERVED
			//  0xF5	RESERVED
			//  0xF6	TUNE REQUEST
			//	0xF9	RESERVED
			//	0xFD	RESERVED
			//	0xFE	ACTIVE SENSING
			//	0xFF	RESET
		}    
		// STATUS BYTE
		else if(!!(ch & 0x80))
		{
			// a status byte cancels sysex state
			sysex_state = SYSEX_NONE;
		
			midi_param = 0;
			midi_status = ch; 
			switch(ch & 0xF0)
			{
			case 0xC0: //  Patch change  1  instrument #   
			case 0xD0: //  Channel Pressure  1  pressure  
				midi_num_params = 1;
				break;    
			case 0xA0: //  Polyphonic aftertouch  2  key  touch  
			case 0x80: //  Note-off  2  key  velocity  
			case 0x90: //  Note-on  2  key  veolcity  
			case 0xB0: //  Continuous controller  2  controller #  controller value  
			case 0xE0: //  Pitch bend  2  lsb (7 bits)  msb (7 bits)  
			default:
				midi_num_params = 2;
				break;        
			}
		}    
		else 
		{
			switch(sysex_state) // are we inside a sysex block?
			{
			// SYSEX MANUFACTURER ID
			case SYSEX_ID0: sysex_state = (ch == MY_SYSEX_ID0)? SYSEX_ID1 : SYSEX_IGNORE; break;
			case SYSEX_ID1: sysex_state = (ch == MY_SYSEX_ID1)? SYSEX_ID2 : SYSEX_IGNORE; break;
			case SYSEX_ID2: sysex_state = (ch == MY_SYSEX_ID2)? SYSEX_PARAMH : SYSEX_IGNORE; break;
			// CONFIG PARAM DELIVERED BY SYSEX
			case SYSEX_PARAMH: sysex_param_hi = ch; ++sysex_state; break;
			case SYSEX_PARAML: sysex_param_lo = ch; ++sysex_state;break;
			case SYSEX_VALUEH: sysex_value_hi = ch; ++sysex_state;break;
			case SYSEX_VALUEL: sysex_param(sysex_param_hi, sysex_param_lo, sysex_value_hi, ch); sysex_state = SYSEX_PARAMH; break;
			case SYSEX_IGNORE: break;			
			// MIDI DATA
			case SYSEX_NONE: 
				if(midi_status)
				{
					// gathering parameters
					midi_params[midi_param++] = ch;
					if(midi_param >= midi_num_params)
					{
						// we have a complete message.. is it one we care about?
						midi_param = 0;
						switch(midi_status&0xF0)
						{
						case 0x80: // note off
						case 0x90: // note on
						//case 0xE0: // pitch bend
						case 0xB0: // cc
						case 0xC0: // program change
							return midi_status; 
						}
					}
				}
			}
		}
	}
	// no message ready yet
	return 0;
}

////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
//
// PUBLIC FUNCTIONS
//
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// SET PWM DUTY
// Outputs A-D uses "software" PWM
// Outputs E-H use the PIC PWM peripheral
void pwm_set(byte which, byte duty, byte gamma_correction) 
{	
	// perform gamma table mapping if needed
	if(gamma_correction) {
		duty = gamma[duty];
	}
	// check if the duty has actually changed
	if(pwm_duty[which] != duty)
	{
		// store the new duty
		pwm_duty[which] = duty;
		switch(which) 
		{
			// output 0-3, ensure fastest switch on if duty is 100
			case 0: if(duty==0xFF) P_OUTA = 1; break;
			case 1: if(duty==0xFF) P_OUTB = 1; break;
			case 2: if(duty==0xFF) P_OUTC = 1; break;
			case 3: if(duty==0xFF) P_OUTD = 1; break;
			// output 4-7, configure the hardware peripheral
			case 4:	ccpr3l = duty; break;
			case 5:	ccpr4l = duty; break;
			case 6:	ccpr1l = duty; break;
			case 7:	ccpr2l = duty; break;
		}
	}
}

////////////////////////////////////////////////////////////
// MAIN
void main()
{ 
	osccon = 0b01111010;	// osc control / 16MHz / internal

	apfcon0.7 = 1; 			// RX is on RA1
	apfcon0.2 = 1; 			// RX is on RA0
	apfcon1.0 = 1; 			// CCP2 is on RA5

	trisa = TRISA_MASK;		// set pin in/out directions
	trisc = TRISC_MASK;

	ansela = 0b00000000;	// disable all analog inputs
	anselc = 0b00000000;

	
	P_WPU = 1; 				// weak pull up on switch input
	option_reg.7 = 0; 		// weak pull up enable

	// we are alive...
	P_LED = 1; delay_ms(100); P_LED = 0; delay_ms(100);
	P_LED = 1; delay_ms(100); P_LED = 0; delay_ms(100);
	
	pwm_init();				// initialise PWM peripheral
	t1con = 0b01000000;		// configure timer 1, used for "fake" PWM
	timer_init();			// configure timer 0, used for millisecond clock
	uart_init();			// configure the serial port	
	
	intcon.7 = 1; 			// enable interrupts
	intcon.6 = 1; 			// enable peripheral interrupts
	t1con.0 = 1;			// start up timer 1

	
	memset(pwm_duty, 0, sizeof(pwm_duty)); // clear the PWM data	
	switch_init();			// initialise switch data	
	storage_read_patch();	// read patch from EEPROM	
	switch_reset();			// set initial states of outputs

	// we are still alive...
	P_LED = 1; delay_ms(100); P_LED = 0; delay_ms(100);
	P_LED = 1; delay_ms(100); P_LED = 0; delay_ms(100);

	// now, main app loop...
	int switch_hold = 0;	
	byte lockout_count = 0;	
	byte pwm = 0;
	for(;;)
	{	
		// once per millisecond tick event
		if(ms_tick) {
			ms_tick = 0;
			
			// service outputs state machine
			switch_tick();
			
			// update LED
			if(led_timeout) {
				if(!--led_timeout) {
					P_LED = 0;
				}
			}
			
			// is switch pressed?
			if(!P_SWITCH) {
			
				// was it pressed before?
				if(!switch_hold) {
				
					// no, so perform a reset and clear any
					// MIDI lockout
					all_reset();
					midi_lockout = 0;
					switch_hold = 1;
					
					// LED long blink
					P_LED = 1;
					led_timeout = LED_LONG_BLINK;
				}
				// has it been held for long time?
				else if(switch_hold > LONG_PRESS_TIME) {
				
					// MIDI lockout!
					midi_lockout = 1;
					lockout_count = 0;
					
					// LED on
					P_LED = 1;
					led_timeout = 0;
				}
				else {
					// counting another ms of switch holdingness
					++switch_hold;
				}
			}
			else {
				// switch is not held
				switch_hold = 0;
			}
						
			// Blink the LED in MIDI lockout mode 
			if(midi_lockout) {
				if(!++lockout_count) {
					P_LED = !P_LED;
				}
			}
		}
		
		// poll for incoming MIDI data
		byte msg = midi_in();		
		if(msg) {
			switch(msg & 0xF0) {
			// MIDI NOTE OFF
			case 0x80:
				switch_on_note(msg&0x0F, midi_params[0], 0);
				break;
			// MIDI NOTE ON
			case 0x90:
				switch_on_note(msg&0x0F, midi_params[0], midi_params[1]);
				break;				
			// CONTINUOUS CONTROLLER
			case 0xB0: 
				switch_on_cc(msg&0x0F, midi_params[0], midi_params[1]);
				break;			
			// PROGRAM CHANGE
			case 0xC0: 
				switch_on_pgm(msg&0x0F, midi_params[0]);
				break;
			}
		}
		
		// manage the software PWM on outputs A-D
		if(!pwm) {
			P_OUTA = !!pwm_duty[0];
			P_OUTB = !!pwm_duty[1];
			P_OUTC = !!pwm_duty[2];
			P_OUTD = !!pwm_duty[3];
		}
		else {
			if(pwm > pwm_duty[0]) P_OUTA = 0;
			if(pwm > pwm_duty[1]) P_OUTB = 0;
			if(pwm > pwm_duty[2]) P_OUTC = 0;
			if(pwm > pwm_duty[3]) P_OUTD = 0;
		}
		++pwm;
	}
}

//
// END
//