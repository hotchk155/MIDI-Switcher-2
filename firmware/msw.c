
// CONFIG OPTIONS 
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#include <system.h>
#include <memory.h>
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000



#include "msw.h"

#define P_SWITCH	porta.3

#define P_TRISA		0b11001011
#define P_TRISC		0b11000000
#define P_WPU		wpua.3





//
// TYPE DEFS
//

//
// GLOBAL DATA
//
				
/*
// Gamma correction table - map 7 bit CC value to 
// 8 bit PWM brightness level									
rom char *gamma = {
0, 0, 0, 0, 0, 0, 0, 0, 
0, 0, 0, 0, 0, 0, 1, 1, 
1, 1, 1, 1, 2, 2, 2, 2, 
3, 3, 3, 3, 4, 4, 5, 5, 
6, 6, 7, 7, 8, 8, 9, 10, 
10, 11, 12, 13, 13, 14, 15, 16, 
17, 18, 19, 20, 21, 22, 24, 25, 
26, 27, 29, 30, 32, 33, 35, 36, 
38, 39, 41, 43, 45, 47, 49, 50, 
52, 55, 57, 59, 61, 63, 66, 68, 
70, 73, 75, 78, 81, 83, 86, 89, 
92, 95, 98, 101, 104, 107, 110, 114, 
117, 120, 124, 127, 131, 135, 138, 142, 
146, 150, 154, 158, 162, 167, 171, 175, 
180, 184, 189, 193, 198, 203, 208, 213, 
218, 223, 228, 233, 239, 244, 249, 255 };
*/

// PIC CONFIG BITS
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC


//
// TYPES
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

//
// LOCAL DATA
//

// define the buffer used to receive MIDI input
#define SZ_RXBUFFER 			64		// size of MIDI receive buffer (power of 2)
#define SZ_RXBUFFER_MASK 		0x3F	// mask to keep an index within range of buffer
volatile byte rx_buffer[SZ_RXBUFFER];	// the MIDI receive buffer
volatile byte rx_head = 0;				// buffer data insertion index
volatile byte rx_tail = 0;				// buffer data retrieval index

// State flags used while receiving MIDI data
byte midi_status = 0;					// current MIDI message status (running status)
byte midi_num_params = 0;				// number of parameters needed by current MIDI message
byte midi_params[2];					// parameter values of current MIDI message
char midi_param = 0;					// number of params currently received
byte midi_ticks = 0;					// number of MIDI clock ticks received
byte sysex_state = SYSEX_NONE;			// whether we are currently inside a sysex block

// Timer related stuff
#define TIMER_0_INIT_SCALAR		5		// Timer 0 initialiser to overlow at 1ms intervals
volatile byte ms_tick = 0;				// once per millisecond tick flag used to synchronise stuff
//volatile int millis = 0;				// millisecond counter

byte sysex_param_hi = 0;
byte sysex_param_lo = 0;
byte sysex_value_hi = 0;

#define LED_SHORT_BLINK		10
#define LED_LONG_BLINK		255
volatile byte g_led_timeout = 0;

byte pwm_duty[8];

////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINE
void interrupt( void )
{
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
void uart_init()
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
// INITIALISE TIMER
void timer_init() {
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
// Set PWM duty (8-bits) on one of the output channels
// Outputs A-D uses "software" PWM
// Outputs E-H use the PIC PWM peripheral
void pwm_set(byte which, byte duty, byte invert) 
{	
	if(invert) {
		duty = 255-duty;
	}
	if(pwm_duty[which] != duty)
	{
		pwm_duty[which] = duty;
		switch(which) 
		{
			case 0: if(duty==0xFF) P_OUTA = 1; break;
			case 1: if(duty==0xFF) P_OUTB = 1; break;
			case 2: if(duty==0xFF) P_OUTC = 1; break;
			case 3: if(duty==0xFF) P_OUTD = 1; break;
			case 4:	ccpr3l = duty; break;
			case 5:	ccpr4l = duty; break;
			case 6:	ccpr1l = duty; break;
			case 7:	ccpr2l = duty; break;
		}
	}
}


////////////////////////////////////////////////////////////
// Configure hardware PWM using CCP modules
void pwm_init() {

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
void sysex_param(byte param_hi, byte param_lo, byte value_hi, byte value_lo) {
	switch_cfg(param_hi, param_lo, value_hi, value_lo);
}
void all_reset() {
	switch_reset();
}





////////////////////////////////////////////////////////////
// GET MESSAGES FROM MIDI INPUT
byte midi_in()
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
		
		P_LED = 1;
		g_led_timeout = LED_SHORT_BLINK;		
		
		// read the character out of buffer
		byte ch = rx_buffer[rx_tail];
		++rx_tail;
		rx_tail&=SZ_RXBUFFER_MASK;

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
					delay_ms(250); 
					delay_ms(250); 
					delay_ms(250); 
					delay_ms(250); 
					P_LED = 0; 
					storage_write_patch();
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
						case 0xE0: // pitch bend
						case 0xB0: // cc
						//case 0xD0: // channel pressure
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
// MAIN
void main()
{ 
	int i;
	
	// osc control / 16MHz / internal
	osccon = 0b01111010;

	apfcon0.7 = 1; // RX is on RA1
	apfcon0.2 = 1; // RX is on RA0
	apfcon1.0 = 1; // CCP2 is on RA5

	trisa = P_TRISA;
	trisc = P_TRISC;

	ansela = 0b00000000;
	anselc = 0b00000000;

	porta = 0;
	portc = 0;
	
	P_WPU = 1; // weak pull up on switch input
	option_reg.7 = 0; // weak pull up enable


	// we are alive...
	P_LED = 1; delay_ms(100); P_LED = 0; delay_ms(100);
	P_LED = 1; delay_ms(100); P_LED = 0; delay_ms(100);
	

	// initialise MIDI comms
	pwm_init();	
	// start up timer 1, which will be used to 
	// time the PWM of ports A-D
	t1con = 0b01000000;
		
	uart_init();
	timer_init();
	

	
	memset(pwm_duty, 0, sizeof(pwm_duty));

	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE	

t1con.0 = 1;
	switch_init();
	
	storage_read_patch();	
	
	switch_reset();

	// we are still alive...
	P_LED = 1; delay_ms(100); P_LED = 0; delay_ms(100);
	P_LED = 1; delay_ms(100); P_LED = 0; delay_ms(100);

	for(;;)
	{	
		// once per millisecond tick event
		if(ms_tick) {
			ms_tick = 0;
			
		
			switch_tick();
			
			// update LED
			if(g_led_timeout) {
				if(!--g_led_timeout) {
					P_LED = 0;
				}
			}
			
			if(!P_SWITCH) {
				all_reset();
				P_LED = 1;
				g_led_timeout = LED_LONG_BLINK;
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
		P_OUTA = pwm_duty[0] && !(tmr1h>pwm_duty[0]);
		P_OUTB = pwm_duty[1] && !(tmr1h>pwm_duty[1]);
		P_OUTC = pwm_duty[2] && !(tmr1h>pwm_duty[2]);
		P_OUTD = pwm_duty[3] && !(tmr1h>pwm_duty[3]);		
	}
}

//
// END
//