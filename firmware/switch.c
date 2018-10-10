//////////////////////////////////////////////////////////////
// SWITCH
#include <memory.h>
#include "msw.h"
// 
// MACRO DEFS
//

//
// TYPE DEFS
//

// enumeration of the different trigger conditions for a switch
enum {
	COND_NONE,			// no external trigger
	COND_NOTE,			// output is triggered by MIDI note
	COND_CC,			// output is triggered by MIDI CC
	COND_PGM			// output is triggered by MIDI program changes
};

enum {
	MOD_NONE,
	MOD_NOTE,
	MOD_CC
};

// parameters that define a note trigger condition
typedef struct {
	byte chan;			// midi channel
	byte note;			// note range minimum match
	byte min_vel;		// note velocity minimum match
	byte max_vel;		// note velocity maximum match
} NOTE_COND;

// parameters that define a cc trigger condition
typedef struct {
	byte chan;			// midi channel
	byte cc_no;			// cc number
	byte min_val;		// cc value minimum match
	byte max_val;		// cc value maximum match
} CC_COND;

typedef struct {
	byte chan;
	byte cc_no;
} CC_MOD;


// enumeration of states for a switch cycle
enum {
	STATE_READY,		// port is waiting for trigger condition
	STATE_HOLD,			// minimum trigge hold time
	STATE_SUSTAIN,		// waiting for trigger condition to end
};

//////////////////////////////////////////////////////////////
// switch configuration
typedef struct {
	byte cond_type;				// the type of external condition
	union {
		NOTE_COND note;
		CC_COND cc;
	} cond;						// parameters associated with trigger condition

	byte dur_mod_type;			// type of external modulation of duration
	union {
		CC_MOD cc;
	} dur_mod;

	byte pwm_mod_type;			// type of external modulation of PWM
	union {
		CC_MOD cc;
	} pwm_mod;

	byte initial_output;	// the output on reset 
	byte invert_output;		// whether the switch output is inverted
	byte sustain;			// sustain ON/OFF flag
	unsigned int hold_time;	// hold time in ms
} SWITCH_CONFIG;

//////////////////////////////////////////////////////////////
// current switch status
typedef struct {
	byte is_triggered;			// whether trigger condition exists
	byte state;					// switch cycle state
	byte cur_duty;				// the maximum duty cycle
	unsigned int cur_hold_time;	// hold time in ms
	unsigned int hold_timeout;	// milliseconds left in hold mode
} SWITCH_STATUS;

typedef struct {
	byte chan;
	byte pgm_no;
	byte mask;
} PGM_CONFIG;

//
// LOCAL DATA
//
static SWITCH_STATUS l_status[SWITCH_MAX];
static SWITCH_CONFIG l_cfg[SWITCH_MAX];
static PGM_CONFIG l_pgm[PGM_MAX];
//
// PRIVATE FUNCTIONS
//

//////////////////////////////////////////////////////////////
// call when trigger condition begins
static void trigger(byte which) {

	SWITCH_STATUS *pstatus = &l_status[which];	
	SWITCH_CONFIG *pcfg = &l_cfg[which];	
	byte duty = pstatus->cur_duty;
	// are we responding to program change?
	if(pcfg->cond_type == COND_PGM) {
		// immediately go to sustain mode. Hold time
		// is not applicable
		pstatus->hold_timeout = 0;
		pstatus->state = STATE_SUSTAIN;
	}
	// is a hold time defined?
	else if(pstatus->cur_hold_time) {
		// start the hold phase
		pstatus->state = STATE_HOLD;
		pstatus->hold_timeout = pstatus->cur_hold_time;
	}
	// is sustain defined?
	else if(pcfg->sustain) {
		// start the sustain phase
		pstatus->state = STATE_SUSTAIN;
	}
	else {
		// no trigger at all 
		duty = 0;
	}
	pwm_set(which, duty, pcfg->invert_output);	
	pstatus->is_triggered = 1;
	
}		


//////////////////////////////////////////////////////////////
static void untrigger(byte which) {

	SWITCH_STATUS *pstatus = &l_status[which];	
	SWITCH_CONFIG *pcfg = &l_cfg[which];	
	
	if(pstatus->state == STATE_SUSTAIN) {
		pwm_set(which, 0, pcfg->invert_output);	
		pstatus->state = STATE_READY;
	}
	pstatus->is_triggered = 0;
	
}		



//
// PUBLIC FUNCTIONS
//

//////////////////////////////////////////////////////////////
// initialise the data in the switch module
void switch_init() {
	for(int which = 0; which < SWITCH_MAX; ++which) {
		memset(&l_status[which], 0, sizeof(SWITCH_STATUS));
		memset(&l_cfg[which], 0, sizeof(SWITCH_CONFIG));
	}
	for(int which = 0; which < PGM_MAX; ++which) {
		memset(&l_pgm[which], 0, sizeof(PGM_CONFIG));
	}	
}

//////////////////////////////////////////////////////////////
// reset all outputs to their initial state
void switch_reset() {
	for(int which = 0; which < SWITCH_MAX; ++which) {
		SWITCH_STATUS *pstatus = &l_status[which];	
		SWITCH_CONFIG *pcfg = &l_cfg[which];	
	
		pstatus->is_triggered = 0;
		pstatus->state = STATE_READY;
		pstatus->cur_duty = 0xFF;
		pwm_set(which,0,pcfg->initial_output);
		if(pcfg->initial_output) {
			trigger(which);
		}
	}
}

//////////////////////////////////////////////////////////////
// once per ms servicing of switches
void switch_tick() {
	// loop through all the switches
	for(int which = 0; which < SWITCH_MAX; ++which) {
		SWITCH_STATUS *pstatus = &l_status[which];	
		SWITCH_CONFIG *pcfg = &l_cfg[which];	
		switch(pstatus->state) {	
			case STATE_READY:
			case STATE_SUSTAIN:
				// nothing to do in these staes
				break;
			case STATE_HOLD:
				// decrement the timeout
				if(pstatus->hold_timeout) {
					--pstatus->hold_timeout;
				}
				if(!pstatus->hold_timeout) {
					// hold has timed out
					if(pcfg->sustain && pstatus->is_triggered) {
						// need to sustain until end of trigger condition
						pstatus->state = STATE_SUSTAIN;
					}
					else {
						// automatic untriggering at the end 
						// of the hold timeout period
						pstatus->state = STATE_READY;
						pwm_set(which,0,pcfg->invert_output);
					}
				}
				break;			
		}
	}
}

//////////////////////////////////////////////////////////////
// handle incoming MIDI note
void switch_on_note(byte chan, byte note, byte vel) {

	// cycle through each of the outputs
	for(int which = 0; which < SWITCH_MAX; ++which) {
		SWITCH_CONFIG *pcfg = &l_cfg[which];		
		SWITCH_STATUS *pstatus = &l_status[which];		
				
		// check for matching note
		if( COND_NOTE == pcfg->cond_type &&
			chan == pcfg->cond.note.chan && 
			note == pcfg->cond.note.note ) {

			// is this a note off?
			if(!vel) {
				// MIDI note OFF
				untrigger(which);
			}
			// is this a note on with matching velocity?
			else if( vel >= pcfg->cond.note.min_vel &&
				(vel <= pcfg->cond.note.max_vel)) {
				
				// modulate the duration if needed			
				if(pcfg->dur_mod_type == MOD_NOTE) {					
					pstatus->cur_hold_time = ((long)pcfg->hold_time * vel)/127;
				}
				else if(pcfg->dur_mod_type == MOD_NONE) {
					pstatus->cur_hold_time = pcfg->hold_time;
				}

				// modulate the duty if needed			
				if(pcfg->pwm_mod_type == MOD_NOTE) {
					pstatus->cur_duty = 2 * vel;
				}
				else if(pcfg->pwm_mod_type == MOD_NONE) {
					pstatus->cur_duty = 0xFF;
				}
				
				// MIDI note ON
				trigger(which);
			}
		}
	}
}

//////////////////////////////////////////////////////////////
// handle incoming MIDI CC
void switch_on_cc(byte chan, byte cc_no, byte value) {

	// cycle through each of the outputs
	for(int which = 0; which < SWITCH_MAX; ++which) {
		SWITCH_CONFIG *pcfg = &l_cfg[which];		
		SWITCH_STATUS *pstatus = &l_status[which];		

		// handle duration modulation via CC
		if( MOD_CC == pcfg->dur_mod_type && 
			cc_no == pcfg->dur_mod.cc.cc_no	&& 
			chan == pcfg->dur_mod.cc.chan ) {						
			pstatus->cur_hold_time = ((long)pcfg->hold_time * value)/127;			
		}

		// handle PWM duty modulation via CC
		if( MOD_CC == pcfg->pwm_mod_type && 
			cc_no == pcfg->pwm_mod.cc.cc_no	&& 
			chan == pcfg->pwm_mod.cc.chan ) {						
			pstatus->cur_duty = 2 * value;
			if(!pstatus->is_triggered) {
				// change duty while note is triggered
				pwm_set(which, pstatus->cur_duty, pcfg->invert_output);
			}
		}

		// handle trigger via CC
		if( COND_CC == pcfg->cond_type && 
			chan == pcfg->cond.cc.chan &&
			cc_no == pcfg->cond.cc.cc_no) { 			
			if( value >= pcfg->cond.cc.min_val &&
				value <= pcfg->cond.cc.max_val ) {
				if(!pstatus->is_triggered) {
					// avoid multiple triggering when CC is changing
					trigger(which);
				}
			}
			else {
				if(pstatus->is_triggered) {
					untrigger(which);
				}
			}
		}
	}
}

//////////////////////////////////////////////////////////////
// Handle incoming program change
void switch_on_pgm(byte chan, byte pgm_no) {

	byte is_found = 0;
	byte mask = 0;
	for(int which = 0; which < PGM_MAX; ++which) {
		if(l_pgm[which].chan == chan && l_pgm[which].pgm_no == pgm_no) {
			mask = l_pgm[which].mask;
			is_found = 1;
			break;
		}
	}
	
	// cycle through each of the outputs
	byte b = 1;	
	for(int which = 0; which < SWITCH_MAX; ++which) {
		SWITCH_CONFIG *pcfg = &l_cfg[which];		
				
		if(pcfg->cond_type != COND_PGM) {
			continue;
		}
		
		if(b & mask) {
			trigger(which);
		}
		else {
			untrigger(which);
		}
		b<<=1;
	}
}

//////////////////////////////////////////////////////////////
static byte switch_cfg_port(byte which, byte param_lo, byte value_hi, byte value_lo) {
	SWITCH_CONFIG *pcfg = &l_cfg[which];		
	switch(param_lo) {
	
	/////////////////////////////////////////
	case PARAML_TRIG_NONE:
		pcfg->cond_type = COND_NONE;
		return 1;

	/////////////////////////////////////////
	case PARAML_TRIG_NOTE:
		if(value_hi < 1 || value_hi > 16) {
			return 0;
		}
		pcfg->cond_type = COND_NOTE;
		pcfg->cond.note.chan = value_hi - 1;
		pcfg->cond.note.note = value_lo;
		pcfg->cond.note.min_vel = 0;
		pcfg->cond.note.max_vel = 127;
		return 1;
	/////////////////////////////////////////
	case PARAML_TRIG_CC:
		if(value_hi < 1 || value_hi > 16) {
			return 0;
		}
		pcfg->cond_type = COND_CC;
		pcfg->cond.cc.chan = value_hi - 1;
		pcfg->cond.cc.cc_no = value_lo;
		pcfg->cond.cc.min_val = 1;		// default is 0=off, >0=on
		pcfg->cond.cc.max_val = 127;
		return 1;
	/////////////////////////////////////////
	case PARAML_TRIG_PGM:
		pcfg->cond_type = COND_PGM;
		return 1;
	/////////////////////////////////////////
	case PARAML_TRIG2_VELOCITY:
		if(value_lo > value_hi || pcfg->cond_type != COND_NOTE) {
			return 0;
		}
		pcfg->cond.note.min_vel = value_lo;
		pcfg->cond.note.max_vel = value_hi;
		return 1;
	/////////////////////////////////////////
	case PARAML_TRIG2_VALUE:
		if(value_lo > value_hi || pcfg->cond_type == COND_CC) {
			return 0;
		}
		pcfg->cond.cc.min_val = value_lo;
		pcfg->cond.cc.max_val = value_hi;
		return 1;
	/////////////////////////////////////////
	case PARAML_ENV_SUSTAIN:
		pcfg->sustain = !!value_lo;
		return 1;
	/////////////////////////////////////////
	case PARAML_ENV_HOLD:	
		pcfg->hold_time = (unsigned int)value_hi<<7|value_lo;
		return 1;
	/////////////////////////////////////////
	case PARAML_DUR_MOD_NONE:
		pcfg->dur_mod_type = MOD_NONE;
		return 1;
	/////////////////////////////////////////
	case PARAML_DUR_MOD_VELOCITY:
		if(pcfg->cond_type != COND_NOTE) {
			return 0;
		}
		pcfg->dur_mod_type = MOD_NOTE;
		return 1;	
	/////////////////////////////////////////
	case PARAML_DUR_MOD_CC:	
		if(value_hi < 1 || value_hi > 16) {
			return 0;
		}
		pcfg->dur_mod_type = MOD_CC;
		pcfg->dur_mod.cc.chan = value_hi-1;
		pcfg->dur_mod.cc.cc_no = value_lo;
		return 1;	
	/////////////////////////////////////////
	case PARAML_PWM_MOD_NONE:
		pcfg->pwm_mod_type = MOD_NONE;
		return 1;
	/////////////////////////////////////////
	case PARAML_PWM_MOD_VELOCITY:
		if(pcfg->cond_type != COND_NOTE) {
			return 0;
		}
		pcfg->pwm_mod_type = MOD_NOTE;
		return 1;	
	/////////////////////////////////////////
	case PARAML_PWM_MOD_CC:	
		if(value_hi < 1 || value_hi > 16) {
			return 0;
		}
		pcfg->pwm_mod_type = MOD_CC;
		pcfg->pwm_mod.cc.chan = value_hi-1;
		pcfg->pwm_mod.cc.cc_no = value_lo;
		return 1;	
	}	
}
//////////////////////////////////////////////////////////////
static byte switch_cfg_pgm(byte which, byte param_lo, byte value_hi, byte value_lo) {
	PGM_CONFIG *pcfg = &l_pgm[which];
	switch(param_lo) {
		case PARAML_PGM_CLEAR:
			pcfg->chan = 0xFF;
			return 1;
		case PARAML_PGM_MATCH: 
			// set the match for the channel and pgm
			// value_hi = channel(1-16)
			// value_lo = pgm (0-127)
			if(value_hi < 1 || value_hi > 16 || value_lo > 127) {
				return 0;
			}
			pcfg->chan = value_hi - 1;
			pcfg->pgm_no = value_lo;
			pcfg->mask = 0;
			return 1;
		case PARAML_PGM_PORT:
			if(value_hi < 1 || value_hi > 7) {
				return 0;
			}
			if(value_lo) {
				pcfg->mask |= (1<<value_hi);
			}
			else {
				pcfg->mask &= ~(1<<value_hi);
			}
			return 1;
	}	
	return 0;
}

//////////////////////////////////////////////////////////////
byte switch_cfg(byte param_hi, byte param_lo, byte value_hi, byte value_lo) {
	if(param_hi >= PARAMH_PORTA && param_hi <= PARAMH_PORTH) {
		return switch_cfg_port(param_hi - PARAMH_PORTA, param_lo, value_hi, value_lo);
	}
	else if(param_hi >= PARAMH_PGM_SLOT1 && param_hi <= PARAMH_PGM_SLOT16) {
		return switch_cfg_pgm(param_hi - PARAMH_PGM_SLOT1, param_lo, value_hi, value_lo);
	}
	else if(param_hi == PARAMH_PGM_ALL) {
		for(int i=0; i<PGM_MAX; ++i) {
			if(!switch_cfg_pgm(i, param_lo, value_hi, value_lo)) {
				return 0;
			}
		}
		return 1;
	}
}

//////////////////////////////////////////////////////////////
byte *switch_storage(int *len) {
	*len = sizeof(l_cfg);
	return (byte*)&l_cfg;
}

//////////////////////////////////////////////////////////////
byte *switch_pgm_storage(int *len) {
	*len = sizeof(l_pgm);
	return (byte*)&l_pgm;
}
