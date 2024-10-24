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
// OUTPUT SWITCHER STATE MACHINE
//
////////////////////////////////////////////////////////////

//
// INCLUDE FILES
//
#include <memory.h>
#include "msw.h"

//
// TYPE DEFS
//

// macro for convenience redirecting default midi channel
#define CHAN_DEFAULT 0xFF
#define IS_CHAN_MATCH(chan, me, def) (((me)==CHAN_DEFAULT)?((chan)==(def)):((chan)==(me)))

// enumeration of the different trigger conditions for a switch
enum {
	COND_NONE	= PARAML_TRIG_NONE,			// no external trigger
	COND_NOTE	= PARAML_TRIG_NOTE,			// output is triggered by MIDI note
	COND_CC		= PARAML_TRIG_CC,			// output is triggered by MIDI CC
	COND_PGM	= PARAML_TRIG_PGM,			// output is triggered by MIDI program changes
	COND_ALWAYS = PARAML_TRIG_ALWAYS		// output is triggered at startup
};

// enumeration of different envelope types
enum {
	ENV_SUSTAIN		= 0,
	ENV_HOLD		= 1,
	ENV_HOLD_SUST	= 2,
	ENV_SUST_HOLD	= 3,
	ENV_RELEASE		= 4,
	ENV_SUST_REL	= 5,
	ENV_LATCH		= 6
};

// enumeration of modulation destinations for velocity and CC
enum {
	MOD_DEST_NONE	= 0,
	MOD_DEST_TIME	= 1,
	MOD_DEST_DUTY	= 2
};

// enumeration of flag bits
enum {
	SWF_USE_DEFAULTS 	= 0x01,
	SWF_GAMMA_CORRECT 	= 0x02,
	SWF_INVERT 			= 0x04
};

// enumeration of states for a switch cycle
enum {
	STATE_READY,		// port is waiting for trigger condition
	STATE_SUSTAIN,		// waiting for trigger condition to end
	STATE_HOLD,			// waiting for hold time to expire
	STATE_RELEASE		// waiting for hold time to expire and scaling duty down
};

//////////////////////////////////////////////////////////////
// STRUCTURE TO HOLD DEFAULT CONFIGUATION
typedef struct {
	byte trig_chan;				// global default trigger channel
	byte pgm_chan;				// global PGM change message channel
	byte env_type;				// global default envelope type
	unsigned int hold_time;		// global default hold time in ms		
	byte vel_mod_dest;			// global default modulation by note velocity
	byte cc_mod_dest;			// global default modulation by cc
	byte cc_mod_chan;			// global default channel for modulating cc
	byte cc_mod_cc;				// global default modulating cc number
	byte flags;					// global default flags
} DEFAULT_CONFIG;

//////////////////////////////////////////////////////////////
// STRUCTURE TO HOLD SWITCH CONFIGUATION
typedef struct {
	byte cond_type;				// the type of external condition
	byte trig_chan;				// trigger channel
	byte trig_match_min;		// trigger note or cc - min
	byte trig_match_max;		// trigger note or cc - max
	byte value_min;				// range of velocity or cc value - min
	byte value_max;				// range of velocity or cc value - max	
	byte env_type;				// envelope type
	unsigned int hold_time;		// hold time in ms		
	byte vel_mod_dest;			// modulation by note velocity
	byte cc_mod_dest;			// modulation by cc
	byte cc_mod_chan;			// channel for modulating cc
	byte cc_mod_cc;				// modulating cc number
	byte flags;					// flags
} SWITCH_CONFIG;

//////////////////////////////////////////////////////////////
// STRUCTURE TO HOLD SWITCH STATUS
typedef struct {
	byte is_triggered;					// whether trigger condition exists now
	byte trig_note;						// the note which caused the trigger
	byte is_cc_in_range;				// whether CC value is within trigger range
	byte state;							// switch cycle state
	byte cur_duty;						// modulated duty cycle
	unsigned int cur_hold_time;			// modulated hold time in ms
	unsigned int init_hold_timeout;		// the hold_time at start of release state
	unsigned int timeout;			// milliseconds left in hold mode
} SWITCH_STATUS;

//////////////////////////////////////////////////////////////
// STRUCTURE TO HOLD PROGRAM CHANGE SLOT CONFIGURATION
typedef struct {
	byte pgm_no;				// program number to match
	byte trigger_mask;			// bit mask of channels to switch
} PGM_CONFIG;

//
// LOCAL DATA
//
static SWITCH_CONFIG l_cfg0;
static SWITCH_CONFIG l_cfg1;
static SWITCH_CONFIG l_cfg2;
static SWITCH_CONFIG l_cfg3;
static SWITCH_CONFIG l_cfg4;
static SWITCH_CONFIG l_cfg5;
static SWITCH_CONFIG l_cfg6;
static SWITCH_CONFIG l_cfg7;
static SWITCH_CONFIG *l_cfg[SWITCH_MAX];	// indirection needed to prevent address truncation :|
static DEFAULT_CONFIG l_default_cfg;		// patch info - default config
static SWITCH_STATUS l_status[SWITCH_MAX];	// current switch status
static PGM_CONFIG l_pgm[PGM_MAX];


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//
// PRIVATE FUNCTIONS
//
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// SET SPECIAL RESET STATE PER OUTPUT
static void on_reset(byte which) {
#ifdef CUSTOM_SWITCHING_01
	switch(which) {
	case 0: 
	case 1: 
		l_cfg[which]->env_type = ENV_LATCH;
		break;
	case 2:
		l_cfg[which]->env_type = ENV_SUSTAIN;
		break;
	}
#endif	
}

//////////////////////////////////////////////////////////////
// SET A SWITCHER OUTPUT
static void set_output(byte which, byte duty, byte flags) {
#ifdef CUSTOM_SWITCHING_01
	if(which <= 2) {
		byte a_status = !l_status[0].is_triggered;
		byte b_status = l_status[0].is_triggered;
		byte c_status = l_status[2].is_triggered;		
		if(l_status[2].is_triggered) {
			a_status = !a_status;
			b_status = !b_status;
		}	
		else if(l_status[1].is_triggered) {
			a_status = 1;
			b_status = 1;
		}
		pwm_set(0, a_status? PWM_FULL : 0, 0);	
		pwm_set(1, b_status? PWM_FULL : 0, 0);
		pwm_set(2, c_status? PWM_FULL : 0, 0);
		return;
	}
#endif	
	if(flags & SWF_INVERT) {
		pwm_set(which, 255-duty, flags & SWF_GAMMA_CORRECT);	
	}
	else {	
		pwm_set(which, duty, flags & SWF_GAMMA_CORRECT);	
	}
}

//////////////////////////////////////////////////////////////
// START OF TRIGGER CONDITION
static void trigger(byte which) {

	SWITCH_STATUS *pstatus = &l_status[which];	
	SWITCH_CONFIG *pcfg = l_cfg[which];	
		
	switch(pcfg->env_type) {
		case ENV_LATCH:
			if(pstatus->is_triggered) { // latched on?
				// turn output off and go back to ready state
				pstatus->state = STATE_READY;
				pstatus->is_triggered = 0;
				set_output(which, 0, pcfg->flags);
				return;
			}
			// otherwise treat as sustain 
		case ENV_SUSTAIN:
		case ENV_SUST_HOLD:
		case ENV_SUST_REL:
			// go to the sustain state
			pstatus->state = STATE_SUSTAIN;
			break;	
		case ENV_RELEASE:
			// start a release curve
			pstatus->timeout = pstatus->cur_hold_time;
			pstatus->init_hold_timeout = pstatus->cur_hold_time;
			pstatus->state = STATE_RELEASE;
			break;
		case ENV_HOLD:
		case ENV_HOLD_SUST:
		default:
			// start a hold 
			pstatus->timeout = pstatus->cur_hold_time;
			pstatus->state = STATE_HOLD;
			break;
	}
		
	// trigger is active
	pstatus->is_triggered = 1;		
	
	// set output ON using modulated duty
	set_output(which, pstatus->cur_duty, pcfg->flags);	
}		

//////////////////////////////////////////////////////////////
// END OF TRIGGER CONDITION
static void untrigger(byte which) {

	SWITCH_STATUS *pstatus = &l_status[which];	
	SWITCH_CONFIG *pcfg = l_cfg[which];	

	// sustain state
	if(STATE_SUSTAIN == pstatus->state) {	
		switch(pcfg->env_type) {
		case ENV_SUST_HOLD:
			// sustain is over - go to hold state
			pstatus->timeout = pstatus->cur_hold_time;
			pstatus->state = STATE_HOLD;
			pstatus->is_triggered = 0;			
			break;	
		case ENV_SUST_REL:
			// sustain is over - go to release state
			pstatus->timeout = pstatus->cur_hold_time;
			pstatus->init_hold_timeout = pstatus->cur_hold_time;
			pstatus->is_triggered = 0;			
			pstatus->state = STATE_RELEASE;
			break;	
		case ENV_LATCH:
			// need another trigger condition to unlatch
			// so nothing to do at this point
			break;
		default:
			// turn output off and go back to ready state
			pstatus->is_triggered = 0;
			pstatus->state = STATE_READY;
			set_output(which, 0, pcfg->flags);			
			break;
		}	
	}			
}		

//////////////////////////////////////////////////////////////
// SET UP OUTPUT FOR A PGM CHANGE MASK
static void trigger_pgm(byte mask) {

	byte b = 0x01;
	// cycle through each of the outputs, triggering outputs
	// that are PGM condition and which match the mask
	for(int i = 0; i < SWITCH_MAX; ++i) {
		if(l_cfg[i]->cond_type == COND_PGM) {
			if(mask & b) {
				trigger(i);
			}
			else {
				untrigger(i);
			}
		}
		b<<=1;
	}
}

//////////////////////////////////////////////////////////////
// HANDLE CONFIG PARAM FOR DEFAULTS
static void switch_cfg_default(byte param_lo, byte value_hi, byte value_lo) {
	switch(param_lo) {
	case PARAML_TRIG2_CHAN:
		l_default_cfg.trig_chan = value_lo - 1;
		break;
	case PARAML_PGM_CHAN:
		l_default_cfg.pgm_chan = value_lo - 1;
		break;
	case PARAML_ENV_TYPE:
		l_default_cfg.env_type = value_lo;
		break;
	case PARAML_ENV_HOLD:
		l_default_cfg.hold_time = (unsigned int)value_hi<<7|value_lo;
		break;
	case PARAML_VEL_MOD_DEST:
		l_default_cfg.vel_mod_dest = value_lo;
		break;
	case PARAML_CC_MOD_DEST:
		l_default_cfg.cc_mod_dest = value_lo;
		break;
	case PARAML_CC_MOD_CHAN:
		l_default_cfg.cc_mod_chan = (value_lo ? (value_lo-1) : CHAN_DEFAULT);
		break;
	case PARAML_CC_MOD_CC:
		l_default_cfg.cc_mod_cc = value_lo;
		break;
	case PARAML_FLAGS:
		l_default_cfg.flags &= SWF_USE_DEFAULTS;
		l_default_cfg.flags |= value_lo;
		break;
	}
}

//////////////////////////////////////////////////////////////
// HANDLE CONFIG PARAM FOR SWITCH PORT
static void switch_cfg_port(byte which, byte param_lo, byte value_hi, byte value_lo) {
	SWITCH_CONFIG *pcfg = l_cfg[which];		
	switch(param_lo) {
	case PARAML_TRIG_NONE:
	case PARAML_TRIG_PGM:
	case PARAML_TRIG_ALWAYS:
		pcfg->cond_type = param_lo;
		break;	
	case PARAML_TRIG_NOTE:
	case PARAML_TRIG_CC:
		pcfg->cond_type = param_lo;
		pcfg->trig_match_min = value_lo;
		pcfg->trig_match_max = value_lo;
		pcfg->value_min = 1;
		pcfg->value_max = 127;
		break;	
	case PARAML_TRIG_NOTE_RANGE:
		pcfg->cond_type = PARAML_TRIG_NOTE;
		pcfg->trig_match_min = value_lo;
		pcfg->trig_match_max = value_hi;
		pcfg->value_min = 1;
		pcfg->value_max = 127;
		break;	
	case PARAML_TRIG2_CHAN:
		pcfg->trig_chan = (value_lo ? (value_lo-1) : CHAN_DEFAULT);
		break;
	case PARAML_TRIG2_VALUE_MIN:
		pcfg->value_min = value_lo;
		pcfg->value_max = value_lo;
		break;
	case PARAML_TRIG2_VALUE_MAX:
		pcfg->value_max = value_lo;
		break;		
	case PARAML_ENV_SET: 
		if(value_lo) {
			pcfg->flags &= ~SWF_USE_DEFAULTS;
		}
		else {
			pcfg->flags |= SWF_USE_DEFAULTS;
		}
		break;
	case PARAML_ENV_TYPE:
		pcfg->env_type = value_lo;
		break;
	case PARAML_ENV_HOLD:
		pcfg->hold_time = (unsigned int)value_hi<<7|value_lo;
		break;
	case PARAML_VEL_MOD_DEST:
		pcfg->vel_mod_dest = value_lo;
		break;
	case PARAML_CC_MOD_DEST:
		pcfg->cc_mod_dest = value_lo;
		break;
	case PARAML_CC_MOD_CHAN:
		pcfg->cc_mod_chan = (value_lo ? (value_lo-1) : CHAN_DEFAULT);
		break;
	case PARAML_CC_MOD_CC:
		pcfg->cc_mod_cc = value_lo;
		break;
	case PARAML_FLAGS:
		pcfg->flags &= SWF_USE_DEFAULTS;
		pcfg->flags |= value_lo;
		break;
	}
}

//////////////////////////////////////////////////////////////
// HANDLE CONFIG PARAM FOR PROGRAM CHANGE
static void  switch_cfg_pgm(byte which, byte param_lo, byte value_hi, byte value_lo) {
	PGM_CONFIG *pcfg = &l_pgm[which];
	switch(param_lo) {
		case PARAML_PGM_MATCH: 
			if(value_hi) {
				pcfg->pgm_no = value_lo; 
			}
			else {
				pcfg->pgm_no = 0xFF; // disable the slot
			}
			pcfg->trigger_mask = 0;
			break;
		case PARAML_PGM_PORT:					
			if(value_lo) {
				pcfg->trigger_mask |= (1<<(value_hi-1));				
			}
			else {
				pcfg->trigger_mask &= ~(1<<(value_hi-1));
			}
			break;
	}	
}

//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//
// PUBLIC FUNCTIONS
//
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////
// SET ALL LOCAL DATA TO INITIAL STATE
void switch_init() {

	l_cfg[0] = &l_cfg0;
	l_cfg[1] = &l_cfg1;
	l_cfg[2] = &l_cfg2;
	l_cfg[3] = &l_cfg3;
	l_cfg[4] = &l_cfg4;
	l_cfg[5] = &l_cfg5;
	l_cfg[6] = &l_cfg6;
	l_cfg[7] = &l_cfg7;

	memset(&l_default_cfg, 0, sizeof(DEFAULT_CONFIG));
	for(int which = 0; which < SWITCH_MAX; ++which) {
		memset(&l_status[which], 0, sizeof(SWITCH_STATUS));
		memset(l_cfg[which], 0, sizeof(SWITCH_CONFIG));
	}
	for(int which = 0; which < PGM_MAX; ++which) {
		memset(&l_pgm[which], 0, sizeof(PGM_CONFIG));
		l_pgm[which].pgm_no = 0xFF; // disable slot by default
	}	
}

//////////////////////////////////////////////////////////////
// RESET ALL OUTPUTS TO INITIAL STATE
void switch_reset() {
	for(int which = 0; which < SWITCH_MAX; ++which) {
		SWITCH_STATUS *pstatus = &l_status[which];	
		SWITCH_CONFIG *pcfg = l_cfg[which];	

		// copy default values against any output that is 
		// configured to use default pulse shaping
		if(pcfg->flags & SWF_USE_DEFAULTS) {
			pcfg->env_type 		= l_default_cfg.env_type;
			pcfg->hold_time 	= l_default_cfg.hold_time;
			pcfg->vel_mod_dest 	= l_default_cfg.vel_mod_dest;
			pcfg->cc_mod_dest 	= l_default_cfg.cc_mod_dest;
			pcfg->cc_mod_chan 	= l_default_cfg.cc_mod_chan;
			pcfg->cc_mod_cc 	= l_default_cfg.cc_mod_cc;
			pcfg->flags 	    = l_default_cfg.flags | SWF_USE_DEFAULTS;
		}
		
		// not triggered
		pstatus->trig_note = 0xff;
		pstatus->is_cc_in_range = 0;
		pstatus->is_triggered = 0;
		pstatus->state = STATE_READY;
		
		// clear modulation of hold time and duty
		pstatus->cur_hold_time = pcfg->hold_time;
		pstatus->cur_duty = 0xFF;

		// call reset hook allowing special reset states 
		on_reset(which);

		if(pcfg->cond_type == COND_ALWAYS) {
			// trigger for "always" mode
			trigger(which);
		}
		else {
			// or turn the output off
			set_output(which, 0, pcfg->flags);						
		}		
	}
	
	// set default program change status
	trigger_pgm(l_pgm[0].trigger_mask);			
	
}

//////////////////////////////////////////////////////////////
// SERVICE SWITCHES ONCE PER MILLISECOND
void switch_tick() {
		
	// loop through all the switches
	for(int which = 0; which < SWITCH_MAX; ++which) {
		SWITCH_STATUS *pstatus = &l_status[which];	
		SWITCH_CONFIG *pcfg = l_cfg[which];	
		switch(pstatus->state) {	
		
		
			case STATE_READY:
			case STATE_SUSTAIN:
				// nothing to do in these staes
				break;
			case STATE_HOLD:
			case STATE_RELEASE:
				// decrement the timeout
				if(pstatus->timeout) {
					--pstatus->timeout;
				}
				if(!pstatus->timeout) {
					// hold has timed out					
					if(pstatus->is_triggered && pcfg->env_type == ENV_HOLD_SUST) {
						// need to sustain until end of trigger condition
						pstatus->state = STATE_SUSTAIN;
					}
					else {
						// automatic untriggering at the end 
						// of the hold timeout period
						pstatus->state = STATE_READY;
						set_output(which, 0, pcfg->flags);			
					}
				}
				else if(pstatus->state == STATE_RELEASE) {
					byte duty = (((long)pstatus->cur_duty * pstatus->timeout)/pstatus->init_hold_timeout);					
					set_output(which, duty, pcfg->flags);			
				}
				break;			
		}
	}
}

//////////////////////////////////////////////////////////////
// HANDLE SWITCHING LOGIC BASED ON MIDI NOTE
void switch_on_note(byte chan, byte note, byte vel) {

	// cycle through each of the outputs
	for(int which = 0; which < SWITCH_MAX; ++which) {
		SWITCH_CONFIG *pcfg = l_cfg[which];		
		SWITCH_STATUS *pstatus = &l_status[which];		
								
		// Check if this note should trigger this output channel... 
		// First see if MIDI channel, note and mode are a match
		if( COND_NOTE == pcfg->cond_type && 
			IS_CHAN_MATCH(chan, pcfg->trig_chan, l_default_cfg.trig_chan) &&
			(note >= pcfg->trig_match_min && note <= pcfg->trig_match_max))
		{
		
			// no check if velocity filter is passed
			if(vel >= pcfg->value_min && vel <= pcfg->value_max) {
			
				// perform modulation based on velocity, as needed
				switch(pcfg->vel_mod_dest) {
				case MOD_DEST_TIME:
					pstatus->cur_hold_time = ((long)pcfg->hold_time * vel)/127;			
					break;
				case MOD_DEST_DUTY:
					pstatus->cur_duty = 2 * vel;
					if(pstatus->is_triggered) {
						set_output(which, pstatus->cur_duty, pcfg->flags);
					}
					break;
				}

				// MIDI note ON
				pstatus->trig_note = note;
				trigger(which);
			}
			else if(pstatus->trig_note == note) {
				// untrigger output if currently triggered
				if(pstatus->is_triggered) {
					untrigger(which);
				}
				pstatus->trig_note = 0;
			}
		}
	}
}

//////////////////////////////////////////////////////////////
// HANDLE SWITCHING LOGIC BASED ON MIDI CC
void switch_on_cc(byte chan, byte cc_no, byte value) {

	// cycle through each of the outputs
	for(int which = 0; which < SWITCH_MAX; ++which) {
		SWITCH_CONFIG *pcfg = l_cfg[which];		
		SWITCH_STATUS *pstatus = &l_status[which];		

		// if there no triggering on this output then nothing to do
		if(pcfg->cond_type == COND_NONE) {
			continue;
		}

		// find out whether this CC should modulate the pulse
		// shaping on this switcher output
		if((pcfg->cc_mod_cc == cc_no) && 
			IS_CHAN_MATCH(chan, pcfg->cc_mod_chan, l_default_cfg.trig_chan)) {
			switch(pcfg->cc_mod_dest) {
				case MOD_DEST_TIME:
					// modulates hold time
					pstatus->cur_hold_time = ((long)pcfg->hold_time * value)/127;			
					break;
				case MOD_DEST_DUTY:
					// modulates duty
					if(value > 64) {
						pstatus->cur_duty = 1 + 2 * value; // ensure we can reach 255
					}
					else {
						pstatus->cur_duty = 2 * value; 
					}
					
					if(pstatus->is_triggered) {
						// if the output is in triggered status then modulate the duty "live"
						set_output(which, pstatus->cur_duty, pcfg->flags);
					}
					break;
			}
		}
		
		// handle trigger via CC
		if( COND_CC == pcfg->cond_type && cc_no == pcfg->trig_match_min &&
			IS_CHAN_MATCH(chan, pcfg->trig_chan, l_default_cfg.trig_chan)) { 			

			// check if the value is within threshold
			if( value >= pcfg->value_min && value <= pcfg->value_max) {
				if(!pstatus->is_cc_in_range) {
					pstatus->is_cc_in_range = 1;
					trigger(which);
				}
			}
			else {
				if(pstatus->is_cc_in_range) {
					pstatus->is_cc_in_range = 0;
					untrigger(which);
				}
			}
		}
	}
}

//////////////////////////////////////////////////////////////
// HANDLE SWITCHING LOGIC BASED ON PROGRAM CHANGE
void switch_on_pgm(byte chan, byte pgm_no) {

	if(chan == l_default_cfg.pgm_chan) {
		byte is_found = 0;
		for(int which = 1; which < PGM_MAX; ++which) {
			if(l_pgm[which].pgm_no == pgm_no) {
				trigger_pgm(l_pgm[which].trigger_mask);
				return;
			}
		}
	}
}

//////////////////////////////////////////////////////////////
// HANDLE CONFIG PARAM FOR PROGRAM CHANGE
void switch_cfg(byte param_hi, byte param_lo, byte value_hi, byte value_lo) {
	if(param_hi == PARAMH_PORT_DEFAULT) {
		switch_cfg_default(param_lo, value_hi, value_lo);
	}
	else if(param_hi >= PARAMH_PORTA && param_hi <= PARAMH_PORTH) {
		switch_cfg_port(param_hi - PARAMH_PORTA, param_lo, value_hi, value_lo);
	}
	else if(param_hi >= PARAMH_PGM_SLOT_BASE && param_hi <= PARAMH_PGM_SLOT_MAX) {
		switch_cfg_pgm(param_hi - PARAMH_PGM_SLOT_BASE, param_lo, value_hi, value_lo);
	}
}

//////////////////////////////////////////////////////////////
// RETURN MEMORY MAP FOR DEFAULT CONFIG
byte *switch_default_storage(int *len) {
	*len = sizeof(l_default_cfg);
	return (byte*)&l_default_cfg;
}

//////////////////////////////////////////////////////////////
// RETURN MEMORY MAP FOR SWITCH CONFIG
byte *switch_storage(int which, int *len) {
	*len = sizeof(SWITCH_CONFIG);
	return (byte*)&l_cfg[which];
}

//////////////////////////////////////////////////////////////
// RETURN MEMORY MAP FOR PGM CONFIG
byte *switch_pgm_storage(int *len) {
	*len = sizeof(l_pgm);
	return (byte*)&l_pgm;
}

//////////////////////////////////////////////////////////////
// SAVE PATCH INFO
void switch_storage_write(int* addr) {
	storage_write((byte*)&l_default_cfg, sizeof(l_default_cfg), addr);
	storage_write((byte*)&l_cfg0, sizeof(l_cfg0), addr);
	storage_write((byte*)&l_cfg1, sizeof(l_cfg1), addr);
	storage_write((byte*)&l_cfg2, sizeof(l_cfg2), addr);
	storage_write((byte*)&l_cfg3, sizeof(l_cfg3), addr);
	storage_write((byte*)&l_cfg4, sizeof(l_cfg4), addr);
	storage_write((byte*)&l_cfg5, sizeof(l_cfg5), addr);
	storage_write((byte*)&l_cfg6, sizeof(l_cfg6), addr);
	storage_write((byte*)&l_cfg7, sizeof(l_cfg7), addr);
	storage_write((byte*)&l_pgm, sizeof(l_pgm), addr);
}

//////////////////////////////////////////////////////////////
// READ PATCH INFO
void switch_storage_read(int* addr) {
	storage_read((byte*)&l_default_cfg, sizeof(l_default_cfg), addr);
	storage_read((byte*)&l_cfg0, sizeof(l_cfg0), addr);
	storage_read((byte*)&l_cfg1, sizeof(l_cfg1), addr);
	storage_read((byte*)&l_cfg2, sizeof(l_cfg2), addr);
	storage_read((byte*)&l_cfg3, sizeof(l_cfg3), addr);
	storage_read((byte*)&l_cfg4, sizeof(l_cfg4), addr);
	storage_read((byte*)&l_cfg5, sizeof(l_cfg5), addr);
	storage_read((byte*)&l_cfg6, sizeof(l_cfg6), addr);
	storage_read((byte*)&l_cfg7, sizeof(l_cfg7), addr);
	storage_read((byte*)&l_pgm, sizeof(l_pgm), addr);
}

//
// END
//