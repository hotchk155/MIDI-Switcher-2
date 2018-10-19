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
// PATCH STORAGE MODULE
//
//////////////////////////////////////////////////////////////

//
// HEADER FILES
//
#include <system.h>
#include <eeprom.h>
#include "msw.h"

//
// LOCAL DATA
//

#define MAGIC_COOKIE 0xA3

//
// LOCAL FUNCTIONS
//

////////////////////////////////////////////////////
// SAVE DATA TO EEPROM
void storage_write(byte *data, int len, int* addr)
{
	while(len > 0) {
		eeprom_write(*addr, *data);
		++(*addr);
		++data;
		--len;
	}
}

////////////////////////////////////////////////////
// READ DATA FROM EEPROM
void storage_read(byte *data, int len, int* addr)
{
	while(len > 0) {
		*data = eeprom_read(*addr);
		++(*addr);
		++data;
		--len;
	}
}

////////////////////////////////////////////////////
// SAVE ALL CONFIG TO EEPROM
void storage_write_patch()
{
	int len = 0;
	eeprom_write(0, MAGIC_COOKIE);
	int storage_ofs = 1;
	switch_storage_write(&storage_ofs);
}

////////////////////////////////////////////////////
// READ CONFIG FRM EEPROM
void storage_read_patch()
{
	if(eeprom_read(0) != MAGIC_COOKIE) {
		return;
	}
	int len = 0;
	int storage_ofs = 1;
	switch_storage_read(&storage_ofs);
}

//
// END
// 