//////////////////////////////////////////////////////////////
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

#define MAGIC_COOKIE 0xA1

//
// LOCAL FUNCTIONS
//

////////////////////////////////////////////////////
// SAVE DATA TO EEPROM
static void storage_write(byte *data, int len, int* addr)
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
static void storage_read(byte *data, int len, int* addr)
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
	storage_write(switch_storage(&len), len, &storage_ofs);
	storage_write(switch_pgm_storage(&len), len, &storage_ofs);
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
	storage_read(switch_storage(&len), len, &storage_ofs);
	storage_read(switch_pgm_storage(&len), len, &storage_ofs);
}

//
// END
// 