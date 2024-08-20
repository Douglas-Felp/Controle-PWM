/*
 * eeprom.c
 *
 *  Created on: Feb 25, 2024
 *      Author: pablo-jean
 */

#include "eeprom.h"

/*
 * Privates
 */

#if defined (EEPROM_DEBUG)
// TODO The printf :)
#define EEPROM_PRINTF(...)			printf(__VA_ARGS__)
#else
#define EEPROM_PRINTF(...)
#endif


/*
 * Publics
 */

eeprom_e eeprom_init(eeprom_t *Eeprom){
	/* sanity check */
	if (Eeprom == NULL){
		EEPROM_PRINTF("ERROR: EEPROM Handle is NULL\n");
		return EE_UNKNOWN;
	}
	if (Eeprom->EeReadFxn == NULL || Eeprom->EeWriteFxn == NULL || Eeprom->DelayMs == NULL){
		EEPROM_PRINTF("ERROR: Functions is not declared on %s function\n", __func__);
		return EE_INVALID_VARS;
	}
	if (Eeprom->memSize == 0 || Eeprom->memAddrSize == 0 || Eeprom->pageSize == 0){
		EEPROM_PRINTF("ERROR: Invalid Parameters on %s function\n", __func__);
		return EE_INVALID_VARS;
	}

	Eeprom->_intern.initialized = EEPROM_INITIALIZE_FLAG;
	Eeprom->_intern.actAddress = 0x00;

	EEPROM_PRINTF("== EEPROM Information ==\n");
	EEPROM_PRINTF(" Memory Size  | %d Bytes\n", (uint)Eeprom->memSize);
	EEPROM_PRINTF(" Page Size    | %d Bytes\n", (uint)Eeprom->pageSize);
	EEPROM_PRINTF(" Address Size | %d\n", (uint)Eeprom->memAddrSize);
	EEPROM_PRINTF(" == EEPROM Initialization Done ==\n");

	return EE_OK;
}

eeprom_e eeprom_write(eeprom_t *Eeprom, uint32_t Address, uint8_t *data, uint32_t len){
	uint32_t remaining, writed, lenToWrite;
	uint32_t actPage;
	uint8_t err;

	/* sanity check */
	if (Eeprom == NULL){
		return EE_UNKNOWN;
	}
	if (Eeprom->_intern.initialized != EEPROM_INITIALIZE_FLAG){
		return EE_NOT_INITIALIZED;
	}
	if ((Address+len) > Eeprom->memSize){
		return EE_OVERFLOW;
	}

	remaining = len;
	writed = 0;
	while (remaining > 0){
		actPage = Address/Eeprom->pageSize;
		lenToWrite = ((actPage+1)*Eeprom->pageSize) - Address;
		if (lenToWrite > remaining){
			lenToWrite = remaining;
		}
		err = Eeprom->EeWriteFxn(Eeprom->i2cAddress,
								 Address,
								 Eeprom->memAddrSize,
								 &data[writed],
								 lenToWrite);
		if (err != 0){
			return EE_ERROR;
		}
		writed += lenToWrite;
		Address += lenToWrite;
		remaining -= lenToWrite;
		Eeprom->_intern.actAddress = Address;
		Eeprom->DelayMs(EEPROM_DELAY_BTW_WRITES);
	}

	return EE_OK;
}

eeprom_e eeprom_read(eeprom_t *Eeprom, uint32_t Address, uint8_t *data, uint32_t len){
	uint8_t err;
	/* sanity check */
	if (Eeprom == NULL){
		return EE_UNKNOWN;
	}
	if (Eeprom->_intern.initialized != EEPROM_INITIALIZE_FLAG){
		return EE_NOT_INITIALIZED;
	}
	if ((Address+len) > Eeprom->memSize){
		return EE_OVERFLOW;
	}

	err = Eeprom->EeReadFxn(Eeprom->i2cAddress,
						    Address,
						    Eeprom->memAddrSize,
						    data,
						    len);
	if (err != 0){
		return EE_ERROR;
	}
	Address += len;
	Eeprom->_intern.actAddress = Address;

	return EE_OK;
}

eeprom_e eeprom_fill_all(eeprom_t *Eeprom, uint8_t fillByte){
	uint8_t *temp, err;
	uint32_t Address;
	/* sanity check */
	if (Eeprom == NULL){
		return EE_UNKNOWN;
	}
	if (Eeprom->_intern.initialized != EEPROM_INITIALIZE_FLAG){
		return EE_NOT_INITIALIZED;
	}

	Address = 0;
	temp = malloc(Eeprom->pageSize);
	if (temp == NULL){
		return EE_ERROR;
	}
	memset(temp, fillByte, Eeprom->pageSize);
	while (Address < Eeprom->memSize){
		err = Eeprom->EeWriteFxn(Eeprom->i2cAddress,
								 Address,
								 Eeprom->memAddrSize,
								 temp,
								 Eeprom->pageSize);
		if (err != 0){
			return EE_ERROR;
		}
		Address += Eeprom->pageSize;
		Eeprom->_intern.actAddress = Address;
		Eeprom->DelayMs(EEPROM_DELAY_BTW_WRITES);
	}
	free(temp);

	return EE_OK;
}
