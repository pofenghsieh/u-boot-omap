/*
 *
 * Bridging C function to make SMC calls to secure world
 *
 * (C) Copyright 2015
 * Texas Instruments, <www.ti.com>
 *
 * Daniel Allred	<d-allred@ti.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */


/*------------------------------ Include Files -------------------------------*/

#include <stdint.h>
#include <stdarg.h>


/*----------------------------- Extern Declarations --------------------------*/

extern int SEC_ENTRY_Pub2SecBridgeEntry(uint32_t appl_id, uint32_t proc_ID, uint32_t flag, va_list ap);


/*------------------------------- Local Macros -------------------------------*/


/*-------------------------- Local Type Definitions --------------------------*/


/*----------------------- Local Variable Declarations ------------------------*/


/*---------------------- Global Variable Declarations ------------------------*/


/*------------------------ Local Function Definitions ------------------------*/


/*----------------------- Global Function Definitions ------------------------*/


/*_____________________________________________________________________________
* FUNCTION: SEC_ENTRY_Pub2SecDispatcher
*
* DESCRIPTION: C function responsible for formatting the parameters to pass
* to the Secure ROM Code.
*
* INPUTS:
*   appl_id : Secure HAL service ID
*   proc_ID : Process ID
*   flag    : Calling flag
*   ...     : Secure HAL service arguments
*
* OUTPUT:
*   NONE
*
* RETURNS:
*   Secure HAL service return value
*____________________________________________________________________________*/
int SEC_ENTRY_Pub2SecDispatcher(uint32_t appl_id, uint32_t proc_ID, uint32_t flag, ...)
{
	va_list ap;
	uint32_t return_value;
	va_start(ap, flag);

	return_value = SEC_ENTRY_Pub2SecBridgeEntry(appl_id, proc_ID, flag, ap);
	va_end(ap);

	return return_value;
}


/*------------------------------- End Of File --------------------------------*/
