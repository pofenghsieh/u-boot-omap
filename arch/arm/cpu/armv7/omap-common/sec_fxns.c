/*
 *
 * Common security functions that rely on OMAP ROM secure world code
 *
 * (C) Copyright 2015
 * Texas Instruments, <www.ti.com>
 *
 * Daniel Allred    <d-allred@ti.com>
 *
 * SPDX-License-Identifier: GPL-2.0+
 */


/*------------------------------ Include Files -------------------------------*/

#include <common.h>
#include <stdarg.h>

#include <asm/arch/sys_proto.h>
#include <asm/omap_common.h>


/*----------------------------- Extern Declarations --------------------------*/

extern int SEC_ENTRY_Pub2SecDispatcher(uint32_t appl_id, uint32_t proc_ID, uint32_t flag, ...);


/*------------------------------- Local Macros -------------------------------*/

#define SIGNATURE_LENGTH				(0x118)

#if defined(CONFIG_OMAP54XX)
/* API Index for OMAP5, DRA7xx */
#define API_HAL_KM_VERIFYCERTIFICATESIGNATURE_INDEX     (0x0000000E)
#else
/* API Index for OMAP4, J5, J5Eco, Aegis, Subarctic */
#define API_HAL_KM_VERIFYCERTIFICATESIGNATURE_INDEX     (0x0000000C)
#endif


/*-------------------------- Local Type Definitions --------------------------*/


/*----------------------- Local Variable Declarations ------------------------*/


/*---------------------- Global Variable Declarations ------------------------*/


/*------------------------ Local Function Definitions ------------------------*/


/*----------------------- Global Function Definitions ------------------------*/

int secure_boot_verify_image(const void *image, size_t size)
{
	int result = 1;
	uint32_t load_addr, sig_addr;

	load_addr = (uint32_t)image;
	size -= SIGNATURE_LENGTH;   /* Subtract out the signature size */
	sig_addr = load_addr + size;

	/* Check if image load address is 32-bit aligned */
	if (0 != (0x3 & load_addr)) {
		puts("Image is not 4-byte aligned.\n");
		result = 1;
		goto auth_exit;
	}

	/* Image size also should be multiple of 4 */
	if (0 != (0x3 & size)) {
		puts("Image size is not 4-byte aligned.\n");
		result = 1;
		goto auth_exit;
	}

	/*
	 * Call ROM HAL API to verify certificate signature.
	 * This is kind of an abuse of the API as the image
	 * is not a certificate, but rather just a signed data blob.
	 */
	result = SEC_ENTRY_Pub2SecDispatcher(
		API_HAL_KM_VERIFYCERTIFICATESIGNATURE_INDEX,
		0, 0, 4, load_addr, size,
		sig_addr, 0xFFFFFFFF);
auth_exit:
		if (result != 0) {
			puts("Authentication failed!\n");
			printf("Return Value = %08X\n", result);
			hang();
		}

	return result;
}

/*------------------------------- End Of File --------------------------------*/
