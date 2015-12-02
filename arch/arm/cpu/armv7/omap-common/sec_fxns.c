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
#include <fdt_support.h>


/*----------------------------- Extern Declarations --------------------------*/

extern int SEC_ENTRY_Pub2SecBridgeEntry(uint32_t appl_id, uint32_t proc_ID, uint32_t flag, uint32_t *params);


/*------------------------------- Local Macros -------------------------------*/

#define SIGNATURE_LENGTH				(0x118)

#if defined(CONFIG_OMAP54XX)
/* API Index for OMAP5, DRA7xx */
#define API_HAL_KM_VERIFYCERTIFICATESIGNATURE_INDEX     (0x0000000E)

#define PPA_HAL_SERVICES_START_INDEX                    (0x200)
#define PPA_SERV_HAL_SETUP_SEC_RESVD_REGION             (PPA_HAL_SERVICES_START_INDEX + 25)
#define PPA_SERV_HAL_SETUP_EMIF_FW_REGION               (PPA_HAL_SERVICES_START_INDEX + 26)
#define PPA_SERV_HAL_LOCK_EMIF_FW                       (PPA_HAL_SERVICES_START_INDEX + 27)

#else
/* API Index for OMAP4, J5, J5Eco, Aegis, Subarctic */
#define API_HAL_KM_VERIFYCERTIFICATESIGNATURE_INDEX     (0x0000000C)
#endif


/*-------------------------- Local Type Definitions --------------------------*/


/*----------------------- Local Variable Declarations ------------------------*/

static uint32_t LOCAL_smcParams[5] __aligned(ARCH_DMA_MINALIGN);


/*---------------------- Global Variable Declarations ------------------------*/


/*------------------------ Local Function Definitions ------------------------*/

static void LOCAL_prepParams(uint32_t numParams, ...)
{
	int i;
	va_list ap;
	va_start(ap, numParams);

	LOCAL_smcParams[0] = numParams;
	for (i=0; i<numParams; i++)	{
		LOCAL_smcParams[i+1] = va_arg(ap, uint32_t);
	}

	va_end(ap);

#if !defined(CONFIG_SPL_BOOT)
#if !defined(CONFIG_SYS_DCACHE_OFF)
	flush_dcache_range(
		(unsigned int) &LOCAL_smcParams[0],
		((unsigned int)&LOCAL_smcParams[0]) + roundup(sizeof(LOCAL_smcParams), ARCH_DMA_MINALIGN));
#endif
#endif
}

/*----------------------- Global Function Definitions ------------------------*/

int secure_boot_verify_image(const void *image, size_t size)
{
	int result = 1;
	uint32_t load_addr, sig_addr;

#if !defined(CONFIG_SPL_BOOT)
#if !defined(CONFIG_SYS_DCACHE_OFF)
	/* Perform cache writeback on input buffer */
	flush_dcache_range(
		(unsigned int) image,
		((unsigned int)image) + roundup(size, ARCH_DMA_MINALIGN));
#endif
#endif

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
	debug("%s: load_addr = %x, size = %x, sig_addr = %x\n", __func__,
		load_addr, size, sig_addr);

	LOCAL_prepParams(4, load_addr, size,
        sig_addr, 0xFFFFFFFF);
	result = SEC_ENTRY_Pub2SecBridgeEntry(
		API_HAL_KM_VERIFYCERTIFICATESIGNATURE_INDEX,
		0, 0, LOCAL_smcParams);
auth_exit:
		if (result != 0) {
			puts("Authentication failed!\n");
			printf("Return Value = %08X\n", result);
			hang();
		}

	return result;
}

int secure_memory_reserve(uint32_t startAddr, uint32_t size)
{
	int result = 1;

	/*
	 * Call PPA HAL API to reserve a chunk of EMIF SDRAM
	 * for secure world use. This region should be carved out
	 * from use by any public code. EMIF firewall region 7
     * will be used to protect this block of memory.
	 */
	debug("%s: startAddr = %x, size = %x", __func__,
		startAddr, size);

	LOCAL_prepParams(2, startAddr, size);
	result = SEC_ENTRY_Pub2SecBridgeEntry(
		PPA_SERV_HAL_SETUP_SEC_RESVD_REGION,
        0, 0, LOCAL_smcParams);

	if (result != 0) {
		puts("Secure Memory Reservation failed!\n");
		printf("Return Value = %08X\n", result);
	}

	return result;
}

int secure_emif_firewall_setup(uint8_t regionNum, uint32_t startAddr, uint32_t size,
							uint32_t accessPerm, uint32_t initiatorPerm)
{
	int result = 1;

	/*
	 * Call PPA HAL API to do any other general firewall
	 * configuration for regions 1-6 of the EMIF firewall.
	 */
	debug("%s: regionNum = %x, startAddr = %x, size = %x", __func__,
		regionNum, startAddr, size);

	LOCAL_prepParams(4, (startAddr & 0xFFFFFFF0) | (regionNum & 0x0F), size,
		accessPerm, initiatorPerm);
	result = SEC_ENTRY_Pub2SecBridgeEntry(
		PPA_SERV_HAL_SETUP_EMIF_FW_REGION,
        0, 0, LOCAL_smcParams);

    if (result != 0) {
        puts("Secure EMIF Firewall Setup failed!\n");
        printf("Return Value = %08X\n", result);
    }

    return result;
}

int secure_emif_firewall_lock(void)
{
	int result = 1;

	/*
	 * Call PPA HAL API to lock the EMIF firewall configurations.
	 * After this API is called, none of the PPA HAL APIs for
	 * configuring the EMIF firewalls will be usable again (that
	 * is, calls to those APIs will return failure and have no 
     * effect).
	 */
	LOCAL_prepParams(0);

	result = SEC_ENTRY_Pub2SecBridgeEntry(
		PPA_SERV_HAL_LOCK_EMIF_FW,
		0, 0, LOCAL_smcParams);

    if (result != 0) {
        puts("Secure EMIF Firewall Lock failed!\n");
        printf("Return Value = %08X\n", result);
    }

    return result;
}

#if !defined(CONFIG_SECURE_FW_MEMORY_SIZE)
#define CONFIG_SECURE_FW_MEMORY_SIZE (0)
#endif
#if !defined(CONFIG_SECURE_NOFW_MEMORY_SIZE)
#define CONFIG_SECURE_NOFW_MEMORY_SIZE (0)
#endif
int hs_device_fixup_fdt(void *fdt)
{
	if (HS_DEVICE == get_device_type()) {
		int offs;
		const char *path;
		u32 temp[16];

		u32 sec_mem_size  = (CONFIG_SECURE_FW_MEMORY_SIZE + CONFIG_SECURE_NOFW_MEMORY_SIZE);
		u32 sec_mem_start = (CONFIG_SYS_SDRAM_BASE + omap_sdram_size()) - sec_mem_size;

		/* Only add reserved memory region if size is non zero */
		if (sec_mem_size) {
			/* Delete any original sec_rsvd node */
			path = "/reserved-memory/sec_rsvd";
			offs = fdt_path_offset(fdt,path);
			if (offs >= 0) {
				fdt_del_node(fdt,offs);
			}

			/* Add new sec_rsvd node */
			path = "/reserved-memory";
			offs = fdt_path_offset(fdt, path);
			if (offs < 0) {
				debug("Node %s not found\n", path);
			}
			else {
				offs = fdt_add_subnode(fdt, offs,"sec_rsvd");
				temp[0] = cpu_to_fdt32(sec_mem_start);
				temp[1] = cpu_to_fdt32(sec_mem_size);
				fdt_setprop_string(fdt, offs, "compatible", "sec-rsvd");
				fdt_setprop_string(fdt, offs, "status", "okay");
				fdt_setprop(fdt, offs, "no-map", NULL, 0);
				fdt_setprop(fdt, offs, "reg", temp, 2*sizeof(temp[0]));
			}
		}

		/* Reserve IRQs that are used/needed by secure world */
		path = "/ocp/crossbar";
		offs = fdt_path_offset(fdt, path);
		if (offs < 0) {
			debug("Node %s not found\n", path);
		}
		else {
			int len;
			int cnt = 0;
			const u32* pData;

			pData = fdt_getprop(fdt, offs, "ti,irqs-skip", &len);
			if (pData) {
				int i;
				cnt = len/sizeof(u32);
				for (i=0; i<cnt; i++) {
					temp[i+3] = pData[i];
				}
			}
			/* Add 8, 15, 118 to skip list for HS parts */
			temp[0] = cpu_to_fdt32(8);
			temp[1] = cpu_to_fdt32(15);
			temp[2] = cpu_to_fdt32(118);
			fdt_delprop(fdt, offs, "ti,irqs-skip");
			fdt_setprop(fdt, offs, "ti,irqs-skip", temp, (cnt+3)*sizeof(temp[0]));
		}

		/* Make HW RNG reserved for secure world use */
		path = "/ocp/rng";
		offs = fdt_path_offset(fdt, path);
		if (offs < 0) {
			debug("Node %s not found\n", path);
		}
		else {
			fdt_setprop_string(fdt, offs, "status", "disabled");
		}
    }

	return 0;
}


/*------------------------------- End Of File --------------------------------*/
