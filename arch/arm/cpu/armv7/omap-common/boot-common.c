/*
 * boot-common.c
 *
 * Common bootmode functions for omap based boards
 *
 * Copyright (C) 2011, Texas Instruments, Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR /PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <spl.h>
#include <spi_flash.h>
#ifdef	CONFIG_SPL_EDMA_SUPPORT
#include <edma.h>
#endif
#include <asm/omap_common.h>
#include <asm/arch/omap.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>

#include <bootimg.h>
#include <libfdt.h>
#include <fdt_support.h>

DECLARE_GLOBAL_DATA_PTR;

struct spl_image_info spl_image;

u32 omap_sysboot(void)
{
	return ((*(u32 *)(CTRL_CORE_BOOTSTRAP)) & SYS_BOOT_DEVICE);
}

#ifdef CONFIG_SPL_BUILD

u32 spl_boot_device(void)
{
	return (u32) (gd->arch.omap_boot_params.omap_bootdevice);
}

u32 spl_boot_mode(void)
{
	return gd->arch.omap_boot_params.omap_bootmode;
}

void spl_board_init(void)
{
#ifdef CONFIG_SPL_NAND_SUPPORT
	gpmc_init();
#endif
#if defined(CONFIG_AM33XX) && defined(CONFIG_SPL_MUSB_NEW_SUPPORT)
	arch_misc_init();
#endif

#ifdef CONFIG_SPL_EARLY_BOOT
	board_init();
#endif
}

int board_mmc_init(bd_t *bis)
{
	switch (spl_boot_device()) {
	case BOOT_DEVICE_MMC1:
		omap_mmc_init(0, 0, 0, -1, -1);
		break;
#ifdef CONFIG_SPL_USB_BOOT_SUPPORT
	case BOOT_DEVICE_USB:
#endif
	case BOOT_DEVICE_SPI:
	case BOOT_DEVICE_SPI_4:
#ifdef CONFIG_SPL_EARLY_BOOT
		omap_mmc_init(1, 0, 0, -1, -1);
#endif
        break;
	case BOOT_DEVICE_MMC2:
	case BOOT_DEVICE_MMC2_2:
		omap_mmc_init(1, 0, 0, -1, -1);
		break;
	}
	return 0;
}

void __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	typedef void __noreturn (*image_entry_noargs_t)(u32 *);
	image_entry_noargs_t image_entry =
			(image_entry_noargs_t) spl_image->entry_point;

	debug("image entry point: 0x%X\n", spl_image->entry_point);
	/* Pass the saved boot_params from rom code */
	image_entry((u32 *)&gd->arch.omap_boot_params);
}

#ifdef CONFIG_SPL_EARLY_BOOT

#define TOSTRING(x) #x
#define STR(x) TOSTRING(x)

struct boot_img_hdr spl_kernel_boot;
struct fdt_header *working_fdt;

void set_working_fdt_addr(void *addr)
{
	working_fdt = addr;
}

static int fdt_valid(void)
{
	int  err;

	if (working_fdt == NULL) {
		printf ("The address of the fdt is invalid (NULL).\n");
		return 0;
	}

	err = fdt_check_header(working_fdt);
	if (err == 0)
		return 1;	/* valid */

	if (err < 0) {
		printf("libfdt fdt_check_header(): %s", fdt_strerror(err));
		/*
		 * Be more informative on bad version.
		 */
		if (err == -FDT_ERR_BADVERSION) {
			if (fdt_version(working_fdt) <
			    FDT_FIRST_SUPPORTED_VERSION) {
				printf (" - too old, fdt %d < %d",
					fdt_version(working_fdt),
					FDT_FIRST_SUPPORTED_VERSION);
				working_fdt = NULL;
			}
			if (fdt_last_comp_version(working_fdt) >
			    FDT_LAST_SUPPORTED_VERSION) {
				printf (" - too new, fdt %d > %d",
					fdt_version(working_fdt),
					FDT_LAST_SUPPORTED_VERSION);
				working_fdt = NULL;
			}
			return 0;
		}
		printf("\n");
		return 0;
	}
	return 1;
}

static int spl_fdt(int argc, char * const argv[])
{
	if (argv[1][0] == 'a') {
		unsigned long addr;
		/*
		 * Set the address [and length] of the fdt.
		 */
		if (argc == 2) {
			if (!fdt_valid()) {
				return 1;
			}
			printf("The address of the fdt is %p\n", working_fdt);
			return 0;
		}

		addr = simple_strtoul(argv[2], NULL, 16);
		set_working_fdt_addr((void *)addr);

		if (!fdt_valid()) {
			return 1;
		}

		if (argc >= 4) {
			int  len;
			int  err;
			/*
			 * Optional new length
			 */
			len = simple_strtoul(argv[3], NULL, 16);
			if (len < fdt_totalsize(working_fdt)) {
				printf ("New length %d < existing length %d, "
					"ignoring.\n",
					len, fdt_totalsize(working_fdt));
			} else {
				/*
				 * Open in place with a new length.
				 */
				err = fdt_open_into(working_fdt, working_fdt, len);
				if (err != 0) {
					printf ("libfdt fdt_open_into(): %s\n",
						fdt_strerror(err));
				}
			}
		}

		return 0;
	}

	if (!working_fdt) {
		puts(
			"No FDT memory address configured. Please configure\n"
			"the FDT address via \"fdt addr <address>\" command.\n"
			"Aborting!\n");
		return 1;
	}

	if (strncmp(argv[1], "re", 2) == 0) {
		fdt_resize(working_fdt);
	}else if (argv[1][0] == 'c') {
		unsigned long initrd_start = 0, initrd_end = 0;

		if ((argc != 2) && (argc != 4))
			return 1;

		if (argc == 4) {
			initrd_start = simple_strtoul(argv[2], NULL, 16);
			initrd_end = simple_strtoul(argv[3], NULL, 16);
		}

		fdt_chosen(working_fdt, 1);
		fdt_initrd(working_fdt, initrd_start, initrd_end, 1);
	}

	return 0;
}

#ifdef CONFIG_SPL_EARLY_BOOT_ANDROID
u32 spl_boot_linux(void)
{
	struct mmc *mmc;

	u32 cfg_machine_type = CONFIG_BOARD_MACH_TYPE;
	void (*theKernel)(int zero, int arch, void *);
	char* fdt_addr[3] = { "fdt", "addr", STR(DEVICE_TREE) };
	char* fdt_resize[2] = { "fdt", "resize"};
	char* fdt_chosen[4] = { "fdt", "chosen", NULL, NULL};
	char start[32];
	char end[32];

	spl_mmc_init(&mmc);

# if defined(CONFIG_BOOTIPU1) || defined(CONFIG_LATE_ATTACH_BOOTIPU1) || \
					defined(CONFIG_LATE_ATTACH_BOOTIPU2)
	/* Clock and MMU initialization done in spl_board_init */
	spl_mmc_load_image_raw(mmc, LOAD_IPU);
	if (spl_boot_ipu()){
		puts("Error loading IPU!, fall back to u-boot...\n");
		return 1;
	}
# endif
	spl_mmc_load_image_raw(mmc, LOAD_KERNEL);
	debug("kernel: start %x end %x\n",
		spl_kernel_boot.kernel_addr,
		spl_kernel_boot.kernel_size);
	if (memcmp(spl_kernel_boot.magic, BOOT_MAGIC, 8)) {
		puts("spl_boot_linux: Bad boot image magic\n");
		return 1;
	}

	/*
	 * Account 2048 bytes from the Android boot img header
	 * overhead generated by mkbootimg; this data is  needed
	 * for proper authentication
	 */
	authenticate_image_signature(
		spl_kernel_boot.kernel_addr-2048,
		spl_kernel_boot.kernel_size+2048);

	spl_mmc_load_image_raw(mmc, LOAD_RAMDISK);
	debug("ramdisk: start %x end %x\n",
		spl_kernel_boot.ramdisk_addr,
		spl_kernel_boot.ramdisk_size);
	/*
	 * If image is signed and authentication passed, need to remove
	 * signature block for ftd to work correctly.
	*/
	if(authenticate_image_signature(
				spl_kernel_boot.ramdisk_addr,
				spl_kernel_boot.ramdisk_size) == 0)
		 spl_kernel_boot.ramdisk_size -= 280;

	spl_mmc_load_image_raw(mmc, LOAD_DTB);

	//Set the initrd_start and initrd_end inside the FDT
	if (spl_fdt(3, fdt_addr)){
		puts("Could not set FDT address\n");
		return 1;
	}
	if (spl_fdt(4, fdt_resize)) {
		 printf("Could not resize FDT\n");
		return 1;
	}

	fdt_chosen[2] = start;
	fdt_chosen[3] = end;
	sprintf(start, "0x%x", (unsigned int)spl_kernel_boot.ramdisk_addr);
	sprintf(end, "0x%x", (unsigned int)(spl_kernel_boot.ramdisk_addr +
					 spl_kernel_boot.ramdisk_size));
	if (spl_fdt(4, fdt_chosen)) {
		 printf("Could not set initrd_start and initrd_end\n");
		return 1;
	}

	/* Jump to kernel */
	printf("Starting kernel...\n");
	theKernel = (void (*)(int, int, void *))(spl_kernel_boot.kernel_addr);
	theKernel(0, cfg_machine_type, (void *)DEVICE_TREE);

	// We shouldn't get here!
	return 1;
}
#elif defined(CONFIG_SPL_EARLY_BOOT)
u32 spl_boot_linux()
{
	struct spi_flash *flash;
	struct mmc *mmc;
	image_header_t image_header;
	void (*entry_point)(int zero, int arch, void *);
	const int DTB_SIZE = 0x10000;

	spl_mmc_init(&mmc);
	if (!mmc){
		puts("spl: mmc device not found!!\n");
		hang();
	}

#if defined(CONFIG_SPL_EDMA_SUPPORT)
      edma3_init(0);
      edma3_request_channel(EDMA3_CHANNEL_TYPE_DMA,1,1,0);
#endif

	flash = spi_flash_probe(CONFIG_SPL_SPI_BUS, CONFIG_SPL_SPI_CS,
				CONFIG_SF_DEFAULT_SPEED, SPI_MODE_3);
	if (!flash) {
		puts("SPI probe failed.\n");
		hang();
	}

    /* Early boot changes*/
	/*TODO: Add IPU changes here*/

    /*1) Read & parse the uImage header(image_header_t)*/
    spi_flash_read(flash, CONFIG_SYS_SPI_UIMAGE_OFFS,sizeof(image_header_t),(void*)&image_header);
    spl_image.flags |= SPL_COPY_PAYLOAD_ONLY;
    spl_parse_image_header(&image_header);

    /* 2) Load the kernel */
    spi_flash_read(flash,CONFIG_SYS_SPI_UIMAGE_OFFS + sizeof(image_header_t),
                   spl_image.size,(void*)spl_image.load_addr);
    entry_point = (void(*)(int,int,void*))spl_image.entry_point;

    /*3) Load the dtb file*/
    spi_flash_read(flash, CONFIG_SYS_SPI_DTB_OFFS, DTB_SIZE, \
				  (void*)DEVICE_TREE);

    /*4) Jump to kernel entry point */
    printf("\n starting kernel ...\n");
    entry_point(0,CONFIG_BOARD_MACH_TYPE,(void*)DEVICE_TREE);

}
#endif

static void spl_bootimg_print_image_hdr (const struct boot_img_hdr *hdr)
{
	puts ("Loaded kernel info\n");
	printf ("   Image magic:   %s\n", hdr->magic);

	printf ("   kernel_size:   0x%x\n", hdr->kernel_size);
	printf ("   kernel_addr:   0x%x\n", hdr->kernel_addr);

	printf ("   rdisk_size:   0x%x\n", hdr->ramdisk_size);
	printf ("   rdisk_addr:   0x%x\n", hdr->ramdisk_addr);

	printf ("   second_size:   0x%x\n", hdr->second_size);
	printf ("   second_addr:   0x%x\n", hdr->second_addr);

	printf ("   tags_addr:   0x%x\n", hdr->tags_addr);
	printf ("   page_size:   0x%x\n", hdr->page_size);

	printf ("   name:      %s\n", hdr->name);
	printf ("   cmdline:   %s\n", hdr->cmdline);
}


void spl_parse_kernel_image_header(const struct boot_img_hdr *hdr)
{
	spl_image.os = IH_OS_LINUX;
	spl_image.name = (const char *) hdr->name;
	spl_image.load_addr = (u32) hdr->kernel_addr;
	spl_image.entry_point = hdr->kernel_addr;
	spl_image.size = hdr->kernel_size;

	memcpy(&spl_kernel_boot.magic, &hdr->magic,
				sizeof(spl_kernel_boot));

	spl_bootimg_print_image_hdr(hdr);
}
#endif /* CONFIG_SPL_EARLY_BOOT */

#endif
