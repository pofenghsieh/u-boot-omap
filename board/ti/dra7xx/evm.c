/*
 * (C) Copyright 2013
 * Texas Instruments Incorporated, <www.ti.com>
 *
 * Lokesh Vutla <lokeshvutla@ti.com>
 *
 * Based on previous work by:
 * Aneesh V       <aneesh@ti.com>
 * Steve Sakoman  <steve@sakoman.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <palmas.h>
#include <sata.h>
#include <asm/gpio.h>
#include <asm/arch/gpio.h>
#include <usb.h>
#include <linux/usb/gadget.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sata.h>
#include <environment.h>
#include <dwc3-uboot.h>
#include <dwc3-omap-uboot.h>
#include <ti-usb-phy-uboot.h>

#include "mux_data.h"

#ifdef CONFIG_DRIVER_TI_CPSW
#include <cpsw.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

/* GPIO 7_11 */
#define GPIO_DDR_VTT_EN 203

const struct omap_sysinfo sysinfo = {
	"Board: DRA7xx\n"
};

#if defined(CONFIG_LATE_ATTACH_BOOTIPU2)
#define L4_CFG_TARG                  0x4A000000
#define L4_WKUP_TARG                 0x4AE00000
#define IPU2_TARGET_TARG             0x55000000
#define IPU1_TARGET_TARG             0x58800000
#define CTRL_MODULE_CORE             (L4_CFG_TARG + 0x2000)
#define CM_CORE_AON                  (L4_CFG_TARG + 0x5000)
#define CM_CORE                      (L4_CFG_TARG + 0x8000)
#define PRM                          (L4_WKUP_TARG + 0x6000)
#define MPU_CM_CORE_AON              (CM_CORE_AON + 0x300)
#define IPU_CM_CORE_AON              (CM_CORE_AON + 0x500)
#define RTC_CM_CORE_AON              (CM_CORE_AON + 0x740)
#define VPE_CM_CORE_AON              (CM_CORE_AON + 0x760)
#define COREAON_CM_CORE              (CM_CORE + 0x600)
#define CORE_CM_CORE                 (CM_CORE + 0x700)
#define CAM_CM_CORE                  (CM_CORE + 0x1000)
#define DSS_CM_CORE                  (CM_CORE + 0x1100)
#define L3INIT_CM_CORE               (CM_CORE + 0x1300)
#define L4PER_CM_CORE                (CM_CORE + 0x1700)
#define CKGEN_PRM                    (PRM + 0x100)
#define IPU_PRM                      (PRM + 0x500)
#define CORE_PRM                     (PRM + 0x700)
#define WKUPAON_CM                   (PRM + 0x1800)

#define CM_L3MAIN1_CLKSTCTRL         (CORE_CM_CORE + 0x000)
#define CM_IPU2_CLKSTCTRL            (CORE_CM_CORE + 0x200)
#define CM_DMA_CLKSTCTRL             (CORE_CM_CORE + 0x300)
#define CM_EMIF_CLKSTCTRL            (CORE_CM_CORE + 0x400)
#define CM_L4CFG_CLKSTCTRL           (CORE_CM_CORE + 0x600)

#define CM_DSS_CLKSTCTRL             (DSS_CM_CORE + 0x00)
#define CM_CAM_CLKSTCTRL             (CAM_CM_CORE + 0x00)
#define CM_COREAON_CLKSTCTRL         (COREAON_CM_CORE + 0x00)
#define CM_L3INIT_CLKSTCTRL          (L3INIT_CM_CORE + 0x00)
#define CM_GMAC_CLKSTCTRL            (L3INIT_CM_CORE + 0xC0)
#define CM_L4PER_CLKSTCTRL           (L4PER_CM_CORE + 0x000)
#define CM_L4PER_TIMER11_CLKCTRL     (CM_L4PER_CLKSTCTRL + 0x30)
#define CM_L4PER_TIMER3_CLKCTRL      (CM_L4PER_CLKSTCTRL + 0x40)
#define CM_L4PER_TIMER4_CLKCTRL      (CM_L4PER_CLKSTCTRL + 0x48)
#define CM_L4PER_TIMER9_CLKCTRL      (CM_L4PER_CLKSTCTRL + 0x50)
#define CM_L4PER2_CLKSTCTRL          (L4PER_CM_CORE + 0x1FC)
#define CM_L4PER3_CLKSTCTRL          (L4PER_CM_CORE + 0x210)
#define CM_MPU_CLKSTCTRL             (MPU_CM_CORE_AON + 0x00)
#define CM_RTC_CLKSTCTRL             (RTC_CM_CORE_AON + 0x00)
#define CM_VPE_CLKSTCTRL             (VPE_CM_CORE_AON + 0x00)
#define CM_WKUPAON_CLKSTCTRL         (WKUPAON_CM + 0x00)

#define RM_IPU1_RSTCTRL              (IPU_PRM + 0x10)
#define RM_IPU1_RSTST                (IPU_PRM + 0x14)
#define CM_IPU1_CLKSTCTRL            (IPU_CM_CORE_AON + 0x0)
#define CM_IPU1_IPU1_CLKCTRL         (IPU_CM_CORE_AON + 0x20)
#define CM_IPU2_IPU2_CLKCTRL         (CORE_CM_CORE + 0x220)
#define CM_IPU_CLKSTCTRL             (IPU_CM_CORE_AON + 0x40)
#define CM_IPU_MCASP1_CLKCTRL        (IPU_CM_CORE_AON + 0x50)
#define CM_IPU_TIMER5_CLKCTRL        (IPU_CM_CORE_AON + 0x58)
#define CM_IPU_TIMER6_CLKCTRL        (IPU_CM_CORE_AON + 0x60)
#define CM_IPU_TIMER7_CLKCTRL        (IPU_CM_CORE_AON + 0x68)
#define CM_IPU_TIMER8_CLKCTRL        (IPU_CM_CORE_AON + 0x70)
#define CM_L4PER2_L4_PER2_CLKCTRL    (L4PER_CM_CORE + 0x0C)
#define CM_L4PER3_L4_PER3_CLKCTRL    (L4PER_CM_CORE + 0x14)
#define CM_L4PER_I2C1_CLKCTRL        (L4PER_CM_CORE + 0xA0)
#define CM_L4PER_I2C2_CLKCTRL        (L4PER_CM_CORE + 0xA8)
#define CM_L4PER_I2C3_CLKCTRL        (L4PER_CM_CORE + 0xB0)
#define CM_L4PER_I2C4_CLKCTRL        (L4PER_CM_CORE + 0xB8)
#define CM_L4PER_L4_PER1_CLKCTRL     (L4PER_CM_CORE + 0xC0)
#define CM_CAM_VIP1_CLKCTRL          (CAM_CM_CORE + 0x20)
#define CM_CAM_VIP2_CLKCTRL          (CAM_CM_CORE + 0x28)
#define CM_CAM_VIP3_CLKCTRL          (CAM_CM_CORE + 0x30)
#define CM_DMA_DMA_SYSTEM_CLKCTRL    (CORE_CM_CORE + 0x320)
#define CM_L3INSTR_L3_MAIN_2_CLKCTRL (CORE_CM_CORE + 0x728)
#define CM_DSS_DSS_CLKCTRL           (DSS_CM_CORE + 0x20)
#define CM_VPE_VPE_CLKCTRL           (VPE_CM_CORE_AON + 0x04)

#define RM_IPU2_RSTCTRL              (CORE_PRM + 0x210)
#define RM_IPU2_RSTST                (CORE_PRM + 0x214)

#define CTRL_CORE_CONTROL_IO_2       (CTRL_MODULE_CORE + 0x558)

#define IPU1_BASE_ADDR               (IPU1_TARGET_TARG + 0x20000)
#define IPU1_MMU_CFG                 (IPU1_TARGET_TARG + 0x80000)
#define IPU2_MMU_CFG                 (IPU2_TARGET_TARG + 0x80000)

#define CTRL_CORE_CORTEX_M4_MMUADDRTRANSLTR 0x4A002358
#define CTRL_CORE_CORTEX_M4_MMUADDRLOGICTR  0x4A00235C

# ifdef CONFIG_LATE_ATTACH_BOOTIPU2
#define IPU_MMU_REGS         IPU2_MMU_CFG + 0x2000
# endif

#define MMU_REVISION                         IPU_MMU_REGS + 0x00
#define MMU_IRQSTATUS                        IPU_MMU_REGS + 0x18
#define MMU_IRQENABLE                        IPU_MMU_REGS + 0x1c
#define MMU_WALKING_ST                       IPU_MMU_REGS + 0x40
#define MMU_CNTL                             IPU_MMU_REGS + 0x44
#define MMU_FAULT_AD                         IPU_MMU_REGS + 0x48
#define MMU_TTB                              IPU_MMU_REGS + 0x4c
#define MMU_LOCK                             IPU_MMU_REGS + 0x50
#define MMU_LD_TLB                           IPU_MMU_REGS + 0x54
#define MMU_CAM                              IPU_MMU_REGS + 0x58
#define MMU_RAM                              IPU_MMU_REGS + 0x5c
#define MMU_GFLUSH                           IPU_MMU_REGS + 0x60
#define MMU_FLUSH_ENTRY                      IPU_MMU_REGS + 0x64
#define MMU_READ_CAM                         IPU_MMU_REGS + 0x68
#define MMU_READ_RAM                         IPU_MMU_REGS + 0x6c
#define MMU_EMU_FAULT_AD                     IPU_MMU_REGS + 0x70
#define MMU_GPR                              IPU_MMU_REGS + 0x88

#define PAGESIZE_1M                          0x0
#define PAGESIZE_64K                         0x1
#define PAGESIZE_4K                          0x2
#define PAGESIZE_16M                         0x3
#define LE                                   0
#define BE                                   1
#define ELEMSIZE_8                           0x0
#define ELEMSIZE_16                          0x1
#define ELEMSIZE_32                          0x2
#define MIXED_TLB                            0x0
#define MIXED_CPU                            0x1

#define PGT_SECTION_SIZE                     0x00100000
#define PGT_SUPERSECTION_SIZE                0x01000000

#define PGT_L1_DESC_PAGE                     0x00001
#define PGT_L1_DESC_SECTION                  0x00002
#define PGT_L1_DESC_SUPERSECTION             0x40002

#define PGT_L1_DESC_SECTION_MASK             0xfff00000
#define PGT_L1_DESC_SUPERSECTION_MASK        0xff000000

#define PGT_L1_DESC_SECTION_INDEX_SHIFT      20
#define PGT_L1_DESC_SUPERSECTION_INDEX_SHIFT 24

#define DRA7_RPROC_CMA_BASE_IPU1             0x9d000000
#define DRA7_RPROC_CMA_BASE_IPU2             0x95800000

#define DRA7_RPROC_CMA_SIZE_IPU1             0x02000000
#define DRA7_RPROC_CMA_SIZE_IPU2             0x03800000

/*
 * The page table (16 KB) is placed at the end of the CMA reserved area.
 * It's possible that this location is needed by the firmware (in which
 * case the firmware is using pretty much *all* of the reserved area), but
 * there doesn't seem to be a better location to place it.
*/
#ifdef CONFIG_LATE_ATTACH_BOOTIPU2
#define PAGE_TABLE_SIZE 0x00004000
#define PAGE_TABLE_PHYS (DRA7_RPROC_CMA_BASE_IPU2 + DRA7_RPROC_CMA_SIZE_IPU2 - PAGE_TABLE_SIZE)
#endif

#define PAGE_SHIFT 12
#define PAGE_SIZE  (1 << PAGE_SHIFT)

#define BITS_PER_BYTE 8
#undef BITS_PER_LONG
#define BITS_PER_LONG (sizeof (long) * BITS_PER_BYTE)
#define BITS_TO_LONGS(nbits) (((nbits) + BITS_PER_LONG - 1) / BITS_PER_LONG)

#define BIT_WORD(nr) ((nr) / BITS_PER_LONG)
#define BITOP_WORD(nr) BIT_WORD(nr)

#define BITMAP_FIRST_WORD_MASK(start) (~0UL << ((start) % BITS_PER_LONG))
#define BITMAP_LAST_WORD_MASK(nbits)                                    \
(                                                                       \
        ((nbits) % BITS_PER_LONG) ?                                     \
                (1UL<<((nbits) % BITS_PER_LONG))-1 : ~0UL               \
)

unsigned int *page_table = (unsigned int *)PAGE_TABLE_PHYS;

#ifdef CONFIG_LATE_ATTACH_BOOTIPU2
unsigned long mem_base = DRA7_RPROC_CMA_BASE_IPU2;
unsigned long mem_size = DRA7_RPROC_CMA_SIZE_IPU2;
unsigned long mem_bitmap[BITS_TO_LONGS(DRA7_RPROC_CMA_SIZE_IPU2 >> PAGE_SHIFT)];
unsigned long mem_count = DRA7_RPROC_CMA_SIZE_IPU2 >> PAGE_SHIFT;
#endif

void bitmap_set(unsigned long *map, int start, int nr)
{
	unsigned long *p = map + BIT_WORD(start);
	const int size = start + nr;
	int bits_to_set = BITS_PER_LONG - (start % BITS_PER_LONG);
	unsigned long mask_to_set = BITMAP_FIRST_WORD_MASK(start);

	while (nr - bits_to_set >= 0) {
		*p |= mask_to_set;
		nr -= bits_to_set;
		bits_to_set = BITS_PER_LONG;
		mask_to_set = ~0UL;
		p++;
	}
	if (nr) {
		mask_to_set &= BITMAP_LAST_WORD_MASK(size);
		*p |= mask_to_set;
	}
}

static unsigned long __ffs(unsigned long word)
{
	int num = 0;

	if ((word & 0xffff) == 0) {
		num += 16;
		word >>= 16;
	}
	if ((word & 0xff) == 0) {
		num += 8;
		word >>= 8;
	}
	if ((word & 0xf) == 0) {
		num += 4;
		word >>= 4;
	}
	if ((word & 0x3) == 0) {
		num += 2;
		word >>= 2;
	}
	if ((word & 0x1) == 0)
		num += 1;
	return num;
}

#define ffz(x) __ffs(~(x))

/*
 * This has an "evm_" prefix since without the prefix it clashes with the
 * non-existant-yet-declared "find_next_zero_bit" in
 * arch/arm/include/asm/bitops.h.
 */
unsigned long evm_find_next_zero_bit(const unsigned long *addr, unsigned long size, unsigned long offset)
{
	const unsigned long *p = addr + BITOP_WORD(offset);
	unsigned long result = offset & ~(BITS_PER_LONG-1);
	unsigned long tmp;

	if (offset >= size)
		return size;
	size -= result;
	offset %= BITS_PER_LONG;
	if (offset) {
		tmp = *(p++);
		tmp |= ~0UL >> (BITS_PER_LONG - offset);
		if (size < BITS_PER_LONG)
		goto found_first;
		if (~tmp)
			goto found_middle;
		size -= BITS_PER_LONG;
		result += BITS_PER_LONG;
	}
	while (size & ~(BITS_PER_LONG-1)) {
		if (~(tmp = *(p++)))
			goto found_middle;
		result += BITS_PER_LONG;
		size -= BITS_PER_LONG;
	}
	if (!size)
		return result;
	tmp = *p;

found_first:
	tmp |= ~0UL << size;
	if (tmp == ~0UL)        /* Are any bits zero? */
		return result + size;   /* Nope. */
found_middle:
	return result + ffz(tmp);
}

unsigned long find_next_bit(const unsigned long *addr, unsigned long size,
				unsigned long offset)
{
	const unsigned long *p = addr + BITOP_WORD(offset);
	unsigned long result = offset & ~(BITS_PER_LONG-1);
	unsigned long tmp;

	if (offset >= size)
		return size;
	size -= result;
	offset %= BITS_PER_LONG;
	if (offset) {
		tmp = *(p++);
		tmp &= (~0UL << offset);
		if (size < BITS_PER_LONG)
		goto found_first;
		if (tmp)
		goto found_middle;
		size -= BITS_PER_LONG;
		result += BITS_PER_LONG;
	}
	while (size & ~(BITS_PER_LONG-1)) {
		if ((tmp = *(p++)))
			goto found_middle;
		result += BITS_PER_LONG;
		size -= BITS_PER_LONG;
	}
	if (!size)
		return result;
	tmp = *p;

found_first:
	tmp &= (~0UL >> (BITS_PER_LONG - size));
	if (tmp == 0UL)         /* Are any bits set? */
		return result + size;   /* Nope. */
found_middle:
	return result + __ffs(tmp);
}

unsigned long bitmap_find_next_zero_area(unsigned long *map,
                                         unsigned long size,
                                         unsigned long start,
                                         unsigned int nr,
                                         unsigned long align_mask)
{
	unsigned long index, end, i;
again:
	index = evm_find_next_zero_bit(map, size, start);

	/* Align allocation */
	index = (index + align_mask) & ~align_mask;

	end = index + nr;
	if (end > size)
		return end;
	i = find_next_bit(map, end, index);
	if (i < end) {
		start = i + 1;
		goto again;
	}
	return index;
}

void *alloc_mem(unsigned long len, unsigned long align)
{
	unsigned long mask;
	unsigned long pageno;
	int count;

	count = ((len + (PAGE_SIZE - 1)) & ~(PAGE_SIZE - 1)) >> PAGE_SHIFT;
	mask = (1 << align) - 1;
	pageno = bitmap_find_next_zero_area(mem_bitmap, mem_count, 0, count,
	                                    mask);
	debug("alloc_mem: count %d mask %#lx pageno %#lx\n", count, mask,
	       pageno);

	if (pageno >= mem_count)
		return NULL;

	bitmap_set(mem_bitmap, pageno, count);
	return (void *)(mem_base + (pageno << PAGE_SHIFT));
}

unsigned int config_pagetable(unsigned int virt, unsigned int phys, unsigned int len)
{
	unsigned int index = virt >> PGT_L1_DESC_SECTION_INDEX_SHIFT;
	unsigned int l = len;
	unsigned int desc;

	while (l > 0) {
		desc = (phys & PGT_L1_DESC_SECTION_MASK) | PGT_L1_DESC_SECTION;
		page_table[index++] = desc;

		l -= PGT_SECTION_SIZE;
		phys += PGT_SECTION_SIZE;
	}

	return len;
}

static void config_iommu(void)
{
	u32 reg = 0;

	debug("config_iommu: page_table %p\n", page_table);

	/* Clear the entire pagetable location before programming the address into
	 * the MMU */
	memset(page_table,0x00,PAGE_TABLE_SIZE);

	__raw_writel((int)page_table, MMU_TTB);
	reg = __raw_readl(MMU_GPR);
	__raw_writel(reg | 0x1, MMU_GPR); /* enable bus-error back */

	/*
	 * Enable the MMU IRQs during MMU programming for the late attach case.
	 * This is to allow the MMU fault to be detected by the kernel.
	 */
	__raw_writel(0x1E, MMU_IRQENABLE); /* MULTIHITFAULT|EMMUMISS|TRANSLATIONFAULT|TABLEWALKFAULT */

	__raw_writel(0x6, MMU_CNTL);	/* emutlbupdate|TWLENABLE|MMUENABLE */
}

void reset_ipu(void)
{

#ifdef CONFIG_LATE_ATTACH_BOOTIPU2
	/* bring the IPU2 out of reset */
	__raw_writel(0x0, RM_IPU2_RSTCTRL);

	/* check module is functional or not */
	while(((__raw_readl(RM_IPU2_RSTST)&0x7)!=0x7));
	__raw_writel(0x7, RM_IPU2_RSTST);
#endif
}

void enable_ipu(void)
{
	u32 reg;

	/* enable CORE domain transitions */
	__raw_writel(0x2, CM_CAM_CLKSTCTRL);
	__raw_writel(0x2, CM_L3INIT_CLKSTCTRL);
	__raw_writel(0x2, CM_GMAC_CLKSTCTRL);
	__raw_writel(0x2, CM_EMIF_CLKSTCTRL);
	__raw_writel(0x2, CM_L4CFG_CLKSTCTRL);
	__raw_writel(0x2, CM_DMA_CLKSTCTRL);
	__raw_writel(0x2, CM_COREAON_CLKSTCTRL);
	__raw_writel(0x2, CM_DSS_CLKSTCTRL);
#if defined(CONFIG_LATE_ATTACH_BOOTIPU2)
	__raw_writel(0x2, CM_L4PER_CLKSTCTRL);
#endif
	/* enable power domain transitions (sw_wkup mode) */
#ifdef CONFIG_LATE_ATTACH_BOOTIPU2
	__raw_writel(0x2, CM_IPU2_CLKSTCTRL);
#endif
	__raw_writel(0x2, CM_IPU_CLKSTCTRL);
	__raw_writel(0x2, CM_VPE_CLKSTCTRL);

	/* Enable IPU module peripherals */
	reg = __raw_readl(CM_CAM_VIP1_CLKCTRL);
	__raw_writel((reg & ~0x00000003)|0x1, CM_CAM_VIP1_CLKCTRL);
	reg = __raw_readl(CM_CAM_VIP2_CLKCTRL);
	__raw_writel((reg & ~0x00000003)|0x1, CM_CAM_VIP2_CLKCTRL);
	reg = __raw_readl(CM_CAM_VIP3_CLKCTRL);
	__raw_writel((reg & ~0x00000003)|0x1, CM_CAM_VIP3_CLKCTRL);
	reg = __raw_readl(CM_L3INSTR_L3_MAIN_2_CLKCTRL);
	__raw_writel((reg & ~0x00000003)|0x1, CM_L3INSTR_L3_MAIN_2_CLKCTRL);
	reg = __raw_readl(CM_L4PER_I2C1_CLKCTRL);
	__raw_writel((reg & ~0x00000003)|0x2, CM_L4PER_I2C1_CLKCTRL);
	reg = __raw_readl(CM_L4PER_I2C2_CLKCTRL);
	__raw_writel((reg & ~0x00000003)|0x2, CM_L4PER_I2C2_CLKCTRL);
	reg = __raw_readl(CM_VPE_VPE_CLKCTRL);
	__raw_writel((reg & ~0x00000003)|0x1, CM_VPE_VPE_CLKCTRL);

# ifdef CONFIG_LATE_ATTACH_BOOTIPU2
	/* Using SYS CLK2 as the clock source */
	reg = __raw_readl(CM_L4PER_TIMER3_CLKCTRL);
	__raw_writel((reg & ~0x0F000003)|0x02000002, CM_L4PER_TIMER3_CLKCTRL);
# endif

# ifdef CONFIG_LATE_ATTACH_BOOTIPU2
	/* Using SYS_CLK1_32K_CLK as the clock source for both the watch dog
	 * timers. IPU will eventually configure these timers to run from
	 * SYS_CLK2. If we use SYS_CLK2 from the boot loader, the timer will
	 * overflow and trigger a watchdog interrupt even before the kernel has
	 * a chance to connect to IPU
	 */

	reg = __raw_readl(CM_L4PER_TIMER4_CLKCTRL);
	__raw_writel((reg & ~0x0F000003)|0x01000002, CM_L4PER_TIMER4_CLKCTRL);

	reg = __raw_readl(CM_L4PER_TIMER9_CLKCTRL);
	__raw_writel((reg & ~0x0F000003)|0x01000002, CM_L4PER_TIMER9_CLKCTRL);
# endif

	/* enable DSS */
	reg = __raw_readl(CTRL_CORE_CONTROL_IO_2);
	__raw_writel((reg | 0x1), CTRL_CORE_CONTROL_IO_2);
	reg = __raw_readl(CM_DSS_DSS_CLKCTRL);
	__raw_writel(((reg & ~0x00000003) | 0x00003F00 | 0x2), CM_DSS_DSS_CLKCTRL);

	/* checking if DSS is enabled */
	while ((__raw_readl(CM_DSS_DSS_CLKCTRL) & 0x00030000) != 0);

#ifdef CONFIG_LATE_ATTACH_BOOTIPU2
	/* enable IPU2 clocks / hw_auto mode */
	reg = __raw_readl(CM_IPU2_IPU2_CLKCTRL);
	__raw_writel(((reg & ~0x00000003) | 0x01000000 | 0x1), CM_IPU2_IPU2_CLKCTRL);

	/* checking if IPU2 module is enabled */
	while ((__raw_readl(CM_IPU2_IPU2_CLKCTRL) & 0x00030000) == 0x00030000);

	/* checking if clocks are enabled */
	while (((__raw_readl(CM_IPU2_CLKSTCTRL) & 0x100) >> 8) != 1);
#endif
}

void ipu_systemReset(void)
{

#ifdef CONFIG_LATE_ATTACH_BOOTIPU2
	/*Assert the IPU2 - CPU0,CPU1 & MMU,cache*/
	__raw_writel(0x7, RM_IPU2_RSTCTRL);

	/* Bring the IPU Unicache/MMU out of reset*/
	__raw_writel(0x7, RM_IPU2_RSTST);
	__raw_writel(0x3, RM_IPU2_RSTCTRL);

	while((__raw_readl(RM_IPU2_RSTST) & 0x4) != 0x4);
#endif
}

extern int valid_elf_image(unsigned long addr);
extern unsigned long load_elf_image_phdr_rproc(unsigned long addr);

u32 spl_boot_ipu(void)
{
	u32 load_elf_status=0;
#if defined(CONFIG_LATE_ATTACH_BOOTIPU2)
	/* Enable IPU clocks */
	enable_ipu();
	ipu_systemReset();
	config_iommu();
#endif
	if (valid_elf_image(IPU_LOAD_ADDR)) {
		load_elf_status = load_elf_image_phdr_rproc(IPU_LOAD_ADDR);
		if(load_elf_status == 0){
			printf("load_elf_image_phdr returned error\n");
			return 1;
		}
		reset_ipu();
		return 0;
	} else {
		printf("Not a valid elf image at 0x%x\n", IPU_LOAD_ADDR);
	}
	return 1;
}
#endif

/*
 * Adjust I/O delays on the Tx control and data lines of each MAC port. This
 * is a workaround in order to work properly with the DP83865 PHYs on the EVM.
 * In 3COM RGMII mode this PHY applies it's own internal clock delay, so we
 * essentially need to counteract the DRA7xx internal delay, and we do this
 * by delaying the control and data lines. If not using this PHY, you probably
 * don't need to do this stuff!
 */
static void dra7xx_adj_io_delay(const struct io_delay *io_dly)
{
	int i = 0;
	u32 reg_val;
	u32 delta;
	u32 coarse;
	u32 fine;

	writel(CFG_IO_DELAY_UNLOCK_KEY, CFG_IO_DELAY_LOCK);

	while(io_dly[i].addr) {
		writel(CFG_IO_DELAY_ACCESS_PATTERN & ~CFG_IO_DELAY_LOCK_MASK,
		       io_dly[i].addr);
		delta = io_dly[i].dly;
		reg_val = readl(io_dly[i].addr) & 0x3ff;
		coarse = ((reg_val >> 5) & 0x1F) + ((delta >> 5) & 0x1F);
		coarse = (coarse > 0x1F) ? (0x1F) : (coarse);
		fine = (reg_val & 0x1F) + (delta & 0x1F);
		fine = (fine > 0x1F) ? (0x1F) : (fine);
		reg_val = CFG_IO_DELAY_ACCESS_PATTERN |
				CFG_IO_DELAY_LOCK_MASK |
				((coarse << 5) | (fine));
		writel(reg_val, io_dly[i].addr);
		i++;
	}

	writel(CFG_IO_DELAY_LOCK_KEY, CFG_IO_DELAY_LOCK);
}

/**
 * @brief board_init
 *
 * @return 0
 */
int board_init(void)
{
	gpmc_init();
	gd->bd->bi_boot_params = (0x80000000 + 0x100); /* boot param addr */

	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	if (omap_revision() == DRA722_ES1_0)
		setenv("board_name", "dra72x");
	else
		setenv("board_name", "dra7xx");
#endif
	init_sata(0);
	return 0;
}

static void do_set_mux32(u32 base,
			 struct pad_conf_entry const *array, int size)
{
	int i;
	struct pad_conf_entry *pad = (struct pad_conf_entry *)array;

	for (i = 0; i < size; i++, pad++)
		writel(pad->val, base + pad->offset);
}

void set_muxconf_regs_essential(void)
{
	do_set_mux32((*ctrl)->control_padconf_core_base,
		     core_padconf_array_essential,
		     sizeof(core_padconf_array_essential) /
		     sizeof(struct pad_conf_entry));
}

#if !defined(CONFIG_SPL_BUILD) && defined(CONFIG_GENERIC_MMC)
int board_mmc_init(bd_t *bis)
{
	omap_mmc_init(0, 0, 0, -1, -1);
	omap_mmc_init(1, 0, 0, -1, -1);
	return 0;
}
#endif

#ifdef CONFIG_USB_DWC3
static struct dwc3_device usb_otg_ss1 = {
	.maximum_speed = USB_SPEED_SUPER,
	.base = DRA7_USB_OTG_SS1_BASE,
	.needs_fifo_resize = false,
	.index = 0,
};

static struct dwc3_omap_device usb_otg_ss1_glue = {
	.base = (void *)DRA7_USB_OTG_SS1_GLUE_BASE,
	.utmi_mode = DWC3_OMAP_UTMI_MODE_SW,
	.vbus_id_status = OMAP_DWC3_VBUS_VALID,
	.index = 0,
};

static struct ti_usb_phy_device usb_phy1_device = {
	.pll_ctrl_base = (void *)DRA7_USB3_PHY1_PLL_CTRL,
	.usb2_phy_power = (void *)DRA7_USB2_PHY1_POWER,
	.usb3_phy_power = (void *)DRA7_USB3_PHY1_POWER,
	.index = 0,
};

static struct dwc3_device usb_otg_ss2 = {
	.maximum_speed = USB_SPEED_SUPER,
	.base = DRA7_USB_OTG_SS2_BASE,
	.needs_fifo_resize = false,
	.index = 1,
};

static struct dwc3_omap_device usb_otg_ss2_glue = {
	.base = (void *)DRA7_USB_OTG_SS2_GLUE_BASE,
	.utmi_mode = DWC3_OMAP_UTMI_MODE_SW,
	.vbus_id_status = OMAP_DWC3_VBUS_VALID,
	.index = 1,
};

static struct ti_usb_phy_device usb_phy2_device = {
	.usb2_phy_power = (void *)DRA7_USB2_PHY2_POWER,
	.index = 1,
};

int board_usb_init(int index, enum usb_init_type init)
{
	switch (index) {
	case 0:
		if (init == USB_INIT_DEVICE) {
			usb_otg_ss1.dr_mode = USB_DR_MODE_PERIPHERAL;
			usb_otg_ss1_glue.vbus_id_status = OMAP_DWC3_VBUS_VALID;
		} else {
			usb_otg_ss1.dr_mode = USB_DR_MODE_HOST;
			usb_otg_ss1_glue.vbus_id_status = OMAP_DWC3_ID_GROUND;
		}

		ti_usb_phy_uboot_init(&usb_phy1_device);
		dwc3_omap_uboot_init(&usb_otg_ss1_glue);
		dwc3_uboot_init(&usb_otg_ss1);
		break;
	case 1:
		if (init == USB_INIT_DEVICE) {
			usb_otg_ss2.dr_mode = USB_DR_MODE_PERIPHERAL;
			usb_otg_ss2_glue.vbus_id_status = OMAP_DWC3_VBUS_VALID;
		} else {
			usb_otg_ss2.dr_mode = USB_DR_MODE_HOST;
			usb_otg_ss2_glue.vbus_id_status = OMAP_DWC3_ID_GROUND;
		}

		ti_usb_phy_uboot_init(&usb_phy2_device);
		dwc3_omap_uboot_init(&usb_otg_ss2_glue);
		dwc3_uboot_init(&usb_otg_ss2);
		break;
	default:
		printf("Invalid Controller Index\n");
	}

	return 0;
}

int board_usb_cleanup(int index, enum usb_init_type init)
{
	switch (index) {
	case 0:
	case 1:
		ti_usb_phy_uboot_exit(index);
		dwc3_uboot_exit(index);
		dwc3_omap_uboot_exit(index);
		break;
	default:
		printf("Invalid Controller Index\n");
	}
	return 0;
}

int usb_gadget_handle_interrupts(void)
{
	u32 status;

	status = dwc3_omap_uboot_interrupt_status(0);
	if (status)
		dwc3_uboot_handle_interrupt(0);

	return 0;
}
#endif

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_OS_BOOT)
int spl_start_uboot(void)
{
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_relocate_spec();
	if (getenv_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif

#ifdef CONFIG_DRIVER_TI_CPSW

/* Delay value to add to calibrated value */
#define RGMII0_TXCTL_DLY_VAL		((0x3 << 5) + 0x8)
#define RGMII0_TXD0_DLY_VAL		((0x3 << 5) + 0x8)
#define RGMII0_TXD1_DLY_VAL		((0x3 << 5) + 0x2)
#define RGMII0_TXD2_DLY_VAL		((0x4 << 5) + 0x0)
#define RGMII0_TXD3_DLY_VAL		((0x4 << 5) + 0x0)
#define VIN2A_D13_DLY_VAL		((0x3 << 5) + 0x8)
#define VIN2A_D17_DLY_VAL		((0x3 << 5) + 0x8)
#define VIN2A_D16_DLY_VAL		((0x3 << 5) + 0x2)
#define VIN2A_D15_DLY_VAL		((0x4 << 5) + 0x0)
#define VIN2A_D14_DLY_VAL		((0x4 << 5) + 0x0)

extern u32 *const omap_si_rev;

static void cpsw_control(int enabled)
{
	/* VTP can be added here */

	return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 2,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 3,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 2,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};

int board_eth_init(bd_t *bis)
{
	int ret;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;
	uint32_t ctrl_val;
	const struct io_delay io_dly[] = {
		{CFG_RGMII0_TXCTL, RGMII0_TXCTL_DLY_VAL},
		{CFG_RGMII0_TXD0, RGMII0_TXD0_DLY_VAL},
		{CFG_RGMII0_TXD1, RGMII0_TXD1_DLY_VAL},
		{CFG_RGMII0_TXD2, RGMII0_TXD2_DLY_VAL},
		{CFG_RGMII0_TXD3, RGMII0_TXD3_DLY_VAL},
		{CFG_VIN2A_D13, VIN2A_D13_DLY_VAL},
		{CFG_VIN2A_D17, VIN2A_D17_DLY_VAL},
		{CFG_VIN2A_D16, VIN2A_D16_DLY_VAL},
		{CFG_VIN2A_D15, VIN2A_D15_DLY_VAL},
		{CFG_VIN2A_D14, VIN2A_D14_DLY_VAL},
		{0}
	};

	/* Adjust IO delay for RGMII tx path */
	dra7xx_adj_io_delay(io_dly);

	/* try reading mac address from efuse */
	mac_lo = readl((*ctrl)->control_core_mac_id_0_lo);
	mac_hi = readl((*ctrl)->control_core_mac_id_0_hi);
	mac_addr[0] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = mac_hi & 0xFF;
	mac_addr[3] = (mac_lo & 0xFF0000) >> 16;
	mac_addr[4] = (mac_lo & 0xFF00) >> 8;
	mac_addr[5] = mac_lo & 0xFF;

	if (!getenv("ethaddr")) {
		printf("<ethaddr> not set. Validating first E-fuse MAC\n");

		if (is_valid_ether_addr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

	mac_lo = readl((*ctrl)->control_core_mac_id_1_lo);
	mac_hi = readl((*ctrl)->control_core_mac_id_1_hi);
	mac_addr[0] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = mac_hi & 0xFF;
	mac_addr[3] = (mac_lo & 0xFF0000) >> 16;
	mac_addr[4] = (mac_lo & 0xFF00) >> 8;
	mac_addr[5] = mac_lo & 0xFF;

	if (!getenv("eth1addr")) {
		if (is_valid_ether_addr(mac_addr))
			eth_setenv_enetaddr("eth1addr", mac_addr);
	}

	ctrl_val = readl((*ctrl)->control_core_control_io1) & (~0x33);
	ctrl_val |= 0x22;
	writel(ctrl_val, (*ctrl)->control_core_control_io1);

	if (*omap_si_rev == DRA722_ES1_0)
		cpsw_data.active_slave = 1;

	ret = cpsw_register(&cpsw_data);
	if (ret < 0)
		printf("Error %d registering CPSW switch\n", ret);

	return ret;
}
#endif

#ifdef CONFIG_BOARD_EARLY_INIT_F
/* VTT regulator enable */
static inline void vtt_regulator_enable(void)
{
	if (omap_hw_init_context() == OMAP_INIT_CONTEXT_UBOOT_AFTER_SPL)
		return;

	/* Do not enable VTT for DRA722 */
	if (omap_revision() == DRA722_ES1_0)
		return;

	/*
	 * EVM Rev G and later use gpio7_11 for DDR3 termination.
	 * This is safe enough to do on older revs.
	 */
	gpio_request(GPIO_DDR_VTT_EN, "ddr_vtt_en");
	gpio_direction_output(GPIO_DDR_VTT_EN, 1);
}

int board_early_init_f(void)
{
	vtt_regulator_enable();
	return 0;
}
#endif
