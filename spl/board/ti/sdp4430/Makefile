#
# (C) Copyright 2006-2007
# Stefan Roese, DENX Software Engineering, sr@denx.de.
#
# (C) Copyright 2008
# Guennadi Liakhovetki, DENX Software Engineering, <lg@denx.de>
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston,
# MA 02111-1307 USA
#
splobj	:= $(OBJTREE)/spl/
include $(TOPDIR)/config.mk
SOBJS =
COBJS =

# serial console
$(obj)eabi_compat.c:
	@rm -f $@
	@ln -s $(TOPDIR)/arch/arm/lib/eabi_compat.c $@

$(obj)string.c:
	@rm -f $@
	@ln -s $(TOPDIR)/lib/string.c $@

$(obj)vsprintf.c:
	@rm -f $@
	@ln -s $(TOPDIR)/lib/vsprintf.c $@

$(obj)console.c:
	@rm -f $@
	@ln -s $(TOPDIR)/common/console.c $@

$(obj)stdio.c:
	@rm -f $@
	@ln -s $(TOPDIR)/common/stdio.c $@

$(obj)serial.c:
	@rm -f $@
	@ln -s $(TOPDIR)/drivers/serial/serial.c $@

$(obj)ns16550.c:
	@rm -f $@
	@ln -s $(TOPDIR)/drivers/serial/ns16550.c $@

$(obj)div64.c:
	@rm -f $@
	@ln -s $(TOPDIR)/lib/div64.c $@

$(obj)ctype.c:
	@rm -f $@
	@ln -s $(TOPDIR)/lib/ctype.c $@

COBJS	+= serial.o ns16550.o string.o vsprintf.o console.o stdio.o
COBJS	+= ctype.o eabi_compat.o div64.o

# armv7
$(obj)start.S:
	@rm -f $@
	@ln -s $(TOPDIR)/arch/arm/cpu/armv7/start.S $@

$(obj)syslib.c:
	@rm -f $@
	@ln -s $(TOPDIR)/arch/arm/cpu/armv7/syslib.c $@

SOBJS	+= start.o
COBJS	+= syslib.o

# omap-common
$(obj)timer.c:
	@rm -f $@
	@ln -s $(TOPDIR)/arch/arm/cpu/armv7/omap-common/timer.c $@

$(obj)reset.S:
	@rm -f $@
	@ln -s $(TOPDIR)/arch/arm/cpu/armv7/omap-common/reset.S $@

$(obj)spl-omap.c:
	@rm -f $@
	@ln -s $(TOPDIR)/spl/board/ti/spl-omap.c $@

SOBJS	+= reset.o
COBJS	+= timer.o spl-omap.o

$(OBJTREE)/MLO:	$(splobj)u-boot-spl.bin
	$(OBJTREE)/tools/mkimage -T omapimage \
		-a $(CONFIG_SYS_SPL_TEXT_BASE) -d $< $@

LDSCRIPT_SOURCE = $(TOPDIR)/spl/board/ti/spl-omap.lds

# omap4
$(obj)lowlevel_init.S:
	@rm -f $@
	@ln -s $(TOPDIR)/arch/arm/cpu/armv7/omap4/lowlevel_init.S $@

$(obj)omap4_mux_data.h:
	@rm -f $@
	@ln -s $(TOPDIR)/arch/arm/cpu/armv7/omap4/omap4_mux_data.h $@

$(obj)board.c:$(obj)omap4_mux_data.h
	@rm -f $@
	@ln -s $(TOPDIR)/arch/arm/cpu/armv7/omap4/board.c $@

$(obj)clocks.c:
	@rm -f $@
	@ln -s $(TOPDIR)/arch/arm/cpu/armv7/omap4/clocks.c $@

SOBJS	+= lowlevel_init.o
COBJS	+= board.o clocks.o

# rules
LDPPFLAGS += -include $(TOPDIR)/include/config.h
LDSCRIPT = $(splobj)u-boot-spl-generated.lds
$(LDSCRIPT): $(LDSCRIPT_SOURCE)
	$(CPP) $(CPPFLAGS) $(LDPPFLAGS) -ansi -D__ASSEMBLY__ -P - <$^ >$@
LDFLAGS	= -Bstatic -T $(LDSCRIPT) \
	$(PLATFORM_LDFLAGS) --gc-sections
AFLAGS	+= -DCONFIG_PRELOADER -Os -ffixed-r8
CFLAGS	+= -DCONFIG_PRELOADER -Os -ffixed-r8 -ffunction-sections \
	-fdata-sections -march=armv7-a -mthumb
PLATFORM_LIBGCC = -L $(shell dirname `$(CC) $(CFLAGS) \
		-print-libgcc-file-name`) -lgcc

$(obj)%.o:	$(obj)%.S
	$(CC) $(AFLAGS) -c -o $@ $<

$(obj)%.o:	$(obj)%.c
	$(CC) $(CFLAGS) -c -o $@ $<

SRCS	:= $(addprefix $(obj),$(SOBJS:.o=.S) $(COBJS:.o=.c))
OBJS	:= $(addprefix $(obj),$(SOBJS) $(COBJS))
__OBJS	:= $(SOBJS) $(COBJS)
LNDIR	:= $(OBJTREE)/spl/board/$(BOARDDIR)

$(splobj)u-boot-spl.bin:	$(splobj)u-boot-spl
	$(OBJCOPY) ${OBJCFLAGS} -O binary $< $@

$(splobj)u-boot-spl:	$(OBJS) $(LDSCRIPT)
	cd $(LNDIR) && $(LD) $(LDFLAGS) $(__OBJS) \
		$(PLATFORM_LIBGCC) \
		-Map $(splobj)u-boot-spl.map \
		-o $(splobj)u-boot-spl

ALL	= $(OBJTREE)/MLO
all:	$(obj).depend $(ALL)

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend
