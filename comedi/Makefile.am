
SUBDIRS = . kcomedilib drivers

CLEANFILES = Modules.symvers

if CONFIG_KBUILD
module_PROGRAMS =
#ignore errors from kernel's make modules in all-local, to avoid failure of make install
#on nfs filesystem with root_squash
all-local: comedi_kbuild.inc
	-$(MAKE) -I@abs_builddir@ -C $(LINUX_DIR) M=@abs_builddir@ CC="$(LINUX_CC) -I@abs_top_srcdir@/ \
		-I@abs_top_srcdir@/include $(RTAI_CFLAGS) $(RTLINUX_CFLAGS)" V=1 modules
install-data-local: comedi_kbuild.inc
	$(MAKE) -I@abs_builddir@ -C $(LINUX_DIR) M=@abs_builddir@ CC="$(LINUX_CC) -I@abs_top_srcdir@/ \
		-I@abs_top_srcdir@/include $(RTAI_CFLAGS) $(RTLINUX_CFLAGS)" INSTALL_MOD_DIR="comedi" \
		V=1 modules_install
clean-local: comedi_kbuild.inc
	$(MAKE) -I@abs_builddir@ -C $(LINUX_DIR) M=@abs_builddir@ CC="$(LINUX_CC) -I@abs_top_srcdir@/ \
		-I@abs_top_srcdir@/include $(RTAI_CFLAGS) $(RTLINUX_CFLAGS)" V=1 clean
else
module_PROGRAMS = comedi.ko
all-local:
install-data-local:
clean-local:
endif

EXTRA_DIST = \
 Kbuild \
 rt_pend_tq.h

if CONFIG_COMEDI_RT
RT_SOURCES = rt_pend_tq.c rt.c
else
RT_SOURCES =
endif

comedi_ko_SOURCES = \
 comedi_fops.c \
 proc.c \
 range.c \
 drivers.c \
 comedi_ksyms.c \
 $(RT_SOURCES)

comedi_ko_CFLAGS = $(COMEDI_CFLAGS) $(LINUX_CFLAGS) $(RTAI_CFLAGS) $(RTLINUX_CFLAGS)
comedi_ko_LINK = $(top_builddir)/modtool --link -o $@

#comedi_ksyms.o: comedi_ksyms.c
#	$(CC) -o $@ comedi_ksyms.c $(CFLAGS) $(COMEDI_CFLAGS) $(LINUX_CFLAGS) -DEXPORT_SYMTAB

clean-modulePROGRAMS:
	-test -z "$(module_PROGRAMS)" || rm -f $(module_PROGRAMS)
	-rm -rf .mods

comedi_kbuild.inc: Makefile
	echo > $@
if CONFIG_COMEDI_RT
	echo CONFIG_COMEDI_RT=y >> $@
	echo CONFIG_COMEDI_RT_MODULES=m >> $@
endif
if CONFIG_PCMCIA
	echo CONFIG_COMEDI_PCMCIA_MODULES=m >> $@
endif
if CONFIG_USB
	echo CONFIG_COMEDI_USB_MODULES=m >> $@
endif
