
SUBDIRS = . kcomedilib drivers

module_PROGRAMS = comedi.ko

EXTRA_DIST = \
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

