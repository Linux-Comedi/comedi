
SUBDIRS = kcomedilib drivers

module_PROGRAMS = comedi.ko

EXTRA_DIST = \
 rt_pend_tq.h

comedi_ko_SOURCES = \
 comedi_fops.c \
 proc.c \
 range.c \
 drivers.c \
 comedi_ksyms.c

comedi_ko_CFLAGS = $(COMEDI_CFLAGS) $(LINUX_CFLAGS)
comedi_ko_LINK = $(top_builddir)/modtool --link -o $@

#ifsel(CONFIG_COMEDI_RT)
#  objlink(comedi.o rt_pend_tq.o rt.o)
#endif


#comedi_ksyms.o: comedi_ksyms.c
#	$(CC) -o $@ comedi_ksyms.c $(CFLAGS) $(COMEDI_CFLAGS) $(LINUX_CFLAGS) -DEXPORT_SYMTAB

