
module_PROGRAMS = kcomedilib.ko

kcomedilib_ko_SOURCES = data.c ksyms.c dio.c kcomedilib_main.c get.c
kcomedilib_ko_CFLAGS = $(COMEDI_CFLAGS) $(LINUX_CFLAGS) $(RTAI_CFLAGS) $(RTLINUX_CFLAGS)
kcomedilib_ko_LINK = $(top_builddir)/modtool --link -o $@ -i ../.mods/comedi.o.symvers

#ksyms.o: ksyms.c
#	$(CC) -o $@ ksyms.c $(CFLAGS) $(COMEDI_CFLAGS) $(LINUX_CFLAGS) -DEXPORT_SYMTAB


