
module_PROGRAMS = kcomedilib.ko

kcomedilib_ko_SOURCES = data.c ksyms.o dio.c kcomedilib_main.c get.c
kcomedilib_ko_CFLAGS = $(COMEDI_CFLAGS) $(LINUX_CFLAGS)
kcomedilib_ko_LINK = $(top_builddir)/modtool --link -o $@

ksyms.o: ksyms.c
	$(CC) -o $@ ksyms.c $(CFLAGS) $(COMEDI_CFLAGS) $(LINUX_CFLAGS) -DEXPORT_SYMTAB


