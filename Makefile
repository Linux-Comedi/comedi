VERSION = 0
PATCHLEVEL = 7
SUBLEVEL = 62
EXTRAVERSION = -cvs

PROJECT = COMEDI
project = comedi

SUBDIRS := comedi

include ./Makefile.modbuild

INSTALLDIR_DEV=$(DESTDIR)/usr/realtime/

install: modules_install

install_dev:
	install -d $(INSTALLDIR_DEV)/include
	install -m 644 include/linux/comedi.h $(INSTALLDIR_DEV)/include
	install -m 644 include/linux/comedilib.h $(INSTALLDIR_DEV)/include

dev:
	-mknod -m 666 /dev/comedi0 c 98 0
	-mknod -m 666 /dev/comedi1 c 98 1
	-mknod -m 666 /dev/comedi2 c 98 2
	-mknod -m 666 /dev/comedi3 c 98 3

drivers.txt:
	(for each in comedi/drivers/*.c;do scripts/dump_doc $$each;done >Documentation/comedi/drivers.txt)
	scripts/doc_devlist Documentation/comedi/drivers.txt >Documentation/comedi/devices.txt

check:
	(cd comedi/drivers;for each in *.c;do ../../scripts/check_driver $$each;done >../../drivers.check)
	(grep '^.:' drivers.check >drivers.summary)
	

