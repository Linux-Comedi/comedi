
SUBDIRS = comedi

EXTRA_DIST = Documentation include m4 scripts modtool.in

ACLOCAL_AMFLAGS = -I m4

dev:
	-for i in `seq 0 15`; do \
		mknod -m 666 /dev/comedi$$i c 98 $$i \
		; \
	done;

drivers.txt:
	(for each in comedi/drivers/*.c;do scripts/dump_doc $$each;done >Documentation/comedi/drivers.txt)
	scripts/doc_devlist Documentation/comedi/drivers.txt >Documentation/comedi/devices.txt

moo:
	(cd comedi/drivers;for each in *.c;do ../../scripts/check_driver $$each;done >../../drivers.check)
	(grep '^.:' drivers.check >drivers.summary)
	
install-data-hook:
	if [ -z "$(DESTDIR)" ] ; then \
		$(DEPMOD) -ae $(LINUX_KERNELRELEASE); \
	fi

