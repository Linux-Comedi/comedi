
SUBDIRS = comedi

EXTRA_DIST = Documentation include scripts autogen.sh \
	m4/as-linux.m4 \
	m4/as-modtool.m4 \
	m4/rtai.m4 \
	m4/rtlinux.m4

ACLOCAL_AMFLAGS = -I m4

dev:
	-for i in `seq 0 15`; do \
		mknod -m 666 /dev/comedi$$i c 98 $$i \
		; \
	done;

drivers.txt:
	(for each in comedi/drivers/*.c;do scripts/dump_doc $$each;done >Documentation/comedi/drivers.txt)
	scripts/doc_devlist Documentation/comedi/drivers.txt | sort >Documentation/comedi/devices.txt

moo:
	(cd comedi/drivers;for each in *.c;do ../../scripts/check_driver $$each;done >../../drivers.check)
	(grep '^.:' drivers.check >drivers.summary)
	
DISTCLEANFILES = modtool

