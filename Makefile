
# Makefile for comedi

VERS1 = 0
VERS2 = 7
VERS3 = 46

INSTALLDIR=/usr

ifndef LINUXDIR
LINUXDIR = /usr/src/linux
endif

# define the following if you want to compile using RTL
# headers that aren't in the default location
#RTLDIR = /d/ds/cvs/rtlinux

# define the following if you want to compile using RTAI
# headers that aren't in the default location
#RTAIDIR = /home/ds/cvs/rtai

# define the following if you want to compile using PCMCIA
# headers
#PCMCIADIR = /d/ds/stuff/pcmcia-cs-3.1.15

TOPDIR := $(shell if [ "$$PWD" != "" ]; then echo $$PWD; else pwd; fi)

.EXPORT_ALL_VARIABLES:

CFLAGS = -Wall -O2 -Wstrict-prototypes
CFLAGS += -D__KERNEL__ -I $(LINUXDIR)/include -I $(TOPDIR)/include -I .

CONFIG_SHELL := sh

include $(LINUXDIR)/.config
ifdef PCMCIADIR
CONFIG_PCMCIA=y
endif

ifeq (.config,$(wildcard .config))
include .config
include .uts_version
all2:	modules comedi_config
else
all2:	config
endif

ifeq ($(CONFIG_SMP),y)
CFLAGS += -D__SMP__
endif

ifeq ($(CONFIG_COMEDI_RTL),y)
ifdef RTLDIR
CFLAGS += -I $(RTLDIR)/include
else
CFLAGS += -I /usr/include/rtlinux
endif
CFLAGS += -D__RTL__
endif

ifeq ($(CONFIG_COMEDI_RTAI),y)
ifdef RTAIDIR
CFLAGS += -I $(RTAIDIR)/include
else
CFLAGS += -I /usr/include/rtai
endif
endif

ifdef PCMCIADIR
CFLAGS += -I $(PCMCIADIR)/include
endif

SUBDIRS := comedi

DOCFILES= README INSTALL drivers `find doc -type f`

comedi_config: dummy
	$(MAKE) -C comedi_config

config:	dummy
	scripts/Configure

install:	dummy
ifeq (/lib/modules/${UTS_VERSION},$(wildcard /lib/modules/${UTS_VERSION}))
	install -d /lib/modules/${UTS_VERSION}/misc
	install modules/*.o /lib/modules/${UTS_VERSION}/misc
	/sbin/depmod -a ${UTS_VERSION}
else
	# ***
	# *** Could not install comedi.o into /lib/modules/${UTS_VERSION}/misc
	# *** Please install by hand.
	# ***
endif
#	install -m 755 comedi_config/comedi_config ${INSTALLDIR}/sbin
#	install -d ${INSTALLDIR}/include
#	(cd include;install -m 644 comedi.h ${INSTALLDIR}/include)
#	install man/comedi.7 ${INSTALLDIR}/man/man7
#	install man/comedi_config.8 ${INSTALLDIR}/man/man8
#	install -d ${INSTALLDIR}/doc/comedi
#	install ${DOCFILES} ${INSTALLDIR}/doc/comedi

lpr:	dummy
	find . -name '*.[chs]'|xargs enscript -2r -pit.ps

dev:	dummy
	-rm /dev/comedi*
	/bin/mknod /dev/comedi0 c 98 0
	/bin/mknod /dev/comedi1 c 98 1
	/bin/mknod /dev/comedi2 c 98 2
	/bin/mknod /dev/comedi3 c 98 3
	chown root.root /dev/comedi*
	chmod 666 /dev/comedi*


MODFLAGS += -DMODULE
ifeq ($(CONFIG_MODVERSIONS),y)
MODFLAGS += -DMODVERSIONS -include $(LINUXDIR)/include/linux/modversions.h
endif


modules:	$(patsubst %, _mod_%, $(SUBDIRS))

$(patsubst %, _mod_%, $(SUBDIRS)) : dummy
	-mkdir modules
	$(MAKE) -C $(patsubst _mod_%, %, $@) CFLAGS="$(CFLAGS) $(MODFLAGS)" MAKING_MODULES=1 modules

clean:
	rm -f core `find . -name '*.[oas]'`
	rm -f core `find . -name '.*.flags' -print`
	rm -f comedi/range.h comedi/mk_range comedi/range.def
	rm -f comedi_config/comedi_config

distclean:	clean
	rm -f .depend `find . -name .depend -print`
	rm -f core `find . \( -name '*.orig' -o -name '*.rej' -o -name '*~' \
		-o -name '*.bak' -o -name '#*#' -o -name '.*.orig' \
		-o -name '.*.rej' -o -name '.SUMS' -o -size 0 \) -print` TAGS
	-rm -rf modules
	rm -f .config .uts_version include/config.h

include $(TOPDIR)/Rules.make


dummy:
