
#LINUXDIR=/home/ds/linux/linux-2.4.16-rthal5

all:
	$(MAKE) -C $(LINUXDIR) SUBDIRS=$(PWD) modules

modules:
	echo LINUXDIR=$(LINUXDIR) >flags
	echo ARCH=$(ARCH) >>flags
	echo AFLAGS="$(AFLAGS)" >>flags
	echo LDFLAGS="$(LDFLAGS)" >>flags
	echo ARFLAGS="$(ARFLAGS)" >>flags
	echo CROSS_COMPILE="$(CROSS_COMPILE)" >>flags
	echo KERNELRELEASE="$(KERNELRELEASE)" >>flags
# This is evil Makefile hackery.  Please look the other way.
ifneq (,$(findstring __KERNEL__,$(CC)))
	echo CFLAGS="$(wordlist 2,100,$(CC)) $(CFLAGS)" >>flags
	echo CC=$(word 1,$(CC)) >>flags
else
	echo CFLAGS="$(CFLAGS)" >>flags
	echo CC="$(CC)" >>flags
endif
	echo LD="$(LD)" >>flags
	echo AS="$(AS)" >>flags


