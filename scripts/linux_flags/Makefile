
#LINUXDIR=/home/ds/linux/linux-2.4.16-rthal5

all:
	$(MAKE) -C $(LINUXDIR) SUBDIRS=$(PWD) modules

modules:
	echo LINUXDIR=$(LINUXDIR) >flags
	echo ARCH=$(ARCH) >>flags
	echo CFLAGS="$(CFLAGS)" >>flags
	echo AFLAGS="$(AFLAGS)" >>flags
	echo LDFLAGS="$(LDFLAGS)" >>flags
	echo ARFLAGS="$(ARFLAGS)" >>flags
	echo CROSS_COMPILE="$(CROSS_COMPILE)" >>flags
	echo KERNELRELEASE="$(KERNELRELEASE)" >>flags




