
obj-m += fake.o

$(obj)/fake.c: flags
	touch $(obj)/fake.c

.PHONY: flags
flags:
	echo LINUXDIR=$(LINUXDIR) >$(obj)/flags
	echo MODVERSIONS_H_PATH=$(LINUXDIR)/include/config/modversions.h >>$(obj)/flags
	echo ARCH=$(ARCH) >>$(obj)/flags
	echo AFLAGS="$(AFLAGS)" | sed 's_Iinclude_I"$(LINUXDIR)/include"_g'>>$(obj)/flags
	echo LDFLAGS= >>$(obj)/flags
	echo ARFLAGS="$(ARFLAGS)" >>$(obj)/flags
	echo CROSS_COMPILE="$(CROSS_COMPILE)" >>$(obj)/flags
	echo KERNELRELEASE="$(KERNELRELEASE)" >>$(obj)/flags
	echo CFLAGS="$(CFLAGS)" | sed 's_Iinclude_I"$(LINUXDIR)/include"_g'>>$(obj)/flags
	echo CC="$(CC)" >>$(obj)/flags
	echo LD="$(LD) $(LDFLAGS)" >>$(obj)/flags
	echo AS="$(AS)" >>$(obj)/flags


