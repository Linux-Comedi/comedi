

SUB_DIRS     := drivers
ALL_SUB_DIRS := kcomedilib drivers
MOD_SUB_DIRS := drivers
MOD_IN_SUB_DIRS :=

ifneq ($(CONFIG_COMEDI_KLIB),n)
	MOD_SUB_DIRS += kcomedilib
	SUB_DIRS += kcomedilib
endif

MOD_LIST_NAME := MISC_MODULES

M_OBJS := comedi.o
MI_OBJS := comedi_fops.o dummy.o proc.o range.o drivers.o kvmem.o
MIX_OBJS := comedi_ksyms.o


ifeq ($(CONFIG_COMEDI_REALTIME),y)
MI_OBJS += rt.o
endif

ifeq ($(CONFIG_COMEDI_RTL),y)
MI_OBJS += rtl.o
CFLAGS += -D__RTL__
endif

ifeq ($(CONFIG_COMEDI_RTAI),y)
MI_OBJS += rtai.o
endif

ifeq ($(CONFIG_COMEDI_RTL_V1),y)
MI_OBJS += rtl_v1.o
CFLAGS += -D__RTL__
endif

range-y := range.c
range-$(CONFIG_COMEDI_DT2811)		+= drivers/dt2811.c
range-$(CONFIG_COMEDI_DT2815)		+= drivers/dt2815.c
range-$(CONFIG_COMEDI_DT282x)		+= drivers/dt282x.c
range-$(CONFIG_COMEDI_DT3000)		+= drivers/dt3000.c
range-$(CONFIG_COMEDI_NI_ATMIO)		+= drivers/ni_mio_common.c
ifeq ($(CONFIG_COMEDI_NI_ATMIO),n)
range-$(CONFIG_COMEDI_NI_PCIMIO)	+= drivers/ni_mio_common.c
endif

range-$(CONFIG_COMEDI_PCL711)		+= drivers/pcl711.c
range-$(CONFIG_COMEDI_RTI800)		+= drivers/rti800.c
range-$(CONFIG_COMEDI_DAS1600)		+= drivers/das1600.c





include $(TOPDIR)/Rules.make


comedi.o: $(MI_OBJS) $(MIX_OBJS)
	$(LD) -r -o $@ $(MI_OBJS) $(MIX_OBJS)

range.h: mk_range $(TOPDIR)/.config
	-rm -f range.h
	set -e ; for i in $(range-y) $(range-m) ; do \
	grep -A 99999 -e '^--BEGIN-RANGE-DEFS--' $$i | \
	grep -B 99999 -e '^---END-RANGE-DEFS---' | \
	grep -v -e 'RANGE-DEFS' ; done >range.def
	./mk_range source <range.def >range.h
	./mk_range include <range.def >>range.h

mk_range: mk_range.c
	$(CC) -Wall mk_range.c -o mk_range

$(MIX_OBJS) $(MI_OBJS):	range.h


