

SUB_DIRS     := drivers realtime
ALL_SUB_DIRS := kcomedilib drivers
MOD_SUB_DIRS := drivers
MOD_IN_SUB_DIRS :=

ifneq ($(CONFIG_COMEDI_KLIB),n)
	MOD_SUB_DIRS += kcomedilib
	SUB_DIRS += kcomedilib
endif

ifeq ($(CONFIG_COMEDI_RT),y)
	MOD_SUB_DIRS += realtime
	SUB_DIRS += realtime
endif

MOD_LIST_NAME := MISC_MODULES

M_OBJS := comedi.o
MI_OBJS := comedi_fops.o dummy.o proc.o range.o drivers.o kvmem.o
MIX_OBJS := comedi_ksyms.o


ifeq ($(CONFIG_COMEDI_RT),y)
MI_OBJS += rt.o
endif

ifeq ($(CONFIG_COMEDI_RTL),y)
MI_OBJS += rtl.o
endif

ifeq ($(CONFIG_COMEDI_RTAI),y)
MI_OBJS += rtai.o
endif

ifeq ($(CONFIG_COMEDI_RTL_V1),y)
MI_OBJS += rtl_v1.o
endif

#range-y := range.c
#range-$(CONFIG_COMEDI_DT2801)		+= drivers/dt2801.c
#range-$(CONFIG_COMEDI_DT2811)		+= drivers/dt2811.c
#range-$(CONFIG_COMEDI_DT2815)		+= drivers/dt2815.c
#range-$(CONFIG_COMEDI_DT282x)		+= drivers/dt282x.c
#range-$(CONFIG_COMEDI_DT3000)		+= drivers/dt3000.c
#range-$(CONFIG_COMEDI_NI_ATMIO)		+= drivers/ni_mio_common.c
#ifndef CONFIG_COMEDI_NI_ATMIO
#range-$(CONFIG_COMEDI_NI_PCIMIO)	+= drivers/ni_mio_common.c
#endif

#range-$(CONFIG_COMEDI_PCL711)		+= drivers/pcl711.c
#range-$(CONFIG_COMEDI_RTI800)		+= drivers/rti800.c
#range-$(CONFIG_COMEDI_DAS1600)		+= drivers/das1600.c





include $(TOPDIR)/Rules.make


comedi.o: $(MI_OBJS) $(MIX_OBJS)
	$(LD) -r -o $@ $(MI_OBJS) $(MIX_OBJS)

