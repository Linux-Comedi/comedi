

SUB_DIRS     := drivers
ALL_SUB_DIRS := kcomedilib drivers realtime
MOD_SUB_DIRS := drivers
MOD_IN_SUB_DIRS :=

EXTRA_CFLAGS := -I .

ifneq ($(CONFIG_COMEDI_KLIB),n)
	MOD_SUB_DIRS += kcomedilib
	SUB_DIRS += kcomedilib
endif

ifeq ($(CONFIG_COMEDI_RT),y)
	MOD_SUB_DIRS += realtime
	SUB_DIRS += realtime
endif

MOD_LIST_NAME := MISC_MODULES

ifeq ($(CONFIG_COMEDI),y)
  O_TARGET := comedi.o
  O_OBJS := comedi_fops.o proc.o range.o drivers.o kvmem.o
  OX_OBJS := comedi_ksyms.o
else
  ifeq ($(CONFIG_COMEDI),m)
    M_OBJS := comedi.o
    MI_OBJS := comedi_fops.o proc.o range.o drivers.o kvmem.o
    MIX_OBJS := comedi_ksyms.o
  endif
endif


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



include $(TOPDIR)/Rules.make


ifeq ($(CONFIG_COMEDI),m)
comedi.o: $(MI_OBJS) $(MIX_OBJS)
	$(LD) -r -o $@ $(MI_OBJS) $(MIX_OBJS)
endif

