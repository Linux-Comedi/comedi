

SUB_DIRS     := drivers
ALL_SUB_DIRS := kcomedilib drivers rt_pend_tq
MOD_SUB_DIRS := drivers
MOD_IN_SUB_DIRS :=

EXTRA_CFLAGS := -I .

ifneq ($(CONFIG_COMEDI_KLIB),)
	MOD_SUB_DIRS += kcomedilib
	SUB_DIRS += kcomedilib
endif

ifeq ($(CONFIG_COMEDI_RT),y)
  ifneq ($(CONFIG_COMEDI_PIRQ),y)
	MOD_SUB_DIRS += rt_pend_tq
	SUB_DIRS += rt_pend_tq
	EXTRA_OBJS := rt_pend_tq/rt_pend_tq.o
  endif
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



include $(TOPDIR)/Rules.make


ifeq ($(CONFIG_COMEDI),m)
comedi.o: $(MI_OBJS) $(MIX_OBJS) $(EXTRA_OBJS)
	$(LD) -r -o $@ $(MI_OBJS) $(MIX_OBJS) $(EXTRA_OBJS)
endif

