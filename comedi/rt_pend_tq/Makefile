
ALL_SUB_DIRS :=
MOD_SUB_DIRS :=
SUB_DIRS     :=
MOD_LIST_NAME := MISC_MODULES

EXTRA_CFLAGS := -I ../

export-objs := 

obj-y :=
obj-m :=
obj-n :=
obj-  :=

obj-$(CONFIG_COMEDI_RT)		+= rt_pend_tq.o

obj-m += $(obj-y)

#L_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-y)))
#LX_OBJS	:= $(sort $(filter     $(export-objs), $(obj-y)))
#MI_OBJS	:= $(sort $(filter-out $(export-objs), $(obj-y)))
#MIX_OBJS	:= $(sort $(filter     $(export-objs), $(obj-y)))
M_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-m)))
MX_OBJS		:= $(sort $(filter     $(export-objs), $(obj-m)))



include $(TOPDIR)/Rules.make

ifdef CONFIG_COMEDI_RTL
#rt_pend_tq.o : rt_pend_tq_rtl.c

endif

ifdef CONFIG_COMEDI_RTAI
#rt_pend_tq.o : rt_pend_tq_rtai.c

endif



