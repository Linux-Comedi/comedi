
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

obj-$(CONFIG_COMEDI_VD_TIMER)		+= vd_timer.o
obj-$(CONFIG_COMEDI_VD_DDS)		+= vd_dds.o

obj-m += $(obj-y)

#L_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-y)))
#LX_OBJS	:= $(sort $(filter     $(export-objs), $(obj-y)))
#MI_OBJS	:= $(sort $(filter-out $(export-objs), $(obj-y)))
#MIX_OBJS	:= $(sort $(filter     $(export-objs), $(obj-y)))
M_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-m)))
MX_OBJS		:= $(sort $(filter     $(export-objs), $(obj-m)))



include $(TOPDIR)/Rules.make


realtime.o:	$(obj-y)
	$(LD) -r -o $@ $(obj-y)


