
ALL_SUB_DIRS :=
MOD_SUB_DIRS :=
SUB_DIRS     :=
MOD_LIST_NAME := MISC_MODULES

EXTRA_CFLAGS := -I ../

ifeq ($(CONFIG_COMEDI_RT),y)
MI_OBJS		+= rt_pend_tq.o
endif


include $(TOPDIR)/Rules.make

