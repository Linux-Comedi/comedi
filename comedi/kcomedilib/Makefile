
ALL_SUB_DIRS :=
MOD_SUB_DIRS :=
SUB_DIRS     :=
MOD_LIST_NAME := MISC_MODULES

EXTRA_CFLAGS := -I ../

ifeq ($(CONFIG_COMEDI_KLIB),y)
  O_TARGET := kcomedilib.o
  OX_OBJS := kcomedilib_main.o
  O_OBJS := data.o dio.o
else
  MI_OBJS := data.o dio.o
  MIX_OBJS := kcomedilib_main.o
  M_OBJS := kcomedilib.o
endif


include $(TOPDIR)/Rules.make

kcomedilib.o: $(MI_OBJS) $(MIX_OBJS)
	$(LD) -r -o $@ $(MI_OBJS) $(MIX_OBJS)

