
ALL_SUB_DIRS :=
MOD_SUB_DIRS :=
SUB_DIRS     :=
MOD_LIST_NAME := MISC_MODULES

EXTRA_CFLAGS := -I ../

MI_OBJS := data.o dio.o
MIX_OBJS := kcomedilib_main.o
M_OBJS := kcomedilib.o


include $(TOPDIR)/Rules.make

kcomedilib.o: $(MI_OBJS) $(MIX_OBJS)
	$(LD) -r -o $@ $(MI_OBJS) $(MIX_OBJS)

