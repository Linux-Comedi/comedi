
ALL_SUB_DIRS :=
MOD_SUB_DIRS :=
SUB_DIRS     :=
MOD_LIST_NAME := MISC_MODULES

EXTRA_CFLAGS := -I ../

export-objs := mite.o 8255.o

obj-y :=
obj-m :=
obj-n :=
obj-  :=

obj-$(CONFIG_COMEDI_8255)		+= 8255.o

obj-$(CONFIG_COMEDI_DAS08)		+= das08.o
obj-$(CONFIG_COMEDI_DAS08JR)		+= das08jr.o
obj-$(CONFIG_COMEDI_DAS16)		+= das16.o
obj-$(CONFIG_COMEDI_DAS1600)		+= das1600.o
obj-$(CONFIG_COMEDI_DAS6402)		+= das6402.o

obj-$(CONFIG_COMEDI_DT2801)		+= dt2801.o
obj-$(CONFIG_COMEDI_DT2811)		+= dt2811.o
obj-$(CONFIG_COMEDI_DT2814)		+= dt2814.o
obj-$(CONFIG_COMEDI_DT2815)		+= dt2815.o
obj-$(CONFIG_COMEDI_DT2817)		+= dt2817.o
obj-$(CONFIG_COMEDI_DT282x)		+= dt282x.o
obj-$(CONFIG_COMEDI_DT3000)		+= dt3000.o

obj-$(CONFIG_COMEDI_II_PCI20KC)		+= ii_pci20kc.o

obj-$(CONFIG_COMEDI_MULTIQ3)		+= multiq3.o

obj-$(CONFIG_COMEDI_NI_ATMIO)		+= ni_atmio.o
obj-$(CONFIG_COMEDI_NI_PCIMIO)		+= ni_pcimio.o
obj-$(CONFIG_COMEDI_NI_PCIDIO)		+= ni_pcidio.o
obj-$(CONFIG_COMEDI_MITE)		+= mite.o

obj-$(CONFIG_COMEDI_PCL711)		+= pcl711.o
obj-$(CONFIG_COMEDI_PCL725)		+= pcl725.o
obj-$(CONFIG_COMEDI_PCL726)		+= pcl726.o

obj-$(CONFIG_COMEDI_PARPORT)		+= comedi_parport.o

obj-$(CONFIG_COMEDI_RTI800)		+= rti800.o
obj-$(CONFIG_COMEDI_RTI802)		+= rti802.o

obj-m += $(obj-y)

#L_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-y)))
#LX_OBJS	:= $(sort $(filter     $(export-objs), $(obj-y)))
#MI_OBJS	:= $(sort $(filter-out $(export-objs), $(obj-y)))
#MIX_OBJS	:= $(sort $(filter     $(export-objs), $(obj-y)))
M_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-m)))
MX_OBJS		:= $(sort $(filter     $(export-objs), $(obj-m)))



include $(TOPDIR)/Rules.make


drivers.o:	$(obj-y)
	$(LD) -r -o $@ $(obj-y)

ni_pcimio.o:	ni_mio_common.c
ni_atmio.o:	ni_mio_common.c


