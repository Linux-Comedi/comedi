
ALL_SUB_DIRS :=
MOD_SUB_DIRS :=
SUB_DIRS     :=
MOD_LIST_NAME := MISC_MODULES

EXTRA_CFLAGS := -I ../ -I ./

ifeq ($(CONFIG_COMEDI),y)
O_TARGET	:= drivers.o
endif

export-objs := mite.o 8255.o

obj-y :=
obj-m :=
obj-n :=
obj-  :=

obj-$(CONFIG_COMEDI_8255)		+= 8255.o

obj-$(CONFIG_COMEDI_ADL_PCI9118)	+= adl_pci9118.o

obj-$(CONFIG_COMEDI_DAQBOARD2000)	+= daqboard2000.o

obj-$(CONFIG_COMEDI_DAS08_NEW)		+= das08-new.o
obj-$(CONFIG_COMEDI_DAS16_NEW)		+= das16-new.o
obj-$(CONFIG_COMEDI_DAS08)		+= das08.o
obj-$(CONFIG_COMEDI_DAS08JR)		+= das08jr.o
obj-$(CONFIG_COMEDI_DAS16)		+= das16.o
obj-$(CONFIG_COMEDI_DAS1600)		+= das1600.o
obj-$(CONFIG_COMEDI_DAS6402)		+= das6402.o
obj-$(CONFIG_COMEDI_DAS800)		+= das800.o
obj-$(CONFIG_COMEDI_DAS1800)		+= das1800.o

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
obj-$(CONFIG_COMEDI_NI_MIO_CS)		+= ni_mio_cs.o
obj-$(CONFIG_COMEDI_NI_PCIMIO)		+= ni_pcimio.o
obj-$(CONFIG_COMEDI_NI_PCIDIO)		+= ni_pcidio.o
obj-$(CONFIG_COMEDI_MITE)		+= mite.o
obj-$(CONFIG_COMEDI_NI_ATMIO16D)	+= ni_atmio16d.o

obj-$(CONFIG_COMEDI_PCL711)		+= pcl711.o
obj-$(CONFIG_COMEDI_PCL724)		+= pcl724.o
obj-$(CONFIG_COMEDI_PCL725)		+= pcl725.o
obj-$(CONFIG_COMEDI_PCL726)		+= pcl726.o
obj-$(CONFIG_COMEDI_PCL812)		+= pcl812.o
obj-$(CONFIG_COMEDI_PCL818)		+= pcl818.o

obj-$(CONFIG_COMEDI_QUATECH_DAQP)	+= quatech_daqp_cs.o

obj-$(CONFIG_COMEDI_PARPORT)		+= comedi_parport.o

obj-$(CONFIG_COMEDI_RTI800)		+= rti800.o
obj-$(CONFIG_COMEDI_RTI802)		+= rti802.o

obj-$(CONFIG_COMEDI_SKEL)		+= skel.o

#obj-m += $(obj-y)

O_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-y)))
OX_OBJS		:= $(sort $(filter     $(export-objs), $(obj-y)))
M_OBJS		:= $(sort $(filter-out $(export-objs), $(obj-m)))
MX_OBJS		:= $(sort $(filter     $(export-objs), $(obj-m)))



include $(TOPDIR)/Rules.make


ni_pcimio.o:	ni_mio_common.c
ni_atmio.o:	ni_mio_common.c
ni_mio_cs.o:	ni_mio_common.c


