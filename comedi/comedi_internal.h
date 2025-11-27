
#ifndef _COMEDI_INTERNAL_H
#define _COMEDI_INTERNAL_H

#include <linux/moduleparam.h>
#include <linux/comedidev.h>
#include <linux/comedi-config.h>

extern COMEDI_MODULE_PARAM_BOOL_T comedi_autoconfig;
extern comedi_driver *comedi_drivers;

comedi_device *comedi_alloc_board_minor(struct device *hardware_device);
void comedi_free_board_minor(unsigned minor);
int comedi_alloc_subdevice_minor(comedi_device *dev, comedi_subdevice *s);
void comedi_free_subdevice_minor(comedi_subdevice *s);

void comedi_device_detach(comedi_device * dev);
int comedi_device_attach(comedi_device * dev, comedi_devconfig * it);
int insn_inval(comedi_device * dev, comedi_subdevice * s,
	comedi_insn * insn, lsampl_t * data);
int comedi_buf_alloc(comedi_device * dev, comedi_subdevice * s, unsigned long
	new_size);
unsigned int comedi_buf_write_alloc_strict(comedi_async * async,
	unsigned int nbytes);

int do_rangeinfo_i(comedi_device * dev, comedi_rangeinfo * ri);

#ifdef CONFIG_PROC_FS
void comedi_proc_init(void);
void comedi_proc_cleanup(void);
#else
static inline void comedi_proc_init(void)
{
}
static inline void comedi_proc_cleanup(void)
{
}
#endif

#endif //_COMEDI_INTERNAL_H
