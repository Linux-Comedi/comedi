
#ifndef _COMEDI_INTERNAL_H
#define _COMEDI_INTERNAL_H

#include <linux/moduleparam.h>
#include <linux/comedidev.h>

extern COMEDI_MODULE_PARAM_BOOL_T comedi_autoconfig;
extern comedi_driver *comedi_drivers;

#endif //_COMEDI_INTERNAL_H
