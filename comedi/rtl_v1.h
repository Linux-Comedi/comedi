/*
 *  RTL compatibility,, version 1
 *
 */

#ifndef __COMEDI_RTL_V1_H
#define __COMEDI_RTL_V1_H

#include <asm/rt_irq.h>

#if 0
int rt_printk(const char *fmt, ...);

void rt_printk_cleanup(void);
int rt_printk_init(void);
#endif

void comedi_rtl_v1_init(void);
void comedi_rtl_v1_cleanup(void);


#endif

