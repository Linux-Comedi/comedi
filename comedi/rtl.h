/*
 *  rt_printk.c, hacked from linux/kernel/printk.c
 *
 * Modified for RT support, David Schleef
 */

#ifndef __COMEDI_RTL_H
#define __COMEDI_RTL_H

int rt_printk(const char *fmt, ...);

void rt_printk_cleanup(void);
int rt_printk_init(void);

void comedi_rtl_init(void);
void comedi_rtl_cleanup(void);


#endif

