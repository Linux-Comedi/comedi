/*
 *  RTL compatibility,, version 1
 *
 */

#ifndef __COMEDI_RTAI_H
#define __COMEDI_RTAI_H

#include <rtai/rtai.h>

#if 0
int rt_printk(const char *fmt, ...);

void rt_printk_cleanup(void);
int rt_printk_init(void);
#endif

void comedi_rtai_init(void);
void comedi_rtai_cleanup(void);


#endif

