#ifndef __MULTI_CONFIG_H_
#define __MULTI_CONFIG_H_

#include <modbuild/config.h>
#ifdef CONFIG_COMEDI_RTAI
#include_next <modbuild/config.h>
#endif
#include_next <linux/config.h>

#endif