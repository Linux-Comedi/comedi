#ifndef __MULTI_VERSION_H_
#define __MULTI_VERSION_H_

#include <modbuild/version.h>
#include_next <linux/version.h>

#ifndef KERNEL_VERSION
#define KERNEL_VERSION(a,b,c) (((a) << 16) + ((b) << 8) + (c))
#endif

#endif