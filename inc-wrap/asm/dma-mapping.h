/*
 * asm/dma-mapping.h compatibility header
 */

#ifndef __COMPAT_ASM_DMA_MAPPING_H_
#define __COMPAT_ASM_DMA_MAPPING_H_

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
/* Early 2.6.x kernels may need these included before <asm/dma-mapping.h> */
#include <asm/io.h>
#include <asm/scatterlist.h>
#endif /* LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8) */

#include_next <asm/dma-mapping.h>

#endif
