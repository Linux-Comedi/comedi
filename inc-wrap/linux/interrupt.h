/*
 * linux/interrupt.h compatibility header
 */

#ifndef COMEDI_COMPAT__LINUX__INTERRUPT_H__INCLUDED__
#define COMEDI_COMPAT__LINUX__INTERRUPT_H__INCLUDED__

#include_next <linux/interrupt.h>

#include <linux/version.h>

#if !defined(IRQ_NONE) && LINUX_VERSION_CODE < KERNEL_VERSION(2,6,29)
typedef void irqreturn_t;
#define IRQ_NONE
#define IRQ_HANDLED
#define IRQ_RETVAL(x) (void)(x)
#endif

#ifndef IRQF_SHARED
#define IRQF_DISABLED           SA_INTERRUPT
#define IRQF_SAMPLE_RANDOM      SA_SAMPLE_RANDOM
#define IRQF_SHARED             SA_SHIRQ
#define IRQF_PROBE_SHARED       SA_PROBEIRQ
#define IRQF_PERCPU             SA_PERCPU
#ifdef SA_TRIGGER_MASK
#define IRQF_TRIGGER_NONE       0
#define IRQF_TRIGGER_LOW        SA_TRIGGER_LOW
#define IRQF_TRIGGER_HIGH       SA_TRIGGER_HIGH
#define IRQF_TRIGGER_FALLING    SA_TRIGGER_FALLING
#define IRQF_TRIGGER_RISING     SA_TRIGGER_RISING
#define IRQF_TRIGGER_MASK       SA_TRIGGER_MASK
#else
#define IRQF_TRIGGER_NONE       0
#define IRQF_TRIGGER_LOW        0
#define IRQF_TRIGGER_HIGH       0
#define IRQF_TRIGGER_FALLING    0
#define IRQF_TRIGGER_RISING     0
#define IRQF_TRIGGER_MASK       0
#endif
#endif

/* if interrupt handler prototype has pt_regs* parameter */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,19)
#define PT_REGS_ARG , struct pt_regs *regs
#define PT_REGS_CALL , regs
#define PT_REGS_NULL , NULL
#else
#define PT_REGS_ARG
#define PT_REGS_CALL
#define PT_REGS_NULL
#endif

#endif
