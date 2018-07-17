/*
 * linux/poll.h compatibility header
 */

#ifndef __COMPAT_LINUX_POLL_H_
#define __COMPAT_LINUX_POLL_H_

#include <linux/version.h>

#include_next <linux/poll.h>

/*
 * For poll() file operation, the EPOLL* constants should be used for kernel
 * versions 4.16 onwards, and the POLL* constants should be used for earlier
 * kernel versions.
 *
 * Define our own COMEDI_EPOLL constants to select the ones appropriate for
 * the kernel version.
 *
 * Also typedef comedi_poll_t to __poll_t for kernel versions 4.16 onwards,
 * or to unsigned int for earlier kernel versions.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,16,0)
#define COMEDI_EPOLLIN		POLLIN
#define COMEDI_EPOLLPRI		POLLPRI
#define COMEDI_EPOLLOUT		POLLOUT
#define COMEDI_EPOLLERR		POLLERR
#define COMEDI_EPOLLHUP		POLLHUP
#define COMEDI_EPOLLNVAL	POLLNVAL
#define COMEDI_EPOLLRDNORM	POLLRDNORM
#define COMEDI_EPOLLRDBAND	POLLRDBAND
#define COMEDI_EPOLLWRNORM	POLLWRNORM
#define COMEDI_EPOLLWRBAND	POLLWRBAND
#define COMEDI_EPOLLMSG		POLLMSG
#define COMEDI_EPOLLRDHUP	POLLRDHUP
typedef unsigned int comedi_poll_t;
#else
#define COMEDI_EPOLLIN		EPOLLIN
#define COMEDI_EPOLLPRI		EPOLLPRI
#define COMEDI_EPOLLOUT		EPOLLOUT
#define COMEDI_EPOLLERR		EPOLLERR
#define COMEDI_EPOLLHUP		EPOLLHUP
#define COMEDI_EPOLLNVAL	EPOLLNVAL
#define COMEDI_EPOLLRDNORM	EPOLLRDNORM
#define COMEDI_EPOLLRDBAND	EPOLLRDBAND
#define COMEDI_EPOLLWRNORM	EPOLLWRNORM
#define COMEDI_EPOLLWRBAND	EPOLLWRBAND
#define COMEDI_EPOLLMSG		EPOLLMSG
#define COMEDI_EPOLLRDHUP	EPOLLRDHUP
typedef __poll_t comedi_poll_t;
#endif

#endif /* __COMPAT_LINUX_POLL_H */
