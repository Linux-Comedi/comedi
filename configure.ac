
AC_PREREQ([2.64])

dnl For second parameter of AC_INIT, use [$major.$minor.$micro] for releases,
dnl otherwise [$major.$minor.$micro.$nano], where $nano is 1 for Git,
dnl $nano is >= 2 for prereleases.
AC_INIT([Comedi],[0.7.76.1],[comedi_list@googlegroups.com],[comedi],
	[http://www.comedi.org/])
AC_CONFIG_SRCDIR([comedi/comedi_fops.c])
AC_CONFIG_HEADERS(config.h)
AC_CONFIG_HEADERS(include/linux/comedi-config.h)

dnl Replace package version number, getting from .tarball-version file,
dnl or from git repository, or from .tarball-version-git file.
dnl Note that the contents of the .tarball-version-git file are replaced
dnl in archive files created by 'git archive'.
VERSION_NUMBER=`cd $srcdir && scripts/git-version-gen .tarball-version`
dnl If new version number is "UNKNOWN" keep the existing number
AS_IF([test "${VERSION_NUMBER}" == "UNKNOWN"],
      [VERSION_NUMBER=$PACKAGE_VERSION])
gl_INIT_PACKAGE_VERSION([$VERSION_NUMBER])

AC_CANONICAL_TARGET([])

AM_INIT_AUTOMAKE([foreign])
AM_MAINTAINER_MODE([enable])

PACKAGE_RPMVERSION=`echo "$PACKAGE_VERSION" | tr - _`
AC_SUBST(PACKAGE_RPMVERSION)

COMEDI_MAJORVERSION=[`expr $PACKAGE_VERSION : '^\([0-9][0-9]*\)' '|' 0 + 0`]
COMEDI_MINORVERSION=[`expr $PACKAGE_VERSION : \
	'^[0-9][0-9]*\.\([0-9][0-9]*\)' '|' 0 + 0`]
COMEDI_MICROVERSION=[`expr $PACKAGE_VERSION : \
	'^[0-9][0-9]*\.[0-9][0-9]*\.\([0-9][0-9]*\)' '|' 0 + 0`]
COMEDI_NANOVERSION=[`expr $PACKAGE_VERSION : \
	'^[0-9][0-9]*\.[0-9][0-9]*\.[0-9][0-9]*\.\([0-9][0-9]*\)' '|' 0 + 0`]

AC_DEFINE_UNQUOTED(COMEDI_MAJORVERSION, $COMEDI_MAJORVERSION, [Comedi major version])
AC_DEFINE_UNQUOTED(COMEDI_MINORVERSION, $COMEDI_MINORVERSION, [Comedi minor version])
AC_DEFINE_UNQUOTED(COMEDI_MICROVERSION, $COMEDI_MICROVERSION, [Comedi micro version])
AC_DEFINE_UNQUOTED(COMEDI_NANOVERSION, $COMEDI_NANOVERSION, [Comedi nano version])
AC_DEFINE_UNQUOTED(COMEDI_RELEASE, ["$PACKAGE_VERSION"], [Comedi version number string])

AC_PROG_CC()
AM_PROG_CC_C_O
AC_PATH_TOOL([STRIP],[strip])
AC_PATH_PROG([DEPMOD], [depmod], [no], [$PATH:/sbin:/usr/sbin:/usr/local/sbin])
if test "$DEPMOD" == "no" ; then
	AC_MSG_ERROR([unable to find the 'depmod' program.])
fi

AS_LINUX()

dnl Check for (hopefully) supported kernel versions
dnl TODO The minimum supported kernel version is yet to be determined, but
dnl 2.4 kernels are definitely no longer supported, and we do not care about
dnl 2.5 (development) kernels.
AC_CHECK_KERNEL_VERSION([2.6],
	[support_kernel="no"], [support_kernel="yes"], [support_kernel="yes"])
if test "$support_kernel" != "yes"; then
	AC_MSG_ERROR([kernel version not supported!])
fi

AS_LINUX_MODTOOL()

AC_ARG_ENABLE([modulesdir-prefix],
	[AS_HELP_STRING([--enable-modulesdir-prefix],
		[enable prefix for the kernel modules installation directory
		 (modulesdir) (needed for "make distcheck")])],
	[ENABLE_MODULESDIR_PREFIX=$enableval], [ENABLE_MODULESDIR_PREFIX="no"])
AM_CONDITIONAL([COMEDI_ENABLE_MODULESDIR_PREFIX],
	[test "${ENABLE_MODULESDIR_PREFIX}" == "yes"])

AC_ARG_ENABLE([rtai], [AS_HELP_STRING([--disable-rtai], [disable use of rtai])],
	[ENABLE_RTAI=$enableval], [ENABLE_RTAI="yes"])
DS_RTAI([HAVE_RTAI=yes],[HAVE_RTAI=no])
if test "$HAVE_RTAI" == "yes" -a "$ENABLE_RTAI" == "yes"; then
	USE_RTAI="yes"
else
	USE_RTAI="no"
fi
DS_RTLINUX([USE_RTLINUX=yes],[USE_RTLINUX=no])
AM_CONDITIONAL([COMEDI_CONFIG_RT],
	[test ${USE_RTAI} = "yes" -o ${USE_RTLINUX} = "yes"])
if test ${USE_RTAI} = "yes" -o ${USE_RTLINUX} = "yes" ; then
	AC_DEFINE([COMEDI_CONFIG_RT],[true],[Define to enable Comedi's real-time support])
fi


CC=$LINUX_CC

#ERROR_CFLAGS="-Werror"

[COMEDI_CFLAGS="$ERROR_CFLAGS -I\$(top_srcdir)/include \
 -I\$(top_srcdir)/inc-wrap \
 -DKBUILD_MODNAME=\$(shell basename \$@ .o | sed 's/\(.*\)_ko.*/\1/g')\
 -DKBUILD_BASENAME=\$(shell basename \$< .c)"]
AC_SUBST(COMEDI_CFLAGS)

AS_CHECK_LINUX_CONFIG_OPTION([CONFIG_PCI],[HAVE_PCI="yes"],[HAVE_PCI="yes"],[HAVE_PCI="no"])
AM_CONDITIONAL([CONFIG_PCI],[test "$HAVE_PCI" = "yes"])
AC_ARG_ENABLE([pci],[AS_HELP_STRING([--disable-pci], [disable support for PCI devices])],
	[ENABLE_PCI=$enableval],[ENABLE_PCI="maybe"])
if test "$HAVE_PCI" = "yes" ; then
	if test "$ENABLE_PCI" = "no" ; then
		AC_MSG_NOTICE([PCI support disabled])
		USE_PCI="no"
	else
		AC_DEFINE([COMEDI_CONFIG_PCI],[true],[Define if PCI support is enabled])
		USE_PCI="yes"
	fi
else
	if test "$ENABLE_PCI" = "yes" ; then
		AC_MSG_ERROR([Kernel does not support PCI])
	fi
	USE_PCI="no"
fi
AM_CONDITIONAL([COMEDI_CONFIG_PCI],[test "$USE_PCI" = "yes"])

COMEDI_CHECK_PCMCIA_DRIVER_PROBE([$LINUX_SRC_DIR],
	[AS_CHECK_LINUX_CONFIG_OPTION([CONFIG_PCMCIA],
		[HAVE_PCMCIA="yes"],[HAVE_PCMCIA="yes"],[HAVE_PCMCIA="no"])],
	[HAVE_PCMCIA="no"])
AM_CONDITIONAL([CONFIG_PCMCIA],[test "$HAVE_PCMCIA" = "yes"])
AC_ARG_ENABLE([pcmcia],[AS_HELP_STRING([--disable-pcmcia], [disable support for PCMCIA devices])],
	[ENABLE_PCMCIA=$enableval],[ENABLE_PCMCIA="maybe"])
if test "$HAVE_PCMCIA" = "yes" ; then
	if test "$ENABLE_PCMCIA" = "no" ; then
		AC_MSG_NOTICE([PCMCIA support disabled])
		USE_PCMCIA="no"
	else
		AC_DEFINE([COMEDI_CONFIG_PCMCIA],[true],[Define if using PCMCIA support])
		USE_PCMCIA="yes"
	fi
else
	if test "$ENABLE_PCMCIA" = "yes" ; then
		AC_MSG_ERROR([Kernel does not support PCMCIA or its API is not supported by Comedi])
	fi
	USE_PCMCIA="no"
fi
AM_CONDITIONAL([COMEDI_CONFIG_PCMCIA],[test "$USE_PCMCIA" = "yes"])

COMEDI_CHECK_PCMCIA_DRIVER_NAME([$LINUX_SRC_DIR],
      [HAVE_PCMCIA_DRIVER_NAME="yes"], [HAVE_PCMCIA_DRIVER_NAME="no"])
if test "$HAVE_PCMCIA_DRIVER_NAME" = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_PCMCIA_DRIVER_NAME],[true],[Define if Linux kernel source has name member in struct pcmcia_driver])
fi

COMEDI_CHECK_PCMCIA_LOOP_TUPLE([$LINUX_SRC_DIR],
      [HAVE_PCMCIA_LOOP_TUPLE="yes"], [HAVE_PCMCIA_LOOP_TUPLE="no"])
if test "$HAVE_PCMCIA_LOOP_TUPLE" = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_PCMCIA_LOOP_TUPLE],[true],[Define if Linux kernel source has pcmcia_loop_tuple function])
fi

COMEDI_CHECK_DS_DEV_NODE_T([$LINUX_SRC_DIR],
      [HAVE_DS_DEV_NODE_T="yes"], [HAVE_DS_DEV_NODE_T="no"])
if test "$HAVE_DS_DEV_NODE_T" = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_DS_DEV_NODE_T],[true],[Define if Linux kernel source has dev_node_t in pcmcia/ds.h (removed in 2.6.35)])
fi

COMEDI_CHECK_CS_IRQ_REQ_T([$LINUX_SRC_DIR],
      [HAVE_CS_IRQ_REQ_T="yes"], [HAVE_CS_IRQ_REQ_T="no"])
if test "$HAVE_CS_IRQ_REQ_T" = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_CS_IRQ_REQ_T],[true],[Define if Linux kernel source has irq_req_t in pcmcia/cs.h (removed in 2.6.35)])
fi

COMEDI_CHECK_CS_IO_REQ_T([$LINUX_SRC_DIR],
      [HAVE_CS_IO_REQ_T="yes"], [HAVE_CS_IO_REQ_T="no"])
if test "$HAVE_CS_IO_REQ_T" = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_CS_IO_REQ_T],[true],[Define if Linux kernel source has io_req_t in pcmcia/cs.h (removed in 2.6.36)])
fi

COMEDI_CHECK_CS_MEMREQ_T([$LINUX_SRC_DIR],
      [HAVE_CS_MEMREQ_T="yes"], [HAVE_CS_MEMREQ_T="no"])
if test "$HAVE_CS_MEMREQ_T" = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_CS_MEMREQ_T],[true],[Define if Linux kernel source has memreq_t in pcmcia/cs.h (removed in 2.6.36)])
fi

COMEDI_CHECK_HAVE_CS_TYPES_H([$LINUX_SRC_DIR],
      [HAVE_CS_TYPES_H="yes"], [HAVE_CS_TYPES_H="no"])
if test "$HAVE_CS_TYPES_H" = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_CS_TYPES_H],[true],[Define if Linux kernel source has pcmcia/cs_types.h (removed in 2.6.36)])
fi

COMEDI_CHECK_HAVE_CS_H([$LINUX_SRC_DIR],
      [HAVE_CS_H="yes"], [HAVE_CS_H="no"])
if test "$HAVE_CS_TYPES_H" = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_CS_H],[true],[Define if Linux kernel source has pcmcia/cs.h (removed in 2.6.37)])
fi

COMEDI_CHECK_HAVE_GENERIC_BOOL_TYPE([$LINUX_SRC_DIR],
      [HAVE_GENERIC_BOOL_TYPE="yes"], [HAVE_GENERIC_BOOL_TYPE="no"])
if test "$HAVE_GENERIC_BOOL_TYPE" = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_GENERIC_BOOL_TYPE],[true],[Define if Linux kernel source has generic 'bool' type in linux/types.h])
fi

COMEDI_CHECK_REQUEST_FIRMWARE_NOWAIT_HAS_GFP([$LINUX_SRC_DIR],
	[REQUEST_FIRMWARE_NOWAIT_HAS_GFP="yes"],
	[REQUEST_FIRMWARE_NOWAIT_HAS_GFP="no"])
if test "$REQUEST_FIRMWARE_NOWAIT_HAS_GFP" = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_REQUEST_FIRMWARE_NOWAIT_HAS_GFP],[true],[Define if Linux kernel 'request_firmware_nowait' function has 'gfp' parameter])
fi

AS_CHECK_LINUX_CONFIG_OPTION([CONFIG_USB],[HAVE_USB="yes"],[HAVE_USB="yes"],[HAVE_USB="no"])
AM_CONDITIONAL([CONFIG_USB],[test "$HAVE_USB" = "yes"])
AC_ARG_ENABLE([usb],[AS_HELP_STRING([--disable-usb], [disable support for USB devices])],
	[ENABLE_USB=$enableval],[ENABLE_USB="maybe"])
if test "$HAVE_USB" = "yes" ; then
	if test "$ENABLE_USB" = "no" ; then
		AC_MSG_NOTICE([USB support disabled])
		USE_USB="no"
	else
		AC_DEFINE([COMEDI_CONFIG_USB],[true],[Define if using USB support])
		USE_USB="yes"
	fi
else
	if test "$ENABLE_USB" = "yes" ; then
		AC_MSG_ERROR([Kernel does not support USB])
	fi
	USE_USB="no"
fi
AM_CONDITIONAL([COMEDI_CONFIG_USB],[test "$USE_USB" = "yes"])

COMEDI_CHECK_HAVE_SET_FS([$LINUX_SRC_DIR],[HAVE_SET_FS="yes"],[HAVE_SET_FS="no"])
if test "$HAVE_SET_FS" = "yes" ; then
	AC_DEFINE([COMEDI_CONFIG_SET_FS],[true],[Define if Linux kernel supports set_fs()])
else
	AC_MSG_NOTICE([serial2002 module cannot be built for this kernel])
fi
AM_CONDITIONAL([COMEDI_CONFIG_SET_FS],[test "$HAVE_SET_FS" = "yes"])

COMEDI_CHECK_LINUX_KBUILD([$LINUX_SRC_DIR], [ENABLE_KBUILD="yes"], [ENABLE_KBUILD="no"])
AC_ARG_ENABLE([kbuild], [AS_HELP_STRING([--enable-kbuild], [force use of kernel's Kbuild system to build modules (needs recent 2.6 kernel)])],
	[ENABLE_KBUILD=$enableval], [])
AM_CONDITIONAL([CONFIG_KBUILD], [test ${ENABLE_KBUILD} = "yes"])
COMEDI_CHECK_HAVE_MUTEX_H([$LINUX_SRC_DIR], [HAVE_MUTEX_H="yes"], [HAVE_MUTEX_H="no"])
if test ${HAVE_MUTEX_H} = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_MUTEX_H], [true], [Define if Linux kernel has mutex.h header])
fi
COMEDI_CHECK_HAVE_LINUX_SEMAPHORE_H([$LINUX_SRC_DIR], [HAVE_LINUX_SEMAPHORE_H="yes"], [HAVE_LINUX_SEMAPHORE_H="no"])
if test ${HAVE_LINUX_SEMAPHORE_H} = "yes" ; then
	AC_DEFINE([COMEDI_COMPAT_HAVE_LINUX_SEMAPHORE_H], [true], [Define if Linux kernel has linux/semaphore.h header])
fi
AS_LINUX_CONFIG_OPTION_MODULE(CONFIG_ISA)

AC_DEFINE(COMEDI_CONFIG_DEBUG, true, [Define if debugging is enabled])
AC_DEFINE(COMEDI_CONFIG_8255, true, [Define if 8255 support is enabled])

DISTCHECK_CONFIGURE_FLAGS="--enable-modulesdir-prefix"
AC_SUBST(DISTCHECK_CONFIGURE_FLAGS)

AC_CONFIG_FILES([
Makefile
comedi-kmod.spec
comedi/Makefile
comedi/comedi_kbuild_copy.inc
comedi/drivers/Makefile
comedi/drivers/addi-data/Makefile
comedi/kcomedilib/Makefile
dkms.conf
include/Makefile
include/linux/Makefile
inc-wrap/Makefile
inc-wrap/linux/Makefile
inc-wrap/linux/sched/Makefile
inc-wrap/asm/Makefile
inc-wrap/pcmcia/Makefile
])
AC_OUTPUT

