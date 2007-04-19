
AC_INIT
AC_CONFIG_SRCDIR([comedi/comedi_fops.c])
AC_CONFIG_HEADERS(config.h)

AC_CANONICAL_TARGET([])

if test x$prefix = xNONE ; then
  prefix=/
fi

PACKAGE=comedi
COMEDI_MAJORVERSION=0
COMEDI_MINORVERSION=7
COMEDI_MICROVERSION=73
# set the nanoversion to 0 for releases, 1 for CVS, >= 2 for prereleases
COMEDI_NANOVERSION=0

if test x$COMEDI_NANOVERSION = x0 ; then
  VERSION=$COMEDI_MAJORVERSION.$COMEDI_MINORVERSION.$COMEDI_MICROVERSION
else
  VERSION=$COMEDI_MAJORVERSION.$COMEDI_MINORVERSION.$COMEDI_MICROVERSION.$COMEDI_NANOVERSION
fi

AM_INIT_AUTOMAKE($PACKAGE,$VERSION)
AM_MAINTAINER_MODE

AC_SUBST(PACKAGE)
AC_SUBST(VERSION)
AC_DEFINE_UNQUOTED(COMEDI_MAJORVERSION, $COMEDI_MAJORVERSION, [Comedi major version])
AC_DEFINE_UNQUOTED(COMEDI_MINORVERSION, $COMEDI_MINORVERSION, [Comedi minor version])
AC_DEFINE_UNQUOTED(COMEDI_MICROVERSION, $COMEDI_MICROVERSION, [Comedi micro version])

AC_PROG_CC()
AM_PROG_CC_C_O
AC_PATH_TOOL([STRIP],[strip])
AC_PATH_PROG([DEPMOD], [depmod], [no], [$PATH:/sbin:/usr/sbin:/usr/local/sbin])
if test "$DEPMOD" == "no" ; then
	AC_MSG_ERROR([unable to find the 'depmod' program.])
fi

AS_LINUX()
AS_LINUX_MODTOOL()

AC_ARG_ENABLE([rtai], [  --disable-rtai	disable use of rtai],
	[ENABLE_RTAI=$enableval], [ENABLE_RTAI="yes"])
DS_RTAI([HAVE_RTAI=yes],[HAVE_RTAI=no])
if test "$HAVE_RTAI" == "yes" -a "$ENABLE_RTAI" == "yes"; then
	USE_RTAI="yes"
else
	USE_RTAI="no"
fi
DS_RTLINUX([USE_RTLINUX=yes],[USE_RTLINUX=no])
AM_CONDITIONAL([CONFIG_COMEDI_RT],
	[test ${USE_RTAI} = "yes" -o ${USE_RTLINUX} = "yes"])
if test ${USE_RTAI} = "yes" -o ${USE_RTLINUX} = "yes" ; then
	AC_DEFINE([CONFIG_COMEDI_RT],[true],[Define to enable Comedi's real-time support])
fi


CC=$LINUX_CC

#ERROR_CFLAGS="-Werror"

[COMEDI_CFLAGS="$ERROR_CFLAGS -I\$(top_srcdir)/include/\
 -DKBUILD_MODNAME=\$(shell basename \$@ .o | sed 's/\(.*\)_ko.*/\1/g')\
 -DKBUILD_BASENAME=\$(shell basename \$< .c)"]
AC_SUBST(COMEDI_CFLAGS)

COMEDI_CHECK_PCMCIA_PROBE([$LINUX_DIR], [ENABLE_PCMCIA="yes"], [ENABLE_PCMCIA="no"])
AC_ARG_ENABLE([pcmcia],[  --disable-pcmcia	Disable support for PCMCIA devices],
	[ENABLE_PCMCIA=$enableval],[])
if test "$ENABLE_PCMCIA" = "yes" ; then
	AS_LINUX_CONFIG_OPTION_MODULE(CONFIG_PCMCIA)
else
	AC_MSG_NOTICE([PCMCIA support disabled])
	AM_CONDITIONAL(CONFIG_PCMCIA,false)
fi
AC_ARG_ENABLE([usb],[  --disable-usb		Disable support for USB devices],
	[ENABLE_USB=$enableval],[ENABLE_USB="yes"])
if test "$ENABLE_USB" = "yes" ; then
	AS_LINUX_CONFIG_OPTION_MODULE(CONFIG_USB)
else
	AC_MSG_NOTICE([USB support disabled])
	AM_CONDITIONAL(CONFIG_USB,false)
fi
COMEDI_CHECK_LINUX_KBUILD([$LINUX_DIR], [ENABLE_KBUILD="yes"], [ENABLE_KBUILD="no"])
AC_ARG_ENABLE([kbuild], [  --enable-kbuild	Force use (or not) of kernel's Kbuild system to build modules (needs recent 2.6 kernel)],
	[ENABLE_KBUILD=$enableval], [])
AM_CONDITIONAL([CONFIG_KBUILD], [test ${ENABLE_KBUILD} = "yes"])
AS_LINUX_CONFIG_OPTION_MODULE(CONFIG_ISA)
AS_LINUX_CONFIG_OPTION_MODULE(CONFIG_PCI)

AC_DEFINE(CONFIG_COMEDI_DEBUG, true, [Define if debugging is enabled])
AC_DEFINE(CONFIG_COMEDI_8255, true, [Define if 8255 support is enabled])

DISTCHECK_CONFIGURE_FLAGS="--with-modulesdir=\\\$\$\(prefix\)/modules"
AC_SUBST(DISTCHECK_CONFIGURE_FLAGS)

AC_CONFIG_FILES([
Makefile
comedi/Makefile
comedi/drivers/Makefile
comedi/drivers/addi-data/Makefile
comedi/kcomedilib/Makefile
])
AC_OUTPUT

