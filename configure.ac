
AC_INIT(comedi/comedi_fops.c)

AM_CONFIG_HEADER(config.h)

AC_CANONICAL_TARGET([])

if test x$prefix = xNONE ; then
  prefix=/
fi

PACKAGE=comedi
COMEDI_MAJORVERSION=0
COMEDI_MINORVERSION=7
COMEDI_MICROVERSION=67
# set the nanoversion to 0 for releases, 1 for CVS, >= 2 for prereleases
COMEDI_NANOVERSION=1

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
AC_PATH_PROG(STRIP,strip)
AC_PATH_PROG([DEPMOD], [depmod], [no], [PATH:/sbin:/usr/sbin:/usr/local/sbin])
if test "$DEPMOD" == "no" ; then
	AC_MSG_ERROR([unable to find the 'depmod' program.])
fi

AS_LINUX()
AS_LINUX_MODTOOL()

DS_RTAI([USE_RTAI=yes],[USE_RTAI=no])
DS_RTLINUX([USE_RTLINUX=yes],[USE_RTLINUX=no])
AM_CONDITIONAL([CONFIG_COMEDI_RT],
	[test ${USE_RTAI} = "yes" -o ${USE_RTLINUX} = "yes"])
if test ${USE_RTAI} = "yes" -o ${USE_RTLINUX} = "yes" ; then
	AC_DEFINE([CONFIG_COMEDI_RT],[true],[Define to enable Comedi's real-time support])
fi


CC=$LINUX_CC

#ERROR_CFLAGS="-Werror"

[COMEDI_CFLAGS="$ERROR_CFLAGS -I\$(top_srcdir)/include/\
 -DKBUILD_MODNAME=\$(shell echo \$@ | sed 's/\(.*\)\.o/\1/g' | sed 's/\(.*\)_ko.*/\1/g')\
 -DKBUILD_BASENAME=\$(shell echo \$< | sed 's/\(.*\)\.c/\1/g')"]
AC_SUBST(COMEDI_CFLAGS)

AC_ARG_ENABLE([pcmcia],[  --disable-pcmcia	Disable support for PCMCIA devices],
	[ENABLE_PCMCIA=$enableval],[ENABLE_PCMCIA="probe"])
if test "$ENABLE_PCMCIA" = "probe" ; then
	if test "x$(echo "$LINUX_KERNELRELEASE" | grep '^2.6')" != x ; then
		AC_MSG_NOTICE([Disabling PCMCIA drivers because they haven't been ported to Linux-2.6])
		ENABLE_PCMCIA=no
	else
		AC_MSG_NOTICE([moo])
		ENABLE_PCMCIA=yes
	fi

fi
if test "$ENABLE_PCMCIA" = "yes" ; then
	AS_LINUX_CONFIG_OPTION_MODULE(CONFIG_PCMCIA)
else
	AC_MSG_NOTICE([PCMCIA support disabled])
	AM_CONDITIONAL(CONFIG_PCMCIA,false)
fi
AS_LINUX_CONFIG_OPTION_MODULE(CONFIG_ISA)
AS_LINUX_CONFIG_OPTION_MODULE(CONFIG_PCI)
AS_LINUX_CONFIG_OPTION_MODULE(CONFIG_USB)

AC_DEFINE(CONFIG_COMEDI_DEBUG, true, [Define if debugging is enabled])
AC_DEFINE(CONFIG_COMEDI_8255, true, [Define if 8255 support is enabled])

AC_OUTPUT([
Makefile
comedi/Makefile
comedi/drivers/Makefile
comedi/kcomedilib/Makefile
])
#comedi/drivers/addi-data/Makefile

