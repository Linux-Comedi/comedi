
AC_INIT(comedi/comedi_fops.c)

AM_CONFIG_HEADER(config.h)

AC_CANONICAL_TARGET([])

PACKAGE=comedi
VERSION=0.7.67

AM_INIT_AUTOMAKE($PACKAGE,$VERSION)
AM_MAINTAINER_MODE

AC_SUBST(PACKAGE)
AC_SUBST(VERSION)

AC_PROG_CC

#DS_LINUX_2_4()
DS_LINUX_2_6()

#ERROR_CFLAGS="-Werror"

COMEDI_CFLAGS="$ERROR_CFLAGS -I\$(top_srcdir)/include/"
AC_SUBST(COMEDI_CFLAGS)

moduledir="/lib/modules/${LINUX_KERNELRELEASE}/comedi"
modulePROGRAMS_INSTALL="\$(top_srcdir)/modtool --install"
AC_SUBST(moduledir)
AC_SUBST(modulePROGRAMS_INSTALL)

#DS_LINUX_CONFIG_OPTION_MODULE(CONFIG_PCMCIA)
AM_CONDITIONAL(CONFIG_PCMCIA,false)

AC_OUTPUT([
Makefile
modtool
comedi/Makefile
comedi/drivers/Makefile
comedi/kcomedilib/Makefile
])

chmod +x modtool

