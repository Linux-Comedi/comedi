
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

LINUX="/home/ds/linux/condor-kernel-2.4.20/linux-2.4.21"

LINUX_LD="ld -m elf_i386 -r -e stext"
LINUX_CFLAGS="-D__KERNEL__ -I/home/ds/linux/condor-kernel-2.4.20/linux-2.4.21/include -Wall -Wstrict-prototypes -Wno-trigraphs -O2 -fno-strict-aliasing -fno-common -fomit-frame-pointer -pipe -mpreferred-stack-boundary=2 -march=athlon -DMODULE"

AC_SUBST(LINUX_LD)
AC_SUBST(LINUX_CFLAGS)


COMEDI_CFLAGS="-Werror -I\$(top_srcdir)/include/"
AC_SUBST(COMEDI_CFLAGS)

moduledir="\$(libdir)/comedi"

AC_SUBST(moduledir)


AC_OUTPUT([
Makefile
modtool
comedi/Makefile
comedi/drivers/Makefile
comedi/kcomedilib/Makefile
])

chmod +x modtool

