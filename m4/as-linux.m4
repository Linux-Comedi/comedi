dnl as-linux.m4 0.0.1
dnl autostars m4 macro for detecting a Linux source tree (or
dnl equivalent) for compiling modules.
dnl
dnl David Schleef <ds@schleef.org>
dnl Frank Mori Hess <fmhess@users.sourceforge.net>
dnl thomas@apestaart.org
dnl
dnl AS_LINUX()
dnl
dnl this macro adds the options --with-linuxdir and --with-linux-config.
dnl this macro defines:
dnl LINUX_DIR
dnl   The directory where the Linux source resides.
dnl CONFIG_FILE
dnl   The Linux config file
dnl LINUX_ARCH
dnl   $(ARCH) in kernel Makefiles
dnl LINUX_AFLAGS
dnl   $(AFLAGS) in kernel Makefiles
dnl LINUX_LDFLAGS
dnl   Linker flags used by Linux
dnl LINUX_ARFLAGS
dnl   Archiver flags used by Linux
dnl LINUX_CROSS_COMPILE
dnl   Cross-compiler prefix, if any. (example: "powerpc-linux-")
dnl LINUX_KERNELRELEASE
dnl   Kernel release name (2.4.5-pre4-ac5-rmk), $(KERNELRELEASE) in
dnl   Linux Makefiles.
dnl LINUX_CFLAGS
dnl   CFLAGS used by Linux.  Includes both $CFLAGS and $MODFLAGS from
dnl   kernel Makefiles. Also includes warnings and optimization.
dnl LINUX_CC
dnl   Compiler used by Linux.
dnl LINUX_LD
dnl   Path to linker (possibly with options) used by Linux.
dnl LINUX_AS
dnl   Assembler used by Linux.
dnl LINUX_MODULE_EXT
dnl   Module extension (.o or .ko)
dnl
dnl 
dnl End of search list.



dnl check if we can find a source dir for the Linux kernel
dnl defines LINUX_DIR to the absolute location of a usable kernel source tree
AC_DEFUN([AS_LINUX_DIR],
[
	AC_ARG_WITH([linuxdir],
		[AC_HELP_STRING([--with-linuxdir=DIR],
			[specify path to Linux source directory])],
		[LINUX_DIR="${withval}"],
		[LINUX_DIR=default])

	if test "${LINUX_DIR}" != "default" ; then
		AS_TRY_LINUX_DIR([${LINUX_DIR}], , AC_MSG_ERROR([Linux dir not found]) )
	fi

	if test "${LINUX_DIR}" = "default" ; then
		dir="/lib/modules/`uname -r`/build";
		AS_TRY_LINUX_DIR([${dir}], [LINUX_DIR=${dir}], )
	fi
	if test "${LINUX_DIR}" = "default" ; then
		dir="../linux";
		AS_TRY_LINUX_DIR([${dir}], [LINUX_DIR=${dir}], )
	fi
	if test "${LINUX_DIR}" = "default" ; then
		dir="/usr/src/linux";
		AS_TRY_LINUX_DIR([${dir}], [LINUX_DIR=${dir}], )
	fi

	if test "${LINUX_DIR}" = "default" ; then
		AC_MSG_ERROR([Linux source directory not found])
	fi

	AC_SUBST(LINUX_DIR)
])

dnl check if the given candidate path for a linux source tree is usable
AC_DEFUN([AS_TRY_LINUX_DIR],
	[AC_MSG_CHECKING(for Linux in $1)

	if test -f "$1/Makefile" ; then
		result=yes
		$2
	else
		result="not found"
		$3
	fi

	AC_MSG_RESULT($result)
])

dnl check if we can find a config file for the Linux kernel
dnl defines CONFIG_FILE to the absolute location of a usable kernel source tree
dnl uses LINUX_DIR to find either .config or decent configs in configs/
AC_DEFUN([AS_CONFIG_FILE],
[
	AC_ARG_WITH([linux-config],
		[AC_HELP_STRING([--with-linux-config=FILE],
			[specify path to Linux configuration file])],
		[CONFIG_FILE="${withval}"],
		[CONFIG_FILE=default])

	if test "${CONFIG_FILE}" != "default" ; then
		AS_TRY_CONFIG_FILE([${CONFIG_FILE}], , AC_MSG_ERROR([Linux config not found]) )
	fi

        dnl if default specified, first check for the regular .config file
        dnl in LINUX_DIR created by manual configuration
	if test "${CONFIG_FILE}" = "default" ; then
		file="$LINUX_DIR/.config";
		AS_TRY_CONFIG_FILE([${file}], [CONFIG_FILE=${file}], )
	fi
        dnl second, try to guess what config file to use for the current kernel
	if test "${CONFIG_FILE}" = "default" ; then
		dnl Red Hat stores configuration files for their built kernels
		dnl named kernel-(version)-(arch)(extra).config
		dnl where arch is athlon, i386, i586, i686, x86_64
		dnl and (extra) is empty or -smp, -BOOT, -bigmem
		dnl haven't seen combinations of extra yet as of FC1
		version=`uname -r | cut -d- -f1`
		machine=`uname -m`
		extra=
		uname -r | grep smp && extra="-smp"
		uname -r | grep bigmem && extra="-bigmem"
		uname -r | grep BOOT && extra="-BOOT"
		file="$LINUX_DIR/configs/kernel-$version-$machine$extra.config"
		AS_TRY_CONFIG_FILE([${file}], [CONFIG_FILE=${file}], )
	fi
	if test "${CONFIG_FILE}" = "default" ; then
		AC_MSG_ERROR([
The kernel source tree at ${LINUX_DIR} is not configured,
and no configuration files in config/ matching your kernel were found.
Fix before continuing or specify a config file using --with-linux-config.])
	fi

	AC_SUBST(CONFIG_FILE)
])

dnl check if the given candidate config file is usable
AC_DEFUN([AS_TRY_CONFIG_FILE],
	[AC_MSG_CHECKING(for configuration in $1)

	if test -f "$1" ; then
		result=yes
		$2
	else
		result="not found"
		$3
	fi

	AC_MSG_RESULT($result)
])

dnl check if RED_HAT_LINUX_KERNEL is defined
dnl RH/Fedora defines this in linux/rhconfig.h, included from linux/version.h
dnl if this is present, a few extra defines need to be present to make sure
dnl symbol versioning is correct
dnl uses LINUX_DIR to find rhconfig.h
AC_DEFUN([AS_CHECK_REDHAT],
[
	AC_MSG_CHECKING(Red Hat/Fedora kernel)
        HAVE_REDHAT_KERNEL=false
        ac_save_CFLAGS="$CFLAGS"
        CFLAGS="$CFLAGS -I${LINUX_DIR}/include/linux"
        AC_COMPILE_IFELSE(AC_LANG_PROGRAM([
#include "rhconfig.h"
int code = RED_HAT_LINUX_KERNEL;
	]),
        AC_MSG_RESULT(found); HAVE_REDHAT_KERNEL=true,
        AC_MSG_RESULT(not found))
	dnl restore CFLAGS
        CFLAGS="$ac_save_CFLAGS"

	dnl check for Red Hat define flags to use - see /sbin/mkkerneldoth

	dnl initialize the booleans we want to detect
	ENTERPRISE='0'
	SMP='0'
	UP='0'
	BIGMEM='0'
	HUGEMEM='0'

	dnl get variables from the currently running kernel as default 
	KERNEL_TYPE=`uname -r | sed 's_^.*\(smp\|enterprise\|bigmem\|hugemem\)$_-\1_;t;s_.*__;'`
	KERNEL_RELEASE=`uname -r | sed 's|smp\|enterprise\|bigmem\|hugemem||g'`
	KERNEL_ARCH=`uname -m`

	dnl check the config file and override KERNEL_ARCH
        AS_CHECK_LINUX_CONFIG_OPTION(CONFIG_M386, KERNEL_ARCH=i386)
        AS_CHECK_LINUX_CONFIG_OPTION(CONFIG_M586, KERNEL_ARCH=i586)
        AS_CHECK_LINUX_CONFIG_OPTION(CONFIG_M686, KERNEL_ARCH=i686)
	dnl for some reason the i686 bigmem config file has PENTIUM
        AS_CHECK_LINUX_CONFIG_OPTION(CONFIG_MPENTIUMIII, KERNEL_ARCH=i686)
        AS_CHECK_LINUX_CONFIG_OPTION(CONFIG_MK7, KERNEL_ARCH=athlon)

	dnl check the config file and override KERNEL_TYPE
        AS_CHECK_LINUX_CONFIG_OPTION(CONFIG_SMP, KERNEL_TYPE=-smp)
	dnl bigmem is also smp, so this check is done after smp to override
        AS_CHECK_LINUX_CONFIG_OPTION(CONFIG_HIGHMEM64G, KERNEL_TYPE=-bigmem)

	dnl FIXME: need to check hugemem and enterprise config files, which
	dnl aren't provided in Fedora Core 1 !

	case "$KERNEL_TYPE" in
		-smp) SMP='1';;
		-enterprise) ENTERPRISE='1';;
		-bigmem) BIGMEM='1';;
		-hugemem) HUGEMEM='1';;
		*) UP='1';;
	esac
	REDHAT_CFLAGS="-D__MODULE_KERNEL_$KERNEL_ARCH=1"
        REDHAT_CFLAGS="$REDHAT_CFLAGS -D__BOOT_KERNEL_ENTERPRISE=$ENTERPRISE"
        REDHAT_CFLAGS="$REDHAT_CFLAGS -D__BOOT_KERNEL_UP=$UP"
        REDHAT_CFLAGS="$REDHAT_CFLAGS -D__BOOT_KERNEL_SMP=$SMP"
        REDHAT_CFLAGS="$REDHAT_CFLAGS -D__BOOT_KERNEL_BIGMEM=$BIGMEM"
        REDHAT_CFLAGS="$REDHAT_CFLAGS -D__BOOT_KERNEL_HUGEMEM=$HUGEMEM"
])

dnl main entry point
dnl checks the version, and figures out all flags to use to make modules.
AC_DEFUN([AS_LINUX],
[
	AS_LINUX_DIR()
        AS_CONFIG_FILE()
	AS_CHECK_REDHAT()

	AC_MSG_CHECKING([Linux major/minor version])

	if [[ ! -f "${LINUX_DIR}/include/linux/version.h" ]];then
		AC_MSG_ERROR([The header file include/linux/version.h does not exist.
For 2.6 kernels, it can be generated by running 'make prepare' in
the kernel source directory.])
	fi
        dnl the next set of tests is for figuring out version major/minor
        dnl we make sure we have the right version.h by faking out CFLAGS
        ac_save_CFLAGS="$CFLAGS"
        CFLAGS="$CFLAGS -I${LINUX_DIR}/include/linux"
        dnl make sure we find version.h and it contains LINUX_VERSION_CODE
        AC_COMPILE_IFELSE(AC_LANG_PROGRAM([
#include "version.h"
int code = LINUX_VERSION_CODE;
]),
 :, AC_MSG_ERROR([${LINUX_DIR}/include/linux/version.h does not contain LINUX_VERSION_CODE]))


        dnl figure out the linux kernel version major and minor
        dnl using the LINUX_VERSION_CODE defined in include/linux/version.h
        AC_RUN_IFELSE(AC_LANG_PROGRAM([
#include "version.h"
#define KERNEL_VERSION_MAJOR(code) ((code) >> 16)
],[
  return KERNEL_VERSION_MAJOR(LINUX_VERSION_CODE);
]),
		LINUX_VERSION_MAJOR=0,
		LINUX_VERSION_MAJOR=$?)
        AC_RUN_IFELSE(AC_LANG_PROGRAM([
#include "version.h"
#define KERNEL_VERSION_MINOR(code) (((code) >> 8) & 0xff)
],[
  return KERNEL_VERSION_MINOR(LINUX_VERSION_CODE);
]),
		LINUX_VERSION_MINOR=0,
		LINUX_VERSION_MINOR=$?)
        AC_MSG_RESULT($LINUX_VERSION_MAJOR.$LINUX_VERSION_MINOR)
	dnl restore CFLAGS
        CFLAGS="$ac_save_CFLAGS"

	case $LINUX_VERSION_MAJOR.$LINUX_VERSION_MINOR in
		2.6)
			AS_LINUX_2_6()
			;;
		2.[[01234]])
			AS_LINUX_2_4()
			;;
		*)
			AC_MSG_ERROR([Unknown Linux major.minor $LINUX_VERSION_MAJOR.$LINUX_VERSION_MINOR])
			;;
	esac
])

AC_DEFUN([AS_LINUX_2_6],
[
	AC_MSG_CHECKING(for Linux CFLAGS)

	tmpdir="`pwd`/tmp-noicrwa"

	rm -rf ${tmpdir} || :
	mkdir ${tmpdir}

	cat >${tmpdir}/Makefile <<EOF
obj-m += fake.o

\$(obj)/fake.c: flags
	touch \$(obj)/fake.c

.PHONY: flags
flags:
	echo LINUX_ARCH=\"\$(ARCH)\" >>\$(obj)/flags
	echo LINUX_AFLAGS=\"\$(AFLAGS)\" | sed 's_Iinclude_I"\$(LINUXDIR)/include"_g'>>\$(obj)/flags
	echo LINUX_LDFLAGS=\"\" >>\$(obj)/flags
	echo LINUX_ARFLAGS=\"\$(ARFLAGS)\" >>\$(obj)/flags
	echo LINUX_CROSS_COMPILE=\"\$(CROSS_COMPILE)\" >>\$(obj)/flags
	echo LINUX_KERNELRELEASE=\"\$(KERNELRELEASE)\" >>\$(obj)/flags
	echo LINUX_CFLAGS=\"\$(CFLAGS)\" | sed 's_Iinclude_I"\$(LINUXDIR)/include"_g'>>\$(obj)/flags
	echo LINUX_CFLAGS_MODULE=\"\$(CFLAGS_MODULE)\" >>\$(obj)/flags
	echo LINUX_CC=\"\$(CC)\" >>\$(obj)/flags
	echo LINUX_LD=\"\$(LD) \$(LDFLAGS) \$(LDFLAGS_MODULE)\" >>\$(obj)/flags
	echo LINUX_AS=\"\$(AS)\" >>\$(obj)/flags
EOF

	echo ${MAKE-make} -C ${LINUX_DIR} V=1 SUBDIRS=${tmpdir} LINUXDIR=${LINUX_DIR} MODVERDIR=${tmpdir} modules >&5 2>&5
	${MAKE-make} -C ${LINUX_DIR} V=1 SUBDIRS=${tmpdir} LINUXDIR=${LINUX_DIR} MODVERDIR=${tmpdir} modules >&5 2>&5
	. ${tmpdir}/flags
	rm -rf ${tmpdir}

	LINUX_MODULE_EXT=".ko"

	LINUX_CFLAGS="$LINUX_CFLAGS $LINUX_CFLAGS_MODULE"

	AC_SUBST(LINUX_ARCH)
	AC_SUBST(LINUX_AFLAGS)
	AC_SUBST(LINUX_LDFLAGS)
	AC_SUBST(LINUX_ARFLAGS)
	AC_SUBST(LINUX_CROSS_COMPILE)
	AC_SUBST(LINUX_KERNELRELEASE)
	AC_SUBST(LINUX_CFLAGS)
	AC_SUBST(LINUX_CC)
	AC_SUBST(LINUX_LD)
	AC_SUBST(LINUX_AS)
	AC_SUBST(LINUX_MODULE_EXT)

	AC_MSG_RESULT([ok])
])


AC_DEFUN([AS_LINUX_2_4],
[
	AC_MSG_CHECKING(for Linux 2.4 make flags)
	dnl we try to figure out the CFLAGS by invoking the Makefile on
        dnl a test dir
        dnl we use the correct config file by substituting the MAKEFILES
        dnl Makefile variable, which originally points to .config

	if [[ ! -f "${LINUX_DIR}/.hdepend" ]];then
		AC_MSG_ERROR([
You need to run 'make dep' on the kernel source before continuing.])
	fi

        TMPFILE=`mktemp /tmp/linux.XXXXXXXX` || exit 1

	POPDIR=`(pwd)`
        cd ${LINUX_DIR}
( sed "s|\.config|${CONFIG_FILE}|g" Makefile && cat <<EOF ) | make -f - get_cflags > ${TMPFILE}

get_cflags:
	@echo LINUX_ARCH=\"\$(ARCH)\"
	@echo LINUX_AFLAGS=\"\$(AFLAGS)\" | sed 's_Iinclude_I\"\$(LINUXDIR)/include\"_g'
	@echo LINUX_LDFLAGS=\"\"
	@echo LINUX_ARFLAGS=\"\$(ARFLAGS)\"
	@echo LINUX_CROSS_COMPILE=\"\$(CROSS_COMPILE)\"
	@echo LINUX_KERNELRELEASE=\"\$(KERNELRELEASE)\"
	@echo LINUX_CFLAGS=\"\$(CFLAGS)\" | sed 's_Iinclude_I\"\$(LINUXDIR)/include\"_g'
	@echo LINUX_MODFLAGS=\"\$(MODFLAGS)\"
	@echo LINUX_CC=\"\$(CC)\"
	@echo LINUX_LD=\"\$(LD) \$(LDFLAGS)\"
	@echo LINUX_AS=\"\$(AS)\"
EOF
	. ${TMPFILE}
	rm -rf ${TMPFILE}
	cd $POPDIR

	LINUX_MODULE_EXT=".o"

	LINUX_CFLAGS="$LINUX_CFLAGS $LINUX_MODFLAGS"

	dnl if we have REDHAT_CFLAGS, put them in LINUX_CFLAGS
	if test "x$REDHAT_CFLAGS" != "x";
	then
		LINUX_CFLAGS="$LINUX_CFLAGS $REDHAT_CFLAGS"
	fi
	AC_SUBST(LINUX_ARCH)
	AC_SUBST(LINUX_AFLAGS)
	AC_SUBST(LINUX_LDFLAGS)
	AC_SUBST(LINUX_ARFLAGS)
	AC_SUBST(LINUX_CROSS_COMPILE)
	AC_SUBST(LINUX_KERNELRELEASE)
	AC_SUBST(LINUX_CFLAGS)
	AC_SUBST(LINUX_CC)
	AC_SUBST(LINUX_LD)
	AC_SUBST(LINUX_AS)
	AC_SUBST(LINUX_MODULE_EXT)

	AC_MSG_RESULT([ok])

	AC_MSG_CHECKING(for Linux 2.4 CFLAGS)
	AC_MSG_RESULT($LINUX_CFLAGS)
	AC_MSG_CHECKING(for Linux 2.4 LDFLAGS)
	AC_MSG_RESULT($LINUX_LDFLAGS)
])

AC_DEFUN([AS_CHECK_LINUX_CONFIG_OPTION],
[
	AC_MSG_CHECKING([Linux config option $1])

	if grep '^$1=y$' ${CONFIG_FILE} >/dev/null 2>/dev/null; then
		result=yes
		$2
	else if grep '^$1=m$' ${CONFIG_FILE} >/dev/null 2>/dev/null; then
		result=module
		$3
	else
		result=no
		$4
	fi
	fi

	AC_MSG_RESULT([$result])
])

AC_DEFUN([AS_LINUX_CONFIG_OPTION],
[
	AS_CHECK_LINUX_CONFIG_OPTION([$1],
		[$1=yes],
		[$1=module],
		[$1=no])

	AM_CONDITIONAL([$1],[test "${$1}" = yes])
])

AC_DEFUN([AS_LINUX_CONFIG_OPTION_MODULE],
[
	AS_CHECK_LINUX_CONFIG_OPTION([$1],
		[$1=yes],
		[$1=module],
		[$1=no])

	AM_CONDITIONAL([$1],[test "${$1}" = yes -o "${$1}" = module])
])

