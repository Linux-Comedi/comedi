dnl as-linux.m4 0.2.0

dnl autostars m4 macro for detecting a Linux source tree (or
dnl equivalent) for compiling modules.

dnl David Schleef <ds@schleef.org>
dnl Frank Mori Hess <fmhess@users.sourceforge.net>
dnl Thomas Vander Stichele <thomas@apestaart.org>

dnl $Id$

dnl AS_LINUX()
dnl
dnl this macro adds the options
dnl --with-linuxdir        to specify a kernel build tree location
dnl --with-linuxsrcdir     to specify a kernel source tree location
dnl --with-linuxconfig     to specify a kernel .config file
dnl --with-kernel-release  to specify an alternative uname -r
dnl --with-machine         to specify an alternative uname -m
dnl --with-rpm-target      to specify to match the given rpm --target option
dnl --with-modulesdir      to specify the base install location of modules
dnl --with-modulesdeveldir to specify the base install location of build stuff

dnl this macro defines:
dnl LINUX_DIR
dnl   The directory where the Linux build tree resides.
dnl LINUX_SRC_DIR
dnl   The directory where the Linux source tree resides.
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
dnl LINUX_MODULE_STYLE
dnl   Style of module building (2.4.0, 2.6.0, 2.6.6)
dnl LINUX_MODULE_EXT
dnl   Module extension (.o or .ko)
dnl LINUX_MODPOST
dnl   path to modpost script
dnl MODULESDIR
dnl   base install path for kernel modules
dnl MODULESDEVELDIR
dnl   base install path for kernel module development files
dnl
dnl End of search list.

dnl main entry point
dnl checks the version, and figures out all flags to use to make modules.
AC_DEFUN([AS_LINUX],
[
	dnl check if user supplied a uname -r, and if not use the running one
	AS_LINUX_KERNEL_RELEASE()
	dnl check if user supplied a uname -m, and if not use the running one
	AS_LINUX_MACHINE()
	dnl check if the user supplied an rpm target arch
	dnl override the LINUX_MACHINE value if he did
	AS_LINUX_RPM_TARGET()

	dnl find the kernel build tree for the given uname -r
	AS_LINUX_DIR()
	dnl override kernel release uname -r value with info from build tree
	AS_LINUX_OVERRIDE_KERNEL_RELEASE($LINUX_DIR)
	dnl find the kernel source tree from the build tree or --with-linuxsrcdir
	AS_LINUX_SRC_DIR($LINUX_DIR)
	dnl check if user supplied a config file; if not, guess a good one
	AS_LINUX_CONFIG($LINUX_DIR, $LINUX_KERNEL_RELEASE, $LINUX_MACHINE)
	dnl check if we're building on pre-FC2 Red Hat/Fedora,
	dnl and add some flags if we are
	AS_CHECK_REDHAT_PRE_FC2()
	dnl check for where to install modules
	AS_LINUX_MODULESDIR($LINUX_KERNEL_RELEASE)
	dnl check for where to install module development files
	AS_LINUX_MODULESDEVELDIR($LINUX_DIR)
	dnl check for the MAJOR/MINOR version of Linux
	AS_LINUX_VERSION_MAJOR_MINOR($LINUX_SRC_DIR)

	dnl now call the correct macro to get compiler flags
	dnl the versioned AS_LINUX macros just use the global variables
	dnl this could be cleaned up later on if we feel like it
	case $LINUX_VERSION_MAJOR.$LINUX_VERSION_MINOR in
		2.6 | [[3-9]].[[0-9]]*)
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


dnl check if we can find a build dir for the Linux kernel
dnl defines LINUX_DIR to the absolute location of a usable kernel build tree
AC_DEFUN([AS_LINUX_DIR],
[
	AC_ARG_WITH([linuxdir],
		[AC_HELP_STRING([--with-linuxdir=DIR],
			[specify path to Linux build directory])],
		[LINUX_DIR="${withval}"],
		[LINUX_DIR=default])

	if test "${LINUX_DIR}" != "default" ; then
		AS_TRY_LINUX_DIR([${LINUX_DIR}], , AC_MSG_ERROR([Linux build dir not found]) )
	fi

	if test "${LINUX_DIR}" = "default" ; then
		dir="/lib/modules/$LINUX_KERNEL_RELEASE/build";
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
		AC_MSG_ERROR([Linux build directory not found])
	fi

	AC_SUBST(LINUX_DIR)
])

dnl check if the given candidate path for a linux build tree is usable
AC_DEFUN([AS_TRY_LINUX_DIR],
	[AC_MSG_CHECKING(for Linux build in $1)

	if test -f "$1/Makefile" ; then
		result=yes
		$2
	else
		result="not found"
		$3
	fi

	AC_MSG_RESULT($result)
])

dnl get the kernel source directory
dnl $1 is the kernel build directory
dnl defines LINUX_SRC_DIR to the absolute location of kernel source tree
AC_DEFUN([AS_LINUX_SRC_DIR],
[
	AC_ARG_WITH([linuxsrcdir],
		[AC_HELP_STRING([--with-linuxsrcdir=DIR],
			[specify path to Linux source directory])],
		[LINUX_SRC_DIR="${withval}"],
		[LINUX_SRC_DIR=default])

	if test "${LINUX_SRC_DIR}" != "default" ; then
		AS_TRY_LINUX_SRC_DIR([${LINUX_SRC_DIR}], , AC_MSG_ERROR([Linux source dir not found]) )
	fi

	if test "${LINUX_SRC_DIR}" = "default" ; then
		AC_MSG_CHECKING(for separate Linux source and build directory)
		if test -d "$1/include/sound"; then
			# build directory looks like a source directory
			dir=""
		else
			dir=`sed -n -e 's/^KERNELSRC *:= *\(.*\)/\1/p' "$1/Makefile"`
			if test -z "$dir"; then
				# 2.6.25
				dir=`sed -n -e 's/^MAKEARGS *:= *-C *\([[^[:space:]]]*\).*/\1/p' "$1/Makefile"`
			fi
			if test -z "$dir"; then
				# 4.20
				dir=`sed -n -e '/^__sub-make:$/,/^$/s/.* -C *\([[^[:space:]]]*\).*/\1/p' "$1/Makefile"`
			fi
			if test -z "$dir"; then
				# 5.2
				if test "`grep -cv '^[[[:space:]]]*\(#.*\|\)$' "$1/Makefile"`" = "1"; then
					dir=`sed -n -e 's/^include[[[:space:]]][[[:space:]]]*\(.*\)\/Makefile$/\1/p' "$1/Makefile"`
				fi
			fi
		fi
		if test -z "$dir"; then
			AC_MSG_RESULT([no])
			LINUX_SRC_DIR="$1"
		else
			AC_MSG_RESULT([yes])
			case "$dir" in
			.*) dir="$1/$dir" ;;
			esac
			AS_TRY_LINUX_SRC_DIR([${dir}], [LINUX_SRC_DIR=${dir}], )
		fi
	fi

	if test "${LINUX_SRC_DIR}" = "default" ; then
		AC_MSG_ERROR([Linux source directory not found])
	fi

	AC_SUBST(LINUX_SRC_DIR)
])

dnl check if the given candidate path for a linux source tree is usable
AC_DEFUN([AS_TRY_LINUX_SRC_DIR],
	[AC_MSG_CHECKING(for Linux source in $1)

	if test -f "$1/Makefile" ; then
		result=yes
		$2
	else
		result="not found"
		$3
	fi

	AC_MSG_RESULT($result)
])

dnl allow for specifying a kernel release (uname -r) to build for
dnl use uname -r of running one if not specified
dnl store result in LINUX_KERNEL_RELEASE
AC_DEFUN([AS_LINUX_KERNEL_RELEASE],
[
	AC_ARG_WITH([kernel-release],
		[AC_HELP_STRING([--with-kernel-release=RELEASE],
			[specify the "uname -r"-value to build for])],
		[LINUX_KERNEL_RELEASE="${withval}"],
		[LINUX_KERNEL_RELEASE=`uname -r`])
        if test "x$LINUX_KERNEL_RELEASE" = "xyes";
        then
		LINUX_KERNEL_RELEASE=`uname -r`
        fi
        AC_MSG_NOTICE([Using $LINUX_KERNEL_RELEASE as the uname -r value])
])

dnl replaces LINUX_KERNEL_RELEASE once the Linux build directory is known
dnl first argument is the Linux build directory
AC_DEFUN([AS_LINUX_OVERRIDE_KERNEL_RELEASE],
[
	AC_REQUIRE([AC_PROG_CPP])
	INCDIR="$1/include"
	UTSINC="${INCDIR}/generated/utsrelease.h"
	if ! test -f "${UTSINC}"; then
		UTSINC="${INCDIR}/linux/utsrelease.h"
	fi
	if ! test -f "${UTSINC}"; then
		UTSINC="${INCDIR}/linux/version.h"
	fi
	if test -f "${UTSINC}"; then
		echo UTS_RELEASE | cat "${UTSINC}" - > conftest.c
		RELEASE=`$CPP -I "${INCDIR}" conftest.c | tail -n 1 |
			sed 's/^"\(.*\)"$/\1/'`
		if test "${RELEASE}" != "UTS_RELEASE" -a "${RELEASE}" != "" \
			-a "${RELEASE}" != "${LINUX_KERNEL_RELEASE}"; then
			AC_MSG_NOTICE([Overriding uname -r value with ${RELEASE}])
			LINUX_KERNEL_RELEASE="${RELEASE}"
		fi

	fi
])

dnl allow for specifying a machine (uname -m) to build for
dnl use uname -m, of running one if not specified
dnl store result in LINUX_MACHINE
AC_DEFUN([AS_LINUX_MACHINE],
[
	AC_ARG_WITH([machine],
		[AC_HELP_STRING([--with-machine=MACHINE],
			[specify the "uname -m"-value to build for])],
		[LINUX_MACHINE="${withval}"],
		[LINUX_MACHINE=`uname -m`])
        if test "x$LINUX_MACHINE" = "xyes";
        then
		LINUX_MACHINE=`uname -m`
        fi
        AC_MSG_NOTICE([Using $LINUX_MACHINE as the uname -m value])
])
dnl allow for specifying an rpm target arch
dnl if none specified, try to guess one from running rpm querying for
dnl the kernel with the uname -r
dnl this is so configure without arguments works out of the box
dnl FIXME: investigate if uname -p is a correct guess for this, and if
dnl we should have a flag for specifying it instead

dnl this macro possibly overrides LINUX_MACHINE

dnl first argument is the kernel release building for
AC_DEFUN([AS_LINUX_RPM_TARGET],
[
	RELEASE=$LINUX_KERNEL_RELEASE
	AC_ARG_WITH([rpm-target],
		[AC_HELP_STRING([--with-rpm-target=TARGET],
			[specify the target arch to build for])],
		[LINUX_RPM_TARGET="${withval}"],
		[LINUX_RPM_TARGET=])
	if test "x$LINUX_RPM_TARGET" = "x"
	then
		dnl if we have rpm, try to guess the target of the kernel
		dnl we want to build for using rpm
		AC_PATH_PROG([RPM], [rpm], [no])
		if test "x$RPM" != "xno" ; then
			if $RPM -q kernel-$RELEASE > /dev/null
			then
			  LINUX_RPM_TARGET=`$RPM -q --queryformat %{arch} kernel-$RELEASE`
			else
			  AC_MSG_NOTICE([Cannot guess target arch, consider setting it using --with-rpm-target])
			fi
		fi
	fi

	dnl now override LINUX_MACHINE if LINUX_RPM_TARGET is set
	if test "x$LINUX_RPM_TARGET" != "x"
	then
		dnl override LINUX_MACHINE based on this
		dnl FIXME: add other possible Red Hat/Fedora/rpm targets here
		LINUX_MACHINE=
		case "$LINUX_RPM_TARGET" in
			i?86) LINUX_MACHINE=i386;;
			athlon) LINUX_MACHINE=i386;;
			x86_64) LINUX_MACHINE=x86_64;;
		esac
		if test "x$LINUX_MACHINE" = "x"
		then
		AC_MSG_ERROR(Could not guess uname -m value from target $LINUX_RPM_TARGET)
		fi
	fi
])


dnl check if we can find a config file for the Linux kernel
dnl defines LINUX_CONFIG to the absolute location of a usable
dnl kernel source config file

dnl for rpm distros, it can try and guess the correct config file by
dnl checking the global LINUX_RPM_TARGET variable set somewhere else
dnl (FIXME: move this to an argument instead ?)

dnl first argument is LINUX_DIR to check for possible configs
dnl second argument is kernel-release (uname -r)
dnl third argument is machine (uname -m)

AC_DEFUN([AS_LINUX_CONFIG],
[
	LINUXDIR=[$1]
	KERNEL_RELEASE=[$2]
	MACHINE=[$3]
	AC_ARG_WITH([linuxconfig],
		[AC_HELP_STRING([--with-linuxconfig=FILE],
			[specify path to Linux configuration file])],
		[LINUX_CONFIG="${withval}"],
		[LINUX_CONFIG=default])

	dnl if a file got specified, try it as a linux config file
	if test "${LINUX_CONFIG}" != "default" ; then
		AS_TRY_LINUX_CONFIG($LINUX_CONFIG, $MACHINE,
                                    ,, AC_MSG_ERROR([Linux config not found]) )
	fi

        dnl if no file specified, first check for the regular .config file
        dnl in LINUXDIR created by manual configuration
	if test "${LINUX_CONFIG}" = "default" ; then
		file="$LINUXDIR/.config";
		AS_TRY_LINUX_CONFIG($file, $MACHINE,
		                    [LINUX_CONFIG=${file}], )
	fi
        dnl second, try to guess what config file to use for the current kernel
	dnl FIXME: the possible arch is from rpmbuild --target, and is
	dnl different from the value of ARCH (Makefile) or MACHINE (uname -m)
	dnl so we should have a redhat flag to specify the target to find
	dnl the correct config file
	if test "${LINUX_CONFIG}" = "default" && test "x$LINUX_RPM_TARGET" != "x"; then
		dnl Red Hat stores configuration files for their built kernels
		dnl named kernel-(version)-(arch)(extra).config
		dnl where arch is athlon, i386, i586, i686, x86_64
		dnl and (extra) is empty or -smp, -BOOT, -bigmem
		dnl haven't seen combinations of extra yet as of FC1
		KVERSION=`echo $KERNEL_RELEASE | cut -d- -f1`
		extra=
		echo $KERNEL_RELEASE | grep smp && EXTRA="-smp"
		echo $KERNEL_RELEASE | grep bigmem && EXTRA="-bigmem"
		echo $KERNEL_RELEASE | grep BOOT && EXTRA="-BOOT"
		file="$LINUXDIR/configs/kernel-$KVERSION-$LINUX_RPM_TARGET$EXTRA.config"
		AS_TRY_LINUX_CONFIG($file, $MACHINE,
		                    [LINUX_CONFIG=${file}], )
	fi
	if test "${LINUX_CONFIG}" = "default" ; then
		AC_MSG_ERROR([
The kernel source tree at ${LINUXDIR} is not configured,
and no configuration files in config/ matching your kernel were found.
Fix before continuing or specify a config file using --with-configfile.])
	fi

	AC_SUBST(LINUX_CONFIG)
])

dnl check if the given candidate config file is usable
dnl FIXME: it would be nice if it could check if it matches the
dnl given machine (uname -m)
AC_DEFUN([AS_TRY_LINUX_CONFIG],
[
	CFG=[$1]
	MACHINE=[$2]
	AC_MSG_CHECKING($CFG)

	if test -f "$CFG" ; then
		result=yes
		ifelse([$3], , :, [$3])
	else
		result="not found"
		ifelse([$4], , :, [$4])
	fi
	AC_MSG_RESULT($result)
])

dnl check if RED_HAT_LINUX_KERNEL is defined
dnl pre-FC2 RH/Fedora defines this in linux/rhconfig.h
dnl included from linux/version.h
dnl if this is present, a few extra defines need to be present to make sure
dnl symbol versioning is correct
dnl uses LINUX_DIR to find rhconfig.h
AC_DEFUN([AS_CHECK_REDHAT_PRE_FC2],
[
	AC_MSG_CHECKING(Pre-FC2 Red Hat/Fedora kernel)
        HAVE_REDHAT_KERNEL=false
        ac_save_CFLAGS="$CFLAGS"
        CFLAGS="$CFLAGS -I${LINUX_DIR}/include/linux"
        AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#include "rhconfig.h"
int code = RED_HAT_LINUX_KERNEL;
	]])],
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
	KERNEL_TYPE=`uname -r | sed 's_^.*\(smp\|enterprise\|bigmem\|hugemem\)$_\1_;t;s_.*__;'`
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
        AS_CHECK_LINUX_CONFIG_OPTION(CONFIG_SMP, KERNEL_TYPE=smp)
	dnl bigmem is also smp, so this check is done after smp to override
        AS_CHECK_LINUX_CONFIG_OPTION(CONFIG_HIGHMEM64G, KERNEL_TYPE=bigmem)

	dnl FIXME: need to check hugemem and enterprise config files, which
	dnl aren't provided in Fedora Core 1 !

	case "$KERNEL_TYPE" in
		smp) SMP='1';;
		enterprise) ENTERPRISE='1';;
		bigmem) BIGMEM='1';;
		hugemem) HUGEMEM='1';;
		*) UP='1';;
	esac
	REDHAT_CFLAGS="-D__MODULE_KERNEL_$KERNEL_ARCH=1"
        REDHAT_CFLAGS="$REDHAT_CFLAGS -D__BOOT_KERNEL_ENTERPRISE=$ENTERPRISE"
        REDHAT_CFLAGS="$REDHAT_CFLAGS -D__BOOT_KERNEL_UP=$UP"
        REDHAT_CFLAGS="$REDHAT_CFLAGS -D__BOOT_KERNEL_SMP=$SMP"
        REDHAT_CFLAGS="$REDHAT_CFLAGS -D__BOOT_KERNEL_BIGMEM=$BIGMEM"
        REDHAT_CFLAGS="$REDHAT_CFLAGS -D__BOOT_KERNEL_HUGEMEM=$HUGEMEM"

	LINUX_REDHAT_CFLAGS="$REDHAT_CFLAGS"

	AC_SUBST(LINUX_KERNEL_TYPE, "$KERNEL_TYPE")
])

dnl add an argument to specify/override the module install path
dnl if nothing specified, it will be /lib/modules/(kernelrel)
AC_DEFUN([AS_LINUX_MODULESDIR],
[
	KERNEL_RELEASE=[$1]
	AC_ARG_WITH([modulesdir],
		[AC_HELP_STRING([--with-modulesdir=DIR],
			[specify path to kernel-specific modules install directory])],
		[MODULESDIR="${withval}"],
		[MODULESDIR=default])

	if test "${MODULESDIR}" = "default" ; then
		MODULESDIR="/lib/modules/${KERNEL_RELEASE}"
	fi
	dnl make it available to Makefiles so it can be used in ...dir
	AC_SUBST(modulesdir, $MODULESDIR)
	AC_MSG_NOTICE([Putting kernel modules under ${MODULESDIR}])
])

dnl add an argument to specify/override the module devel install path
dnl if nothing specified, it will be the passed in LINUXDIR)
AC_DEFUN([AS_LINUX_MODULESDEVELDIR],
[
	LINUXDIR=[$1]
	AC_ARG_WITH([modulesdeveldir],
		[AC_HELP_STRING([--with-modulesdeveldir=DIR],
			[specify path to kernel-specific module development install directory])],
		[MODULESDEVELDIR="${withval}"],
		[MODULESDEVELDIR=default])

	if test "${MODULESDEVELDIR}" = "default" ; then
		MODULESDEVELDIR="${LINUXDIR}"
	fi
	dnl make it available to Makefiles so it can be used in ...dir
	AC_SUBST(modulesdeveldir, $MODULESDEVELDIR)
	AC_MSG_NOTICE([Putting kernel module development files under ${MODULESDEVELDIR}])
])

AC_DEFUN([AS_LINUX_2_6],
[
	AC_MSG_CHECKING(for Linux CFLAGS)

	{
	  tmpdir=`(umask 077 && mktemp -d -q "./confstatXXXXXX") 2>/dev/null` &&
	  test -n "$tmpdir" && test -d "$tmpdir"
	} ||
	{
	  tmpdir=./confstat$$-$RANDOM
	  (umask 077 && mkdir $tmpdir)
	} ||
	{
	  echo "$me: cannot create a temporary directory in ." >&2
	  { (exit 1); exit 1; }
	}

	tmpdir=`pwd`/$tmpdir

	cat >${tmpdir}/Makefile <<EOF
obj-m += fake.o

\$(obj)/fake.c: flags
	touch \$(obj)/fake.c

.PHONY: flags
flags:
	echo LINUX_ARCH=\"\$(ARCH)\" >>\$(obj)/flags
	echo LINUX_AFLAGS=\"\$(AFLAGS)\" | sed 's,include,"\$(LINUXDIR)/include",g'>>\$(obj)/flags
	echo LINUX_LDFLAGS=\"\" >>\$(obj)/flags
	echo LINUX_ARFLAGS=\"\$(ARFLAGS)\" >>\$(obj)/flags
	echo LINUX_CROSS_COMPILE=\"\$(CROSS_COMPILE)\" >>\$(obj)/flags
	echo LINUX_KERNELRELEASE=\"\$(KERNELRELEASE)\" >>\$(obj)/flags
	echo LINUX_CFLAGS=\"\$(CFLAGS) \$(CPPFLAGS)\" \
		| sed -e 's,-Iinclude,-I\$(LINUXDIR)/include,g' -e 's,-include include,-include \$(LINUXDIR)/include,g' >>\$(obj)/flags
	echo LINUX_CFLAGS_MODULE=\"\$(CFLAGS_MODULE)\" >>\$(obj)/flags
	echo LINUX_CC=\"\$(CC)\" >>\$(obj)/flags
	echo LINUX_LD=\"\$(LD) \$(LDFLAGS) \$(LDFLAGS_MODULE)\" >>\$(obj)/flags
	echo LINUX_AS=\"\$(AS)\" >>\$(obj)/flags
	echo LINUX_MODLIB=\"\$(MODLIB)\" >>\$(obj)/flags
EOF

	echo ${MAKE-make} -C ${LINUX_DIR} V=1 M=${tmpdir} SUBDIRS=${tmpdir} LINUXDIR=${LINUX_DIR} modules >&5 2>&5
	${MAKE-make} -C ${LINUX_DIR} V=1 M=${tmpdir} SUBDIRS=${tmpdir} LINUXDIR=${LINUX_DIR} modules >&5 2>&5
	. ${tmpdir}/flags
	rm -rf ${tmpdir}

	LINUX_MODULE_EXT=".ko"
	LINUX_MODULE_STYLE="2.6.6"

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
	AC_SUBST(LINUX_MODULE_STYLE)
	AC_SUBST(LINUX_MODPOST)
	AC_SUBST(LINUX_MODLIB)

	AC_MSG_RESULT([$LINUX_CFLAGS])

	AC_PATH_PROG([LINUX_MODPOST], ["modpost"], ["no"], ["$LINUX_DIR/scripts:$LINUX_DIR/scripts/mod"])
])


AC_DEFUN([AS_LINUX_2_4],
[
	AC_MSG_CHECKING(for Linux 2.4 make flags)
	dnl we try to figure out the CFLAGS by invoking the Makefile on
        dnl a test dir
        dnl we use the correct config file by substituting the MAKEFILES
        dnl Makefile variable, which originally points to .config

	if [[ ! -e "${LINUX_DIR}/.hdepend" ]];then
		AC_MSG_WARN([
No .hdepend file found, you may need to run 'make dep' on the kernel source before continuing.])
	fi

	{
	  tmpdir=`(umask 077 && mktemp -d -q "./confstatXXXXXX") 2>/dev/null` &&
	  test -n "$tmpdir" && test -d "$tmpdir"
	} ||
	{
	  tmpdir=./confstat$$-$RANDOM
	  (umask 077 && mkdir $tmpdir)
	} ||
	{
	  echo "$me: cannot create a temporary directory in ." >&2
	  { (exit 1); exit 1; }
	}
	tmpdir=`pwd`/$tmpdir

	#sed "s|\.config|${CONFIG_FILE}|g" ${LINUX_DIR}/Makefile >$tmpdir/Makefile
	cat >$tmpdir/Makefile <<EOF
modules:
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
	make -C ${LINUX_DIR} SUBDIRS=${tmpdir} modules | grep ^LINUX_ >${tmpdir}/ack
	if (($?)); then
		echo make modules with LINUX_DIR=$(LINUX_DIR) failed.
		exit 1
	fi
	. ${tmpdir}/ack
	rm -rf ${tmpdir}

	LINUX_MODULE_EXT=".o"
	LINUX_MODULE_STYLE="2.4.0"

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
	AC_SUBST(LINUX_MODULE_STYLE)

	AC_MSG_RESULT([ok])

	AC_MSG_CHECKING(for Linux 2.4 CFLAGS)
	AC_MSG_RESULT($LINUX_CFLAGS)
	AC_MSG_CHECKING(for Linux 2.4 LDFLAGS)
	AC_MSG_RESULT($LINUX_LDFLAGS)
])

AC_DEFUN([AS_CHECK_LINUX_CONFIG_OPTION],
[
	AC_MSG_CHECKING([Linux config option $1])

	if grep '^$1=y$' ${LINUX_CONFIG} >/dev/null 2>/dev/null; then
		result=yes
		$2
	else if grep '^$1=m$' ${LINUX_CONFIG} >/dev/null 2>/dev/null; then
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

dnl check for the major/minor version of the Linux source by checking
dnl the Makefile
dnl first argument is the linux directory
dnl sets LINUX_VERSION_MAJOR and LINUX_VERSION_MINOR
AC_DEFUN([AS_LINUX_VERSION_MAJOR_MINOR],
[
	LINUXDIR=[$1]
	AC_MSG_CHECKING([Linux major/minor version])

	if [[ ! -f "${LINUXDIR}/Makefile" ]];then
		AC_MSG_ERROR([The Linux kernel Makefile does not exist.])
	fi
        dnl the next set of tests is for figuring out version major/minor
        dnl use VERSION and PATCHLEVEL in the kernel Makefile
	LINUX_VERSION_MAJOR=`sed -n 's/^VERSION = \([[0-9]]*\)/\1/p' "${LINUXDIR}/Makefile"`
	LINUX_VERSION_MINOR=`sed -n 's/^PATCHLEVEL = \([[0-9]]*\)/\1/p' "${LINUXDIR}/Makefile"`
	if [[ -z "$LINUX_VERSION_MAJOR" -o -z "$LINUX_VERSION_MINOR" ]]; then
		AC_MSG_ERROR([No major/minor version information found in Linux kernel Makefile.])
	fi
        AC_MSG_RESULT($LINUX_VERSION_MAJOR.$LINUX_VERSION_MINOR)
])

# COMEDI_CHECK_LINUX_KBUILD([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel source tree is recent enough to support "Kbuild" files.
AC_DEFUN([COMEDI_CHECK_LINUX_KBUILD],
[
	AC_MSG_CHECKING([for Kbuild support in $1])
	dnl If $1/scripts/Makefile.build refers to $(<something>)/Kbuild
	dnl then we support Kbuild (2.6.10 onwards).
	if grep -q '/Kbuild' "$1/scripts/Makefile.build" 2>/dev/null; then
		AC_MSG_RESULT([yes])
		$2
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_PCMCIA_PROBE([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel pcmcia support is new enough to have a probe member in the pcmcia_driver
# struct.
AC_DEFUN([COMEDI_CHECK_PCMCIA_DRIVER_PROBE],
[
	AC_REQUIRE([AC_PROG_EGREP])
	AC_MSG_CHECKING([$1 for probe in pcmcia_driver struct])
	cat "$1/include/pcmcia/ds.h" | tr \\n ' ' | [$EGREP "struct[[:space:]]+pcmcia_driver[[:space:]]*[{][^}]*probe"] > /dev/null
	if (($?)); then
		AC_MSG_RESULT([no])
		$3
	else
		AC_MSG_RESULT([yes])
		$2
	fi
])

# COMEDI_CHECK_PCMCIA_DRIVER_NAME([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel pcmcia support is new enough to have a name member in the pcmcia_driver
# struct.
AC_DEFUN([COMEDI_CHECK_PCMCIA_DRIVER_NAME],
[
	AC_REQUIRE([AC_PROG_EGREP])
	AC_MSG_CHECKING([$1 for name in pcmcia_driver struct])
	cat "$1/include/pcmcia/ds.h" | tr \\n ' ' | [$EGREP "struct[[:space:]]+pcmcia_driver[[:space:]]*[{][^}]*name"] > /dev/null
	if (($?)); then
		AC_MSG_RESULT([no])
		$3
	else
		AC_MSG_RESULT([yes])
		$2
	fi
])

# COMEDI_CHECK_PCMCIA_LOOP_TUPLE([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel pcmcia support is new enough to have the pcmcia_loop_tuple
# function.
#
# RedHat back-ported it to their 2.6.32 kernel but didn't change prototypes
# of pcmcia_request_window() and pcmcia_map_mem_page() at the same time.
# They kept the older mechanism using pcmcia_get_first_tuple() and
# pcmcia_get_next_tuple(), so ignore backported pcmcia_loop_tuple() if
# pcmcia_get_next_tuple() is defined.
AC_DEFUN([COMEDI_CHECK_PCMCIA_LOOP_TUPLE],
[
	AC_MSG_CHECKING([$1 for pcmcia_loop_tuple function])
	if [grep -q 'int[[:space:]]\+pcmcia_loop_tuple[[:space:]]*(' "$1/include/pcmcia/ds.h"] 2>/dev/null ; then
		if [grep -q '^[[:space:]]*#[[:space:]]*define[[:space:]]\+pcmcia_get_first_tuple(' "$1/include/pcmcia/ds.h"] 2>/dev/null ; then
			AC_MSG_RESULT([yes but ignoring backport])
			$3
		else
			AC_MSG_RESULT([yes])
			$2
		fi
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_DS_DEV_NODE_T([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel pcmcia support is old enough to have the dev_node_t type
# in pcmcia/ds.h.
AC_DEFUN([COMEDI_CHECK_DS_DEV_NODE_T],
[
	AC_MSG_CHECKING([$1 for dev_node_t in pcmcia/ds.h (removed in 2.6.35)])
	if [grep -q dev_node_t "$1/include/pcmcia/ds.h"] 2>/dev/null ; then
		AC_MSG_RESULT([yes])
		$2
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_CS_IRQ_REQ_T([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel pcmcia support is old enough to have the irq_req_t type
# in pcmcia/cs.h.
AC_DEFUN([COMEDI_CHECK_CS_IRQ_REQ_T],
[
	AC_MSG_CHECKING([$1 for irq_req_t in pcmcia/cs.h (removed in 2.6.35)])
	if [grep -q irq_req_t "$1/include/pcmcia/cs.h"] 2>/dev/null ; then
		AC_MSG_RESULT([yes])
		$2
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_CS_IO_REQ_T([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel pcmcia support is old enough to have the io_req_t type
# in pcmcia/cs.h.
AC_DEFUN([COMEDI_CHECK_CS_IO_REQ_T],
[
	AC_MSG_CHECKING([$1 for io_req_t in pcmcia/cs.h (removed in 2.6.36)])
	if [grep -q io_req_t "$1/include/pcmcia/cs.h"] 2>/dev/null ; then
		AC_MSG_RESULT([yes])
		$2
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_CS_MEMREQ_T([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel pcmcia support is old enough to have the memreq_t type
# in pcmcia/cs.h.
AC_DEFUN([COMEDI_CHECK_CS_MEMREQ_T],
[
	AC_MSG_CHECKING([$1 for memreq_t in pcmcia/cs.h (removed in 2.6.36)])
	if [grep -q memreq_t "$1/include/pcmcia/cs.h"] 2>/dev/null ; then
		AC_MSG_RESULT([yes])
		$2
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_HAVE_CS_TYPES_H([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel has <pcmcia/cs_types.h> file.
AC_DEFUN([COMEDI_CHECK_HAVE_CS_TYPES_H],
[
	AC_MSG_CHECKING([$1 for include/pcmcia/cs_types.h (removed in 2.6.36)])
	if test -f "$1/include/pcmcia/cs_types.h"; then
		AC_MSG_RESULT([yes])
		$2
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_HAVE_CS_H([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel has <pcmcia/cs.h> file.
AC_DEFUN([COMEDI_CHECK_HAVE_CS_H],
[
	AC_MSG_CHECKING([$1 for include/pcmcia/cs.h (removed in 2.6.37)])
	if test -f "$1/include/pcmcia/cs.h"; then
		AC_MSG_RESULT([yes])
		$2
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_HAVE_MUTEX_H([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel has <linux/mutex.h> file.
AC_DEFUN([COMEDI_CHECK_HAVE_MUTEX_H],
[
	AC_MSG_CHECKING([$1 for include/linux/mutex.h])
	if test -f "$1/include/linux/mutex.h"; then
		AC_MSG_RESULT([yes])
		$2
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_HAVE_LINUX_SEMAPHORE_H([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
# -------------------------------------------------------------
#
# Check if kernel has <linux/semaphore.h> file.
AC_DEFUN([COMEDI_CHECK_HAVE_LINUX_SEMAPHORE_H],
[
	AC_MSG_CHECKING([$1 for include/linux/semaphore.h])
	if test -f "$1/include/linux/semaphore.h"; then
		AC_MSG_RESULT([yes])
		$2
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_HAVE_GENERIC_BOOL_TYPE([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
#
# -------------------------------------------------------------
# Check if kernel is new enough to have generic 'bool' type in <linux/types.h>
# and 'false' and 'true' enum constants in <linux/stddef.h>.  This was added
# in vanilla 2.6.19, but backported to RHEL5 2.6.18.
AC_DEFUN([COMEDI_CHECK_HAVE_GENERIC_BOOL_TYPE],
[
	AC_MSG_CHECKING([$1 for 'bool' in include/linux/types.h])
	if grep -q 'bool;' "$1/include/linux/types.h" 2>/dev/null; then
		AC_MSG_RESULT([yes])
		$2
	else
		AC_MSG_RESULT([no])
		$3
	fi
])

# COMEDI_CHECK_REQUEST_FIRMWARE_NOWAIT_HAS_GFP([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
#
# -------------------------------------------------------------
# Check if the kernel's request_firmware_nowait() function has the gfp
# parameter.  This was added in vanilla 2.6.33 but Red Hat backported it
# to their 2.6.32 kernel.
AC_DEFUN([COMEDI_CHECK_REQUEST_FIRMWARE_NOWAIT_HAS_GFP],
[
	AC_REQUIRE([AC_PROG_EGREP])
	AC_MSG_CHECKING([$1 for gfp parameter in request_firmware_nowait()])
	$EGREP -q gfp_t "$1/include/linux/firmware.h"
	if (($?)); then
		AC_MSG_RESULT([no])
		$3
	else
		AC_MSG_RESULT([yes])
		$2
	fi
])

# COMEDI_CHECK_HAVE_SET_FS([LINUX_SOURCE_PATH], [ACTION-IF-FOUND], [ACTION-IF-NOT-FOUND])
#
# -------------------------------------------------------------
# Check if the kernel supports set_fs(KERNEL_DS).  This is gradually being
# removed from the kernel.  In kernel version 5.10 it is being removed for
# at least powerpc, riscv and x86 architectures.  Note that the serial2002
# driver relies on it.  In particular its TTY ioctl handling cannot be
# reworked to avoid it.
AC_DEFUN([COMEDI_CHECK_HAVE_SET_FS],
[
	AC_REQUIRE([AC_PROG_EGREP])
	# CONFIG_SET_FS first appeared in kernel 5.10, but could go away
	# once set_fs() support has been removed from all architectures,
	# so this test may not be reliable in the future.
	AS_CHECK_LINUX_CONFIG_OPTION([CONFIG_SET_FS],[set_fs_avail="yes"],[set_fs_avail="yes"],[set_fs_avail="maybe"])
	if test "$set_fs_avail" = "maybe"; then
		# If CONFIG_SET_FS is not configured, that might be due to
		# a pre-5.10 kernel or it might be due to the option being
		# removed in a post-5.10 kernel.  If CONFIG_SET_FS is checked
		# by include/asm-generic/uaccess.h then assume the kernel is
		# pre-5.10 and that set_fs() is supported.  This test will
		# break if post-5.10 kernels remove CONFIG_SET_FS altogether.
		AC_MSG_CHECKING([$1 for set_fs() support])
		$EGREP -q CONFIG_SET_FS "$1/include/asm-generic/uaccess.h" 2>/dev/null
		if (($?)); then
			set_fs_avail="yes"
		else
			set_fs_avail="no"
		fi
		AC_MSG_RESULT($set_fs_avail)
	fi
	if test "$set_fs_avail" = "yes"; then
		$2
	else
		$3
	fi
])
