
AC_DEFUN(DS_RTAI,
[
	AC_ARG_WITH([rtaidir],
		[AC_HELP_STRING([--with-rtaidir=DIR],
			[specify path to RTAI source directory])],
		[RTAI_DIR="${withval}"],
		[RTAI_DIR=/usr/src/rtai])

	AS_LINUX_CONFIG_OPTION_MODULE([CONFIG_RTHAL])

	if test "${CONFIG_RTHAL}" != "no" ; then
		AC_MSG_CHECKING([RTAI directory ${RTAI_DIR}])
		if [[ -d ${RTAI_DIR}/include ]] ; then
			RTAI_CFLAGS="-I${RTAI_DIR}/include"
		else
			if [ -d ${RTAI_DIR}/rtai-core/include ] ; then
				RTAI_CFLAGS="-I${RTAI_DIR}/rtai-core/include"
			else
				AC_MSG_ERROR([incorrect RTAI directory?])
			fi
		fi
		$1
		AC_MSG_RESULT([found])
		AC_DEFINE([CONFIG_COMEDI_RTAI],[true],[Define if kernel is RTAI patched])
	else
		$2
	fi
	AC_SUBST(RTAI_CFLAGS)

])

