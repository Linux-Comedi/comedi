
AC_DEFUN([DS_RTAI],
[
	AC_ARG_WITH([rtaidir],
		[AC_HELP_STRING([--with-rtaidir=DIR],
			[specify path to RTAI installation or build directory])],
		[RTAI_DIR="${withval}"],
		[RTAI_DIR=/usr/realtime])

	AS_LINUX_CONFIG_OPTION_MODULE([CONFIG_RTHAL])
	AS_LINUX_CONFIG_OPTION_MODULE([CONFIG_ADEOS])
	AS_LINUX_CONFIG_OPTION_MODULE([CONFIG_IPIPE])
	
	if test "${ENABLE_RTAI}" = "yes" -a \( "${CONFIG_RTHAL}" != "no" -o "${CONFIG_ADEOS}" != "no" -o "${CONFIG_IPIPE}" != "no" \); then
		AC_MSG_CHECKING([RTAI directory ${RTAI_DIR}])
		RTAI_CFLAGS=""
		RTAI_CONFIG=""
		if [[ -d ${RTAI_DIR}/base -a \
			-f ${RTAI_DIR}/GNUmakefile -a \
			-f ${RTAI_DIR}/base/scripts/rtai-config ]]; then
			RTAI_CONFIG="${SHELL} ${RTAI_DIR}/base/scripts/rtai-config"
			RTAI_CFLAGS="-I${RTAI_DIR} -I${RTAI_DIR}/base/include"
			RTAI_VPATH=`sed -n -e 's/^VPATH *= *\(.*\)/\1/p' ${RTAI_DIR}/GNUmakefile`
			if [[ -n "${RTAI_VPATH}" -a "${RTAI_VPATH}" != "." ]]; then
				RTAI_CFLAGS="${RTAI_CFLAGS} -I${RTAI_DIR}/${RTAI_VPATH}/base/include"
			fi
		else
			if [[ -x "${RTAI_DIR}/bin/rtai-config" ]]; then
				RTAI_CONFIG="${RTAI_DIR}/bin/rtai-config"
			fi
			if [[ -d ${RTAI_DIR}/include/rtai ]] ; then # for Debian
				RTAI_CFLAGS="-I${RTAI_DIR}/include/rtai"
			elif [[ -d ${RTAI_DIR}/include ]] ; then
				RTAI_CFLAGS="-I${RTAI_DIR}/include"
			elif [[ -d ${RTAI_DIR}/rtai-core/include ]] ; then
				RTAI_CFLAGS="-I${RTAI_DIR} -I${RTAI_DIR}/rtai-core/include"
			fi
		fi
		if [[ -z "${RTAI_CFLAGS}" -o -z "${RTAI_CONFIG}" ]]; then
			AC_MSG_ERROR([incorrect RTAI directory?])
		fi
		$1
		AC_MSG_RESULT([found])
		FUSION_TEST=`${RTAI_CONFIG} --version | cut -d"-" -f2`
		if test "${FUSION_TEST}" = "fusion"
		then
			AC_DEFINE([COMEDI_CONFIG_FUSION],[true],[Define if kernel is RTAI patched])
		else
			AC_DEFINE([COMEDI_CONFIG_RTAI],[true],[Define if kernel is RTAI patched])
		fi

	else
		$2
	fi
	AC_SUBST(RTAI_CFLAGS)
])

