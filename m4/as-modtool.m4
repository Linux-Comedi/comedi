dnl as-modtool.m4 0.0.1
dnl autostars m4 macro for building modtool, a linker for Linux kernel
dnl modules
dnl
dnl David Schleef <ds@schleef.org>
dnl Frank Mori Hess <fmhess@users.sourceforge.net>
dnl thomas@apestaart.org
dnl
dnl AS_LINUX_MODTOOL()
dnl
dnl this macro defines:
dnl moduledir
dnl modulePROGRAMS_INSTALL
dnl modulePROGRAMS_UNINSTALL
dnl
dnl End of search list.

dnl
dnl FIXME:
dnl  How do you specify that the building of modtool should go to the
dnl  end of the configure script?
dnl

AC_DEFUN([AS_LINUX_MODTOOL],
[
	#AS_LINUX()

	moduledir="\$(libdir)/modules/\$(LINUX_KERNELRELEASE)/comedi"
	modulePROGRAMS_INSTALL="\$(top_builddir)/modtool --install"
	modulePROGRAMS_UNINSTALL="\$(top_builddir)/modtool --uninstall"
	AC_SUBST(moduledir)
	AC_SUBST(modulePROGRAMS_INSTALL)

	AC_MSG_NOTICE(creating modtool)
	cat >modtool <<EOF
#!/bin/sh

LINUX_LD="$LINUX_LD"
CC="$LINUX_CC"
INSTALL="$INSTALL"
LINUX_MODULE_EXT="$LINUX_MODULE_EXT"
STRIP="$STRIP"

mode=\$[1]
shift

case \$mode in
--link)
	echo \$LINUX_LD -r \$[*]
	\$LINUX_LD -r \$[*]
	;;
--install)
	module_src=\$[1]
	module_dest=\`echo \$[2] | sed "s/\.ko$/\${LINUX_MODULE_EXT}/"\`
	echo \$INSTALL -m644 "\$module_src" "\$module_dest"
	\$INSTALL -m644 "\$module_src" "\$module_dest"
	\$STRIP -g "\$module_dest"
	;;
--uninstall)
	module_src=\$[1]
	module_dest=\`echo \$[2] | sed "s/\.ko$/\${LINUX_MODULE_EXT}/"\`
	echo uninstall "\$module_src" "\$module_dest"
	rm -f "\$module_dest"
	;;
*)
	echo Unknown mode \$mode >&2
	exit 1
esac

EOF
	chmod +x modtool

])


