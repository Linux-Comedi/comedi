#!/bin/sh

# function saves 'COPYING' and 'INSTALL' as they get clobbered.
save () {
	COPYING_ok=false
	INSTALL_ok=false
	cp COPYING COPYING.save && COPYING_ok=true
	cp INSTALL INSTALL.save && INSTALL_ok=true
}

# function restores 'COPYING' and 'INSTALL' which have probably been clobbered.
restore () {
	$COPYING_ok && mv COPYING.save COPYING
	$INSTALL_ok && mv INSTALL.save INSTALL
}

# save the files before they get clobbered.
save
trap "restore; exit 130" 1 2 15

# run autoreconf. The -i and -f flags causes files to be overwritten.
autoreconf_ok=false
autoreconf -i -f && autoreconf_ok=true

# unclobber the overwritten files.
restore
trap - 1 2 15

# re-run configure script if autoreconf was successful.
$autoreconf_ok && ./configure --enable-maintainer-mode $*

