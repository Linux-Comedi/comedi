#!/bin/sh
# genchanges.sh - generate a changes file for a deb file generated via
#	the make-kpkg utility

# KSRC KMAINT and KEMAIL are expected to be passed through the environment

set -e
umask 022

KVERS=`cat debian/KVERS`
MODVERS=`cat debian/MODVERS`
ARCH=`dpkg --print-architecture`

# determine the maintainer's name
for name in "$KMAINT" "$DEBFULLNAME" "$DEBNAME"
do test -n "$name" && break; done
for email in "$KEMAIL" "$DEBEMAIL"
do test -n "$email" && break; done
if [ "$name" -a "$email" ]; then maint="$name <$email>"
elif [ "$email" ]; then maint="$email"
else maint=""; fi
    
dpkg-genchanges -b ${maint:+-e"$maint"} -u"$KSRC/.." \

# the changes file's name
chfile="$KSRC/../pcmcia-modules-${KVERS}_${MODVERS}_${ARCH}.changes"

dpkg-genchanges -b ${KMAINT:+-e"$maint"} -u"$KSRC/.." \
	-cdebian/control.tmp > "$chfile.pt"
if test -e "${GNUPGHOME:-$HOME/.gnupg/secring.gpg}"; then
    gpg -ast ${email:+-u"$email"} \
	--clearsign < "$chfile.pt" > "$chfile"
else
    pgp -fast ${email:+-u"$email"} +clearsig=on \
	< "$chfile.pt" > "$chfile"
fi
rm "$chfile.pt"
