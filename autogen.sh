#!/bin/sh
if [ ! -f NEWS ] ; then
        touch NEWS
fi
if [ ! -f AUTHORS ] ; then
        touch AUTHORS
fi
if [ ! -f ChangeLog ] ; then
        touch ChangeLog
fi

aclocal -I m4
autoheader
autoconf
automake --add-missing --copy

rm -f config.cache

./configure --enable-maintainer-mode $*

