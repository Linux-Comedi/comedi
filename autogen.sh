#!/bin/sh

aclocal-1.7 -I m4
autoheader
autoconf
automake-1.7 --add-missing --copy

rm -f config.cache

./configure --enable-maintainer-mode $*

