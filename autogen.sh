#!/bin/sh

autoreconf -i -f &&
./configure --enable-maintainer-mode $*

