#!/bin/sh

autoreconf &&
./configure --enable-maintainer-mode $*

