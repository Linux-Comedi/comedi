VERSION = 0
PATCHLEVEL = 7
SUBLEVEL = 48
EXTRAVERSION =

PROJECT = COMEDI
project = comedi

SUBDIRS := comedi

include ./Makefile.modbuild

dev:
	mknod -m=666 /dev/comedi0 c 98 0
	mknod -m=666 /dev/comedi1 c 98 0
	mknod -m=666 /dev/comedi2 c 98 0
	mknod -m=666 /dev/comedi3 c 98 0

