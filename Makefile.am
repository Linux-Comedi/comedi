
SUBDIRS = comedi

EXTRA_DIST = autogen.sh \
	m4/as-linux.m4 \
	m4/as-modtool.m4 \
	m4/rtai.m4 \
	m4/rtlinux.m4 \
	Documentation/comedi/command \
	Documentation/comedi/FAQ \
	Documentation/comedi/notes/atmio_notes \
	Documentation/comedi/notes/README \
	Documentation/comedi/notes/boards \
	Documentation/comedi/notes/regmaps \
	Documentation/comedi/notes/testing \
	Documentation/comedi/Hardware_Driver.HOWTO \
	Documentation/comedi/counter-spec \
	Documentation/comedi/hacks \
	Documentation/comedi/insn_config \
	Documentation/comedi/pci.ids \
	Documentation/comedi/redhat-notes \
	Documentation/comedi/drivers.txt \
	Documentation/comedi/devices.txt \
	Documentation/Configure.help \
	Documentation/Configure.help.append \
	include/asm/pgtable.h \
	include/asm/div64.h \
	include/asm/dma.h \
	include/asm/page.h \
	include/asm/uaccess.h \
	include/asm/types.h \
	include/asm/semaphore.h \
	include/linux/comedi.h \
	include/linux/comedi_rt.h \
	include/linux/comedidev.h \
	include/linux/comedilib.h \
	include/linux/config.h \
	include/linux/devfs_fs_kernel.h \
	include/linux/fs.h \
	include/linux/highmem.h \
	include/linux/init.h \
	include/linux/interrupt.h \
	include/linux/ioport.h \
	include/linux/irq.h \
	include/linux/isapnp.h \
	include/linux/kdev_t.h \
	include/linux/kern_compat.h \
	include/linux/kmod.h \
	include/linux/mm.h \
	include/linux/module.h \
	include/linux/pci.h \
	include/linux/poll.h \
	include/linux/sched.h \
	include/linux/slab.h \
	include/linux/spinlock.h \
	include/linux/timer.h \
	include/linux/usb.h \
	include/linux/version.h \
	include/linux/vmalloc.h \
	include/linux/wait.h \
	include/linux/wrapper.h \
	include/linux/compiler.h \
	include/linux/device.h \
	include/pcmcia/ds.h \
	scripts/check_driver \
	scripts/check_cmdtest \
	scripts/check_kernel \
	scripts/call_trace \
	scripts/doc_devlist \
	scripts/dump_doc

ACLOCAL_AMFLAGS = -I m4

dev:
	-for i in `seq 0 15`; do \
		mknod -m 666 /dev/comedi$$i c 98 $$i \
		; \
	done;

Documentation/comedi/drivers.txt Documentation/comedi/devices.txt:
	(for each in comedi/drivers/*.c;do scripts/dump_doc $$each;done >Documentation/comedi/drivers.txt)
	scripts/doc_devlist Documentation/comedi/drivers.txt | sort >Documentation/comedi/devices.txt

moo:
	(cd comedi/drivers;for each in *.c;do ../../scripts/check_driver $$each;done >../../drivers.check)
	(grep '^.:' drivers.check >drivers.summary)
	
DISTCLEANFILES = modtool

install-data-hook:
	if test x$(DESTDIR) = x ; then \
	  if test x$(prefix) = x/ ; then \
	    $(DEPMOD) -ae $(LINUX_KERNELRELEASE) ; \
	  fi \
	fi

