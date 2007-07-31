
SUBDIRS = comedi

EXTRA_DIST = autogen.sh \
	AUTHORS \
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
	include/linux/compiler.h \
	include/linux/config.h \
	include/linux/delay.h \
	include/linux/device.h \
	include/linux/interrupt.h \
	include/linux/isapnp.h \
	include/linux/kernel.h \
	include/linux/kref.h \
	include/linux/mm.h \
	include/linux/mod_devicetable.h \
	include/linux/module.h \
	include/linux/moduleparam.h \
	include/linux/pci.h \
	include/linux/pci_ids.h \
	include/linux/pnp.h \
	include/linux/sched.h \
	include/linux/time.h \
	include/linux/types.h \
	include/linux/usb.h \
	include/linux/version.h \
	include/linux/wrapper.h \
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
	(for each in `find $(srcdir)/comedi/drivers -name "*\.c" | $(GREP) -v mod\.c`;do $(srcdir)/scripts/dump_doc $$each;\
		done >$(builddir)/Documentation/comedi/drivers.txt)
	$(srcdir)/scripts/doc_devlist $(builddir)/Documentation/comedi/drivers.txt | sort >\
		$(builddir)/Documentation/comedi/devices.txt

moo:
	(for each in `find $(srcdir)/comedi/drivers -name "*\.c" | $(GREP) -v mod\.c`;\
		do $(srcdir)/scripts/check_driver $$each;done >$(builddir)/drivers.check)
	($(GREP) '^.:' $(builddir)/drivers.check >$(builddir)/drivers.summary)

DISTCLEANFILES = modtool

install-data-hook:
	@echo "***"
	@echo "***"
	@echo "*** Don't forget to run 'depmod -a'"
	@echo "***"
	@echo "***"

