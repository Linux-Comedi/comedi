
SUBDIRS = comedi include

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
	(for each in `find $(srcdir)/comedi/drivers -name "*\.c" | $(GREP) -v mod\.c | sort`;do $(srcdir)/scripts/dump_doc $$each;\
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

