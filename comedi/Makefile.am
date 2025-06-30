
SUBDIRS = . kcomedilib drivers

CLEANFILES = Modules.symvers Module.symvers

COMEDI_MOD_DIR = "comedi"

if COMEDI_ENABLE_MODULESDIR_PREFIX

MOD_PATH_PREFIX = $(prefix)

else

MOD_PATH_PREFIX =

endif

if CONFIG_KBUILD

COMEDI_PREPEND_CFLAGS = \
	-I@abs_top_builddir@/include \
	-I@abs_top_srcdir@/include -I@abs_top_srcdir@/inc-wrap \
	$(RTAI_CFLAGS) $(RTLINUX_CFLAGS)

export COMEDI_PREPEND_CFLAGS

module_PROGRAMS =
all-local: comedi_kbuild.inc
#if builddir is not srcdir, make symbolic links in the builddir to the source files so
#the kernel's build system doesn't get lost.
	if test "$(srcdir)" != "."; then \
		for dir in . drivers kcomedilib drivers/addi-data; do \
			for file in `ls @abs_srcdir@/$$dir/*\.[ch] @abs_srcdir@/$$dir/Kbuild | $(EGREP) -v \.mod\.c`; do \
				LINK_NAME=$$dir/`basename "$$file"`; \
				if test ! "$$file" -ef "$$LINK_NAME"; then ln -vsnf "$$file" "$$LINK_NAME"; fi; \
			done; \
		done; \
	fi
# Invoke kernel's build system.
# For nfs filesystem with root_squash, you may need to change the following
# "$(MAKE)" to "-$(MAKE)" to avoid failure of make install.
	$(MAKE) -I@abs_builddir@ -C $(LINUX_DIR) M=@abs_builddir@ modules
install-data-local: comedi_kbuild.inc
	$(MAKE) -I@abs_builddir@ -C $(LINUX_DIR) M=@abs_builddir@ \
		INSTALL_MOD_DIR=$(COMEDI_MOD_DIR) \
		INSTALL_MOD_PATH="$(DESTDIR)$(MOD_PATH_PREFIX)" modules_install
clean-local:
	if test -f comedi_kbuild.inc ; then \
	  $(MAKE) -I@abs_builddir@ -C $(LINUX_DIR) M=@abs_builddir@ clean ; \
	fi
	-rm -f comedi_kbuild.inc

# This nonsense is because comedi_kbuild.inc was being removed too early by
# `make distclean` when make is running parallel jobs.
comedi_kbuild.inc: comedi_kbuild_copy.inc
	cp $< $@

else
module_PROGRAMS = comedi.ko
all-local:
install-data-local:
clean-local:
endif

EXTRA_DIST = \
 Kbuild \
 rt_pend_tq.h \
 comedi_fops.h \
 comedi_compat32.h

if COMEDI_CONFIG_RT
RT_SOURCES = rt_pend_tq.c rt.c
else
RT_SOURCES =
endif

comedi_ko_SOURCES = \
 comedi_fops.c \
 proc.c \
 range.c \
 drivers.c \
 comedi_compat32.c \
 comedi_ksyms.c \
 $(RT_SOURCES)

comedi_ko_CFLAGS = $(COMEDI_CFLAGS) $(LINUX_CFLAGS) $(RTAI_CFLAGS) $(RTLINUX_CFLAGS)
comedi_ko_LINK = $(top_builddir)/modtool --link -o $@

#comedi_ksyms.o: comedi_ksyms.c
#	$(CC) -o $@ comedi_ksyms.c $(CFLAGS) $(COMEDI_CFLAGS) $(LINUX_CFLAGS) -DEXPORT_SYMTAB

clean-modulePROGRAMS:
	-test -z "$(module_PROGRAMS)" || rm -f $(module_PROGRAMS)
	-rm -rf .mods
