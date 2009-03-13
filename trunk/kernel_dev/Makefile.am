SUBDIRS = mbsim

DIST_SUBDIRS = mbsim doc
EXTRA_DIST = doc/macros.sty

#bin_SCRIPTS = mbsim-config

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = mbsim.pc

# wrapper target for doxygen in dir doc
.PHONY: doc clean-doc install-doc uninstall-doc

doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) all)

clean-doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) clean)

install-doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) install)

uninstall-doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) uninstall)
