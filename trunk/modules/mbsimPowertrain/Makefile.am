if COND_XML
  MAYBE_SCHEMA = schema
endif

SUBDIRS = mbsimPowertrain $(MAYBE_SCHEMA)

DIST_SUBDIRS = mbsimPowertrain doc schema

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = mbsimPowertrain.pc

plugindir = $(datadir)/mbsimxml/plugins
plugin_DATA = mbsimPowertrain.plugin.xml

# wrapper target for doxygen in dir doc
.PHONY: doc clean-doc install-doc uninstall-doc xmldoc clean-xmldoc intall-xmldoc uninstall-xmldoc

doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) all)

clean-doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) clean)

install-doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) install)

uninstall-doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) uninstall)
