SUBDIRS = mbsimFlexibleBody schema

DIST_SUBDIRS = mbsimFlexibleBody schema

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = mbsimFlexibleBody.pc

licdir = @datadir@/mbsimFlexibleBody
lic_DATA = COPYING NOTICE

mbsimmodulesdir = $(datadir)/mbsimmodules
mbsimmodules_DATA = mbsimFlexibleBody.mbsimmodule.xml

# wrapper target for doxygen in dir doc
.PHONY: doc clean-doc install-doc uninstall-doc xmldoc clean-xmldoc intall-xmldoc uninstall-xmldoc

doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) all)
xmldoc:
	(cd xmldoc && $(MAKE) $(AM_MAKEFLAGS) all)

clean-doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) clean)
clean-xmldoc:
	(cd xmldoc && $(MAKE) $(AM_MAKEFLAGS) clean)

install-doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) install)
install-xmldoc:
	(cd xmldoc && $(MAKE) $(AM_MAKEFLAGS) install)

uninstall-doc:
	(cd doc && $(MAKE) $(AM_MAKEFLAGS) uninstall)
uninstall-xmldoc:
	(cd xmldoc && $(MAKE) $(AM_MAKEFLAGS) uninstall)

# create clang compile database if compiledb command is found
all-local: compile_commands.json
check-local: compile_commands.json
install-data-local: compile_commands.json
install-exec-local: compile_commands.json
compile_commands.json: config.log
	command -v compiledb &> /dev/null && compiledb -n -o compile_commands.json make && ln -fs $(abs_top_builddir)/compile_commands.json $(top_srcdir)/compile_commands.json || echo "Failed to create clang compile DB"
