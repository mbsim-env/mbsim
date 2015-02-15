SUBDIRS = 3rdparty general createfmu mbsimfmi

# generate deplib files for octave oct-files
install-exec-hook:
	OCTLIBDIR=$$($(mkoctfile) -p OCTLIBDIR | dos2unix | tr '\\' '/'); \
        test "$(build)" != "$(host)" && OCTLIBDIR=$$(readlink -f $$(winepath -u $$OCTLIBDIR)); \
	for i in $$OCTLIBDIR/oct/*/*.oct; do \
	  if test $$i -nt $(prefix)/lib/$$(basename $$i).deplibs; then \
	    echo "Create dependency files for $$i"; \
	    python $(prefix)/share/mbxmlutils/python/deplibs.py $$i > $(prefix)/lib/$$(basename $$i).deplibs || exit; \
	  fi; \
	done
	if test $(prefix)/bin/casadi_interface.oct -nt $(prefix)/lib/casadi_interface.oct.deplibs; then \
	  echo "Create dependency files for $(prefix)/bin/casadi_interface.oct"; \
	  python $(prefix)/share/mbxmlutils/python/deplibs.py $(prefix)/bin/casadi_interface.oct > $(prefix)/lib/casadi_interface.oct.deplibs || exit; \
	fi
