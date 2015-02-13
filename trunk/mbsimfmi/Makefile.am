SUBDIRS = 3rdparty general createfmu mbsimfmi

# generate deplib files for octave oct-files
install-exec-hook:
	OCTLIBDIR=$$($(mkoctfile) -p OCTLIBDIR | dos2unix | tr '\\' '/'); \
	for i in $$OCTLIBDIR/oct/*/*.oct; do \
	  if test $$i -nt $(prefix)/lib/$$(basename $$i).deplibs; then \
	    echo "Create dependency files for $$i"; \
	    $(prefix)/share/mbxmlutils/python/deplibs.py $$i > $(prefix)/lib/$$(basename $$i).deplibs || exit; \
	  fi; \
	done
