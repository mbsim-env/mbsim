SUBDIRS = schema .



if COND_MBXMLUTILS
  MAYBE_MBSIMXMLPP = mbsimxmlpp
endif
bin_SCRIPTS = $(MAYBE_MBSIMXMLPP)



bin_PROGRAMS = mbsimxml

mbsimxml_CPPFLAGS = $(DEPS_CFLAGS)
mbsimxml_LDFLAGS = -L../mbsimtinyxml -L../mbsim
mbsimxml_LDADD = -lmbsim $(OPENMBVCPPINTERFACE_LIBS) -lmbsimtinyxml

mbsimxml_SOURCES = mbsimxml.cc



# doc
.PHONY: doc doc_install doc_uninstall
doc:
	cd doc; \
	$(MAKE) $(AM_MAKEFLAGS)
doc_install:
	cd doc; \
	$(MAKE) $(AM_MAKEFLAGS) install
doc_uninstall:
	cd doc; \
	$(MAKE) $(AM_MAKEFLAGS) uninstall
