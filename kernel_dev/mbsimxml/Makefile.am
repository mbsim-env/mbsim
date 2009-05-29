SUBDIRS = schema .



if COND_MBXMLUTILS
  MAYBE_MBSIMXML = mbsimxml
endif
bin_SCRIPTS = $(MAYBE_MBSIMXML)



# avoid double linking again lib*tinyxml if openmbvcppinterface is used
if COND_OPENMBVCPPINTERFACE
else
  MAYBE_MBSIMTINYXML = -lmbsimtinyxml
endif

bin_PROGRAMS = mbsimflatxml

mbsimflatxml_CPPFLAGS = $(DEPS_CFLAGS)
mbsimflatxml_LDFLAGS = -L../mbsimtinyxml -L../mbsim
mbsimflatxml_LDADD = -lmbsim $(OPENMBVCPPINTERFACE_LIBS) $(MAYBE_MBSIMTINYXML)

mbsimflatxml_SOURCES = mbsimflatxml.cc



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
