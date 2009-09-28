bin_SCRIPTS = mbsimxml mbsimxml.bat


lib_LTLIBRARIES = libmbsimflatxml.la
libmbsimflatxml_la_SOURCES = mbsimflatxml.cc
libmbsimflatxml_la_CPPFLAGS = $(DEPS_CFLAGS)
libmbsimflatxml_la_LDFLAGS = 
libmbsimflatxml_la_LIBADD = $(DEPS_LIBS) $(MBSIMMODULES_LIBS)
mbsimxmlincludedir = $(includedir)/mbsimxml
mbsimxmlinclude_HEADERS = mbsimflatxml.h headermodules.h


bin_PROGRAMS = mbsimflatxml

mbsimflatxml_CPPFLAGS = $(DEPS_CFLAGS)
mbsimflatxml_LDFLAGS = 
mbsimflatxml_LDADD = -lmbsimflatxml

mbsimflatxml_SOURCES = mbsimflatxml-main.cc
