lib_LTLIBRARIES = libmbsimflatxml.la
libmbsimflatxml_la_SOURCES = mbsimflatxml.cc
libmbsimflatxml_la_CPPFLAGS = $(DEPS_CFLAGS) $(MBSIMMODULES_CFLAGS)
libmbsimflatxml_la_LDFLAGS = 
libmbsimflatxml_la_LIBADD = $(DEPS_LIBS) $(MBSIMMODULES_LIBS)
mbsimxmlincludedir = $(includedir)/mbsimxml
mbsimxmlinclude_HEADERS = mbsimflatxml.h headermodules.h


bin_PROGRAMS = mbsimflatxml mbsimxml

mbsimflatxml_CPPFLAGS = $(DEPS_CFLAGS) $(MBSIMMODULES_CFLAGS)
mbsimflatxml_LDFLAGS = -L. -Wl,-rpath -Wl,\$$ORIGIN/../lib
mbsimflatxml_LDADD = ./libmbsimflatxml.la -lmbsim -lboost_system  -lboost_filesystem
mbsimflatxml_SOURCES = mbsimflatxml-main.cc

mbsimxml_LDFLAGS = -Wl,-rpath -Wl,\$$ORIGIN/../lib
mbsimxml_LDADD = -lboost_system -lboost_filesystem
mbsimxml_SOURCES = mbsimxml.cc

