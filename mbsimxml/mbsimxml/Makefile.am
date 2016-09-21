lib_LTLIBRARIES = libmbsimflatxml.la libmbsimxml.la
libmbsimflatxml_la_SOURCES = mbsimflatxml.cc
libmbsimflatxml_la_CPPFLAGS = $(DEPS_CFLAGS)
libmbsimflatxml_la_LIBADD = $(DEPS_LIBS) -l@BOOST_FILESYSTEM_LIB@ -l@BOOST_SYSTEM_LIB@ $(EXTRA_LIBS)
mbsimflatxmlincludedir = $(includedir)/mbsimxml
mbsimflatxmlinclude_HEADERS = mbsimflatxml.h

libmbsimxml_la_SOURCES = mbsimxml.cc
libmbsimxml_la_CPPFLAGS = $(DEPS_CFLAGS)
libmbsimxml_la_LIBADD = $(DEPS_LIBS)
mbsimxmlincludedir = $(includedir)/mbsimxml
mbsimxmlinclude_HEADERS = mbsimxml.h


bin_PROGRAMS = mbsimflatxml mbsimxml

mbsimflatxml_CPPFLAGS = $(DEPS_CFLAGS)
mbsimflatxml_LDFLAGS = -L.
mbsimflatxml_LDADD = $(DEPS_LIBS) ./libmbsimflatxml.la -lmbsim -l@BOOST_TIMER_LIB@ -l@BOOST_CHRONO_LIB@
mbsimflatxml_SOURCES = mbsimflatxml-main.cc

mbsimxml_CPPFLAGS = $(DEPS_CFLAGS)
mbsimxml_LDADD = ./libmbsimxml.la $(DEPS_LIBS) -l@BOOST_FILESYSTEM_LIB@ -l@BOOST_SYSTEM_LIB@
mbsimxml_SOURCES = mbsimxml-main.cc

schemadir = @MBXMLUTILSSCHEMA@/http___www_mbsim-env_de_MBSimXML
dist_schema_DATA = mbsimproject.xsd

mbsimmoduleschemadir = @MBXMLUTILSSCHEMA@/http___www_mbsim-env_de_MBSimModule
dist_mbsimmoduleschema_DATA = mbsimmodule.xsd
