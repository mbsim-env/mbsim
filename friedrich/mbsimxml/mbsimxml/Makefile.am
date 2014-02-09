lib_LTLIBRARIES = libmbsimflatxml.la
libmbsimflatxml_la_SOURCES = mbsimflatxml.cc
libmbsimflatxml_la_CPPFLAGS = $(DEPS_CFLAGS)
libmbsimflatxml_la_LDFLAGS = 
libmbsimflatxml_la_LIBADD = $(DEPS_LIBS) -l@BOOST_FILESYSTEM_LIB@ -l@BOOST_SYSTEM_LIB@
mbsimxmlincludedir = $(includedir)/mbsimxml
mbsimxmlinclude_HEADERS = mbsimflatxml.h


bin_PROGRAMS = mbsimflatxml mbsimxml

# generate PIC code also for the mbsimflatxml executable: this is required for dlopen with RTLD_DEEPBIND!!!
mbsimflatxml_CPPFLAGS = $(DEPS_CFLAGS) $(MBSIMXML_MINGW)
mbsimflatxml_LDFLAGS = -L. -Wl,-rpath -Wl,\$$ORIGIN/../lib
mbsimflatxml_LDADD = $(DEPS_LIBS) ./libmbsimflatxml.la -lmbsim
mbsimflatxml_SOURCES = mbsimflatxml-main.cc

mbsimxml_CPPFLAGS = $(DEPS_CFLAGS)
mbsimxml_LDFLAGS = -Wl,-rpath -Wl,\$$ORIGIN/../lib
mbsimxml_LDADD = $(DEPS_LIBS) -l@BOOST_FILESYSTEM_LIB@ -l@BOOST_SYSTEM_LIB@
mbsimxml_SOURCES = mbsimxml.cc

schemadir = @MBXMLUTILSSCHEMA@/http___mbsim_berlios_de_MBSimXML
dist_schema_DATA = mbsimproject.xsd
