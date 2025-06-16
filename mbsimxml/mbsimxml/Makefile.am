if COND_WIN32
  MAYBE_WIN32_mbsimxml_OBJ = mbsimxml.o
  MAYBE_WIN32_mbsimflatxml_OBJ = mbsimflatxml.o
endif

# windows rc rules
mbsimflatxml.o: $(srcdir)/mbsimflatxml.rc $(srcdir)/mbsimflatxml.manifest
	$(WINDRES) $< -o $@
mbsimxml.o: $(srcdir)/mbsimxml.rc $(srcdir)/mbsimxml.manifest
	$(WINDRES) $< -o $@

lib_LTLIBRARIES = libmbsimflatxml.la libmbsimxml.la
libmbsimflatxml_la_SOURCES = mbsimflatxml.cc
libmbsimflatxml_la_CPPFLAGS = $(DEPS_CFLAGS) $(PYCPPWRAPPER_CFLAGS) -D_hypot=hypot # -D_hypot is a MSVS/python bug workaround
libmbsimflatxml_la_LIBADD = $(DEPS_LIBS) $(PYCPPWRAPPER_LIBS) -l@BOOST_FILESYSTEM_LIB@ -l@BOOST_SYSTEM_LIB@ $(EXTRA_LIBS)
mbsimflatxmlincludedir = $(includedir)/mbsimxml
mbsimflatxmlinclude_HEADERS = namespace.h mbsimflatxml.h

libmbsimxml_la_SOURCES = mbsimxml.cc project.cc
libmbsimxml_la_CPPFLAGS = $(DEPS_CFLAGS) $(PYCPPWRAPPER_CFLAGS) -D_hypot=hypot # -D_hypot is a MSVS/python bug workaround
libmbsimxml_la_LIBADD = $(DEPS_LIBS) $(PYCPPWRAPPER_LIBS)
mbsimxmlincludedir = $(includedir)/mbsimxml
mbsimxmlinclude_HEADERS = mbsimxml.h set_current_path.h project.h


bin_PROGRAMS = mbsimflatxml mbsimxml

mbsimflatxml_CPPFLAGS = $(DEPS_CFLAGS)
mbsimflatxml_LDFLAGS = -L.  $(PYCPPWRAPPER_LIBS_MAIN)
mbsimflatxml_LDADD = $(DEPS_LIBS) ./libmbsimflatxml.la $(PYCPPWRAPPER_LIBS) -lmbsim -l@BOOST_TIMER_LIB@ -l@BOOST_CHRONO_LIB@ $(MAYBE_WIN32_mbsimflatxml_OBJ)
mbsimflatxml_SOURCES = mbsimflatxml-main.cc

mbsimxml_CPPFLAGS = $(DEPS_CFLAGS) $(MBXMLUTILS_CFLAGS)
mbsimxml_LDFLAGS = $(PYCPPWRAPPER_LIBS_MAIN)
mbsimxml_LDADD = ./libmbsimflatxml.la ./libmbsimxml.la $(DEPS_LIBS) $(PYCPPWRAPPER_LIBS) $(MBXMLUTILS_LIBS) -l@BOOST_FILESYSTEM_LIB@ -l@BOOST_SYSTEM_LIB@ $(MAYBE_WIN32_mbsimxml_OBJ)
mbsimxml_SOURCES = mbsimxml-main.cc

schemadir = @MBXMLUTILSSCHEMA@/http___www_mbsim-env_de_MBSimXML
dist_schema_DATA = mbsimproject.xsd

mbsimmoduleschemadir = @MBXMLUTILSSCHEMA@/http___www_mbsim-env_de_MBSimModule
dist_mbsimmoduleschema_DATA = mbsimmodule.xsd mbsimmoduleCatalog.xml

# freedesktop.org
freedesktoporgdir = @datadir@/mbsim-env/freedesktop.org
freedesktoporg_DATA = freedesktop.org/mbsim-env.mbsimxml.desktop freedesktop.org/mbsim-env.mbsimcreatefmu-me.desktop freedesktop.org/mbsim-env.mbsimcreatefmu-cosim.desktop freedesktop.org/mbsim-env.mbsimxmldoc.desktop freedesktop.org/mbsim-env.mbsimxml.xml freedesktop.org/mimeapps-mbsimxml.list icons/mbsimxml.svg icons/mbsimxmlfmu.svg icons/mbsimxmldoc.svg icons/mbsimxmldoc.ico
bin_SCRIPTS = freedesktop.org/desktopIntegration.sh freedesktop.org/desktopIntegration.bat freedesktop.org/desktopIntegration.py
