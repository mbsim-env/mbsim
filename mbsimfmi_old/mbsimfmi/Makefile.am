lib_LTLIBRARIES = libmbsimfmi.la
libmbsimfmi_la_SOURCES = \
  fmiModelFunctions.cc \
  model_instance.cc \
  system.cc \
  fmi_xml_export.cc \
  fmi_utils.cc \
  fmi_scalar_variable.cc \
  fmi_scalar_variable_container.cc
libmbsimfmi_la_CPPFLAGS = $(DEPS_CFLAGS)
libmbsimfmi_la_LDFLAGS = -version-info @LIBMBSIMFMI_VERSION_INFO@ -Wl,-rpath,\$$ORIGIN
libmbsimfmi_la_LIBADD = $(DEPS_LIBS) -lmbsimflatxml -lmbsimControl -l@BOOST_FILESYSTEM_LIB@ -l@BOOST_SYSTEM_LIB@ $(EXTRA_LIBS)

mbsimfmiincludedir = $(includedir)/mbsimfmi
mbsimfmiinclude_HEADERS = \
  fmiModelTypes.h \
  fmiModelFunctions.h \
  model_instance.h \
  system.h \
  fmi_xml_export.h \
  fmi_utils.h \
  fmi_scalar_variable.h \
  fmi_scalar_variable_container.h \
  fmi_scalar_variable_container_impl.h \
  fmi_scalar_variable_unit.h \
  fmi_scalar_variable_unit_impl.h \
  fmi_enum.h
  
  
bin_PROGRAMS = fmixmlexport

# generate PIC code also for the mbsimflatxml executable: this is required for dlopen with RTLD_DEEPBIND!!!
fmixmlexport_CPPFLAGS = $(DEPS_CFLAGS) $(MBSIMXML_MINGW)
fmixmlexport_LDFLAGS = -L. -Wl,-rpath -Wl,\$$ORIGIN/../lib
fmixmlexport_LDADD = $(DEPS_LIBS) ./libmbsimfmi.la -lmbsim -lmbsimflatxml
fmixmlexport_SOURCES = fmixmlexport-main.cc
