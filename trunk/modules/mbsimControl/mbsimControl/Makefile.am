SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimControl.la
libmbsimControl_la_LDFLAGS = -version-info @LIBMBSIM_CONTROL_VERSION_INFO@
libmbsimControl_la_SOURCES = \
  signal_.cc\
  object_sensors.cc\
  frame_sensors.cc\
  function_sensor.cc\
  link_sensors.cc\
  signal_manipulation.cc\
  signal_processing_system.cc\
  signal_processing_system_sensor.cc\
  linear_transfer_system.cc\
  massless_spring_damper.cc\
  actuator.cc\
  dummyof.cc\
  obsolet_hint.cc


libmbsimControl_la_CPPFLAGS = -DTIXML_USE_STL
libmbsimControl_la_LIBADD = $(DEPS_LIBS)
INCLUDES = $(DEPS_CFLAGS)

mbsimControlincludedir = $(includedir)/mbsimControl
mbsimControlinclude_HEADERS = \
  signal_.h\
  extern_signal_source.h\
  sensor.h\
  object_sensors.h\
  frame_sensors.h\
  function_sensor.h\
  link_sensors.h\
  signal_manipulation.h\
  signal_processing_system.h\
  signal_processing_system_sensor.h\
  linear_transfer_system.h\
  massless_spring_damper.h\
  defines.h\
  actuator.h
