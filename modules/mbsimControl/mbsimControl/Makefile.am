SUBDIRS =  .
  
lib_LTLIBRARIES = libmbsimControl.la
libmbsimControl_la_LDFLAGS = -avoid-version
libmbsimControl_la_SOURCES = \
  signal_.cc\
  object_sensors.cc\
  rigid_body_sensors.cc\
  frame_sensors.cc\
  function_sensor.cc\
  link_sensors.cc\
  mechanical_link_sensors.cc\
  contact_sensor.cc\
  tyre_contact_sensor.cc\
  constraint_sensors.cc\
  signal_sensors.cc\
  signal_manipulation.cc\
  linear_transfer_system.cc\
  nonlinear_transfer_system.cc\
  massless_spring_damper.cc\
  signal_function.cc\
  extern_signal_sink.cc\
  frame_chaser_observer.cc \
  signal_observer.cc \
  motion_observer.cc \
  switch.cc \
  stop.cc \
  duration.cc \
  state_machine.cc \
  state_machine_sensor.cc \
  linear_system_analyzer.cc

libmbsimControl_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS)
libmbsimControl_la_LIBADD = $(DEPS_LIBS)

mbsimControlincludedir = $(includedir)/mbsimControl
mbsimControlinclude_HEADERS = \
  namespace.h\
  signal_.h\
  extern_signal_source.h\
  extern_signal_sink.h\
  sensor.h\
  object_sensors.h\
  rigid_body_sensors.h\
  frame_sensors.h\
  function_sensor.h\
  link_sensors.h\
  mechanical_link_sensors.h\
  contact_sensor.h\
  tyre_contact_sensor.h\
  constraint_sensors.h\
  signal_sensors.h\
  signal_manipulation.h\
  linear_transfer_system.h\
  nonlinear_transfer_system.h\
  massless_spring_damper.h\
  signal_function.h\
  frame_chaser_observer.h \
  signal_observer.h \
  motion_observer.h \
  switch.h \
  stop.h \
  duration.h \
  state_machine.h \
  state_machine_sensor.h \
  linear_system_analyzer.h

include $(prefix)/share/mbxmlutils/python/deplibs.mk
install-exec-hook: deplibs.target
