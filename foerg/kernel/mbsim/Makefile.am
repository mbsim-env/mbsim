SUBDIRS =  plot_elements utils integrators contact_kinematics contours .
 
lib_LTLIBRARIES = libmbsim.la
libmbsim_la_LDFLAGS = -version-info @LIBMBSIM_VERSION_INFO@
libmbsim_la_SOURCES = \
body.cc\
contour.cc\
contour_pairing.cc\
dynamic_system.cc\
dynamic_system_solver.cc\
element.cc\
environment.cc\
frame.cc\
group.cc\
kinematics.cc\
extern_generalized_io.cc\
link.cc\
link_mechanics.cc\
mbsim_event.cc\
object.cc\
extra_dynamic.cc\
rigid_body.cc\
graph.cc

libmbsim_la_LIBADD = integrators/libintegrators.la utils/libutils.la contact_kinematics/libcontactkinematics.la contours/libcontour.la plot_elements/libplotelements.la -lgfortran

libmbsim_la_CPPFLAGS = -DTIXML_USE_STL

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(MBSIMNUMERICS_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(MBSIMNUMERICS_LIBS)

mbsimincludedir = $(includedir)/mbsim
mbsiminclude_HEADERS = \
body.h\
contour.h\
contour_pairing.h\
contour_pdata.h\
discretization_interface.h\
dynamic_system.h\
dynamic_system_solver.h\
element.h\
environment.h\
frame.h\
group.h\
kinematics.h\
extern_generalized_io.h\
link.h\
link_mechanics.h\
mbsim_event.h\
modelling_interface.h\
object.h\
extra_dynamic.h\
rigid_body.h\
graph.h
