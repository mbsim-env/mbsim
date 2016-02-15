SUBDIRS =  numerics utils integrators analysers contact_kinematics frames contours objects functions observers .
 
lib_LTLIBRARIES = libmbsim.la
libmbsim_la_SOURCES = \
element.cc\
constitutive_laws.cc\
contact.cc\
maxwell_contact.cc\
single_contact.cc\
contour.cc\
environment.cc\
frame.cc\
functions_contact.cc\
dynamic_system.cc\
group.cc\
dynamic_system_solver.cc\
joint.cc\
gear.cc\
spring_damper.cc\
friction.cc\
extern_generalized_io.cc\
link.cc\
frame_link.cc\
floating_frame_link.cc\
contour_link.cc\
rigid_body_link.cc\
kinetic_excitation.cc\
kinematic_excitation.cc\
mbsim_event.cc\
object.cc\
objectfactory.cc\
graph.cc\
constraint.cc\
observer.cc\
solver.cc

libmbsim_la_LIBADD = integrators/libintegrators.la
libmbsim_la_LIBADD += analysers/libanalysers.la 
libmbsim_la_LIBADD += utils/libutils.la 
libmbsim_la_LIBADD += contact_kinematics/libcontactkinematics.la 
libmbsim_la_LIBADD += frames/libframe.la
libmbsim_la_LIBADD += contours/libcontour.la
libmbsim_la_LIBADD += objects/libobject.la
libmbsim_la_LIBADD += numerics/libnumerics.la 
libmbsim_la_LIBADD += functions/libfunction.la 
libmbsim_la_LIBADD += observers/libobserver.la 
libmbsim_la_LIBADD += -lgfortran $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libmbsim_la_LIBADD += -l@BOOST_SYSTEM_LIB@

libmbsim_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)

mbsimincludedir = $(includedir)/mbsim
mbsiminclude_HEADERS = \
contact.h\
maxwell_contact.h\
single_contact.h\
contour.h\
constitutive_laws.h\
discretization_interface.h\
dynamic_system.h\
dynamic_system_solver.h\
element.h\
environment.h\
frame.h\
functions_contact.h\
group.h\
joint.h\
gear.h\
spring_damper.h\
friction.h\
extern_generalized_io.h\
link.h\
frame_link.h\
floating_frame_link.h\
contour_link.h\
rigid_body_link.h\
kinetic_excitation.h\
kinematic_excitation.h\
mbsim_event.h\
modelling_interface.h\
object.h\
objectfactory.h\
graph.h\
constraint.h\
observer.h\
solver.h
