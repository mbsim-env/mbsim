SUBDIRS =  numerics utils integrators contact_kinematics contours observers .
 
lib_LTLIBRARIES = libmbsim.la
libmbsim_la_LDFLAGS = -version-info @LIBMBSIM_VERSION_INFO@
libmbsim_la_SOURCES = \
body.cc\
constitutive_laws.cc\
contact.cc\
single_contact.cc\
contour.cc\
dynamic_system.cc\
dynamic_system_solver.cc\
element.cc\
environment.cc\
frame.cc\
functions_contact.cc\
group.cc\
joint.cc\
gear.cc\
gearing.cc\
kinematics.cc\
spring_damper.cc\
rotational_spring_damper.cc\
extern_generalized_io.cc\
link.cc\
link_mechanics.cc\
kinetic_excitation.cc\
kinematic_excitation.cc\
mbsim_event.cc\
object.cc\
objectfactory.cc\
xmlnamespacemapping.cc\
extra_dynamic.cc\
rigid_body.cc\
graph.cc\
constraint.cc \
dummyof.cc \
observer.cc

libmbsim_la_LIBADD = integrators/libintegrators.la utils/libutils.la contact_kinematics/libcontactkinematics.la contours/libcontour.la numerics/libnumerics.la observers/libobserver.la -lgfortran

libmbsim_la_CPPFLAGS = -DTIXML_USE_STL

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(MBSIMNUMERICS_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(MBSIMNUMERICS_LIBS)

mbsimincludedir = $(includedir)/mbsim
mbsiminclude_HEADERS = \
body.h\
contact.h\
single_contact.h\
contour.h\
contour_pdata.h\
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
gearing.h\
kinematics.h\
spring_damper.h\
rotational_spring_damper.h\
extern_generalized_io.h\
link.h\
link_mechanics.h\
kinetic_excitation.h\
kinematic_excitation.h\
mbsim_event.h\
modelling_interface.h\
object.h\
objectfactory.h\
objectfactory_instance.h\
xmlnamespacemapping.h\
extra_dynamic.h\
rigid_body.h\
graph.h\
constraint.h \
observer.h
