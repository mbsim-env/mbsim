SUBDIRS =  utils integrators contact_kinematics flexible_body contours .
 
lib_LTLIBRARIES = libmbsim.la
libmbsim_la_LDFLAGS = -version-info @LIBMBSIM_VERSION_INFO@
libmbsim_la_SOURCES = actuator.cc\
body.cc\
class_factory.cc\
constitutive_laws.cc\
contact.cc\
contour.cc\
data_interface_base.cc\
dynamic_system.cc\
dynamic_system_solver.cc\
element.cc\
environment.cc\
flexible_body.cc\
frame.cc\
functions_contact.cc\
group.cc\
joint.cc\
kinematics.cc\
spring_damper.cc\
link.cc\
link_mechanics.cc\
load.cc\
kinetic_excitation.cc\
mbsim_event.cc\
object.cc\
objectfactory.cc\
order_one_dynamics.cc\
rigid_body.cc\
tree.cc\
userfunction.cc

libmbsim_la_LIBADD = integrators/libintegrators.la utils/libutils.la contact_kinematics/libcontactkinematics.la flexible_body/libflexiblebody.la contours/libcontour.la

libmbsim_la_CPPFLAGS = -DTIXML_USE_STL

INCLUDES = $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)
LDADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)

mbsimincludedir = $(includedir)/mbsim
mbsiminclude_HEADERS = actuator.h\
body.h\
contact.h\
contour.h\
contour_pdata.h\
class_factory.h\
constitutive_laws.h\
data_interface_base.h\
discretization_interface.h\
dynamic_system.h\
dynamic_system_solver.h\
element.h\
environment.h\
extradynamic_interface.h\
frame.h\
functions_contact.h\
group.h\
joint.h\
kinematics.h\
spring_damper.h\
link.h\
link_interface.h\
link_mechanics.h\
load.h\
kinetic_excitation.h\
flexible_body.h\
mbsim_event.h\
modelling_interface.h\
object.h\
objectfactory.h\
object_interface.h\
order_one_dynamics.h\
rigid_body.h\
tree.h\
userfunction.h\
userfunction_contour.h

