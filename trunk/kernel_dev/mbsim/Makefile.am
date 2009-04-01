SUBDIRS =  utils integrators contact_kinematics flexible_body .
 
lib_LTLIBRARIES = libmbsim.la
libmbsim_la_LDFLAGS = -version-info @LIBMBSIM_VERSION_INFO@
libmbsim_la_SOURCES = body.cc\
mbsim_event.cc\
link.cc\
object.cc\
multi_body_system.cc\
subsystem.cc\
group.cc\
rigid_body.cc\
joint.cc\
contact.cc\
load.cc\
actuator.cc\
element.cc\
tree.cc\
contour.cc\
frame.cc\
data_interface_base.cc\
extra_dynamic_interface.cc\
userfunction.cc\
userfunction_contour.cc\
functions_contact.cc\
class_factory.cc\
kinematics.cc\
constitutive_laws.cc\
flexible_body.cc\
special_classes.cc

libmbsim_la_LIBADD = integrators/libintegrators.la utils/libutils.la contact_kinematics/libcontactkinematics.la flexible_body/libflexiblebody.la

INCLUDES = $(DEPS_CFLAGS) $(AMVIS_CFLAGS)
LDADD = $(DEPS_LIBS) $(AMVIS_LIBS)

mbsimincludedir = $(includedir)/mbsim
mbsiminclude_HEADERS = body.h\
mbsim_event.h\
link.h\
object.h\
multi_body_system.h\
subsystem.h\
group.h\
rigid_body.h\
joint.h\
contact.h\
load.h\
actuator.h\
element.h\
tree.h\
frame.h\
contour.h\
contour_pdata.h\
data_interface_base.h\
extra_dynamic_interface.h\
userfunction.h\
userfunction_contour.h\
functions_contact.h\
class_factory.h\
kinematics.h\
constitutive_laws.h\
interfaces.h\
flexible_body.h\
special_classes.h\
compatibility_classes/body_rigid.h\
compatibility_classes/body_rigid_abs.h\
compatibility_classes/body_rigid_rel.h\
compatibility_classes/tree_rigid.h\
compatibility_classes/contact_rigid.h\
compatibility_classes/impact_rigid.h\
compatibility_classes/contact_flexible.h\
compatibility_classes/connection_rigid.h\
compatibility_classes/connection_flexible.h

