SUBDIRS = body_flexible body_flexible/finite_elements utils integrators contact_kinematics .
 
lib_LTLIBRARIES = libmbsim.la
libmbsim_la_LDFLAGS = -version-info @LIBMBSIM_VERSION_INFO@
libmbsim_la_SOURCES = body.cc\
link.cc\
hitsphere_link.cc\
object.cc\
multi_body_system.cc\
body_rigid.cc\
body_rigid_abs.cc\
body_rigid_rel.cc\
body_rigid_rel_onflex.cc\
body_rigid_constrained.cc\
body_flexible.cc\
connection.cc\
connection_flexible.cc\
connection_rigid.cc\
contact.cc\
contact_rigid.cc\
contact_rigid_bilateral.cc\
contact_flexible.cc\
contact_flexible_bilateral.cc\
impact_rigid.cc\
load.cc\
element.cc\
tree.cc\
tree_rigid.cc\
tree_flexroot.cc\
contour.cc\
port.cc\
data_interface_base.cc\
extra_dynamic_interface.cc\
userfunction.cc\
userfunction_contour.cc\
functions_contact.cc

libmbsim_la_LIBADD = body_flexible/libbodyflexible.la integrators/libintegrators.la utils/libutils.la contact_kinematics/libcontactkinematics.la 

INCLUDES = $(DEPS_CFLAGS) $(AMVIS_CFLAGS)
LDADD = $(DEPS_LIBS) $(AMVIS_LIBS)

mbsimincludedir = $(includedir)/mbsim
mbsiminclude_HEADERS = body.h\
link.h\
hitsphere_link.h\
object.h\
multi_body_system.h\
body_rigid.h\
body_rigid_abs.h\
body_rigid_rel.h\
body_rigid_rel_onflex.h\
body_rigid_constrained.h\
body_flexible.h\
connection.h\
connection_flexible.h\
connection_rigid.h\
contact.h\
contact_rigid.h\
contact_rigid_bilateral.h\
contact_flexible.h\
contact_flexible_bilateral.h\
impact_rigid.h\
load.h\
element.h\
tree.h\
tree_rigid.h\
tree_flexroot.h\
port.h\
contour.h\
contour_pdata.h\
data_interface_base.h\
extra_dynamic_interface.h\
userfunction.h\
userfunction_contour.h\
functions_contact.h

