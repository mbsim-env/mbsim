SUBDIRS =  utils integrators contact_kinematics .
 
lib_LTLIBRARIES = libmbsim.la
libmbsim_la_LDFLAGS = -version-info @LIBMBSIM_VERSION_INFO@
libmbsim_la_SOURCES = body.cc\
link.cc\
hitsphere_link.cc\
object.cc\
multi_body_system.cc\
rigid_body.cc\
connection.cc\
flexible_connection.cc\
rigid_connection.cc\
contact.cc\
rigid_contact.cc\
bilateral_rigid_contact.cc\
flexible_contact.cc\
bilateral_flexible_contact.cc\
load.cc\
element.cc\
tree.cc\
contour.cc\
coordinate_system.cc\
data_interface_base.cc\
extra_dynamic_interface.cc\
userfunction.cc\
userfunction_contour.cc\
functions_contact.cc

libmbsim_la_LIBADD = integrators/libintegrators.la utils/libutils.la contact_kinematics/libcontactkinematics.la 

INCLUDES = $(DEPS_CFLAGS) $(AMVIS_CFLAGS)
LDADD = $(DEPS_LIBS) $(AMVIS_LIBS)

mbsimincludedir = $(includedir)/mbsim
mbsiminclude_HEADERS = body.h\
link.h\
hitsphere_link.h\
object.h\
multi_body_system.h\
rigid_body.h\
connection.h\
flexible_connection.h\
rigid_connection.h\
contact.h\
rigid_contact.h\
bilateral_rigid_contact.h\
flexible_contact.h\
bilateral_flexible_contact.h\
load.h\
element.h\
tree.h\
coordinate_system.h\
contour.h\
contour_pdata.h\
data_interface_base.h\
extra_dynamic_interface.h\
userfunction.h\
userfunction_contour.h\
functions_contact.h

