SUBDIRS =  numerics utils integrators analysers contact_kinematics frames contours objects links constraints functions observers .
 
lib_LTLIBRARIES = libmbsim.la
libmbsim_la_SOURCES = element.cc\
		      constitutive_laws.cc\
		      contour.cc\
		      environment.cc\
		      frame.cc\
		      dynamic_system.cc\
		      group.cc\
		      dynamic_system_solver.cc\
		      link.cc\
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
libmbsim_la_LIBADD += links/liblink.la
libmbsim_la_LIBADD += constraints/libconstraint.la
libmbsim_la_LIBADD += numerics/libnumerics.la 
libmbsim_la_LIBADD += functions/libfunction.la 
libmbsim_la_LIBADD += observers/libobserver.la 
libmbsim_la_LIBADD += -lgfortran $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS)
libmbsim_la_LIBADD += -l@BOOST_SYSTEM_LIB@

libmbsim_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS)

mbsimincludedir = $(includedir)/mbsim
mbsiminclude_HEADERS = element.h\
		       constitutive_laws.h\
		       contour.h\
		       discretization_interface.h\
		       dynamic_system.h\
		       dynamic_system_solver.h\
		       environment.h\
		       frame.h\
		       group.h\
		       link.h\
		       mbsim_event.h\
		       modelling_interface.h\
		       object.h\
		       objectfactory.h\
		       graph.h\
		       constraint.h\
		       observer.h\
		       solver.h
