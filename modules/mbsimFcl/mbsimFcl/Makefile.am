SUBDIRS = .
  
lib_LTLIBRARIES = libmbsimFcl.la
libmbsimFcl_la_LDFLAGS = -avoid-version
libmbsimFcl_la_SOURCES = \
  fcl_utils.cc \
  contact_utils.cc \
  fclcontour_fclcontour.cc \
  fcl_contour.cc \
  fcl_box.cc \
  fcl_sphere.cc \
  fcl_plane.cc \
  fcl_mesh.cc

libmbsimFcl_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(EIGEN3_CFLAGS) $(CCD_CFLAGS) $(FCL_CFLAGS)
libmbsimFcl_la_LIBADD = $(DEPS_LIBS) $(CCD_LIBS) $(FCL_LIBS)

mbsimFclincludedir = $(includedir)/mbsimFcl
mbsimFclinclude_HEADERS = \
  namespace.h \
  fcl_utils.h \
  contact_utils.h \
  fclcontour_fclcontour.h \
  fcl_contour.h \
  fcl_box.h \
  fcl_sphere.h \
  fcl_plane.h \
  fcl_mesh.h

include $(prefix)/share/mbxmlutils/python/deplibs.mk
install-exec-hook: deplibs.target
