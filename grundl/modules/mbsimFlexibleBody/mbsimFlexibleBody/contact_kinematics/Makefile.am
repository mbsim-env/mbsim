noinst_LTLIBRARIES = libcontactkinematics.la
libcontactkinematics_la_SOURCES = circlehollow_cylinderflexible.cc\
																	circle_nurbsdisk2s.cc\
																	circlesolid_flexibleband.cc\
																	point_cylinderflexible.cc\
																	point_flexibleband.cc\
																	point_nurbsdisk2s.cc\
																	point_contour2s.cc


contactkinematicsincludedir = $(includedir)/mbsimFlexibleBody/contact_kinematics

libcontactkinematics_la_CPPFLAGS = -I$(top_srcdir) $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)
libcontactkinematics_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)

contactkinematicsinclude_HEADERS = circlehollow_cylinderflexible.h\
																	 circle_nurbsdisk2s.h\
																	 circlesolid_flexibleband.h\
																	 point_cylinderflexible.h\
																	 point_flexibleband.h\
																	 point_nurbsdisk2s.h\
																	 point_contour2s.h

