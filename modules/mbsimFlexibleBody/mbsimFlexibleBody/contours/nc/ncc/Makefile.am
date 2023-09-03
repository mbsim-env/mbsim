SUBDIRS = .

noinst_LTLIBRARIES = libneutralcontourcomponents.la
libneutralcontourcomponents_la_SOURCES = nurbs_1s.cc\
					 nurbs_position_1s.cc\
					 nurbs_angle_1s.cc\
					 nurbs_velocity_1s.cc\
					 nurbs_dotangle_1s.cc\
					 nurbs_local_position_1s.cc\
					 nurbs_2s.cc\
					 nurbs_position_2s.cc\
					 nurbs_velocity_2s.cc\
					 nurbs_local_position_2s.cc

libneutralcontourcomponents_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)

libneutralcontourcomponents_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)


neutralcontourcomponentsincludedir = $(includedir)/mbsimFlexibleBody/contours/nc/ncc/
neutralcontourcomponentsinclude_HEADERS = nurbs_1s.h\
					  nurbs_position_1s.h\
					  nurbs_angle_1s.h\
					  nurbs_velocity_1s.h\
					  nurbs_dotangle_1s.h\
					  nurbs_local_position_1s.h\
					  nurbs_2s.h\
					  nurbs_position_2s.h\
					  nurbs_velocity_2s.h\
					  nurbs_local_position_2s.h
