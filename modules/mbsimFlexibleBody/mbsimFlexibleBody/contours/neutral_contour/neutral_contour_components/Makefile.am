SUBDIRS = .

noinst_LTLIBRARIES = libneutralcontourcomponents.la
libneutralcontourcomponents_la_SOURCES = neutral_nurbs_1s.cc\
					 neutral_nurbs_position_1s.cc\
					 neutral_nurbs_angle_1s.cc\
					 neutral_nurbs_velocity_1s.cc\
					 neutral_nurbs_dotangle_1s.cc\
					 neutral_nurbs_local_position_1s.cc\
					 neutral_nurbs_2s.cc\
					 neutral_nurbs_position_2s.cc\
					 neutral_nurbs_velocity_2s.cc\
					 neutral_nurbs_local_position_2s.cc

libneutralcontourcomponents_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)

libneutralcontourcomponents_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)


neutralcontourcomponentsincludedir = $(includedir)/mbsimFlexibleBody/contours/neutral_contour/neutral_contour_components/
neutralcontourcomponentsinclude_HEADERS = neutral_nurbs_1s.h\
					  neutral_nurbs_position_1s.h\
					  neutral_nurbs_angle_1s.h\
					  neutral_nurbs_velocity_1s.h\
					  neutral_nurbs_dotangle_1s.h\
					  neutral_nurbs_local_position_1s.h\
					  neutral_nurbs_2s.h\
					  neutral_nurbs_position_2s.h\
					  neutral_nurbs_velocity_2s.h\
					  neutral_nurbs_local_position_2s.h
