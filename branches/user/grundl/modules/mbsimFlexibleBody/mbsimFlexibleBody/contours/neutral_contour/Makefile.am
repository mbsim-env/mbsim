SUBDIRS = neutral_contour_components .

noinst_LTLIBRARIES = libneutralcontour.la
libneutralcontour_la_SOURCES = contour_1s_neutral_cosserat.cc\
							   contour_1s_neutral_linear_external_FFR.cc\
							   contour_2s_neutral_linear_external_FFR.cc	

libneutralcontour_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)

libneutralcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)
libneutralcontour_la_LIBADD += neutral_contour_components/libneutralcontourcomponents.la

neutralcontourincludedir = $(includedir)/mbsimFlexibleBody/contours/neutral_contour/
neutralcontourinclude_HEADERS = contour_1s_neutral_factory.h\
								contour_1s_neutral_cosserat.h\
								contour_1s_neutral_linear_external_FFR.h\
								contour_2s_neutral_factory.h\
								contour_2s_neutral_linear_external_FFR.h
								
									 
							
