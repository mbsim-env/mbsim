SUBDIRS = ncc .

noinst_LTLIBRARIES = libneutralcontour.la
libneutralcontour_la_SOURCES = 1s_neutral_cosserat.cc\
			       1s_neutral_linear_external_FFR.cc\
			       2s_neutral_linear_external_FFR.cc

libneutralcontour_la_CPPFLAGS = -I$(top_srcdir) -DTIXML_USE_STL $(DEPS_CFLAGS) $(OPENMBVCPPINTERFACE_CFLAGS) $(NURBS_CFLAGS)

libneutralcontour_la_LIBADD = $(DEPS_LIBS) $(OPENMBVCPPINTERFACE_LIBS) $(NURBS_LIBS)
libneutralcontour_la_LIBADD += ncc/libneutralcontourcomponents.la

neutralcontourincludedir = $(includedir)/mbsimFlexibleBody/contours/nc/
neutralcontourinclude_HEADERS = 1s_neutral_cosserat.h\
				1s_neutral_linear_external_FFR.h\
				2s_neutral_linear_external_FFR.h
