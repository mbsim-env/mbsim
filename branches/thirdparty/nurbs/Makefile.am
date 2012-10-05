EXTRA_DIST = nurbs.cpp curve.cpp hnurbs.cpp hnurbsS.cpp hnurbsS_sp.cpp matrixRT.cpp nurbs.cpp nurbsArray.cpp nurbsGL.cpp nurbsS.cpp nurbsS_sp.cpp nurbsSub.cpp nurbs_sp.cpp surface.cpp tri_spline.cpp

MAGICK_INCLUDE =  $(X_CFLAGS) @MAGICK_CPPFLAGS@

INCLUDES = $(MAGICK_INCLUDE) -I@top_srcdir@/include @matrix_include@ $(GL_CFLAGS)

pkginclude_HEADERS = curve.h matrixRT.h nurbsSub.h hnurbs.h nurbs.h nurbs_sp.h hnurbsS.h nurbsGL.h surface.h hnurbsS_sp.h nurbsS.h nurbsS_sp.h tri_spline.h nurbs_global.h

float_sources = f_nurbs.cpp f_nurbsS.cpp f_curve.cpp f_surface.cpp f_hnurbsS.cpp matrixRT.cpp f_nurbs_sp.cpp f_nurbsS_sp.cpp f_hnurbsS_sp.cpp nurbsGL.cpp f_nurbsArray.cpp f_nurbsSub.cpp f_tri_spline.cpp

double_sources = d_nurbs.cpp d_nurbsS.cpp d_curve.cpp d_surface.cpp d_hnurbsS.cpp matrixRT.cpp d_nurbs_sp.cpp d_nurbsS_sp.cpp d_hnurbsS_sp.cpp d_nurbsArray.cpp  d_nurbsSub.cpp d_tri_spline.cpp

ldflags = -version-info 1:0:1

#EXTRA_LTLIBRARIES = libnurbsf.la libnurbsd.la
#lib_LTLIBRARIES = @build_lib@


#libnurbsf_la_SOURCES = f_nurbs.cpp f_nurbsS.cpp f_curve.cpp f_surface.cpp f_hnurbsS.cpp matrixRT.cpp f_nurbs_sp.cpp f_nurbsS_sp.cpp f_hnurbsS_sp.cpp nurbsGL.cpp f_nurbsArray.cpp f_nurbsSub.cpp

#libnurbsd_la_SOURCES = d_nurbs.cpp d_nurbsS.cpp d_curve.cpp d_surface.cpp d_hnurbsS.cpp matrixRT.cpp d_nurbs_sp.cpp d_nurbsS_sp.cpp d_hnurbsS_sp.cpp d_nurbsArray.cpp  d_nurbsSub.cpp

#libnurbsf_la_LDFLAGS = -version-info 0:0:0
#libnurbsd_la_LDFLAGS = -version-info 0:0:0

#if BUILD_FLOAT_DOUBLE
#lib_LTLIBRARIES = libnurbsf.la libnurbsd.la
#libnurbsf_la_SOURCES = $(float_sources)
#libnurbsf_la_LDFLAGS = $(ldflags)
#libnurbsd_la_SOURCES = $(double_sources)
#libnurbsd_la_LDFLAGS = $(ldflags)
#else
#if BUILD_FLOAT
#lib_LTLIBRARIES = libnurbsf.la
#libnurbsf_la_SOURCES = $(float_sources)
#libnurbsf_la_LDFLAGS = $(ldflags)
#endif
#if BUILD_DOUBLE
#lib_LTLIBRARIES = libnurbsd.la
#libnurbsd_la_SOURCES = $(double_sources)
#libnurbsd_la_LDFLAGS = $(ldflags)
#endif
#endif

lib_LTLIBRARIES = libnurbsf.la libnurbsd.la
libnurbsf_la_SOURCES = $(float_sources)
libnurbsf_la_LDFLAGS = $(ldflags)
libnurbsd_la_SOURCES = $(double_sources)
libnurbsd_la_LDFLAGS = $(ldflags)

