#CXXFLAGS = @CXXFLAGS@ -DMATRIX_TEMPLATE_INSTANTIATION

EXTRA_DIST = barray.cpp   barray2d.cpp  matrix.cpp  vector.cpp  

pkginclude_HEADERS = barray.h galloc2d.h barray2d.h hpoint_nd.h matrixTool.h coordinate.h point_nd.h cvector.h specialType.h error.h list.h specialVcc.h galloc.h matrix.h vector.h matrix_global.h

INCLUDES = -I@top_srcdir@/include @CPPUNIT_CPPFLAGS@

lib_LTLIBRARIES = libmatrix.la
libmatrix_la_SOURCES = error.cpp\
                       cvector.cpp\
                       list.cpp\
                       point_nd.cpp\
                       hpoint_nd.cpp\
                       coordinate.cpp\
                       matrixTool.cpp\
                       barray_int.cpp\
                       barray2d_int.cpp\
                       matrix_int.cpp\
                       vector_int.cpp\
                       barray_float.cpp\
                       barray2d_float.cpp\
                       matrix_float.cpp\
                       vector_float.cpp\
                       barray_double.cpp\
                       barray2d_double.cpp\
                       matrix_double.cpp\
                       vector_double.cpp\
                       barray_uchar.cpp\
                       barray2d_uchar.cpp\
                       matrix_uchar.cpp\
                       vector_uchar.cpp\
                       barray_char.cpp\
                       barray2d_char.cpp\
                       matrix_char.cpp\
                       vector_char.cpp\
                       barray_point.cpp\
                       barray2d_point.cpp\
                       matrix_point.cpp\
                       vector_point.cpp\
                       barray_hpoint.cpp\
                       barray2d_hpoint.cpp\
                       matrix_hpoint.cpp\
                       vector_hpoint.cpp\
                       barray_complex.cpp\
                       barray2d_complex.cpp\
                       matrix_complex.cpp\
                       vector_complex.cpp\
                       barray_coordinate.cpp\
                       barray2d_coordinate.cpp\
                       barray_void.cpp


libmatrix_la_LDFLAGS = -version-info 1:0:0


