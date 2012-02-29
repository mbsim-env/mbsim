MAGICK_INCLUDE =  $(X_CFLAGS) @magick_include@
MAGICK_LIB = $(X_PRE_LIBS) @magick_lib@ $(X_LIBS) -lXt -lX11

LAPACKLIBS = @lapack_libs@
COMPLEXLIB = @complex_lib@
#MATRIXLIB = @matrix_lib@

INCLUDES = $(MAGICK_INCLUDE) -I@top_srcdir@/matrix -I@top_srcdir@/image
LIBSINC = $(MAGICK_LIB) 

EXTRALIBS = -lm -lc
ALLLIBS = @magick_libs@ $(EXTRALIBS)

noinst_PROGRAMS = randomImg
LDADD = ../../image/libmatrixI.la ../../matrix/libmatrix.la $(ALLLIBS)
LDFLAGS = $(LIBSINC)

randomImg_SOURCES = randomImg.cpp
