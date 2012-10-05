EXTRA_DIST = tleastS.png

XINCLUDE = @X_CFLAGS@
XLIBS = @X_PRE_LIBS@ @X_LIBS@ -lXt -lX11
MAGICK_INCLUDE =  $(XINCLUDE) @magick_include@
MAGICK_LIB = @magick_lib@
MAGICK_LIBS = @magick_libs@ 

COMPLEXLIB = @complex_lib@

MATRIX_INCLUDE = @matrix_include@
MATRIX_LIB_FLAG = @matrix_lib_flag@
MATRIXLIB = @matrix_lib_flag@ @matrix_lib@ @complex_lib@

OPENGL_LIBS = -lglut $(GL_LFLAGS) $(GL_LIBS) $(X_EXTRA_LIBS)

#INCLUDES = $(MAGICK_INCLUDE) $(GL_CFLAGS) @matrix_include@ @nurbs_include@
INCLUDES = @matrix_include@ @nurbs_include@ @option_include@
#EXTRALIBS = -lm -lc
#ALLLIBS = @nurbs_libf@ $(MATRIXLIB) @option_libs@ $(EXTRALIBS)
ALLLIBS = -lglut @nurbs_libf@ $(MATRIXLIB) @option_libs@

TESTS = tnurbs tnlength tnurbsS thnurbsS tnsSweep tmatrixRT tdrawAa  tnInterp tnApprox tnMovePoint tnsMovePoint topengl tleastS tnurbs_sp  tnurbsS_sp tclose tnurbsSub tmeasures trispline
noinst_PROGRAMS = tnurbs tnlength tnurbsS thnurbsS tnsSweep tmatrixRT tdrawAa  tnInterp tnApprox tnMovePoint tnsMovePoint topengl tleastS tnurbs_sp  tnurbsS_sp tclose tnurbsSub tmeasures  trispline 

LDADD = $(ALLLIBS)
#LDFLAGS = $(LIBSINC)

tnurbs_SOURCES = tnurbs.cpp
tnlength_SOURCES = tnlength.cpp
tnurbsS_SOURCES = tnurbsS.cpp
thnurbsS_SOURCES = thnurbsS.cpp
tnsSweep_SOURCES = tnsSweep.cpp
tmatrixRT_SOURCES = tmatrixRT.cpp
tdrawAa_SOURCES = tdrawAa.cpp
tnInterp_SOURCES = tnInterp.cpp
tnApprox_SOURCES = tnApprox.cpp
tnMovePoint_SOURCES = tnMovePoint.cpp
tnsMovePoint_SOURCES = tnsMovePoint.cpp
topengl_SOURCES = topengl.cpp
tleastS_SOURCES = tleastS.cpp
tnurbs_sp_SOURCES = tnurbs_sp.cpp
tnurbsS_sp_SOURCES = tnurbsS_sp.cpp
tclose_SOURCES = tclose.cpp
tnurbsSub_SOURCES = tnurbsSub.cpp
tmeasures_SOURCES = tmeasures.cpp
trispline_SOURCES = trispline.cpp


