EXTRA_DIST = nurbs.dsp

INCLUDES = 	-I@top_srcdir@/matrix \
		-I@top_srcdir@/include \
		-I@top_srcdir@/nurbs \
		-I@top_srcdir@/image \
		-I@top_srcdir@/numerical \
		@CPPUNIT_CPPFLAGS@

OPENGL_LIBS = @X_LIBS@ @GL_LIBS@

LDADD = @CPPUNIT_LIBS@ -ldl @top_srcdir@/matrix/libmatrix.la @top_srcdir@/image/libmatrixI.la @top_srcdir@/numerical/libmatrixN.la @top_srcdir@/nurbs/libnurbsf.la $(OPENGL_LIBS) -lm -lc

if BUILD_CPPUNIT_TESTS
test_program = test_nurbs
else
test_program = 
endif

noinst_PROGRAMS = $(test_program)
test_nurbs_SOURCES = test_main.cpp test_nurbs.cpp


