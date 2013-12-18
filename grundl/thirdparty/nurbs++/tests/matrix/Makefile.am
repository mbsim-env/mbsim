EXTRA_DIST = Matrix.dsp

INCLUDES = @CPPUNIT_CPPFLAGS@ -I@top_srcdir@/matrix -I@top_srcdir@/include
LDADD = @CPPUNIT_LIBS@ -ldl @top_srcdir@/matrix/libmatrix.la -lm -lc

if BUILD_CPPUNIT_TESTS
test_program = test_matrix
else
test_program = 
endif

noinst_PROGRAMS = $(test_program)

test_matrix_SOURCES = test_main.cpp test_matrix.cpp test_barray2d.cpp



