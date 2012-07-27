EXTRA_DIST = rec_filter.cpp filter.cpp image.cpp

MAGICK_INCLUDE =  @MAGICK_CPPFLAGS@
INCLUDES = $(MAGICK_INCLUDE) -I@top_srcdir@/include -I@top_srcdir@/matrix

CXXFLAGS = @CXXFLAGS@

pkginclude_HEADERS = image.h rec_filter.h color.h filter.h

lib_LTLIBRARIES = libmatrixI.la
libmatrixI_la_SOURCES = color.cpp image_.cpp  rec_filter_.cpp filter_.cpp
libmatrixI_la_LDFLAGS = -version-info 1:0:0
