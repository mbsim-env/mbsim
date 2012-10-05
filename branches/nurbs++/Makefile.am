AUTOMAKE_OPTIONS = dist-bzip2 dist-zip
ACLOCAL_AMFLAGS = -I m4

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = nurbs++.pc

SUBDIRS= include matrix numerical image nurbs tests
#EXTRA_SUBDIRS = examples
DIST_SUBDIRS = include matrix numerical image nurbs examples tests m4

#bin_SCRIPTS = nurbs++-config
#man_MANS = nurbs++-config.1

EXTRA_DIST = config_mvc.bat 
#	nurbs++-config.1

dist-hook:
	cp -pr $(top_srcdir)/examples/vc6 $(distdir)/examples
	rm -rf `find $(distdir)/examples -name CVS`

m4sources = \
	m4/has_image_magick.m4 \
	m4/has_cppunit.m4 \
	m4/has_opengl.m4

m4datadir = $(datadir)/aclocal
#m4data_DATA = nurbs++.m4

