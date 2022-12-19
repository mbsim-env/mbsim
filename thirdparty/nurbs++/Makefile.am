AUTOMAKE_OPTIONS = dist-bzip2 dist-zip
ACLOCAL_AMFLAGS = -I m4

pkgconfigdir = $(libdir)/pkgconfig
pkgconfig_DATA = nurbs++.pc

licdir = @datadir@/nurbs++
lic_DATA = COPYING

SUBDIRS= include matrix numerical image nurbs #tests
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


# create clang compile database if compiledb command is found
all-local: compile_commands.json
check-local: compile_commands.json
install-data-local: compile_commands.json
install-exec-local: compile_commands.json
compile_commands.json: config.log
	command -v compiledb &> /dev/null && compiledb -n -o compile_commands.json make && ln -fs $(abs_top_builddir)/compile_commands.json $(top_srcdir)/compile_commands.json || echo "Failed to create clang compile DB"
