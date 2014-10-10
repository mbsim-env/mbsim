CLEANFILES = -r html/

docdir = @datadir@/doc/@PACKAGE@

doc_DATA = html/*

html/*:
all-local: clean doxyfile
	@doxygen@ doxyfile
