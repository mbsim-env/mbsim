CLEANFILES = -r html/

docdir = @datadir@/doc/@PACKAGE@-@VERSION@

doc_DATA = html/*

html/*:
all-local: clean
	@doxygen@ doxyfile
