CLEANFILES = -r html/ Latex/

docdir = @datadir@/doc/@PACKAGE@-@VERSION@

doc_DATA = html/*

html/*:
all-local: clean
	@doxygen@ doxyfile
