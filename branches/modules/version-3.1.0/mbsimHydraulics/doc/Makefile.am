CLEANFILES = -r html/ Latex/

docdir = @datadir@/doc/@PACKAGE@

doc_DATA = html/*

html/*:
all-local: clean
	@doxygen@ doxyfile
