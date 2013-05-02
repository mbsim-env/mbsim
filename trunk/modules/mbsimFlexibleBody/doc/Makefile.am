CLEANFILES = -r html/

docdir = @datadir@/doc/@PACKAGE@

doc_DATA = html/*

html/*:
all-local: clean
	TEXINPUTS="@abs_top_srcdir@/doc:" @doxygen@ doxyfile
