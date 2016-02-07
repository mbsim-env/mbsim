all: .mbsimflexiblebody.done

xml/index.xml: $(top_srcdir)/schema/mbsimflexiblebody.xsd $(top_srcdir)/mbsimFlexibleBody/*.h
	doxygen Doxyfile

.mbsimflexiblebody.done: xml/index.xml $(top_srcdir)/schema/mbsimflexiblebody.xsd $(top_srcdir)/mbsimFlexibleBody/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimFlexibleBody/mbsimflexiblebody.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimFlexibleBody MBSim::


install-data-local:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimflexiblebody.done
