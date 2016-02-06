all: .mbsiminterface.done

xml/index.xml: $(top_srcdir)/schema/mbsiminterface.xsd $(top_srcdir)/mbsimInterface/*.h
	doxygen Doxyfile

.mbsiminterface.done: xml/index.xml $(top_srcdir)/schema/mbsiminterface.xsd $(top_srcdir)/mbsimInterface/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___www_mbsim-env_de_MBSimInterface/mbsiminterface.xsd xml $(srcdir)/figures:$(srcdir)/../doc MBSimInterface MBSim::


install-data-local:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsiminterface.done
