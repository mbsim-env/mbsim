all: .mbsimflexiblebody.done

xml/index.xml: $(top_srcdir)/schema/mbsimflexiblebody.xsd $(top_srcdir)/mbsimFlexibleBody/*.h
	doxygen Doxyfile

.mbsimflexiblebody.done: xml/index.xml $(top_srcdir)/schema/mbsimflexiblebody.xsd $(top_srcdir)/mbsimFlexibleBody/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimFlexibleBody/mbsimflexiblebody.xsd xml figures:../doc MBSimFlexibleBody MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimflexiblebody.done
