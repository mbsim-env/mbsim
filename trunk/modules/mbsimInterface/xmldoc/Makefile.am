all: .mbsiminterface.done

xml/index.xml: $(top_srcdir)/schema/mbsiminterface.xsd $(top_srcdir)/mbsimInterface/*.h
	doxygen Doxyfile

.mbsiminterface.done: xml/index.xml $(top_srcdir)/schema/mbsiminterface.xsd $(top_srcdir)/mbsimInterface/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimInterface/mbsiminterface.xsd xml figures:../doc MBSimInterface MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsiminterface.done
