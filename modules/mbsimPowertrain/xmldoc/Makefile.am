all: .mbsimpowertrain.done

xml/index.xml: $(top_srcdir)/schema/mbsimpowertrain.xsd $(top_srcdir)/mbsimPowertrain/*.h
	doxygen $(srcdir)/Doxyfile

.mbsimpowertrain.done: xml/index.xml $(top_srcdir)/schema/mbsimpowertrain.xsd $(top_srcdir)/mbsimPowertrain/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimPowertrain/mbsimpowertrain.xsd xml figures:../doc MBSimPowertrain MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimpowertrain.done
