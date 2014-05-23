all: .mbsimpowertrain.done

xml/index.xml: ../schema/mbsimpowertrain.xsd ../mbsimPowertrain/*.h
	doxygen

.mbsimpowertrain.done: xml/index.xml ../schema/mbsimpowertrain.xsd ../mbsimPowertrain/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimPowertrain/mbsimpowertrain.xsd xml figures:../doc MBSimPowertrain MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimpowertrain.done
