all: .mbsiminterface.done

xml/index.xml: ../schema/mbsiminterface.xsd ../mbsimInterface/*.h
	doxygen

.mbsiminterface.done: xml/index.xml ../schema/mbsiminterface.xsd ../mbsimInterface/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimInterface/mbsiminterface.xsd xml figures:../doc MBSimInterface MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsiminterface.done
