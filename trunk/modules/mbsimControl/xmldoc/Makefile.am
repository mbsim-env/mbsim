all: .mbsimcontrol.done

xml/index.xml: ../schema/mbsimcontrol.xsd ../mbsimControl/*.h
	doxygen

.mbsimcontrol.done: xml/index.xml ../schema/mbsimcontrol.xsd ../mbsimControl/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimControl/mbsimcontrol.xsd xml figures:../doc MBSimControl MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml .mbsimcontrol.done
