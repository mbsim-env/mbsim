all: mbsimcontrol/mbsimcontrol.tex

xml/index.xml: ../schema/mbsimcontrol.xsd ../mbsimControl/*.h
	doxygen

mbsimcontrol/mbsimcontrol.tex: xml/index.xml ../schema/mbsimcontrol.xsd ../mbsimControl/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimControl/mbsimcontrol.xsd xml figures:../doc MBSimControl MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml mbsimcontrol/mbsimcontrol.tex
