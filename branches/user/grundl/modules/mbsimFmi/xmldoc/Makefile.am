all: mbsimfmi/mbsimfmi.tex

xml/index.xml: ../schema/mbsimfmi.xsd ../mbsimInterface/*.h
	doxygen

mbsimfmi/mbsimfmi.tex: xml/index.xml ../schema/mbsimfmi.xsd ../mbsimInterface/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimInterface/mbsimfmi.xsd xml figures:../doc MBSimInterface MBSim::


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml mbsimfmi/mbsimfmi.tex
