# if doxygen docu should be included
#USEDOXYGEN=true
#USEDOXYGENDEP=xml/index.xml
# if doxygen docu should NOT be included
USEDOXYGEN=false
USEDOXYGENDEP=



all: mbsimcontrol/mbsimcontrol.tex

xml/index.xml: ../schema/mbsimcontrol.xsd ../mbsimControl/*.h
	doxygen

mbsimcontrol/mbsimcontrol.tex: $(USEDOXYGENDEP) ../schema/mbsimcontrol.xsd ../mbsimControl/*.h 
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimControl/mbsimcontrol.xsd xml figures:../doc MBSimControl MBSim:: $(USEDOXYGEN)


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml mbsimcontrol/mbsimcontrol.tex
