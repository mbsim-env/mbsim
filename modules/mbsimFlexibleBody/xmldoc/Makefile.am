# if doxygen docu should be included
#USEDOXYGEN=true
#USEDOXYGENDEP=xml/index.xml
# if doxygen docu should NOT be included
USEDOXYGEN=false
USEDOXYGENDEP=



all: mbsimflexiblebody/mbsimflexiblebody.tex

xml/index.xml: ../schema/mbsimflexiblebody.xsd ../mbsimFlexibleBody/*.h
	doxygen

mbsimflexiblebody/mbsimflexiblebody.tex: $(USEDOXYGENDEP) ../schema/mbsimflexiblebody.xsd ../mbsimFlexibleBody/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/ttp___mbsim_berlios_de_MBSimFlexibleBody/mbsimflexiblebody.xsd xml figures:../doc MBSimHydraulics MBSim:: $(USEDOXYGEN)


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml mbsimflexiblebody/mbsimflexiblebody.tex
