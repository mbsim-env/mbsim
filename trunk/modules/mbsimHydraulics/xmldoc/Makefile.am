# if doxygen docu should be included
#USEDOXYGEN=true
#USEDOXYGENDEP=xml/index.xml
# if doxygen docu should NOT be included
USEDOXYGEN=false
USEDOXYGENDEP=



all: mbsimhydraulics/mbsimhydraulics.tex

xml/index.xml: ../schema/mbsimhydraulics.xsd ../mbsimHydraulics/*.h
	doxygen

mbsimhydraulics/mbsimhydraulics.tex: $(USEDOXYGENDEP) ../schema/mbsimhydraulics.xsd ../mbsimHydraulics/*.h
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimHydraulics/mbsimhydraulics.xsd xml figures:../doc MBSimHydraulics MBSim:: $(USEDOXYGEN)


install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml mbsimhydraulics/mbsimhydraulics.tex
