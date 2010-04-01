all: mbsimhydraulics/mbsimhydraulics.tex

xml/index.xml: ../schema/mbsimhydraulics.xsd ../mbsimHydraulics/*.h
	doxygen

mbsimhydraulics/mbsimhydraulics.tex: xml/index.xml
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMADIR@/http___mbsim_berlios_de_MBSimHydraulics/mbsimhydraulics.xsd xml figures:../doc MBSimHydraulics MBSim:: false



install:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=xml/index.xml mbsimhydraulics/mbsimhydraulics.tex
