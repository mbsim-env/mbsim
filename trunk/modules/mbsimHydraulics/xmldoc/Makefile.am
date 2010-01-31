all: mbsimhydraulics/mbsimhydraulics.tex

xml/index.xml: ../schema/mbsimhydraulics.xsd ../mbsimHydraulics/*.h
	doxygen

mbsimhydraulics/mbsimhydraulics.tex: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html @MBXMLUTILSSCHEMA@/http___mbsim_berlios_de_MBSimHydraulics/mbsimhydraulics.xsd xml figures:../doc MBSimHydraulics MBSim:: false



install:
	cp -r http___* $$(pkg-config mbxmlutils --variable DOCDIR)



CLEANFILES=xml/index.xml mbsimhydraulics/mbsimhydraulics.tex
