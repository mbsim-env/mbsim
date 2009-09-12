all: mbsimhydraulics/mbsimhydraulics.tex

../schema/mbsimhydraulics.xsd: ../schema/mbsimhydraulics.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsimhydraulics.xsd

xml/index.xml: ../schema/mbsimhydraulics.xsd ../mbsimHydraulics/*.h
	doxygen

mbsimhydraulics/mbsimhydraulics.tex: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsimhydraulics.xsd xml . MBSimHydraulics MBSim:: false



install:
	cp -r http___* $$(pkg-config mbxmlutils --variable DOCDIR)



CLEANFILES=xml/index.xml mbsimhydraulics/mbsimhydraulics.tex
