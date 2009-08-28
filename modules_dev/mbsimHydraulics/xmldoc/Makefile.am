docdir = @datadir@/@PACKAGE@/doc
nodist_doc_DATA = mbsimhydraulics.html 

../schema/mbsimhydraulics.xsd: ../schema/mbsimhydraulics.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsimhydraulics.xsd

xml/index.xml: ../schema/mbsimhydraulics.xsd ../mbsimHydraulics/*.h
	doxygen

mbsimhydraulics.html: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsimhydraulics.xsd xml .:../doc MBSimHydraulics MBSim:: false
#sed -re "s|$$(pkg-config mbxmlutils --variable DOCDIR)/measurement.html|qrc:measurement.html|" ../doc/openmbv.html > openmbv.qt.html



schemadir = @datadir@/@PACKAGE@/schema
nodist_schema_DATA = mbsimhydraulics.doxygen.xsd 

mbsimhydraulics.doxygen.xsd: mbsimhydraulics.html

