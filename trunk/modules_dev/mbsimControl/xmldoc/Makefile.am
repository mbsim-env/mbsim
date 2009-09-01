docdir = @datadir@/@PACKAGE@/doc
nodist_doc_DATA = mbsimcontrol.xhtml 

../schema/mbsimcontrol.xsd: ../schema/mbsimcontrol.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsimcontrol.xsd

xml/index.xml: ../schema/mbsimcontrol.xsd ../mbsimControl/*.h
	doxygen

mbsimcontrol.xhtml: xml/index.xml
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsimcontrol.xsd xml .:../doc MBSimControl MBSim:: false
#sed -re "s|$$(pkg-config mbxmlutils --variable DOCDIR)/measurement.html|qrc:measurement.html|" ../doc/openmbv.html > openmbv.qt.html



schemadir = @datadir@/@PACKAGE@/schema
nodist_schema_DATA = mbsimcontrol.doxygen.xsd

mbsimcontrol.doxygen.xsd: mbsimcontrol.xhtml

CLEANFILES=mbsimcontrol.xhtml xml/index.xml
