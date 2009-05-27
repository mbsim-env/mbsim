docdir = @datadir@/@PACKAGE@/doc
dist_doc_DATA = mbsim.html

../schema/mbsim.xsd: ../schema/mbsim.xsd.in
	$(top_srcdir)/config.status --file=../schema/mbsim.xsd

mbsim.html: ../schema/mbsim.xsd ../../mbsim/*.h
	doxygen
	$$(pkg-config mbxmlutils --variable BINDIR)/mbxmlutils-xsd2html ../schema/mbsim.xsd xml MBSim MBSim::
#sed -re "s|$$(pkg-config mbxmlutils --variable DOCDIR)/measurement.html|qrc:measurement.html|" ../doc/openmbv.html > openmbv.qt.html



schemadir = @datadir@/@PACKAGE@/schema
dist_schema_DATA = mbsim.doxygen.xsd

mbsim.doxygen.xsd: mbsim.html
