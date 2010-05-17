docdir = @MBXMLUTILSDOCDIR@/http___mbsim_berlios_de_MBSimXML
doc_DATA = mbsimxml.xhtml

mbsimxml.xhtml: genContent.xsl @MBXMLUTILSSCHEMA@/http___mbsim_berlios_de_MBSimXML/mbsimxml.xsd
	@XSLTPROC@ --novalid --output mbsimxml.xhtml genContent.xsl @MBXMLUTILSSCHEMA@/http___mbsim_berlios_de_MBSimXML/mbsimxml.xsd
	@XMLLINT@ --noout --schema @MBXMLUTILSSCHEMA@/http___openmbv_berlios_de_MBXMLUtils/xhtml1-transitional.xsd mbsimxml.xhtml || rm mbsimxml.xhtml

CLEANFILES=mbsimxml.xhtml
