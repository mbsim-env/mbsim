docdir = $(MBXMLUTILSDOCDIR)/http___www_mbsim-env_de_MBSimXML
doc_DATA = mbsimxml.html

# none mbsim modules xsd files
XSDFILES = http___www_mbsim-env_de_MBXMLUtils/mbxmlutils.xsd\
  http___www_mbsim-env_de_MBSim/mbsim.xsd\
  http___www_mbsim-env_de_MBSimXML/mbsimproject.xsd\
  http___www_mbsim-env_de_OpenMBV/openmbv.xsd

all: mbsimxml.html .mbsimxml.done

# list of all xsd files: variables XSDFILES and all MBSim modules
.DELETE_ON_ERROR: xsdfiles.txt
xsdfiles.txt:
	echo "$(XSDFILES)" | tr ' ' '\n' > xsdfiles.txt
	for F in $(MBXMLUTILSBINDIR)/../share/mbsimmodules/*.mbsimmodule.xml; do \
	  @XALAN@ $$F $(srcdir)/getModuleLocation.xsl | sed -re "s-^@MBSIMSCHEMADIR@/--" >> xsdfiles.txt; \
	done

# XML representation of all xsd files
.DELETE_ON_ERROR: files.xml
files.xml: xsdfiles.txt
	echo "<files base=\"$(MBXMLUTILSSCHEMA)\">" > files.xml
	for F in $$(cat xsdfiles.txt); do echo "<file base=\"$$(dirname $$F)\" name=\"$$(basename $$F)\"/>" >> files.xml; done
	echo "</files>" >> files.xml

# generate combined schema (root element xs:dummyRoot)
.DELETE_ON_ERROR: combindedSchema.xsd
combindedSchema.xsd: $(srcdir)/genCombindedSchema.xsl files.xml
	@XALAN@ -o combindedSchema.xsd files.xml $(srcdir)/genCombindedSchema.xsl

# generate the documentation
.DELETE_ON_ERROR: mbsimxml.html
mbsimxml.html: $(srcdir)/genContent.xsl combindedSchema.xsd
	@XALAN@ -o mbsimxml.html -p DATETIME "'$$(date -u +%FT%TZ)'" -p MBSIMXMLVERSION "'@VERSION@'" -p ABS_BUILDDIR "'$(abs_builddir)'" combindedSchema.xsd $(srcdir)/genContent.xsl || exit 1; \
	sed -re "s/ xmlns:[^ ]+ *= *\"[^\"]+\"//g" mbsimxml.html > mbsimxml.html_ || exit 1; \
	mv -f mbsimxml.html_ mbsimxml.html || exit 1
	RET=0; \
	$(prefix)/share/fmatvec/checkHtml/checkHtml.sh mbsimxml.html || RET=1; \
	$(prefix)/share/fmatvec/checkHtml/checkMathJax.sh mbsimxml.html || RET=1; \
	test $$RET -ne 0 && exit 1 || exit 0



.mbsimxml.done: @MBXMLUTILSSCHEMA@/http___www_mbsim-env_de_MBSimXML/mbsimproject.xsd @MBXMLUTILSSCHEMA@/http___www_mbsim-env_de_MBSim/mbsim.xsd
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html.sh @MBXMLUTILSSCHEMA@/http___www_mbsim-env_de_MBSimXML/mbsimproject.xsd none none MBSimXML MBSimXML:: @MBXMLUTILSSCHEMA@/http___www_mbsim-env_de_MBSim/mbsim.xsd

install-data-local: .mbsimxml.done
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=mbsimxml.html combindedSchema.xsd files.xml xsdfiles.txt http___www_mbsim-env_de_MBSimXML/index.html
