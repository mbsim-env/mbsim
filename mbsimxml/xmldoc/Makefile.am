docdir = $(MBXMLUTILSDOCDIR)/http___www_mbsim-env_de_MBSimXML
doc_DATA = mbsimxml.html

# none mbsim modules xsd files
XSDFILES = http___www_mbsim-env_de_MBXMLUtils/physicalvariable.xsd\
  http___www_mbsim-env_de_MBSim/mbsim.xsd\
  http___www_mbsim-env_de_MBSimIntegrator/mbsimintegrator.xsd\
  http___www_mbsim-env_de_MBSimXML/mbsimproject.xsd\
  http___www_mbsim-env_de_OpenMBV/openmbv.xsd

all: mbsimxml.html http___www_mbsim-env_de_MBSimXML/index.html

# list of all xsd files: variables XSDFILES and all modules (plugins)
.DELETE_ON_ERROR: xsdfiles.txt
xsdfiles.txt: $(MBXMLUTILSBINDIR)/../share/mbsimxml/plugins/*.plugin.xml
	echo "$(XSDFILES)" | tr ' ' '\n' > xsdfiles.txt
	for F in $(MBXMLUTILSBINDIR)/../share/mbsimxml/plugins/*.plugin.xml; do \
	  @XALAN@ $$F $(srcdir)/getPluginLocation.xsl | sed -re "s/^@MBSIMSCHEMADIR@//" >> xsdfiles.txt; \
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
	@XALAN@ -o mbsimxml.html -p DATETIME "'$$(date "+%a %b %d %Y %H:%M:%S")'" -p MBSIMXMLVERSION "'@VERSION@'" -p ABS_BUILDDIR "'$(abs_builddir)'" combindedSchema.xsd $(srcdir)/genContent.xsl | exit 1; \
	sed -re "s/ xmlns:[^ ]+ *= *\"[^\"]+\"//g" mbsimxml.html > mbsimxml.html_ || exit 1; \
	mv -f mbsimxml.html_ mbsimxml.html || exit 1
# above line: Remove all namespace declaration from the html output using sed (maybe a bug in Xalan)



http___www_mbsim-env_de_MBSimXML/index.html: @MBXMLUTILSSCHEMA@/http___www_mbsim-env_de_MBSimXML/mbsimproject.xsd
	@MBXMLUTILSBINDIR@/mbxmlutils-xsd2html @MBXMLUTILSSCHEMA@/http___www_mbsim-env_de_MBSimXML/mbsimproject.xsd none none MBSimXML MBSimXML::

install-data-local:
	cp -r http___* @MBXMLUTILSDOCDIR@



CLEANFILES=mbsimxml.html combindedSchema.xsd files.xml xsdfiles.txt http___www_mbsim-env_de_MBSimXML/index.html
