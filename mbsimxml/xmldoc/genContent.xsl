<xsl:stylesheet
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  xmlns:xs="http://www.w3.org/2001/XMLSchema"
  version="1.0">

  <xsl:param name="DATETIME"/>
  <xsl:param name="MBSIMXMLVERSION"/>
  <xsl:param name="ABS_BUILDDIR"/>

  <!-- output method -->
  <xsl:output method="html" encoding="UTF-8"/>

  <!-- generate html link form a attribute -->
  <xsl:template mode="GENLINK" match="@*">
    <xsl:param name="V1" select="../namespace::*[name()=substring-before(current(),':')]"/>
    <xsl:param name="V2" select="translate($V1,'.:/','___')"/>
    <xsl:text>../</xsl:text><xsl:value-of select="$V2"/><xsl:text>/index.html#</xsl:text>
    <xsl:if test="not(contains(.,':'))">
      <xsl:value-of select="."/>
    </xsl:if>
    <xsl:if test="contains(.,':')">
      <xsl:value-of select="substring-after(.,':')"/>
    </xsl:if>
  </xsl:template>

  <xsl:template match="/">
    <!-- html header -->
    <xsl:text disable-output-escaping='yes'>&lt;!DOCTYPE html>
</xsl:text>
    <html lang="en">
    <head>
      <title>MBSimXML - XML Documentation</title>
      <link rel="stylesheet" href="http://maxcdn.bootstrapcdn.com/bootstrap/3.2.0/css/bootstrap.min.css"/>
      <style type="text/css">
        *._element { font-family:monospace; font-weight:bold; }
        *._type { font-family:monospace; }
        ul._content { padding-left:3ex; list-style-type:none; }
        *._displaynone { display:none; }
        *._linkpointer { cursor:pointer; }
      </style>
      <script type="text/javascript" src="http://code.jquery.com/jquery-2.1.1.min.js"> </script>
      <script type="text/javascript">
        <![CDATA[
        $(document).ready(function() {
          $("._expandcollapsecontent").click(function() {
            $($(this).parent().children("ul")[0]).toggleClass("_displaynone");
            $(this).toggleClass("glyphicon-expand");
            $(this).toggleClass("glyphicon-collapse-up");
          });
        });
        ]]>
      </script>
    </head>
    <body style="margin:1em">
    <div class="page-header">
      <h1>MBSimXML - Main MBSim XML Documentation</h1>
    </div>
    <p>This is the entry point for the documentation of MBSimXML. Below is a list of all relevant documentations for MBSimXML ordered by the corresponding XML namespaces.</p>
    <ul>
      <xsl:for-each select="document(concat($ABS_BUILDDIR, '/', 'files.xml'))/files/file">
        <xsl:sort select="@base"/>
        <li><a class="label label-warning" href="../{@base}/index.html">
          <xsl:value-of select="document(concat(document(concat($ABS_BUILDDIR, '/', 'files.xml'))/files/@base, '/', @base, '/', @name))/xs:schema/@targetNamespace"/>
        </a></li>
      </xsl:for-each>
    </ul>
    <p>The root XML element for MBSimXML is:</p>
    <a class="_element" href="../http___mbsim_berlios_de_MBSimXML/index.html#MBSimProject">&lt;MBSimProject&gt;</a>
    <xsl:text> </xsl:text><small><span class="label label-warning">http://mbsim.berlios.de/MBSimXML</span></small>

    <hr/>
    <h2>Legend</h2>
    <table class="table table-condensed">
      <thead>
        <tr><th>Icon</th><th>Description</th></tr>
      </thead>
      <tbody>
        <tr><td><span class="_element">&lt;element&gt;</span></td><td>A XML element of name 'element'</td></tr>
        <tr><td><span class="label label-warning">namespace</span></td><td>A XML namespace of name 'namespace'</td></tr>
        <tr><td><span class="label label-primary">type</span></td><td>A XML element or attribute type of name 'type'</td></tr>
      </tbody>
    </table>

    <hr/>
    <h2>All Elements</h2>
    <p>Below is a list of all known elements from MBSimXML, MBSimXML-Integrator, OpenMBV and the MBSimXML-Modules.</p>
    <ul class="_content">
      <xsl:apply-templates mode="CONTENT" select="/xs:dummyRoot/xs:schema/xs:element[not(@substitutionGroup)]">
        <xsl:sort select="@name"/>
      </xsl:apply-templates>
    </ul>

    <hr/>
    <h2>All Simple Types</h2>
    <p>Below is a list of all known simple types from MBSimXML, MBSimXML-Integrator, OpenMBV and the MBSimXML-Modules.</p>
    <ul class="_content">
      <xsl:apply-templates mode="CONTENT" select="/xs:dummyRoot/xs:schema/xs:simpleType">
        <xsl:sort select="@name"/>
      </xsl:apply-templates>
    </ul>
    <hr/>
    <p class="text-right small">
      <a href="http://validator.w3.org/check?uri=referer">
        <img src="http://www.w3.org/Icons/valid-html401-blue.png" alt="Valid HTML"/>
      </a>
      Generated on <xsl:value-of select="$DATETIME"/> for MBSimXML by <a href="http://mbsim.berlios.de">MBSimXML</a><xsl:text> </xsl:text><xsl:value-of select="$MBSIMXMLVERSION"/>
    </p>
    </body></html>
  </xsl:template>

  <xsl:template mode="CONTENT" match="/xs:dummyRoot/xs:schema/xs:element">
    <xsl:param name="NS_NAME" select="namespace::*[name()=substring-before(current()/@name,':')]"/>
    <xsl:param name="NAME_NAME" select="translate(substring(@name,string-length(substring-before(@name,':'))+1),':','')"/>
    <li>
      <a>
        <xsl:if test="/xs:dummyRoot/xs:schema/xs:element[@substitutionGroup=$NAME_NAME]"><!--MISSING is not NS aware-->
          <xsl:attribute name="class">_expandcollapsecontent glyphicon glyphicon-collapse-up _linkpointer</xsl:attribute>
        </xsl:if>
        <xsl:if test="not(/xs:dummyRoot/xs:schema/xs:element[@substitutionGroup=$NAME_NAME])"><!--MISSING is not NS aware-->
          <xsl:attribute name="class">glyphicon glyphicon-unchecked _linkpointer</xsl:attribute>
        </xsl:if>
      </a>
      <a class="_element">
        <xsl:attribute name="href"><xsl:apply-templates mode="GENLINK" select="@name"/></xsl:attribute>
        &lt;<xsl:value-of select="$NAME_NAME"/>&gt;</a>
      <xsl:text> </xsl:text><small><span class="label label-warning"><xsl:value-of select="$NS_NAME"/></span></small>
      <ul class="_content">
        <li class="_displaynone"/><!-- dummy li element just to honor the html schema which requires at least 1 li inside a ul -->
        <!-- this apply-templates (includeing the CONTENT_WITHATTRFQN template) is equal to select="...[@substitutionGroup=current()/@name]" with namespace aware attributes values -->
        <xsl:apply-templates mode="CONTENT_WITHATTRFQN" select="/xs:dummyRoot/xs:schema/xs:element">
          <xsl:with-param name="FQN" select="concat('{',$NS_NAME,'}',$NAME_NAME)"/>
          <xsl:with-param name="ATTRNAME" select="'substitutionGroup'"/>
          <xsl:sort select="@name"/>
        </xsl:apply-templates>
      </ul>
    </li>
  </xsl:template>
  <!-- just required to workaround the issue that many xslt processors (e.g. Xalan) does not provide the correct parent element for
  a namespace node -->
  <xsl:template match="*" mode="CONTENT_WITHATTRFQN">
    <xsl:param name="FQN"/>
    <xsl:param name="ATTRNAME"/>
    <xsl:param name="ATTR" select="@*[name()=$ATTRNAME]"/>
    <xsl:if test="concat('{',namespace::*[name()=substring-before($ATTR,':')],'}',translate(substring($ATTR,string-length(substring-before($ATTR,':'))+1),':',''))=$FQN">
      <xsl:apply-templates select="." mode="CONTENT"/>
    </xsl:if>
  </xsl:template>

  <xsl:template mode="CONTENT" match="/xs:dummyRoot/xs:schema/xs:simpleType">
    <xsl:param name="NS_NAME" select="namespace::*[name()=substring-before(current()/@name,':')]"/>
    <xsl:param name="NAME_NAME" select="translate(substring(@name,string-length(substring-before(@name,':'))+1),':','')"/>
    <li>
      <a class="glyphicon glyphicon-unchecked _linkpointer"/>
      <a class="label label-primary _type">
        <xsl:attribute name="href"><xsl:apply-templates mode="GENLINK" select="@name"/></xsl:attribute>
        <xsl:value-of select="$NAME_NAME"/></a>
      <xsl:text> </xsl:text><small><span class="label label-warning"><xsl:value-of select="$NS_NAME"/></span></small>
    </li>
  </xsl:template>

</xsl:stylesheet>
