<xsl:stylesheet
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  xmlns:plugin="http://mbsim.berlios.de/MBSimPlugin"
  version="1.0">

  <xsl:output method="text"/>

  <xsl:template match="/">
    <xsl:apply-templates select="/plugin:MBSimPlugin/plugin:schemas/plugin:Schema"/>
  </xsl:template>

  <xsl:template match="/plugin:MBSimPlugin/plugin:schemas/plugin:Schema">
    <xsl:value-of select="@schemaLocation"/><xsl:text>
</xsl:text>
  </xsl:template>

</xsl:stylesheet>
