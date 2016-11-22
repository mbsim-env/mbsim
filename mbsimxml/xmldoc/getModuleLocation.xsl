<xsl:stylesheet
  xmlns:xsl="http://www.w3.org/1999/XSL/Transform"
  xmlns:mbsimmodule="http://www.mbsim-env.de/MBSimModule"
  version="1.0">

  <xsl:output method="text"/>

  <xsl:template match="/">
    <xsl:apply-templates select="/mbsimmodule:MBSimModule/mbsimmodule:schemas/mbsimmodule:File"/>
  </xsl:template>

  <xsl:template match="/mbsimmodule:MBSimModule/mbsimmodule:schemas/mbsimmodule:File">
    <xsl:value-of select="@location"/><xsl:text>
</xsl:text>
  </xsl:template>

</xsl:stylesheet>
