#!/usr/bin/python2

# import mbsim module
import mbsim



# XML namespace of this module (prefixed with { and postfixed with })
NS="{http://mypytest}"



# A python class derived from MBSim::FrameLink
class PySpringDamperXMLInit(mbsim.FrameLink):
  def __init__(self):
    super(PySpringDamperXMLInit, self).__init__("")
    self.saved_ref1="../Frame[I]"
    self.saved_ref2="../Object[Box1]/Frame[C]"
    self.warnCount=0

  def updatelaF(self):
    if self.warnCount==0:
      self.msg(self.Warn, "Test warning from python")
      self.warnCount=self.warnCount+1
    self.lambdaF[0]=-self.c*self.evalGeneralizedRelativePosition()[0]-0.2*self.evalGeneralizedRelativeVelocity()[0]
    self.updlaF = False

  def isActive(self):
    return True

  def gActiveChanged(self):
    return False

  def isSingleValued(self):
    return True

  @staticmethod
  def getSchema():
    import xml.etree.cElementTree as ET
    xsd=ET.Element(mbsim.XS+'sequence')
    xsd.append(ET.Element(mbsim.XS+'element', {'name': "stiffness", 'type': ET.QName(mbsim.PV+"stiffnessScalar")}))
    return xsd

  def initializeUsingXML(self, e):
    super(PySpringDamperXMLInit, self).initializeUsingXML(e)
    self.c=float(e.find(NS+"stiffness").text)

class PySpringDamperPyScriptInit(mbsim.FrameLink):
  def __init__(self):
    super(PySpringDamperPyScriptInit, self).__init__("")
    self.saved_ref1="../Frame[I]"
    self.saved_ref2="../Object[Box2]/Frame[C]"
    self.warnCount=0

  def updatelaF(self):
    if self.warnCount==0:
      self.msg(self.Warn, "Test warning from python")
      self.warnCount=self.warnCount+1
    self.lambdaF[0]=-self.c*self.evalGeneralizedRelativePosition()[0]-0.2*self.evalGeneralizedRelativeVelocity()[0]
    self.updlaF = False

  def isActive(self):
    return True

  def gActiveChanged(self):
    return False

  def isSingleValued(self):
    return True

  @staticmethod
  def getSchema():
    return mbsim.pyScriptSchema()

  def initializeUsingXML(self, e):
    mbsim.pyScriptInitializeUsingXML(self, e, PySpringDamperPyScriptInit)

class PySpringDamperEmpty(mbsim.FrameLink):
  def __init__(self):
    super(PySpringDamperEmpty, self).__init__("")
    self.saved_ref1="../Frame[I]"
    self.saved_ref2="../Object[Box3]/Frame[C]"
    self.c=100
    self.warnCount=0

  def updatelaF(self):
    if self.warnCount==0:
      self.msg(self.Warn, "Test warning from python")
      self.warnCount=self.warnCount+1
    self.lambdaF[0]=-self.c*self.evalGeneralizedRelativePosition()[0]-0.2*self.evalGeneralizedRelativeVelocity()[0]
    self.updlaF = False

  def isActive(self):
    return True

  def gActiveChanged(self):
    return False

  def isSingleValued(self):
    return True



# register the classes as a XML name
mbsim.registerXMLName(PySpringDamperXMLInit)
mbsim.registerXMLName(PySpringDamperPyScriptInit)
mbsim.registerXMLName(PySpringDamperEmpty)



# generate a standard schema
def generateXMLSchemaFile():
  return mbsim.generateXMLSchemaFile(__name__)
