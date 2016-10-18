#!/usr/bin/python2

# import mbsim module
import mbsim



# XML namespace of this module (prefixed with { and postfixed with })
NS="{http://mypytest}"



# A python class derived from MBSim::FrameLink
class PySpringDamper1(mbsim.FrameLink):
  def __init__(self):
    super(PySpringDamper1, self).__init__("")
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

  def initializeUsingXML(self, e):
    super(PySpringDamper1, self).initializeUsingXML(e)
    self.c=float(e.find(NS+"stiffness").text)

  @staticmethod
  def getSchema():
    import xml.etree.cElementTree as ET
    xsd=ET.Element(mbsim.XS+'sequence')
    xsd.append(ET.Element(mbsim.XS+'element', {'name': "stiffness", 'type': ET.QName(mbsim.PV+"stiffnessScalar")}))
    return xsd

class PySpringDamper2(mbsim.FrameLink):
  def __init__(self):
    super(PySpringDamper2, self).__init__("")
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

  def initializeUsingXML(self, e):#mfmf should be the default implemenation if not defined
    mbsim.initializeUsingPythonCode(self, e, PySpringDamper2)



# register the classes as a XML name
mbsim.registerXMLName(PySpringDamper1)
mbsim.registerXMLName(PySpringDamper2)



# generate a standard schema
def generateXMLSchemaFile():
  return mbsim.generateXMLSchemaFile(__name__)
