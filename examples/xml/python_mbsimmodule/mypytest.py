#!/usr/bin/python2

import math
import OpenMBV

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
    self.coilspringOpenMBV=OpenMBV.ObjectFactory.create_CoilSpring()

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

  def init(self, stage):
    if stage==self.plotting:
      self.updatePlotFeatures()
      if self.getPlotFeature(self.plotRecursive)==self.enabled:
        self.plotColumns.push_back("sin")
        if self.getPlotFeature(self.openMBV)==self.enabled:
          self.coilspringOpenMBV.setName(self.name)
          self.parent.getOpenMBVGrp().addObject(self.coilspringOpenMBV)
        super(PySpringDamperPyScriptInit, self).init(stage)
    else:
      super(PySpringDamperPyScriptInit, self).init(stage)

  def plot(self):
    if self.getPlotFeature(self.plotRecursive)==self.enabled:
      self.plotVector.push_back(math.sin(10*self.getTime()))
      if self.getPlotFeature(self.openMBV)==self.enabled:
        WrOFromPoint=self.frame[0].evalPosition()
        WrOToPoint  =self.frame[1].evalPosition()
        data=[]
        data.append(self.getTime());
        data.append(WrOFromPoint[0]+1);
        data.append(WrOFromPoint[1]);
        data.append(WrOFromPoint[2]);
        data.append(WrOToPoint[0]+1);
        data.append(WrOToPoint[1]);
        data.append(WrOToPoint[2]);
        data.append(0.5)
        self.coilspringOpenMBV.append(data)
      super(PySpringDamperPyScriptInit, self).plot()

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

class PyLinearSpringDamper(mbsim.Function_d_d_d):
  def __init__(self):
    super(PyLinearSpringDamper, self).__init__()

  def __call__(self, s, sd):
    return 100*s + 0.2*sd



# register the classes as a XML name
mbsim.registerClass(PySpringDamperXMLInit)
mbsim.registerClass(PySpringDamperPyScriptInit)
mbsim.registerClass(PySpringDamperEmpty)
mbsim.registerClass(PyLinearSpringDamper)



# generate a standard schema
def generateXMLSchemaFile():
  return mbsim.generateXMLSchemaFile(__name__)
