#!/usr/bin/python2

import math
import OpenMBV

# import mbsim module
import mbsim



# XML namespace of this module (prefixed with { and postfixed with })
NS="{http://mypytest}"



# A python class derived from MBSim::FrameLink
class PySpringDamperXMLInit(mbsim.FixedFrameLink):
  def __init__(self):
    super(PySpringDamperXMLInit, self).__init__("")
    self.nF=1
    self.nla=1
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

class PySpringDamperPyScriptInit(mbsim.FixedFrameLink):
  def __init__(self):
    super(PySpringDamperPyScriptInit, self).__init__("")
    self.nF=1
    self.nla=1
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

  def init(self, stage, config):
    if stage==self.plotting:
      if self.getPlotFeature(mbsim.plotRecursive):
        if self.getPlotFeature(pyTestPlotFeature):
          self.plotColumns.push_back("sin")
      if self.getPlotFeature(mbsim.openMBV):
        self.coilspringOpenMBV.setName(self.name)
        self.parent.getOpenMBVGrp().addObject(self.coilspringOpenMBV)
    super(PySpringDamperPyScriptInit, self).init(stage, config)

  def plot(self):
    if self.getPlotFeature(mbsim.plotRecursive):
      if self.getPlotFeature(pyTestPlotFeature):
        self.plotVector.push_back(math.sin(10*self.getTime()))
    if self.getPlotFeature(mbsim.openMBV):
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

class PySpringDamperEmpty(mbsim.FixedFrameLink):
  def __init__(self):
    super(PySpringDamperEmpty, self).__init__("")
    self.nF=1
    self.nla=1
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



# register the classes as a XML name (this makes the class usable from XML)
mbsim.registerClass(PySpringDamperXMLInit)
mbsim.registerClass(PySpringDamperPyScriptInit)
mbsim.registerClass(PySpringDamperEmpty)
mbsim.registerClass(PyLinearSpringDamper)



# create a plot feature enum
pyTestPlotFeature=mbsim.PlotFeatureEnum()

# register the plot feature enum as XML name (this makes the enum usable from XML)
mbsim.registerEnum_PlotFeatureEnum(NS, pyTestPlotFeature, "pyTestPlotFeature")



# generate a standard schema
def generateXMLSchemaFile():
  return mbsim.generateXMLSchemaFile(__name__)
