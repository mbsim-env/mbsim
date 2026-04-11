#!/usr/bin/env python3

import math
import OpenMBV
import numpy

# import mbsim module
import mbsim
import fmatvec
import mbsimControl

import sys
import os
import mbxmlutils



# XML namespace of this module (prefixed with { and postfixed with })
NS="{http://mypytest}"

# NOTE that not _ is allowed in any class name!



# A python class derived from MBSim::FrameLink
class PySpringDamperXMLInit(mbsim.FixedFrameLink):
  def __init__(self):
    super(PySpringDamperXMLInit, self).__init__("")
    self.nF=1
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
    self.warnCount=0
    self.coilspringOpenMBV=OpenMBV.ObjectFactory.create_CoilSpring()
    self.dynamicAttributesOpenMBV=OpenMBV.ObjectFactory.create_DynamicAttributes()
    self.dynamicIvBodyOpenMBV=OpenMBV.ObjectFactory.create_DynamicIvBody()
    self.dynamicIvBodySDOpenMBV=OpenMBV.ObjectFactory.create_DynamicIvBody()
    self.ivScreenAnnotationOpenMBV=OpenMBV.ObjectFactory.create_IvScreenAnnotation()

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
    if stage==self.resolveStringRef:
      # test getByPath_*
      l1=self.getByPath_Link("../Link[SpringRef]")
      l2=self.getByPath_Link("/Link[SpringRef]")
      if l1.getPath()!=l2.getPath():
        raise RuntimeError("getByPath_Link returns wrong output!")
      l3=mbsim.castElementTo_SpringDamper(l1)
      if l3 is None:
        raise RuntimeError("Casting Link to SpringDamper is not possible!")
      super(PySpringDamperPyScriptInit, self).init(stage, config)
    elif stage==self.plotting:
      if self.getPlotFeature(mbsim.plotRecursive):
        if self.getPlotFeature(pyTestPlotFeature):
          self.plotColumns.push_back("sin")
      if self.getPlotFeature(mbsim.openMBV):
        self.coilspringOpenMBV.setName(self.name)
        self.parent.getOpenMBVGrp().addObject(self.coilspringOpenMBV)

        self.dynamicAttributesOpenMBV.setName(self.name+"_DA")
        self.dynamicAttributesOpenMBV.addObjectEnable("//objects/Box1/Box1") # absolute path
        self.dynamicAttributesOpenMBV.addObjectEnable("../objects/Box3/Box3") # relative path
        self.dynamicAttributesOpenMBV.addDynamicColoredBodyTransparency("../objects/Box4/Box4")
        self.dynamicAttributesOpenMBV.addDynamicColoredBodyTransparency("../objects/Box3/Box3", True)
        self.parent.getOpenMBVGrp().addObject(self.dynamicAttributesOpenMBV)

        self.dynamicIvBodyOpenMBV.setName(self.name+"_DIVB")
        self.dynamicIvBodyOpenMBV.setDataSize(2);
        self.dynamicIvBodyOpenMBV.setDataIntSize(1);
        self.dynamicIvBodyOpenMBV.setDataStrSize(2);
        self.dynamicIvBodyOpenMBV.setIvFileName("dynamicivbody.iv");
        self.parent.getOpenMBVGrp().addObject(self.dynamicIvBodyOpenMBV)

        self.dynamicIvBodySDOpenMBV.setName(self.name+"_DIVB_SD")
        self.dynamicIvBodySDOpenMBV.setScalarData(True);
        self.dynamicIvBodySDOpenMBV.setDataSize(2);
        self.dynamicIvBodySDOpenMBV.setDataIntSize(1);
        self.dynamicIvBodySDOpenMBV.setDataStrSize(3);
        self.dynamicIvBodySDOpenMBV.setIvFileName("dynamicivbodysd.iv");
        self.parent.getOpenMBVGrp().addObject(self.dynamicIvBodySDOpenMBV)

        self.ivScreenAnnotationOpenMBV.setName(self.name+"_IVSA")
        self.ivScreenAnnotationOpenMBV.setScale1To1At([1,1])
        self.ivScreenAnnotationOpenMBV.addColumnLabel("ff0")
        self.ivScreenAnnotationOpenMBV.addColumnIntLabel("ii0")
        self.ivScreenAnnotationOpenMBV.addColumnStrLabel("ss0")
        self.ivScreenAnnotationOpenMBV.addColumnStrLabel("ss1")
        self.ivScreenAnnotationOpenMBV.setIvFileName("ivscreenannotation.iv");
        self.parent.getOpenMBVGrp().addObject(self.ivScreenAnnotationOpenMBV)
      super(PySpringDamperPyScriptInit, self).init(stage, config)
    else:
      super(PySpringDamperPyScriptInit, self).init(stage, config)

  def plot(self):
    if self.getPlotFeature(mbsim.plotRecursive):
      if self.getPlotFeature(pyTestPlotFeature):
        self.plotVector.push_back(math.sin(10*self.getTime()))
    if self.getPlotFeature(mbsim.openMBV):
      WrOFromPoint=self.frame[0].evalPosition()
      WrOToPoint  =self.frame[1].evalPosition()
      data=[]
      data.append(self.getTime())
      data.append(WrOFromPoint[0]+1)
      data.append(WrOFromPoint[1])
      data.append(WrOFromPoint[2])
      data.append(WrOToPoint[0]+1)
      data.append(WrOToPoint[1])
      data.append(WrOToPoint[2])
      data.append(0.5)
      self.coilspringOpenMBV.append(data)

      data=[]
      data.append(self.getTime())
      data.append(self.getTime()>0.5)
      data.append(self.getTime()<=0.5)
      data.append(self.getTime())
      self.dynamicAttributesOpenMBV.append(data)

      data=[]
      data.append(self.getTime())
      data.append(self.getTime()/2)
      self.dynamicIvBodyOpenMBV.append(data)
      dataInt=[]
      dataInt.append(round(self.getTime()*10))
      self.dynamicIvBodyOpenMBV.appendInt(dataInt)
      dataStr=[]
      dataStr.append(f"text1_{self.getTime():0.2}")
      dataStr.append(f"text2_{self.getTime():0.2}")
      self.dynamicIvBodyOpenMBV.appendStr(dataStr)

      data=[]
      data.append(self.getTime())
      data.append(self.getTime()/2)
      self.dynamicIvBodySDOpenMBV.append(data)
      dataInt=[]
      dataInt.append(round(self.getTime()*10))
      self.dynamicIvBodySDOpenMBV.appendInt(dataInt)
      dataStr=[]
      dataStr.append(f"text1_{self.getTime():0.2}")
      dataStr.append(f"text2_{self.getTime():0.2}")
      dataStr.append(f"line1 {self.getTime():0.2}\nline2 {self.getTime():0.2}")
      self.dynamicIvBodySDOpenMBV.appendStr(dataStr)

      data=[]
      data.append(self.getTime())
      data.append(self.getTime()/2)
      self.ivScreenAnnotationOpenMBV.append(data)
      dataInt=[]
      dataInt.append(round(self.getTime()*10))
      self.ivScreenAnnotationOpenMBV.appendInt(dataInt)
      dataStr=[]
      dataStr.append(f"text1_{self.getTime():0.2}")
      dataStr.append(f"text2_{self.getTime():0.2}")
      self.ivScreenAnnotationOpenMBV.appendStr(dataStr)
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

class PySignalFunction(mbsim.Function_VecV_d):
  def __init__(self):
    super(PySignalFunction, self).__init__()

  def __call__(self, t):
    return numpy.array([2.0*t,5*t])

  def getRetSize(self):
    return (2,1)

class TransKinematicFunction(mbsim.Function_Vec3_VecV):
  def __init__(self):
    super(TransKinematicFunction, self).__init__()

  def getArgSize(self):
    return 2

  def __call__(self, q):
    return numpy.array([q[0],
                        q[1],
                        0   ])

  def parDer(self, q):
    return numpy.array([[1,0],
                        [0,1],
                        [0,0]])

  def parDerDirDer(self, qd, q):
    return numpy.array([[0,0],
                        [0,0],
                        [0,0]])

class RotKinematicFunction(mbsim.Function_RotMat3_VecV):
  def __init__(self):
    super(RotKinematicFunction, self).__init__()

  def getArgSize(self):
    return 2

  def __call__(self, q):
    return mbxmlutils.rotateAboutX(q[0]) @ mbxmlutils.rotateAboutY(q[1])

  def parDer(self, q):
    return numpy.array([[1,0],
                        [0,1],
                        [0,0]])

  def parDerDirDer(self, qd, q):
    return numpy.array([[0,0],
                        [0,0],
                        [0,0]])

class TransTimeKinematicFunction(mbsim.Function_Vec3_VecV_d):
  def __init__(self):
    super(TransTimeKinematicFunction, self).__init__()

  def getArg1Size(self):
    return 2

  def __call__(self, q, t):
    return numpy.array([q[0],
                        q[1],
                        t   ])

  def parDer1(self, q, t):
    return numpy.array([[1, 0],
                        [0, 1],
                        [0, 0]])

  def parDer2(self, q, t):
    return numpy.array([0,
                        0,
                        1])

  def parDer2ParDer2(self, q, t):
    return numpy.array([0,
                        0,
                        0])

  def parDer1ParDer2(self, q, t):
    return numpy.array([[0, 0],
                        [0, 0],
                        [0, 0]])

  def parDer1DirDer1(self, qd, q, t):
    return numpy.array([[0, 0],
                        [0, 0],
                        [0, 0]])

class RotTimeKinematicFunction(mbsim.Function_RotMat3_VecV_d):
  def __init__(self):
    super(RotTimeKinematicFunction, self).__init__()

  def getArg1Size(self):
    return 2

  def __call__(self, q, t):
    return mbxmlutils.rotateAboutX(q[0]) @ mbxmlutils.rotateAboutY(q[1]) @ mbxmlutils.rotateAboutY(t)

  def parDer1(self, q, t):
    return numpy.array([[1, 0],
                        [0, 1],
                        [0, 0]])

  def parDer2(self, q, t):
    return numpy.array([0,
                        0,
                        1])

  def parDer2ParDer2(self, q, t):
    return numpy.array([0,
                        0,
                        0])

  def parDer1ParDer2(self, q, t):
    return numpy.array([[0, 0],
                        [0, 0],
                        [0, 0]])

  def parDer1DirDer1(self, qd, q, t):
    return numpy.array([[0, 0],
                        [0, 0],
                        [0, 0]])



# A python class derived from MBSim::FrameLink
class PyGeneralizedRelativePositionSensorWithOffsetXMLInit(mbsimControl.GeneralizedRelativePositionSensor):
  def __init__(self):
    super(PyGeneralizedRelativePositionSensorWithOffsetXMLInit, self).__init__("")
    self.offset=0

  def updateSignal(self):
    super(PyGeneralizedRelativePositionSensorWithOffsetXMLInit, self).updateSignal()
    self.s += self.offset

  @staticmethod
  def getSchema():
    import xml.etree.cElementTree as ET
    xsd=ET.Element(mbsim.XS+'sequence')
    xsd.append(ET.Element(mbsim.XS+'element', {'name': "offset", 'type': ET.QName(mbsim.PV+"lengthScalar")}))
    return xsd

  def initializeUsingXML(self, e):
    super(PyGeneralizedRelativePositionSensorWithOffsetXMLInit, self).initializeUsingXML(e)
    self.offset=float(e.find(NS+"offset").text)



# register the classes as a XML name (this makes the class usable from XML)
mbsim.registerClass(PySpringDamperXMLInit)
mbsim.registerClass(PySpringDamperPyScriptInit)
mbsim.registerClass(PySpringDamperEmpty)
mbsim.registerClass(PyLinearSpringDamper)
mbsim.registerClass(PySignalFunction)
mbsim.registerClass(TransKinematicFunction)
mbsim.registerClass(RotKinematicFunction)
mbsim.registerClass(TransTimeKinematicFunction)
mbsim.registerClass(RotTimeKinematicFunction)
mbsim.registerClass(PyGeneralizedRelativePositionSensorWithOffsetXMLInit)



# create a plot feature enum
pyTestPlotFeature=mbsim.PlotFeatureEnum()

# register the plot feature enum as XML name (this makes the enum usable from XML)
mbsim.registerEnum_PlotFeatureEnum(NS, pyTestPlotFeature, "pyTestPlotFeature")



# generate a standard schema
def generateXMLSchemaFile():
  return mbsim.generateXMLSchemaFile(__name__)
