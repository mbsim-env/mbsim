#!/usr/bin/python2

# import mbsim module
import mbsim



# XML namespace of this module (prefixed with { and postfixed with })
NS="{http://mypytest}"



# A python class derived from MBSim::Object
class PyO1(mbsim.Object):

  def __init__(self):
    super(PyO1, self).__init__("")

  def setData(self, v):
    self.value=v



# A python class derived from MBSim::Object
class PyO2(mbsim.Object):

  def __init__(self):
    super(PyO2, self).__init__("")

  def setData(self, v):
    self.value=v

  def initializeUsingXML(self, e, eorg):
    super(PyO2, self).initializeUsingXML(eorg)
    d1=e.find(NS+'data1').text
    print("d1="+str(d1))
    d2=e.find(NS+'data2').text
    print("d2="+str(d2))

  @staticmethod
  def getSchema():
    import xml.etree.cElementTree as ET
    xsd=ET.Element(mbsim.XS+'sequence')
    xsd.append(ET.Element(mbsim.XS+'element', {'name': ET.QName(NS+'data1'), 'type': ET.QName(mbsim.PV+'massScalar')}))
    xsd.append(ET.Element(mbsim.XS+'element', {'name': ET.QName(NS+'data2'), 'type': ET.QName(mbsim.PV+'massScalar')}))
    return xsd



# register the classes as a XML name
mbsim.registerXMLName(PyO1)
mbsim.registerXMLName(PyO2)



# generate a standard schema
def generateXMLSchemaFile():
  return mbsim.generateXMLSchemaFile(__name__)
