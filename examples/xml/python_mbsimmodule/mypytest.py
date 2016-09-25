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
    return '''<xs:sequence>
                <xs:element name="data1" type="pv:massScalar"/>
                <xs:element name="data2" type="pv:massScalar"/>
              </xs:sequence>'''



# register the classes as a XML name
mbsim.registerXMLName(PyO1)
mbsim.registerXMLName(PyO2)



# generate a standard schema
def generateXMLSchemaFile():
  return mbsim.generateXMLSchemaFile(__name__)
