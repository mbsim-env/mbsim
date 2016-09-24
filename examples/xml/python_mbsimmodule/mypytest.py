#!/usr/bin/python2

import mbsim



NS=mbsim.getNS(__name__)



class PyO1(mbsim.Object):

  def __init__(self):
    super(PyO1, self).__init__("")

  def setData(self, v):
    self.value=v



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



mbsim.registerClass(PyO1)
mbsim.registerClass(PyO2)


def generateXMLSchemaFile():
  return mbsim.generateXMLSchemaFile(__name__)
