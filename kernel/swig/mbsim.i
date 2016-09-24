// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") mbsim



// wrap std::string
%include "std_string.i"

// wrap python error to c++ exception
%feature("director:except") {
  if($error) {
    // MISSING we absue PyExc_MemoryError here instead of definding our own exception class
    // which indicates a "pass throught" exception
    if(PyErr_GivenExceptionMatches($error, PyExc_MemoryError)) {
      // pass throught a c++ exception
      PyObject *ptype, *pvalue, *ptraceback;
      PyErr_Fetch(&ptype, &pvalue, &ptraceback);
      Py_DECREF(ptype);
      Py_DECREF(ptraceback);
      PyObject *msg=PyObject_Str(pvalue);
      Py_DECREF(pvalue);
      std::string msgStr=PyUnicode_AsUTF8(msg);
      Py_DECREF(msg);
      throw std::runtime_error(msgStr);
    }
    else
      // wrap a python exception to c++
      throw PythonCpp::PythonException("", 0);
  }
}
// wrap c++ exception to python error
%exception {
  try {
    $action
  }
  catch(const std::exception &e) {
    PyErr_SetString(PyExc_MemoryError, e.what());
    SWIG_fail;
  }
  catch(...) {
    PyErr_SetString(PyExc_MemoryError, "Unknown c++ exception");
    SWIG_fail;
  }
}

// create directors for ...
%feature("director") MBSim::AllocateBase;
%feature("director") MBSim::DeallocateBase;
%feature("director") fmatvec::Atom;
%feature("director") MBSim::Element;
%feature("director") MBSim::Object;

// forward declarations needed in the generated swig c++ code
namespace MBSim {
  class FixedRelativeFrame;
  class RigidContour;
}

// includes needed in the generated swig c++ code
%{
#include <config.h>
#include <mbxmlutilshelper/getinstallpath.h>
#include "mbxmlutils/py2py3cppwrapper.h"
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/join.hpp>
#include "mbsim/objectfactory_part.h"
#include "mbsim/objects/object.h"
%}

// wrap the following headers
%include "mbsim/objectfactory_part.h"
%include "fmatvec/atom.h"
%include "mbsim/element.h"
%include "mbsim/objects/object.h"






%inline %{

// internal helper function create a MBXMLUtils::FQN (MBXMLUtils is not wrapped at all)
MBXMLUtils::FQN _FQN(const std::string &ns, const std::string &name) {
  return MBXMLUtils::FQN(ns, name);
}

// internal helper function to cast a MBSim::AllocateBase to the Python director object
// or None if this MBSim::AllocateBase does not represent a director
PyObject* _dynamic_cast_Director(const MBSim::AllocateBase *x) {
  const Swig::Director *d=dynamic_cast<const Swig::Director*>(x);
  PyObject* ret=d ? d->swig_get_self() : Py_None;
  Py_INCREF(ret);
  return ret;
}

// internal helper function to get the text from a DOMElement. This text is python code for
// all Python MBSim objects.
std::string _getObjectInitCode(xercesc::DOMElement *e, const MBXMLUtils::FQN &name) {
  // return empty string if name does not exist
  xercesc::DOMElement *i=MBXMLUtils::E(e)->getFirstElementChildNamed(name);
  if(!i)
    return "";
  // get code as string
  std::string str=MBXMLUtils::X()%MBXMLUtils::E(i)->getFirstTextChild()->getData();

  // fix python indentation
  std::vector<std::string> lines;
  boost::split(lines, str, boost::is_any_of("\n")); // split to a vector of lines
  size_t indent=std::string::npos;
  size_t lineNr=0;
  for(std::vector<std::string>::iterator it=lines.begin(); it!=lines.end(); ++it, ++lineNr) {
    size_t pos=it->find_first_not_of(' '); // get first none space character
    if(pos==std::string::npos) continue; // not found -> pure empty line -> do not modify
    if(pos!=std::string::npos && (*it)[pos]=='#') continue; // found and first char is '#' -> pure comment line -> do not modify
    // now we have a line with a python statement
    if(indent==std::string::npos) indent=pos; // at the first python statement line use the current indent as indent for all others
    if(it->substr(0, indent)!=std::string(indent, ' ')) // check if line starts with at least indent spaces ...
      // ... if not its an indentation error
      throw MBXMLUtils::DOMEvalException("Unexpected indentation at line "+std::to_string(lineNr)+": "+str, e);
    *it=it->substr(indent); // remove the first indent spaces from the line
  }
  return boost::join(lines, "\n"); // join the lines to a single string
}

//MFMF// internal helper function to get the path of MBSim
//MFMFstd::string _getMBSimInstallPath() {
//MFMF  return MBXMLUtils::getInstallPath().generic_string();
//MFMF}

// internal helper function to get the filename of e
std::string _getDocumentFileNameOf(xercesc::DOMElement *e) {
  std::string uri=MBXMLUtils::X()%e->getOwnerDocument()->getDocumentURI();
  return uri.substr(7); // remove the "file://" part from the uri
}

// internal helper function to get the XPath of e
std::string _getXPathOf(xercesc::DOMElement *e) {
  xercesc::DOMElement *r=e->getOwnerDocument()->getDocumentElement();
  std::string xpath;
  for(xercesc::DOMElement *ee=e; ee!=r; ee=static_cast<xercesc::DOMElement*>(ee->getParentNode())) {
    // get element name
    MBXMLUtils::FQN eleName=MBXMLUtils::E(ee)->getTagName();
    // count same elements on this level before this
    int count=1;
    for(xercesc::DOMElement *pre=ee->getPreviousElementSibling(); pre; pre=pre->getPreviousElementSibling())
      if(MBXMLUtils::E(pre)->getTagName()==eleName)
        count++;
    // construct xpath
    xpath="{"+eleName.first+"}"+eleName.second+"["+std::to_string(count)+"]/"+xpath;
  }
  return xpath.substr(0, xpath.size()-1);
}

%}



%pythoncode %{

# internal helper class to register a director class in the MBSim::ObjectFactory
class _AllocatePython(AllocateBase):
  def __init__(self, className):
    super(_AllocatePython, self).__init__() 
    self.className=className
  def __call__(self):
    return self.className().__disown__()
  def __eq__(self, other):
    otherDirector=_dynamic_cast_Director(other)
    if otherDirector==None:
      return False
    if type(otherDirector)==_AllocatePython and otherDirector.className==self.className:
      return True
    return False;

# internal helper class to register a director class in the MBSim::ObjectFactory
class _DeallocatePython(DeallocateBase):
  def __init__(self):
    super(_DeallocatePython, self).__init__() 
  def __call__(self, e):
    Atom.__swig_destroy__(e)

# internal helper class to register a director class in the MBSim::ObjectFactory
class _GetSingletonPython(AllocateBase):
  def __init__(self, className):
    super(_GetSingletonPython, self).__init__() 
    self.className=className
  def __call__(self):
    return self.classNname.getInstance().__disown__()
  def __eq__(self, other):
    otherDirector=_dynamic_cast_Director(other)
    if otherDirector==None:
      return False
    if type(otherDirector)==_AllocatePython and otherDirector.className==self.className:
      return True
    return False;

# internal helper class to register a director class in the MBSim::ObjectFactory
class _DeallocateSingletonPython(DeallocateBase):
  def __init__(self):
    super(_DeallocateSingletonPython, self).__init__() 
  def __call__(self, e):
    pass

_moduleData={}

# internal helper function to extend a Python class
def _extendClass(className):
  # add initializeUsingXML method if not exist of convert arguments if it exists
  import types
  if not 'initializeUsingXML' in className.__dict__:
    # add initializeUsingXML method which runs the XML text child element as python code
    # where the variable 'self' represents this object
    def implicitInitializeUsingXML(self, e):
      super(className, self).initializeUsingXML(e)
      code=_getObjectInitCode(e, _FQN(getNS(className.__module__, False), "pyinit."+className.__name__))
      if code!="":
        exec(code, {'self': self})
    className.initializeUsingXML=types.MethodType(implicitInitializeUsingXML, None, className)
  else:
    # save user defined initializeUsingXML and replace by implicit one (which convert com DOMElement to pyxml
    userInitializeUsingXML=className.initializeUsingXML
    def wrapperInitializeUsingXML(self, e_xerces):
      import xml.etree.ElementTree
      filename=_getDocumentFileNameOf(e_xerces)
      xpath=_getXPathOf(e_xerces)
      root = xml.etree.ElementTree.parse(filename).getroot()
      e_py=root.find(xpath)
      userInitializeUsingXML(self, e_py, e_xerces)
    className.initializeUsingXML=types.MethodType(wrapperInitializeUsingXML, None, className)

  # create XML schema element and store it in _moduleData
  # store also all required modules in _moduleData
  baseClassName=className.__bases__
  if len(baseClassName)!=1:
    raise RuntimeError('Can only handle classed with one base class.')
  if not hasattr(className, 'getSchema'):
    xsdpart='''
      <xs:sequence>
        <xs:element name="pyinit.%s" type="xs:string" minOccurs="0"/>
      </xs:sequence> 
''' % (className.__name__)
  else:
    xsdpart=className.getSchema()
  xsd='''
<xs:element name="%s" substitutionGroup="%s" type="%sType"/>
<xs:complexType name="%sType">
  <xs:complexContent>
    <xs:extension base="%sType">
%s
    </xs:extension>
  </xs:complexContent>
</xs:complexType>
''' % (className.__name__, baseClassName[0].__module__+":"+baseClassName[0].__name__, className.__name__,
       className.__name__, baseClassName[0].__module__+":"+baseClassName[0].__name__, xsdpart)
  global _moduleData
  if not className.__module__ in _moduleData:
    _moduleData[className.__module__]={'xsdElements': "", 'requiredModules': set()}
  _moduleData[className.__module__]['xsdElements']+=xsd
  _moduleData[className.__module__]['requiredModules'].add(baseClassName[0].__module__)






# get namespace (sourounded with { and } for the python moudle moduleName
def getNS(moduleName, withBrackets=True):
  ns="http://localhost/mbsimmodule/python/"+moduleName
  if withBrackets:
    ns="{"+ns+"}"
  return ns

# register class in the MBSim::ObjectFactory.
# This also extents className with some special members.
def registerClass(className):
  _extendClass(className)
  registerXMLName(_FQN(getNS(className.__module__, False), className.__name__),
    _AllocatePython(className).__disown__(), _DeallocatePython().__disown__())

# register singelton class in the MBSim::ObjectFactory
# This also extents className with some special members.
def registerClassAsSingleton(className):
  _extendClass(className)
  registerXMLName(_FQN(getNS(className.__module__, False), className.__name__),
    _GetSingletonPython(className).__disown__(), _DeallocateSingletonPython().__disown__())

# create the xsd for this module.
def generateXMLSchemaFile(moduleName):
  import os.path
  global _moduleData

  # generated the namespace prefix definition and the namespace import defintion of the xsd file (for all required modules)
  nsPrefixDef=""
  nsImportDef=""
  for module in _moduleData[moduleName]['requiredModules']:
#mfmf    # get some path
#mfmf    moduleFilename=_getMBSimInstallPath()+"/share/mbsimmodules/"+module+".mbsimmodule.xml"
    # handle the mbsim module specially: its not a module its the kernel
    if module=='mbsim':
      ns="http://www.mbsim-env.de/MBSim"
    #mfmf handle other mbsim internal "modules"
#mfmf    # handle modules
#mfmf    elif os.path.isfile(moduleFilename):
#mfmf      import xml.etree.ElementTree
#mfmf      root=xml.etree.ElementTree.parse(moduleFilename).getroot()
#mfmf      ns=root.findall('./{http://www.mbsim-env.de/MBSimModule}schemas/{http://www.mbsim-env.de/MBSimModule}Schema')[0]\
#mfmf        .get('namespace')
    # error case
    else:
      raise RuntimeError("Unable to find the MBSim module file for "+module)

    nsPrefixDef+='xmlns:%s="%s"\n' % (module, ns)
    nsImportDef+='<xs:import namespace="%s"/>\n' % (ns)

  # create xsd file
  xsd='''<?xml version="1.0" encoding="UTF-8"?>
<xs:schema targetNamespace="%s"
  elementFormDefault="qualified"
  attributeFormDefault="unqualified"
  xmlns="%s"
  %s
  xmlns:pv="http://www.mbsim-env.de/MBXMLUtils"
  xmlns:xs="http://www.w3.org/2001/XMLSchema">

  <xs:import namespace="http://www.mbsim-env.de/MBXMLUtils"/>

  %s

  %s

</xs:schema>
''' % (getNS(moduleName, False), getNS(moduleName, False), nsPrefixDef, nsImportDef, _moduleData[moduleName]['xsdElements'])
  return xsd

%}
