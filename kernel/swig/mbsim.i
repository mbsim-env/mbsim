// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") mbsim

%include "fmatvec.i"



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
%feature("director") MBSim::Link;
%feature("director") MBSim::FrameLink;

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
#define PY_ARRAY_UNIQUE_SYMBOL mbxmlutils_pyeval_ARRAY_API
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/join.hpp>
#include "mbsim/objectfactory_part.h"
#include "mbsim/objects/object.h"
#include "mbsim/links/frame_link.h"
#include "mbsim/frames/frame.h"

void checkNumPyDoubleType(int type) {
  if(type!=NPY_SHORT    && type!=NPY_USHORT    &&
     type!=NPY_INT      && type!=NPY_UINT      &&
     type!=NPY_LONG     && type!=NPY_ULONG     &&
     type!=NPY_LONGLONG && type!=NPY_ULONGLONG &&
     type!=NPY_FLOAT    && type!=NPY_DOUBLE    && type!=NPY_LONGDOUBLE)
    throw std::runtime_error("Value is not of type double.");
}

double arrayGetDouble(PyArrayObject *a, int type, int r, int c=-1) {
  switch(type) {
    case NPY_SHORT:      return *static_cast<npy_short*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_USHORT:     return *static_cast<npy_ushort*>    (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_INT:        return *static_cast<npy_int*>       (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_UINT:       return *static_cast<npy_uint*>      (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONG:       return *static_cast<npy_long*>      (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_ULONG:      return *static_cast<npy_ulong*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONGLONG:   return *static_cast<npy_longlong*>  (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_ULONGLONG:  return *static_cast<npy_ulonglong*> (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_FLOAT:      return *static_cast<npy_float*>     (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_DOUBLE:     return *static_cast<npy_double*>    (c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
    case NPY_LONGDOUBLE: return *static_cast<npy_longdouble*>(c==-1 ? PyArray_GETPTR1(a, r) : PyArray_GETPTR2(a, r, c));
  }
  throw std::runtime_error("Value is not of type double (wrong element type).");
}

%}

// wrap std::string and std::vector
%include "std_string.i"
%include "std_vector.i"
%template(VectorFrameP) std::vector<MBSim::Frame*>;

%typemap(in) fmatvec::VecV %{
  if(PyArray_Check($input)) {
    PyArrayObject *a=reinterpret_cast<PyArrayObject*>($input);
    if(PyArray_NDIM(a)!=1)
      throw std::runtime_error("Value is not of type vector (wrong dimension).");
    int type=PyArray_TYPE(a);
    checkNumPyDoubleType(type);
    npy_intp *dims=PyArray_SHAPE(a);
    $1.resize(dims[0]);
    for(int i=0; i<dims[0]; ++i)
      $1(i)=arrayGetDouble(a, type, i);
  }
  else
    throw std::runtime_error("Value is not of type vector (wrong type).");
%}

%typemap(out) fmatvec::VecV %{
  npy_intp dims[1];
  dims[0]=$1.size();
  $result=PyArray_SimpleNew(1, dims, NPY_DOUBLE);
  std::copy(&$1(0), &$1(0)+$1.size(), static_cast<npy_double*>(PyArray_GETPTR1(reinterpret_cast<PyArrayObject*>($result), 0)));
%}

// wrap the following headers
%include "mbsim/objectfactory_part.h"
%include "fmatvec/atom.h"
%include "mbsim/element.h"
%include "mbsim/objects/object.h"
%rename(_lambda) MBSim::Link::lambda; // lambda is a python keyword -> rename it to _lambda
%include "mbsim/links/link.h"
%include "mbsim/links/frame_link.h"
%include "mbsim/frames/frame.h"






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

# get namespace of moudle moudleName (sourounded with { and } for the python moudle moduleName
def _getNSOf(moduleName):
  import sys
  ns=sys.modules[moduleName].NS
  return ns[1:-1]

_moduleData={}

# internal helper function to extend a Python class
def _extendClass(className):
  import xml.etree.cElementTree as ET
  # add initializeUsingXML method if not exist of convert arguments if it exists
  import types
  if not 'initializeUsingXML' in className.__dict__:
    # add initializeUsingXML method which runs the XML text child element as python code
    # where the variable 'self' represents this object
    def implicitInitializeUsingXML(self, e):
      super(className, self).initializeUsingXML(e)
      code=_getObjectInitCode(e, _FQN(_getNSOf(className.__module__), "pyinit."+className.__name__))
      if code!="":
        exec(code, {'self': self})
    className.initializeUsingXML=types.MethodType(implicitInitializeUsingXML, None, className)
  else:
    # save user defined initializeUsingXML and replace by implicit one (which convert com DOMElement to pyxml
    userInitializeUsingXML=className.initializeUsingXML
    def wrapperInitializeUsingXML(self, e_xerces):
      filename=_getDocumentFileNameOf(e_xerces)
      xpath=_getXPathOf(e_xerces)
      root = ET.parse(filename).getroot()
      e_py=root.find(xpath)
      userInitializeUsingXML(self, e_py, e_xerces)
    className.initializeUsingXML=types.MethodType(wrapperInitializeUsingXML, None, className)

  # create XML schema element and store it in _moduleData
  # store also all required modules in _moduleData
  baseClassName=className.__bases__
  if len(baseClassName)!=1:
    raise RuntimeError('Can only handle classed with one base class.')
  if not hasattr(className, 'getSchema'):
    xsdpart=ET.Element(XS+"sequence")
    xsdpart.append(ET.Element(XS+"element", {'name': 'pyinit.'+className.__name__, 'type': ET.QName(XS+'string'), 'minOccurs': '0'}))
  else:
    xsdpart=className.getSchema()
  xsd1=ET.Element(XS+"element", {"name": ET.QName(_getNSOf(className.__module__), className.__name__),
                                 "substitutionGroup": ET.QName(_getNSOf(baseClassName[0].__module__), baseClassName[0].__name__),
                                 "type": ET.QName(_getNSOf(className.__module__), className.__name__+"Type")})
  xsd2=ET.Element(XS+"complexType", {"name": ET.QName(_getNSOf(className.__module__), className.__name__+"Type")})
  cc=ET.Element(XS+"complexContent")
  xsd2.append(cc)
  ext=ET.Element(XS+"extension", {'base': ET.QName(_getNSOf(baseClassName[0].__module__), baseClassName[0].__name__+'Type')})
  cc.append(ext)
  ext.append(xsdpart)
  xsd=[xsd1, xsd2]
  global _moduleData
  if not className.__module__ in _moduleData:
    _moduleData[className.__module__]={'xsdElements': [], 'requiredModules': set()}
  _moduleData[className.__module__]['xsdElements'].extend(xsd)
  _moduleData[className.__module__]['requiredModules'].add(baseClassName[0].__module__)






# register class in the MBSim::ObjectFactory.
# This also extents className with some special members.
def registerXMLName(className):
  _extendClass(className)
  registerXMLName_internal(_FQN(_getNSOf(className.__module__), className.__name__),
    _AllocatePython(className).__disown__(), _DeallocatePython().__disown__())

# register singelton class in the MBSim::ObjectFactory
# This also extents className with some special members.
def registerXMLNameAsSingleton(className):
  _extendClass(className)
  registerXMLName_internal(_FQN(_getNSOf(className.__module__), className.__name__),
    _GetSingletonPython(className).__disown__(), _DeallocateSingletonPython().__disown__())

# create the xsd for this module.
def generateXMLSchemaFile(moduleName):
  import os.path
  import xml.etree.cElementTree as ET
  global _moduleData

  # register namespace/prefix mappings
  ET.register_namespace("pv", PV[1:-1])
  ET.register_namespace("xs", XS[1:-1])
  ET.register_namespace("", "http://mypytest")
  for module in _moduleData[moduleName]['requiredModules']:
    ET.register_namespace(module, _getNSOf(module))
  # create xsd file
  xsd=ET.Element(XS+"schema", {"targetNamespace": _getNSOf(moduleName),
                               "elementFormDefault": "qualified",
                               "attributeFormDefault": "unqualified"})
  xsd.append(ET.Element(XS+"import", {"namespace": PV[1:-1]}))
  for module in _moduleData[moduleName]['requiredModules']:
    xsd.append(ET.Element(XS+"import", {'namespace': _getNSOf(module)}))
  xsd.extend(_moduleData[moduleName]['xsdElements'])
  return xsd

# XML namespace of this module (prefixed with { and postfixed with })
NS="{http://www.mbsim-env.de/MBSim}"

XS="{http://www.w3.org/2001/XMLSchema}"
PV="{http://www.mbsim-env.de/MBXMLUtils}"

%}
