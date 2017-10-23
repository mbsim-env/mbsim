// check target language
#ifndef SWIGPYTHON
  #error "Only Python as target language is supported."
#endif

// The following part is only used to generate a list of unwrapped classes.
// When this is enabled swig will fail but output all classes with are unwrapped.
%include "showUnwrappedClasses.i" 
#ifdef SHOW_UNWRAPPED_CLASSES
  // list here classes with should not be wrapped (these are remove from the list of unwrapped classes: make target "swig-unwrapped")
  WRAPPED_CLASS(MBXMLUtils::FQN)
  WRAPPED_CLASS(MBSim::Function<double(double)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<double(double,double)>::DDRetDArg1DArg2)
  WRAPPED_CLASS(MBSim::Function<double(double,double)>::DDRetDDArg1)
  WRAPPED_CLASS(MBSim::Function<double(double,double)>::DDRetDDArg2)
  WRAPPED_CLASS(MBSim::Function<double(double,double)>::DRetDArg1)
  WRAPPED_CLASS(MBSim::Function<double(double,double)>::DRetDArg2)
  WRAPPED_CLASS(MBSim::Function<double(double)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<double(fmatvec::Vector<fmatvec::Ref,double>,fmatvec::Vector<fmatvec::Ref,double>)>::DDRetDArg1DArg2)
  WRAPPED_CLASS(MBSim::Function<double(fmatvec::Vector<fmatvec::Ref,double>,fmatvec::Vector<fmatvec::Ref,double>)>::DDRetDDArg1)
  WRAPPED_CLASS(MBSim::Function<double(fmatvec::Vector<fmatvec::Ref,double>,fmatvec::Vector<fmatvec::Ref,double>)>::DDRetDDArg2)
  WRAPPED_CLASS(MBSim::Function<double(fmatvec::Vector<fmatvec::Ref,double>,fmatvec::Vector<fmatvec::Ref,double>)>::DRetDArg1)
  WRAPPED_CLASS(MBSim::Function<double(fmatvec::Vector<fmatvec::Ref,double>,fmatvec::Vector<fmatvec::Ref,double>)>::DRetDArg2)
  WRAPPED_CLASS(MBSim::Function<double(int)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<double(int)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::General,fmatvec::Var,fmatvec::Var,double>(fmatvec::VecV)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::General,fmatvec::Var,fmatvec::Var,double>(fmatvec::VecV)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Rotation,fmatvec::Fixed<3>,fmatvec::Fixed<3>,double>(double)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Rotation,fmatvec::Fixed<3>,fmatvec::Fixed<3>,double>(double)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Rotation,fmatvec::Fixed<3>,fmatvec::Fixed<3>,double>(fmatvec::VecV)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Rotation,fmatvec::Fixed<3>,fmatvec::Fixed<3>,double>(fmatvec::VecV,double)>::DDRetDArg1DArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Rotation,fmatvec::Fixed<3>,fmatvec::Fixed<3>,double>(fmatvec::VecV,double)>::DDRetDDArg1)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Rotation,fmatvec::Fixed<3>,fmatvec::Fixed<3>,double>(fmatvec::VecV,double)>::DDRetDDArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Rotation,fmatvec::Fixed<3>,fmatvec::Fixed<3>,double>(fmatvec::VecV,double)>::DRetDArg1)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Rotation,fmatvec::Fixed<3>,fmatvec::Fixed<3>,double>(fmatvec::VecV,double)>::DRetDArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Rotation,fmatvec::Fixed<3>,fmatvec::Fixed<3>,double>(fmatvec::VecV)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Symmetric,fmatvec::Var,fmatvec::Var,double>(double)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Matrix<fmatvec::Symmetric,fmatvec::Var,fmatvec::Var,double>(double)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::SquareMatrix<fmatvec::Ref,double>(fmatvec::Vec)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::SquareMatrix<fmatvec::Ref,double>(fmatvec::Vec)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::SquareMatrix<fmatvec::Var,double>(double)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::SquareMatrix<fmatvec::Var,double>(double)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<1>,double>(double)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<1>,double>(double)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<2>,double>(double)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<2>,double>(double)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(double)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(double)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(fmatvec::Vec2)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(fmatvec::Vec2)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(fmatvec::VecV)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(fmatvec::VecV,double)>::DDRetDArg1DArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(fmatvec::VecV,double)>::DDRetDDArg1)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(fmatvec::VecV,double)>::DDRetDDArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(fmatvec::VecV,double)>::DRetDArg1)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(fmatvec::VecV,double)>::DRetDArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Fixed<3>,double>(fmatvec::VecV)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Ref,double>(double)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Ref,double>(double)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Ref,double>(fmatvec::Vec)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Ref,double>(fmatvec::Vec,double)>::DDRetDArg1DArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Ref,double>(fmatvec::Vec,double)>::DDRetDDArg1)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Ref,double>(fmatvec::Vec,double)>::DDRetDDArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Ref,double>(fmatvec::Vec,double)>::DRetDArg1)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Ref,double>(fmatvec::Vec,double)>::DRetDArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Ref,double>(fmatvec::Vec)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(double)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(double)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV,double)>::DDRetDArg1DArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV,double)>::DDRetDDArg1)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV,double)>::DDRetDDArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV,double)>::DRetDArg1)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV,double)>::DRetDArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV)>::DRetDArg)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV,fmatvec::VecV)>::DDRetDArg1DArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV,fmatvec::VecV)>::DDRetDDArg1)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV,fmatvec::VecV)>::DDRetDDArg2)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV,fmatvec::VecV)>::DRetDArg1)
  WRAPPED_CLASS(MBSim::Function<fmatvec::Vector<fmatvec::Var,double>(fmatvec::VecV,fmatvec::VecV)>::DRetDArg2)
  WRAPPED_CLASS(MBSim::Function<int(fmatvec::Vector<fmatvec::Ref,double>)>::DDRetDDArg)
  WRAPPED_CLASS(MBSim::Function<int(fmatvec::Vector<fmatvec::Ref,double>)>::DRetDArg)
  WRAPPED_CLASS(MBSim::DistanceFunction<double(double)>)
  WRAPPED_CLASS(H5::GroupBase)
  WRAPPED_CLASS(H5::VectorSerie<double>)
  WRAPPED_CLASS(std::ostream)
#endif

%include openmbvcppinterface/OpenMBV_include.i

// create directors for everything
%feature("director");

// disable warning 473
#pragma SWIG nowarn=SWIGWARN_TYPEMAP_DIRECTOROUT_PTR
#pragma SWIG nowarn=SWIGWARN_IGNORE_OPERATOR_DELETE
#pragma SWIG nowarn=SWIGWARN_IGNORE_OPERATOR_DELARR


// add code to the generated code
%{

#include "mbxmlutils/py2py3cppwrapper.h"
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMDocument.hpp>

void _directorExcept(PyObject *error) {
  if(error) {
    // MISSING we abuse PyExc_MemoryError here instead of definding our own exception class
    // which indicates a "pass throught" exception
    if(PyErr_GivenExceptionMatches(error, PyExc_MemoryError)) {
      // pass throught a c++ exception
      PyObject *ptype, *pvalue, *ptraceback;
      PyErr_Fetch(&ptype, &pvalue, &ptraceback);
      Py_XDECREF(ptype);
      Py_XDECREF(ptraceback);
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

// This struct exists once globally (for all python modules; done via python capsule)
class _MapPyXercesDOMElement;
struct _MBSimGlobals {
  _MapPyXercesDOMElement *mapPyXercesDOMElement;
};
static _MBSimGlobals *mbsimGlobals=nullptr;

// Map from Python ElementTree to Xerces DOMElement.
// Only one map at a time can be active. This map is accessible by a PyCapsule.
// This is defined in _typemapInDOMElement and read in _typemapDirectorinDOMElement
class _MapPyXercesDOMElement : public std::pair<PythonCpp::PyO, xercesc::DOMElement*> {
  public:
    _MapPyXercesDOMElement() {
      if(mbsimGlobals->mapPyXercesDOMElement)
        throw std::runtime_error("Global Python ElementTree to xercesc map is already used (Wrong call sequence!?)");
      mbsimGlobals->mapPyXercesDOMElement=this;
    }
    ~_MapPyXercesDOMElement() {
      mbsimGlobals->mapPyXercesDOMElement=nullptr;
    }
    static const _MapPyXercesDOMElement* getGlobal() {
      return mbsimGlobals->mapPyXercesDOMElement;
    }
};

void _typemapDirectorinDOMElement(xercesc::DOMElement *_1, swig::SwigVar_PyObject &_input, _MapPyXercesDOMElement &map) {
  using namespace MBXMLUtils;
  using namespace PythonCpp;
  xercesc::DOMDocument *doc=_1->getOwnerDocument();

  // serialize to memory
  std::string xml;
  DOMParser::serializeToString(doc, xml, false);

  // get the XPath of e
  xercesc::DOMElement *r=doc->getDocumentElement();
  std::string xpath;
  for(xercesc::DOMElement *ee=_1; ee!=r; ee=static_cast<xercesc::DOMElement*>(ee->getParentNode())) {
    // get element name
    FQN eleName=E(ee)->getTagName();
    // count same elements on this level before this
    int count=1;
    for(xercesc::DOMElement *pre=ee->getPreviousElementSibling(); pre; pre=pre->getPreviousElementSibling())
      if(E(pre)->getTagName()==eleName)
        count++;
    // construct xpath
    xpath="{"+eleName.first+"}"+eleName.second+"["+std::to_string(count)+"]/"+xpath;
  }
  xpath=xpath.substr(0, xpath.size()-1);

  // create pythone ElementTree and return corresponding element
  // root = ET.fromstring(xml)
  // return root.find(xpath)
  PyO et(CALLPY(PyImport_ImportModule, "xml.etree.cElementTree"));
  PyO fromstring(CALLPY(PyObject_GetAttrString, et, "fromstring"));
  PyO root(CALLPY(PyObject_CallObject, fromstring, PythonCpp::Py_BuildValue_("(s)", xml)));
  PyO find(CALLPY(PyObject_GetAttrString, root, "find"));
  PyO pye(CALLPY(PyObject_CallObject, find, PythonCpp::Py_BuildValue_("(s)", xpath.c_str())));
  // set mapping
  map.first=pye;
  map.second=_1;
  // set Python input
  _input=pye.incRef().get(); // SwigVar_PyObject steals a reference
}

void _typemapInDOMElement(xercesc::DOMElement *&_1, PyObject *_input) {
  // check mapping
  if(!_MapPyXercesDOMElement::getGlobal() || _MapPyXercesDOMElement::getGlobal()->first!=PythonCpp::PyO(_input, true))
    throw std::runtime_error("Global map from Python ElementTree to xercesc is not defined or wrong (Wrong call sequence!?)");
  // return mapped xerces DOMElement
  _1=_MapPyXercesDOMElement::getGlobal()->second;
}

%}

%init %{
// create a global, cross python modules, mbsim object using python capsule (in a dummy module named _mbsimGlobalsModule)
if(CALLPY(PyDict_Contains, CALLPYB(PyImport_GetModuleDict), PythonCpp::Py_BuildValue_("s", "_mbsimGlobalsModule")))
  // get the global mbsim object
  mbsimGlobals=static_cast<_MBSimGlobals*>(CALLPY(PyCapsule_Import, "_mbsimGlobalsModule._mbsimGlobals", true));
else {
  // create a global mbsim object
  static _MBSimGlobals mbsimGlobalsStore;
  mbsimGlobals=&mbsimGlobalsStore;
  #if PY_MAJOR_VERSION < 3
    PythonCpp::PyO mbsimGlobalsModule(CALLPYB(Py_InitModule, const_cast<char*>("_mbsimGlobalsModule"), nullptr));
  #else
    #error "Python 3 not supported: module initialization is quite different in python 3."
  #endif
  static const char *mbsimGlobalName="_mbsimGlobalsModule._mbsimGlobals"; // PyCapsule_New does not copy this string
  PythonCpp::PyO mbsimGlobalsCapsule(CALLPY(PyCapsule_New, mbsimGlobals, mbsimGlobalName, nullptr));
  CALLPY(PyModule_AddObject, mbsimGlobalsModule, "_mbsimGlobals", mbsimGlobalsCapsule.incRef());
}
%}



// include fmatvec wrapping
%include "fmatvec_include.i"

%typemap(directorin) xercesc::DOMElement* %{
  _MapPyXercesDOMElement mapPyXercesDOMElement$argnum;
  _typemapDirectorinDOMElement($1, $input, mapPyXercesDOMElement$argnum);
%}

%typemap(in) xercesc::DOMElement* {
  try {
    _typemapInDOMElement($1, $input);
  }
  FMATVEC_CATCHARG
}

%import openmbvcppinterface/OpenMBV.i

%include "std_pair.i"
%include "std_map.i"


%ignore swigignore;

// wrap python error to c++ exception
%feature("director:except") %{
  _directorExcept($error);
%}

// wrap c++ exception to python error
%exception %{
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
%}
