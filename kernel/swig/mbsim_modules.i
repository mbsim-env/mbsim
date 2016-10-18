// check target language
#ifndef SWIGPYTHON
  #error "Only Pyhton as target language is supported."
#endif

// disable warning 473
#pragma SWIG nowarn=SWIGWARN_TYPEMAP_DIRECTOROUT_PTR



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

static std::map<PythonCpp::PyO, std::pair<std::shared_ptr<MBXMLUtils::DOMParser>, xercesc::DOMElement*>> domElementMap;

void _typemapDirectorinDOMElement(xercesc::DOMElement *_1, swig::SwigVar_PyObject &_input) {
  using namespace MBXMLUtils;
  using namespace PythonCpp;
  xercesc::DOMDocument *doc=_1->getOwnerDocument();

  // get the uri
  std::string uri=X()%doc->getDocumentURI();
  uri=uri.substr(7); // remove the "file://" part from the uri

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
  // root = ET.parse(uri).getroot()
  // return root.find(xpath)
  PyO et(CALLPY(PyImport_ImportModule, "xml.etree.cElementTree"));
  PyO parse(CALLPY(PyObject_GetAttrString, et, "parse"));
  PyO uristr(CALLPY(PyString_FromString, uri));
  PyO uriarg(PyTuple_Pack(1, uristr.get()));
  PyO tree(CALLPY(PyObject_CallObject, parse, uriarg));
  PyO getroot(CALLPY(PyObject_GetAttrString, tree, "getroot"));
  PyO root(CALLPY(PyObject_CallObject, getroot, nullptr));
  PyO find(CALLPY(PyObject_GetAttrString, root, "find"));
  PyO xpathstr(CALLPY(PyString_FromString, xpath));
  PyO xpatharg(PyTuple_Pack(1, xpathstr.get()));
  PyO pye(CALLPY(PyObject_CallObject, find, xpatharg));
  domElementMap[pye]=std::make_pair(D(doc)->getParser(), _1);
  _input=swig::SwigVar_PyObject(pye.get());
}

void _typemapInDOMElement(xercesc::DOMElement *&_1, PyObject *_input) {
  auto it=domElementMap.find(PythonCpp::PyO(_input));
  if(it==domElementMap.end())
    throw std::runtime_error("No mapping from Python ElementTree to xercesc found (Wrong call sequence!?)");
  _1=it->second.second;
}

%}



// include fmatvec wrapping
%include "fmatvec.i"

// create directors for everything
%feature("director");

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

%typemap(directorin) xercesc::DOMElement* %{
  _typemapDirectorinDOMElement($1, $input);
%}

%typemap(in) xercesc::DOMElement* {
  try {
    _typemapInDOMElement($1, $input);
  }
  FMATVEC_CATCHARG
}
