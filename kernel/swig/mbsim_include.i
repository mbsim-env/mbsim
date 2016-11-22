// check target language
#ifndef SWIGPYTHON
  #error "Only Python as target language is supported."
#endif

%include openmbvcppinterface/OpenMBV_include.i


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

// Map from Python ElementTree to Xerces DOMElement.
// Only one map at a time can be active. This map is accessible by the static variables global.
// This is defined in _typemapInDOMElement and read in _typemapDirectorinDOMElement
class MapPyXercesDOMElement : public std::pair<PythonCpp::PyO, xercesc::DOMElement*> {
  public:
    MapPyXercesDOMElement() {
      if(global)
        throw std::runtime_error("Global Python ElementTree to xercesc map is already used (Wrong call sequence!?)");
      global=this;
    }
    ~MapPyXercesDOMElement() {
      global=nullptr;
    }
    static const MapPyXercesDOMElement* getGlobal() { return global; }
  private:
    static MapPyXercesDOMElement* global;
};
MapPyXercesDOMElement* MapPyXercesDOMElement::global=nullptr;

void _typemapDirectorinDOMElement(xercesc::DOMElement *_1, swig::SwigVar_PyObject &_input, MapPyXercesDOMElement &map) {
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
  PyO uriarg(Py_BuildValue("(s)", uri.c_str()));
  PyO tree(CALLPY(PyObject_CallObject, parse, uriarg));
  PyO getroot(CALLPY(PyObject_GetAttrString, tree, "getroot"));
  PyO root(CALLPY(PyObject_CallObject, getroot, nullptr));
  PyO find(CALLPY(PyObject_GetAttrString, root, "find"));
  PyO xpatharg(Py_BuildValue("(s)", xpath.c_str()));
  PyO pye(CALLPY(PyObject_CallObject, find, xpatharg));
  // set mapping
  map.first=pye;
  map.second=_1;
  // set Python input
  _input=pye.incRef().get(); // SwigVar_PyObject steals a reference
}

void _typemapInDOMElement(xercesc::DOMElement *&_1, PyObject *_input) {
  // check mapping
  if(!MapPyXercesDOMElement::getGlobal() || MapPyXercesDOMElement::getGlobal()->first!=PythonCpp::PyO(_input, true))
    throw std::runtime_error("Global map from Python ElementTree to xercesc is not defined or wrong (Wrong call sequence!?)");
  // return mapped xerces DOMElement
  _1=MapPyXercesDOMElement::getGlobal()->second;
}

%}



// include fmatvec wrapping
%include "fmatvec_include.i"
