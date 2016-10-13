// check target language
#ifndef SWIGPYTHON
  #error "Only Pyhton as target language is supported."
#endif

// add code to the generated code
%{
#include "mbxmlutils/py2py3cppwrapper.h"
%}

// include fmatvec wrapping
%include "fmatvec.i"

// create directors for everything
%feature("director");

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
