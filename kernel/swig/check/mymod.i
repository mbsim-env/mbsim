%module(directors="1", allprotected="1") mymod

// include fmatvec conversions
%include fmatvec.i

// use SWIG_exception to throw a target language exception
%include exception.i

// convert c++ exceptions in function calls to target language exceptions
%exception {
  try {
    $action
  }
  catch(const std::exception &ex) {
    SWIG_exception(SWIG_RuntimeError, (std::string("In function $symname: ")+ex.what()).c_str());
  }
}

// add code to generated file (helper functions and includes)
%{
#include "mymod.h"
%}

// wrap this
%include "mymod.h"
