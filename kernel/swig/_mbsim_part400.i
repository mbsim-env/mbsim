// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part400

// include the general mbsim SWIG configuration (used by all MBSim modules)
%import  fmatvec.i
%include "mbsim_include.i"

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
#include "mbsim/solver.h"
%}

%include "mbsim/solver.h"
