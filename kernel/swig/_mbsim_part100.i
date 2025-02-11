// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part100


// include the general mbsim SWIG configuration (used by all MBSim modules)
%import fmatvec.i
%include "mbsim_include.i"
%import mbsim.i
%import _mbsim_part050.i

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
namespace MBSim {
  class ContourFrame;
  class CriteriaFunction;
  class DampingFunction;
  class FixedContourFrame;
  class FixedRelativeFrame;
  class FloatingContourFrame;
  class FloatingRelativeContourFrame;
  class FloatingRelativeFrame;
  class FrameLink;
  class FuncPairPlanarContourPoint;
  class FunctionBase;
  template<typename Sig> class Function;
  class GlobalCriteriaFunction;
  class GlobalResidualCriteriaFunction;
  class GlobalShiftCriteriaFunction;
  class Link;
  class LocalCriteriaFunction;
  class LocalResidualCriteriaFunction;
  class LocalShiftCriteriaFunction;
  class MechanicalLink;
  class NewtonJacobianFunction;
  class NumericalNewtonJacobianFunction;
  class StandardDampingFunction;
}
#include "mbsim/environment.h"
#include "mbsim/observers/observer.h"
#include "mbsim/functions/piecewise_polynom_function.h"
using namespace MBSim; // SWIGs namespace handling seems to be buggy -> this fixes this
using namespace fmatvec; // SWIGs namespace handling seems to be buggy -> this fixes this
%}



// wrap the following classes
%include "fmatvec/function.h"
%template() fmatvec::Der<double,double>;
%include "mbsim/environment.h"
%include "mbsim/functions/piecewise_polynom_function.h"
%template(PiecewisePolynomFunction_d) MBSim::PiecewisePolynomFunction<double(double)>;
