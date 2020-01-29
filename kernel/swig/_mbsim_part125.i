// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part125


// include the general mbsim SWIG configuration (used by all MBSim modules)
%import fmatvec.i
%include "mbsim_include.i"
%import mbsim.i
%import _mbsim_part050.i
%import _mbsim_part100.i

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
namespace MBSim {
  class ContourFrame;
  class CriteriaFunction;
  class DampingFunction;
  template<typename Sig> class DistanceFunction;
  class Environment;
  class FixedContourFrame;
  class FloatingContourFrame;
  class FloatingRelativeContourFrame;
  class FloatingRelativeFrame;
  class FrameLink;
  class FuncPairPlanarContourPoint;
  class GlobalCriteriaFunction;
  class GlobalResidualCriteriaFunction;
  class GlobalShiftCriteriaFunction;
  class LocalCriteriaFunction;
  class LocalResidualCriteriaFunction;
  class LocalShiftCriteriaFunction;
  class MBSimEnvironment;
  class MechanicalLink;
  class NewtonJacobianFunction;
  class NumericalNewtonJacobianFunction;
  class StandardDampingFunction;
}
#include "mbsim/objects/object.h"
#include "mbsim/objects/body.h"
#include "mbsim/objects/rigid_body.h"
using namespace MBSim; // SWIGs namespace handling seems to be buggy -> this fixes this
using namespace fmatvec; // SWIGs namespace handling seems to be buggy -> this fixes this
%}



// wrap the following classes
%include "mbsim/objects/object.h"
%include "mbsim/objects/body.h"
%include "mbsim/objects/rigid_body.h"
