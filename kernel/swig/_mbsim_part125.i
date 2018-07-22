// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part100


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
  class KinematicCoordinatesObserver;
  class LocalCriteriaFunction;
  class LocalResidualCriteriaFunction;
  class LocalShiftCriteriaFunction;
  class MBSimEnvironment;
  class MechanicalLink;
  class NewtonJacobianFunction;
  class NumericalNewtonJacobianFunction;
  class RelativeKinematicsObserver;
  class StandardDampingFunction;
}
#include "mbsim/observers/rigid_body_system_observer.h"
#include "mbsim/objects/object.h"
#include "mbsim/objects/body.h"
#include "mbsim/objects/rigid_body.h"
using namespace MBSim; // SWIGs namespace handling seems to be buggy -> this fixes this
using namespace fmatvec; // SWIGs namespace handling seems to be buggy -> this fixes this
%}



// wrap the following classes
%include "mbsim/observers/rigid_body_system_observer.h"
%include "mbsim/objects/object.h"
%include "mbsim/objects/body.h"
%include "mbsim/objects/rigid_body.h"
