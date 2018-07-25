// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part150


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
  class FixedContourFrame;
  class FloatingContourFrame;
  class FloatingRelativeContourFrame;
  template<typename Sig> class DistanceFunction;
  class FuncPairPlanarContourPoint;
  class Object;
  class CriteriaFunction;
  class DampingFunction;
  class Environment;
  class FixedRelativeFrame;
  class GlobalCriteriaFunction;
  class GlobalResidualCriteriaFunction;
  class GlobalShiftCriteriaFunction;
  class KinematicCoordinatesObserver;
  class LocalCriteriaFunction;
  class LocalResidualCriteriaFunction;
  class LocalShiftCriteriaFunction;
  class MBSimEnvironment;
  class NewtonJacobianFunction;
  class NumericalNewtonJacobianFunction;
  class Observer;
  class RelativeKinematicsObserver;
  class RigidBodySystemObserver;
  class StandardDampingFunction;
}
#include "mbsim/links/contact.h"
#include "mbsim/links/contour_link.h"
#include "mbsim/links/fixed_frame_link.h"
#include "mbsim/links/floating_frame_link.h"
#include "mbsim/links/directional_spring_damper.h"
#include "mbsim/links/isotropic_rotational_spring_damper.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/links/spring_damper.h"
#include "mbsim/links/rigid_body_link.h"
#include "mbsim/links/dual_rigid_body_link.h"
#include "mbsim/links/generalized_friction.h"
#include "mbsim/links/generalized_spring_damper.h"
#include "mbsim/links/generalized_kinematic_excitation.h"
#include "mbsim/links/generalized_acceleration_excitation.h"
#include "mbsim/links/generalized_position_excitation.h"
#include "mbsim/links/generalized_velocity_excitation.h"
using namespace MBSim; // SWIGs namespace handling seems to be buggy -> this fixes this
using namespace fmatvec; // SWIGs namespace handling seems to be buggy -> this fixes this
%}



// wrap the following classes
%include "mbsim/links/contact.h"
%include "mbsim/links/contour_link.h"
%include "mbsim/links/fixed_frame_link.h"
%include "mbsim/links/floating_frame_link.h"
%include "mbsim/links/directional_spring_damper.h"
%include "mbsim/links/isotropic_rotational_spring_damper.h"
%include "mbsim/links/joint.h"
%include "mbsim/links/kinetic_excitation.h"
%include "mbsim/links/spring_damper.h"
%include "mbsim/links/rigid_body_link.h"
%include "mbsim/links/dual_rigid_body_link.h"
%include "mbsim/links/generalized_friction.h"
%include "mbsim/links/generalized_spring_damper.h"
%include "mbsim/links/generalized_kinematic_excitation.h"
%include "mbsim/links/generalized_acceleration_excitation.h"
%include "mbsim/links/generalized_position_excitation.h"
%include "mbsim/links/generalized_velocity_excitation.h"
