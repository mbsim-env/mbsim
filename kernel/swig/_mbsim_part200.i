// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part200


// include the general mbsim SWIG configuration (used by all MBSim modules)
%import fmatvec.i
%include "mbsim_include.i"
%import mbsim.i
%import _mbsim_part050.i
%import _mbsim_part100.i
%import _mbsim_part125.i
%import _mbsim_part150.i

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
namespace MBSim {
  class Body;
  class Contact;
  class ContourLink;
  class CriteriaFunction;
  class DampingFunction;
  class DirectionalSpringDamper;
  class DualRigidBodyLink;
  class Environment;
  class FixedContourFrame;
  class FloatingContourFrame;
  class FloatingFrameLink;
  class FloatingRelativeContourFrame;
  class FixedFrameLink;
  class FrameLink;
  class GeneralizedAccelerationExcitation;
  class GeneralizedFriction;
  class GeneralizedKinematicExcitation;
  class GeneralizedPositionExcitation;
  class GeneralizedSpringDamper;
  class GeneralizedVelocityExcitation;
  class GlobalCriteriaFunction;
  class GlobalResidualCriteriaFunction;
  class GlobalShiftCriteriaFunction;
  class InverseKineticsJoint;
  class IsotropicRotationalSpringDamper;
  class Joint;
  class KineticExcitation;
  class Link;
  class MechanicalLink;
  class LocalCriteriaFunction;
  class LocalResidualCriteriaFunction;
  class LocalShiftCriteriaFunction;
  class MBSimEnvironment;
  class NewtonJacobianFunction;
  class NumericalNewtonJacobianFunction;
  class Object;
  class Observer;
  class RigidBodyLink;
  class SpringDamper;
  class StandardDampingFunction;
  class FuncPairPlanarContourPoint;
}
#include "mbsim/constraints/constraint.h"
#include "mbsim/constraints/mechanical_constraint.h"
#include "mbsim/constraints/generalized_constraint.h"
#include "mbsim/constraints/joint_constraint.h"
#include "mbsim/constraints/generalized_dual_constraint.h"
#include "mbsim/constraints/generalized_acceleration_constraint.h"
#include "mbsim/constraints/generalized_position_constraint.h"
#include "mbsim/constraints/generalized_velocitiy_constraint.h"
#include "mbsim/contours/contour.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/contours/contour_quad.h"
#include "mbsim/contours/rigid_contour.h"
#include "mbsim/contours/circle.h"
#include "mbsim/contours/compound_contour.h"
#include "mbsim/contours/cuboid.h"
#include "mbsim/contours/room.h"
#include "mbsim/contours/edge.h"
using namespace MBSim; // SWIGs namespace handling seems to be buggy -> this fixes this
using namespace fmatvec; // SWIGs namespace handling seems to be buggy -> this fixes this
%}



// wrap the following classes
%include "mbsim/constraints/constraint.h"
%include "mbsim/constraints/mechanical_constraint.h"
%include "mbsim/constraints/generalized_constraint.h"
%include "mbsim/constraints/joint_constraint.h"
%include "mbsim/constraints/generalized_dual_constraint.h"
%include "mbsim/constraints/generalized_acceleration_constraint.h"
%include "mbsim/constraints/generalized_position_constraint.h"
%include "mbsim/constraints/generalized_velocitiy_constraint.h"
%include "mbsim/contours/contour.h"
%include "mbsim/contours/contour_interpolation.h"
%include "mbsim/contours/contour_quad.h"
%include "mbsim/contours/rigid_contour.h"
%include "mbsim/contours/circle.h"
%include "mbsim/contours/compound_contour.h"
%include "mbsim/contours/cuboid.h"
%include "mbsim/contours/room.h"
%include "mbsim/contours/edge.h"

%inline %{
  namespace MBSim {
    // create cast functions named castElementTo_TYPE where TYPE is the argument
    CAST_MBSIM_ELEMENT_TO(Constraint)
    CAST_MBSIM_ELEMENT_TO(MechanicalConstraint)
    CAST_MBSIM_ELEMENT_TO(GeneralizedConstraint)
    CAST_MBSIM_ELEMENT_TO(JointConstraint)
    CAST_MBSIM_ELEMENT_TO(GeneralizedDualConstraint)
    CAST_MBSIM_ELEMENT_TO(GeneralizedAccelerationConstraint)
    CAST_MBSIM_ELEMENT_TO(GeneralizedPositionConstraint)
    CAST_MBSIM_ELEMENT_TO(GeneralizedVelocityConstraint)
    CAST_MBSIM_ELEMENT_TO(Contour)
    CAST_MBSIM_ELEMENT_TO(ContourInterpolation)
    CAST_MBSIM_ELEMENT_TO(ContourQuad)
    CAST_MBSIM_ELEMENT_TO(RigidContour)
    CAST_MBSIM_ELEMENT_TO(Circle)
    CAST_MBSIM_ELEMENT_TO(CompoundContour)
    CAST_MBSIM_ELEMENT_TO(Cuboid)
    CAST_MBSIM_ELEMENT_TO(Room)
    CAST_MBSIM_ELEMENT_TO(Edge)
  }
%}
