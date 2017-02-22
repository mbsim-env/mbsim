// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part200


// include the general mbsim SWIG configuration (used by all MBSim modules)
%import fmatvec.i
%include "mbsim_include.i"
%import mbsim.i
%import _mbsim_part100.i

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
namespace MBSim {
  class Object;
  class Body;
  class Link;
  class MechanicalLink;
  class FixedFrameLink;
  class FloatingFrameLink;
  class FunctionBase;
  template<typename Sig> class Function;
  class Joint;
  template<typename Sig> class DistanceFunction;
  class Contact;
  class ContourFrame;
  class ContourLink;
  class KinematicCoordinatesObserver;
  class CriteriaFunction;
  class DampingFunction;
  class DirectionalSpringDamper;
  class DualRigidBodyLink;
  class Environment;
  class FixedContourFrame;
  class FixedRelativeFrame;
  class FloatingContourFrame;
  class FloatingRelativeContourFrame;
  class FrameLink;
  class FuncPairPlanarContourPoint;
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
  class KinematicsObserver;
  class LocalCriteriaFunction;
  class LocalResidualCriteriaFunction;
  class LocalShiftCriteriaFunction;
  class MBSimEnvironment;
  class NewtonJacobianFunction;
  class NumericalNewtonJacobianFunction;
  class Observer;
  class RelativeKinematicsObserver;
  class RigidBodyGroupObserver;
  class RigidBodyLink;
  class SpringDamper;
  class StandardDampingFunction;
  class KineticExcitation;
  class RigidBody;
  class FloatingRelativeFrame;
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
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/line_segment.h"
#include "mbsim/contours/planar_contour.h"
#include "mbsim/contours/planar_frustum.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/plate.h"
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/polynomial_frustum.h"
#include "mbsim/contours/spatial_contour.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/graph.h"
#include "mbsim/group.h"
#include "mbsim/dynamic_system_solver.h"
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
%include "mbsim/contours/frustum.h"
%include "mbsim/contours/line.h"
%include "mbsim/contours/line_segment.h"
%include "mbsim/contours/planar_contour.h"
%include "mbsim/contours/planar_frustum.h"
%include "mbsim/contours/plane.h"
%include "mbsim/contours/plate.h"
%include "mbsim/contours/planewithfrustum.h"
%include "mbsim/contours/point.h"
%include "mbsim/contours/polynomial_frustum.h"
%include "mbsim/contours/spatial_contour.h"
%include "mbsim/contours/sphere.h"
%include "mbsim/dynamic_system.h"
%include "mbsim/graph.h"
%include "mbsim/group.h"
%rename(global_) MBSim::DynamicSystemSolver::global; // global is a python keyword -> rename it to global_
%include "mbsim/dynamic_system_solver.h"
