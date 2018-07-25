// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part300


// include the general mbsim SWIG configuration (used by all MBSim modules)
%import fmatvec.i
%include "mbsim_include.i"
%import mbsim.i
%import _mbsim_part050.i
%import _mbsim_part100.i
%import _mbsim_part125.i
%import _mbsim_part150.i
%import _mbsim_part200.i
%import _mbsim_part250.i

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
namespace MBSim {
  class Body;
  class Circle;
  class CompoundContour;
  class Constraint;
  class ContactPolyfun;
  class Contour;
  class ContourFrame;
  class ContourInterpolation;
  class ContourQuad;
  class CriteriaFunction;
  class Cuboid;
  class DampingFunction;
  template<typename Sig> class DistanceFunction;
  class DynamicSystem;
  class Edge;
  class Environment;
  class FixedContourFrame;
  class FixedRelativeFrame;
  class FloatingContourFrame;
  class FloatingRelativeContourFrame;
  class FloatingRelativeFrame;
  class FrameLink;
  class Frustum;
  class GeneralizedAccelerationConstraint;
  class GeneralizedConstraint;
  class GeneralizedDualConstraint;
  class GeneralizedPositionConstraint;
  class GeneralizedVelocityConstraint;
  class GlobalCriteriaFunction;
  class GlobalResidualCriteriaFunction;
  class GlobalShiftCriteriaFunction;
  class Graph;
  class Group;
  class JointConstraint;
  class KinematicCoordinatesObserver;
  class Line;
  class LineSegment;
  class Link;
  class LocalCriteriaFunction;
  class LocalResidualCriteriaFunction;
  class LocalShiftCriteriaFunction;
  class MBSimEnvironment;
  class MechanicalConstraint;
  class MechanicalLink;
  class NewtonJacobianFunction;
  class NumericalNewtonJacobianFunction;
  class Object;
  class Observer;
  class PlanarContour;
  class PlanarFrustum;
  class Plane;
  class PlaneWithFrustum;
  class Plate;
  class Point;
  class PolynomialFrustum;
  class RelativeKinematicsObserver;
  class RigidBody;
  class RigidBodySystemObserver;
  class RigidContour;
  class Room;
  class SpatialContour;
  class Sphere;
  class StandardDampingFunction;
  class FuncPairPlanarContourPoint;
  class Contact;
  class ContourLink;
  class DirectionalSpringDamper;
  class DualRigidBodyLink;
  class FixedFrameLink;
  class FloatingFrameLink;
  class GeneralizedAccelerationExcitation;
  class GeneralizedFriction;
  class GeneralizedKinematicExcitation;
  class GeneralizedPositionExcitation;
  class GeneralizedSpringDamper;
  class GeneralizedVelocityExcitation;
  class InverseKineticsJoint;
  class IsotropicRotationalSpringDamper;
  class Joint;
  class KineticExcitation;
  class RigidBodyLink;
  class SpringDamper;
}
#include "mbsim/constitutive_laws/friction_force_law.h"
#include "mbsim/constitutive_laws/planar_coulomb_friction.h"
#include "mbsim/constitutive_laws/planar_stribeck_friction.h"
#include "mbsim/constitutive_laws/regularized_planar_friction.h"
#include "mbsim/constitutive_laws/regularized_spatial_friction.h"
#include "mbsim/constitutive_laws/spatial_coulomb_friction.h"
#include "mbsim/constitutive_laws/spatial_stribeck_friction.h"
#include "mbsim/constitutive_laws/friction_impact_law.h"
#include "mbsim/constitutive_laws/planar_coulomb_impact.h"
#include "mbsim/constitutive_laws/planar_stribeck_impact.h"
#include "mbsim/constitutive_laws/spatial_coulomb_impact.h"
#include "mbsim/constitutive_laws/spatial_stribeck_impact.h"
#include "mbsim/constitutive_laws/generalized_force_law.h"
#include "mbsim/constitutive_laws/bilateral_constraint.h"
#include "mbsim/constitutive_laws/regularized_bilateral_constraint.h"
#include "mbsim/constitutive_laws/regularized_unilateral_constraint.h"
#include "mbsim/constitutive_laws/unilateral_constraint.h"
#include "mbsim/constitutive_laws/generalized_impact_law.h"
#include "mbsim/constitutive_laws/bilateral_impact.h"
#include "mbsim/constitutive_laws/unilateral_newton_impact.h"
using namespace MBSim; // SWIGs namespace handling seems to be buggy -> this fixes this
using namespace fmatvec; // SWIGs namespace handling seems to be buggy -> this fixes this
%}



// wrap the following classes
%include "mbsim/constitutive_laws/friction_force_law.h"
%include "mbsim/constitutive_laws/planar_coulomb_friction.h"
%include "mbsim/constitutive_laws/planar_stribeck_friction.h"
%include "mbsim/constitutive_laws/regularized_planar_friction.h"
%include "mbsim/constitutive_laws/regularized_spatial_friction.h"
%include "mbsim/constitutive_laws/spatial_coulomb_friction.h"
%include "mbsim/constitutive_laws/spatial_stribeck_friction.h"
%include "mbsim/constitutive_laws/friction_impact_law.h"
%include "mbsim/constitutive_laws/planar_coulomb_impact.h"
%include "mbsim/constitutive_laws/planar_stribeck_impact.h"
%include "mbsim/constitutive_laws/spatial_coulomb_impact.h"
%include "mbsim/constitutive_laws/spatial_stribeck_impact.h"
%include "mbsim/constitutive_laws/generalized_force_law.h"
%include "mbsim/constitutive_laws/bilateral_constraint.h"
%include "mbsim/constitutive_laws/regularized_bilateral_constraint.h"
%include "mbsim/constitutive_laws/regularized_unilateral_constraint.h"
%include "mbsim/constitutive_laws/unilateral_constraint.h"
%include "mbsim/constitutive_laws/generalized_impact_law.h"
%include "mbsim/constitutive_laws/bilateral_impact.h"
%include "mbsim/constitutive_laws/unilateral_newton_impact.h"
