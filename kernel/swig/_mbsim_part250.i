// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part250


// include the general mbsim SWIG configuration (used by all MBSim modules)
%import fmatvec.i
%include "mbsim_include.i"
%import mbsim.i
%import _mbsim_part100.i
%import _mbsim_part200.i

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
namespace MBSim {
  class AbsoluteKinematicsObserver;
  class Body;
  class Circle;
  class CompoundContour;
  class ContourInterpolation;
  class Link;
  class MechanicalLink;
  class ContourLink;
  class ContourQuad;
  class KinematicCoordinatesObserver;
  class CoordinatesObserver;
  class CriteriaFunction;
  class Cuboid;
  class DampingFunction;
  class DirectionalSpringDamper;
  template<typename Sig> class DistanceFunction;
  class DualRigidBodyLink;
  class Edge;
  class Environment;
  class FixedContourFrame;
  class FloatingContourFrame;
  class FixedFrameLink;
  class FloatingFrameLink;
  class FloatingRelativeContourFrame;
  class FloatingRelativeFrame;
  class FrameLink;
  class Constraint;
  class MechanicalConstraint;
  class GeneralizedAccelerationConstraint;
  class GeneralizedAccelerationExcitation;
  class GeneralizedConstraint;
  class GeneralizedDualConstraint;
  class GeneralizedFriction;
  class GeneralizedKinematicExcitation;
  class GeneralizedPositionConstraint;
  class GeneralizedPositionExcitation;
  class GeneralizedSpringDamper;
  class GeneralizedVelocityConstraint;
  class GeneralizedVelocityExcitation;
  class GlobalCriteriaFunction;
  class GlobalResidualCriteriaFunction;
  class GlobalShiftCriteriaFunction;
  class IsotropicRotationalSpringDamper;
  class Joint;
  class JointConstraint;
  class KinematicsObserver;
  class KineticExcitation;
  class LocalCriteriaFunction;
  class LocalResidualCriteriaFunction;
  class LocalShiftCriteriaFunction;
  class MBSimEnvironment;
  class NewtonJacobianFunction;
  class NumericalNewtonJacobianFunction;
  class RelativeKinematicsObserver;
  class RigidBody;
  class RigidBodySystemObserver;
  class RigidBodyLink;
  class Room;
  class SpringDamper;
  class StandardDampingFunction;
  class FuncPairPlanarContourPoint;
}
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
