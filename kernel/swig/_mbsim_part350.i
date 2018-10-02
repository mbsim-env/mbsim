// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part350


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
%import _mbsim_part300.i

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
namespace MBSim {
  class BilateralConstraint;
  class BilateralImpact;
  class Body;
  class Constraint;
  class ContourQuad;
  class Cuboid;
  template<typename Sig> class DistanceFunction;
  class DynamicSystem;
  class Environment;
  class FixedContourFrame;
  class FixedRelativeFrame;
  class FloatingContourFrame;
  class FloatingRelativeContourFrame;
  class FloatingRelativeFrame;
  class FrameLink;
  class GeneralizedAccelerationConstraint;
  class GeneralizedConstraint;
  class GeneralizedDualConstraint;
  class GeneralizedPositionConstraint;
  class GeneralizedVelocityConstraint;
  class Graph;
  class Group;
  class JointConstraint;
  class KinematicCoordinatesObserver;
  class MBSimEnvironment;
  class MechanicalConstraint;
  class Observer;
  class PlanarContour;
  class PlanarCoulombFriction;
  class PlanarCoulombImpact;
  class PlanarStribeckFriction;
  class PlanarStribeckImpact;
  class RegularizedConstraint;
  class RegularizedBilateralConstraint;
  class RegularizedFriction;
  class RegularizedPlanarFriction;
  class RegularizedSpatialFriction;
  class RegularizedUnilateralConstraint;
  class RelativeKinematicsObserver;
  class RigidBody;
  class RigidBodySystemObserver;
  class Room;
  class SpatialContour;
  class SpatialCoulombFriction;
  class SpatialCoulombImpact;
  class SpatialStribeckFriction;
  class SpatialStribeckImpact;
  class UnilateralConstraint;
  class UnilateralNewtonImpact;
  class Contact;
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
#include "mbsim/contact_kinematics/contact_kinematics.h"
#include "mbsim/contact_kinematics/circle_circle.h"
#include "mbsim/contact_kinematics/circle_extrusion.h"
#include "mbsim/contact_kinematics/circle_frustum.h"
#include "mbsim/contact_kinematics/circle_line.h"
#include "mbsim/contact_kinematics/circle_linesegment.h"
#include "mbsim/contact_kinematics/circle_planarcontour.h"
#include "mbsim/contact_kinematics/circle_planarfrustum.h"
#include "mbsim/contact_kinematics/circle_plane.h"
#include "mbsim/contact_kinematics/compoundcontour_compoundcontour.h"
#include "mbsim/contact_kinematics/compoundcontour_contour.h"
#include "mbsim/contact_kinematics/edge_edge.h"
#include "mbsim/contact_kinematics/line_planarcontour.h"
#include "mbsim/contact_kinematics/plate_polynomialfrustum.h"
#include "mbsim/contact_kinematics/point_circle.h"
#include "mbsim/contact_kinematics/point_contourinterpolation.h"
#include "mbsim/contact_kinematics/point_extrusion.h"
#include "mbsim/contact_kinematics/point_frustum.h"
#include "mbsim/contact_kinematics/point_line.h"
#include "mbsim/contact_kinematics/point_linesegment.h"
#include "mbsim/contact_kinematics/point_planarcontour.h"
#include "mbsim/contact_kinematics/point_plane.h"
#include "mbsim/contact_kinematics/point_planewithfrustum.h"
#include "mbsim/contact_kinematics/point_plate.h"
#include "mbsim/contact_kinematics/point_polynomialfrustum.h"
#include "mbsim/contact_kinematics/point_spatialcontour.h"
#include "mbsim/contact_kinematics/point_sphere.h"
#include "mbsim/contact_kinematics/sphere_frustum.h"
#include "mbsim/contact_kinematics/sphere_plane.h"
#include "mbsim/contact_kinematics/sphere_plate.h"
#include "mbsim/contact_kinematics/sphere_polynomialfrustum.h"
#include "mbsim/contact_kinematics/sphere_sphere.h"
#include "mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h"
#include "mbsim/modelling_interface.h"
#include "mbsim/utils/colors.h"
using namespace MBSim; // SWIGs namespace handling seems to be buggy -> this fixes this
using namespace fmatvec; // SWIGs namespace handling seems to be buggy -> this fixes this
%}



// wrap the following classes
%include "mbsim/contact_kinematics/contact_kinematics.h"
%include "mbsim/contact_kinematics/circle_circle.h"
%include "mbsim/contact_kinematics/circle_extrusion.h"
%include "mbsim/contact_kinematics/circle_frustum.h"
%include "mbsim/contact_kinematics/circle_line.h"
%include "mbsim/contact_kinematics/circle_linesegment.h"
%include "mbsim/contact_kinematics/circle_planarcontour.h"
%include "mbsim/contact_kinematics/circle_planarfrustum.h"
%include "mbsim/contact_kinematics/circle_plane.h"
%include "mbsim/contact_kinematics/compoundcontour_compoundcontour.h"
%include "mbsim/contact_kinematics/compoundcontour_contour.h"
%include "mbsim/contact_kinematics/edge_edge.h"
%include "mbsim/contact_kinematics/line_planarcontour.h"
%include "mbsim/contact_kinematics/plate_polynomialfrustum.h"
%include "mbsim/contact_kinematics/point_circle.h"
%include "mbsim/contact_kinematics/point_contourinterpolation.h"
%include "mbsim/contact_kinematics/point_extrusion.h"
%include "mbsim/contact_kinematics/point_frustum.h"
%include "mbsim/contact_kinematics/point_line.h"
%include "mbsim/contact_kinematics/point_linesegment.h"
%include "mbsim/contact_kinematics/point_planarcontour.h"
%include "mbsim/contact_kinematics/point_plane.h"
%include "mbsim/contact_kinematics/point_planewithfrustum.h"
%include "mbsim/contact_kinematics/point_plate.h"
%include "mbsim/contact_kinematics/point_polynomialfrustum.h"
%include "mbsim/contact_kinematics/point_spatialcontour.h"
%include "mbsim/contact_kinematics/point_sphere.h"
%include "mbsim/contact_kinematics/sphere_frustum.h"
%include "mbsim/contact_kinematics/sphere_plane.h"
%include "mbsim/contact_kinematics/sphere_plate.h"
%include "mbsim/contact_kinematics/sphere_polynomialfrustum.h"
%include "mbsim/contact_kinematics/sphere_sphere.h"
%include "mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h"
%include "mbsim/modelling_interface.h"
%include "mbsim/utils/colors.h"
