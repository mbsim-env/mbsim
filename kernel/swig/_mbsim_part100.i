// wrap MBSim with directors (and all protected class data)
%module(directors="1", allprotected="1") _mbsim_part100


// include the general mbsim SWIG configuration (used by all MBSim modules)
%import fmatvec.i
%include "mbsim_include.i"
%import mbsim.i

// includes needed in the generated swig c++ code
%{
#include <config.h> // to use consistent preprocessor defines
namespace MBSim {
 template<typename Sig> class DistanceFunction;
 class FuncPairPlanarContourPoint;
 class DampingFunction;
 class StandardDampingFunction;
 class NewtonJacobianFunction;
 class CriteriaFunction;
 class NumericalNewtonJacobianFunction;
 class GlobalCriteriaFunction;
 class LocalCriteriaFunction;
 class GlobalResidualCriteriaFunction;
 class LocalResidualCriteriaFunction;
 class GlobalShiftCriteriaFunction;
 class LocalShiftCriteriaFunction;
}
#include "mbsim/utils/index.h"
#include "mbsim/links/link.h"
#include "mbsim/links/mechanical_link.h"
#include "mbsim/links/frame_link.h"
#include "mbsim/frames/frame.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/frames/fixed_contour_frame.h"
#include "mbsim/frames/floating_contour_frame.h"
#include "mbsim/frames/floating_relative_contour_frame.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/frames/floating_relative_frame.h"
#include "mbsim/environment.h"
#include "mbsim/observers/observer.h"
#include "mbsim/observers/kinematic_coordinates_observer.h"
#include "mbsim/observers/relative_kinematics_observer.h"
#include "mbsim/observers/rigid_body_group_observer.h"
#include "mbsim/objects/object.h"
#include "mbsim/objects/body.h"
#include "mbsim/objects/rigid_body.h"
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
%rename(lambda_) MBSim::Link::lambda; // lambda is a python keyword -> rename it to lambda_
%include "mbsim/utils/index.h"
%include "mbsim/links/link.h"
%include "mbsim/links/mechanical_link.h"
%include "mbsim/links/frame_link.h"
%include "mbsim/frames/frame.h"
%include "mbsim/frames/contour_frame.h"
%include "mbsim/frames/fixed_contour_frame.h"
%include "mbsim/frames/floating_contour_frame.h"
%include "mbsim/frames/floating_relative_contour_frame.h"
%include "mbsim/frames/fixed_relative_frame.h"
%include "mbsim/frames/floating_relative_frame.h"
%include "mbsim/environment.h"
%include "mbsim/observers/observer.h"
%include "mbsim/observers/kinematic_coordinates_observer.h"
%include "mbsim/observers/relative_kinematics_observer.h"
%include "mbsim/observers/rigid_body_group_observer.h"
%include "mbsim/objects/object.h"
%include "mbsim/objects/body.h"
%include "mbsim/objects/rigid_body.h"
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
