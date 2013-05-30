/*
 * DO NOT ADD ANYTHING IN THIS FILE!!!
 *
 * If you want to add new classes to the ObjectFactory add the corresponding
 *
 * MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(BaseType, NewType, MBSIMNS"NewType")
 *
 * line to the .cc file where the new class is defined!
 * Where NewType is the class name of the new class to be added to the ObjectFactory,
 * BaseType is the TOP LEVEL base class of NewType and
 * MBSIMNS"NewType" is the full qualified XML element name of NewType
 *
 * All below MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(...) line
 * should also be moved the the corrsponding *.cc files. They are just added here
 * temporarily during the change of the ObjectFactory code.
 */
#include "config.h"
#include "mbsim/objectfactory.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/group.h"
#include "mbsim/rigid_body.h"
#include "mbsim/kinetic_excitation.h"
#include "mbsim/spring_damper.h"
#include "mbsim/extern_generalized_io.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/line.h"
#include "mbsim/contours/line_segment.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/contours/frustum2d.h"
#include "mbsim/contours/face.h"
#include "mbsim/contours/edge.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/contours/contour_quad.h"
#include "mbsim/contours/cuboid.h"
#include "mbsim/contours/compound_contour.h"
#include "mbsim/contours/contour1s_analytical.h"
#include "mbsim/utils/function_library.h"
#include "mbsim/integrators/dopri5_integrator.h"
#include "mbsim/integrators/radau5_integrator.h"
#include "mbsim/integrators/lsode_integrator.h"
#include "mbsim/integrators/lsodar_integrator.h"
#include "mbsim/integrators/time_stepping_integrator.h"
#include "mbsim/integrators/time_stepping_ssc_integrator.h"
#include "mbsim/integrators/theta_time_stepping_integrator.h"
#include "mbsim/integrators/euler_explicit_integrator.h"
#include "mbsim/integrators/rksuite_integrator.h"
#include "mbsim/utils/contour_functions.h"
#include "mbsim/utils/function.h"
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
#  include "mbsim/utils/symbolic_function.h"
#endif
#include "mbsim/constraint.h"
#include "mbsim/observers/kinematics_observer.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#  include "openmbvcppinterface/objectfactory.h"
#endif

#define COMMA ,

using namespace std;
using namespace fmatvec;
using namespace MBSim;

/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedForceLaw, BilateralConstraint, MBSIMNS"BilateralConstraint")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedForceLaw, UnilateralConstraint, MBSIMNS"UnilateralConstraint")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedForceLaw, RegularizedBilateralConstraint, MBSIMNS"RegularizedBilateralConstraint")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedForceLaw, RegularizedUnilateralConstraint, MBSIMNS"RegularizedUnilateralConstraint")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedForceLaw, MaxwellUnilateralConstraint, MBSIMNS"MaxwellUnilateralConstraint")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedImpactLaw, BilateralImpact, MBSIMNS"BilateralImpact")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(GeneralizedImpactLaw, UnilateralNewtonImpact, MBSIMNS"UnilateralNewtonImpact")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionForceLaw, SpatialCoulombFriction, MBSIMNS"SpatialCoulombFriction")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionForceLaw, PlanarCoulombFriction, MBSIMNS"PlanarCoulombFriction")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionForceLaw, RegularizedPlanarFriction, MBSIMNS"RegularizedPlanarFriction")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionForceLaw, RegularizedSpatialFriction, MBSIMNS"RegularizedSpatialFriction")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionImpactLaw, SpatialCoulombImpact, MBSIMNS"SpatialCoulombImpact")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(FrictionImpactLaw, PlanarCoulombImpact, MBSIMNS"PlanarCoulombImpact")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, CircleHollow, MBSIMNS"CircleHollow")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, CircleSolid, MBSIMNS"CircleSolid")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Frustum, MBSIMNS"Frustum")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Line, MBSIMNS"Line")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, LineSegment, MBSIMNS"LineSegment")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Plane, MBSIMNS"Plane")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, PlaneWithFrustum, MBSIMNS"PlaneWithFrustum")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Point, MBSIMNS"Point")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Sphere, MBSIMNS"Sphere")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Element, Contour1sAnalytical, MBSIMNS"Contour1sAnalytical")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantJacobian, MBSIMNS"ConstantJacobian")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantFunction1<double COMMA double>, MBSIMNS"ConstantFunction1_SS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, Function1_SS_from_VS, MBSIMNS"Function1_SS_from_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, Polynom1_SS, MBSIMNS"Polynom1_SS")
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<double COMMA double>, MBSIMNS"SymbolicFunction1_SS")
#endif
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantFunction1<Vec COMMA double>, MBSIMNS"ConstantFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, PPolynom<Ref COMMA Ref>, MBSIMNS"PiecewisePolynom1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, QuadraticFunction1_VS<Ref>, MBSIMNS"QuadraticFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SinusFunction1_VS<Ref>, MBSIMNS"SinusFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, PositiveSinusFunction1_VS, MBSIMNS"PositiveSinusFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, StepFunction1_VS, MBSIMNS"StepFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TabularFunction1_VS<Ref COMMA Ref>, MBSIMNS"TabularFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, PeriodicTabularFunction1_VS, MBSIMNS"PeriodicTabularFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SummationFunction1_VS, MBSIMNS"SummationFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, Function1_VS_from_SS<Ref>, MBSIMNS"Function1_VS_from_SS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantFunction1<VecV COMMA double>, MBSIMNS"ConstantFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, QuadraticFunction1_VS<Var>, MBSIMNS"QuadraticFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SinusFunction1_VS<Var>, MBSIMNS"SinusFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TabularFunction1_VS<Var COMMA Var>, MBSIMNS"TabularFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, PPolynom<Var COMMA Var>, MBSIMNS"PiecewisePolynom1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, Function1_VS_from_SS<Var>, MBSIMNS"Function1_VS_from_SS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantFunction1<Vec3 COMMA double>, MBSIMNS"ConstantFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, QuadraticFunction1_VS<Fixed<3> >, MBSIMNS"QuadraticFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SinusFunction1_VS<Fixed<3> >, MBSIMNS"SinusFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TabularFunction1_VS<Var COMMA Fixed<3> >, MBSIMNS"TabularFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, PPolynom<Var COMMA Fixed<3> >, MBSIMNS"PiecewisePolynom1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, Function1_VS_from_SS<Fixed<3> >, MBSIMNS"Function1_VS_from_SS")
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
// creOBJECTFACTORY_ate some common Vec combination for SymbolicFunction1_VS
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec  COMMA double>, MBSIMNS"SymbolicFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<VecV COMMA double>, MBSIMNS"SymbolicFunction1_VS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec3 COMMA double>, MBSIMNS"SymbolicFunction1_VS")
#endif
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantFunction2<double COMMA double COMMA double>, MBSIMNS"ConstantFunction2_SSS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, TabularFunction2_SSS, MBSIMNS"TabularFunction2_SSS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearSpringDamperForce, MBSIMNS"LinearSpringDamperForce")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, NonlinearSpringDamperForce, MBSIMNS"NonlinearSpringDamperForce")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearRegularizedUnilateralConstraint, MBSIMNS"LinearRegularizedUnilateralConstraint")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearRegularizedBilateralConstraint, MBSIMNS"LinearRegularizedBilateralConstraint")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearRegularizedCoulombFriction, MBSIMNS"LinearRegularizedCoulombFriction")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, LinearRegularizedStribeckFriction, MBSIMNS"LinearRegularizedStribeckFriction")
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
// creOBJECTFACTORY_ate some common Vec combination for SymbolicFunction2_VVS
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec  COMMA Vec  COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec  COMMA VecV COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec  COMMA Vec3 COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<VecV COMMA Vec  COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<VecV COMMA VecV COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<VecV COMMA Vec3 COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec3 COMMA Vec  COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec3 COMMA VecV COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<Vec3 COMMA Vec3 COMMA double>, MBSIMNS"SymbolicFunction2_VVS")
#endif
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
// creOBJECTFACTORY_ate some common Vec combination for SymbolicFunction1_VV
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec  COMMA Vec >, MBSIMNS"SymbolicFunction1_VV")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec  COMMA VecV>, MBSIMNS"SymbolicFunction1_VV")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec  COMMA Vec3>, MBSIMNS"SymbolicFunction1_VV")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<VecV COMMA Vec >, MBSIMNS"SymbolicFunction1_VV")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<VecV COMMA VecV>, MBSIMNS"SymbolicFunction1_VV")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<VecV COMMA Vec3>, MBSIMNS"SymbolicFunction1_VV")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec3 COMMA Vec >, MBSIMNS"SymbolicFunction1_VV")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec3 COMMA VecV>, MBSIMNS"SymbolicFunction1_VV")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<Vec3 COMMA Vec3>, MBSIMNS"SymbolicFunction1_VV")
#endif
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
// creOBJECTFACTORY_ate some common Vec combination for SymbolicFunction1_SV
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<double COMMA Vec >, MBSIMNS"SymbolicFunction1_SV")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<double COMMA VecV>, MBSIMNS"SymbolicFunction1_SV")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction1<double COMMA Vec3>, MBSIMNS"SymbolicFunction1_SV")
#endif
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
// creOBJECTFACTORY_ate some common Vec combination for SymbolicFunction2_SSS
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, SymbolicFunction2<double  COMMA double  COMMA double>, MBSIMNS"SymbolicFunction2_SSS")
#endif
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, FlexibilityInfluenceFunction, MBSIMNS"FlexibilityInfluenceFunction")
MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Function, ConstantInfluenceFunction, MBSIMNS"ConstantInfluenceFunction")
/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
