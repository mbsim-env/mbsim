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
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, DynamicSystemSolver, MBSIMNS"DynamicSystemSolver")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Group, MBSIMNS"Group")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, RigidBody, MBSIMNS"RigidBody")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, JointConstraint, MBSIMNS"JointConstraint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, GearConstraint, MBSIMNS"GearConstraint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, KinematicConstraint, MBSIMNS"KinematicConstraint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, LinearTranslation, MBSIMNS"LinearTranslation")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, TranslationInXDirection, MBSIMNS"TranslationInXDirection")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, TranslationInYDirection, MBSIMNS"TranslationInYDirection")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, TranslationInZDirection, MBSIMNS"TranslationInZDirection")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, TranslationInXYDirection, MBSIMNS"TranslationInXYDirection")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, TranslationInXZDirection, MBSIMNS"TranslationInXZDirection")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, TranslationInYZDirection, MBSIMNS"TranslationInYZDirection")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, TranslationInXYZDirection, MBSIMNS"TranslationInXYZDirection")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, TimeDependentTranslation, MBSIMNS"TimeDependentTranslation")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, GeneralTranslation, MBSIMNS"GeneralTranslation")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, StateDependentTranslation, MBSIMNS"StateDependentTranslation")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, RotationAboutFixedAxis, MBSIMNS"RotationAboutFixedAxis")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, RotationAboutXAxis, MBSIMNS"RotationAboutXAxis")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, RotationAboutYAxis, MBSIMNS"RotationAboutYAxis")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, RotationAboutZAxis, MBSIMNS"RotationAboutZAxis")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, TimeDependentRotationAboutFixedAxis, MBSIMNS"TimeDependentRotationAboutFixedAxis")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, CardanAngles, MBSIMNS"CardanAngles")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, RotationAboutAxesXY, MBSIMNS"RotationAboutAxesXY")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, RotationAboutAxesXZ, MBSIMNS"RotationAboutAxesXZ")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, RotationAboutAxesYZ, MBSIMNS"RotationAboutAxesYZ")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, RotationAboutAxesXYZ, MBSIMNS"RotationAboutAxesXYZ")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, EulerAngles, MBSIMNS"EulerAngles")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<SqrMat3 COMMA Vec COMMA double>, TimeDependentCardanAngles, MBSIMNS"TimeDependentCardanAngles")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, KineticExcitation, MBSIMNS"KineticExcitation")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SpringDamper, MBSIMNS"SpringDamper")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Joint, MBSIMNS"Joint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, SingleContact, MBSIMNS"SingleContact")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Contact, MBSIMNS"Contact")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, ExternGeneralizedIO, MBSIMNS"ExternGeneralizedIO")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, AbsoluteKinematicsObserver, MBSIMNS"AbsoluteKinematicsObserver")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Integrator, DOPRI5Integrator, MBSIMINTNS"DOPRI5Integrator")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Integrator, RADAU5Integrator, MBSIMINTNS"RADAU5Integrator")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Integrator, LSODEIntegrator, MBSIMINTNS"LSODEIntegrator")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Integrator, LSODARIntegrator, MBSIMINTNS"LSODARIntegrator")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Integrator, TimeSteppingIntegrator, MBSIMINTNS"TimeSteppingIntegrator")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Integrator, TimeSteppingSSCIntegrator, MBSIMINTNS"TimeSteppingSSCIntegrator")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Integrator, ThetaTimeSteppingIntegrator, MBSIMINTNS"ThetaTimeSteppingIntegrator")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Integrator, EulerExplicitIntegrator, MBSIMINTNS"EulerExplicitIntegrator")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Integrator, RKSuiteIntegrator, MBSIMINTNS"RKSuiteIntegrator")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(GeneralizedForceLaw, BilateralConstraint, MBSIMNS"BilateralConstraint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(GeneralizedForceLaw, UnilateralConstraint, MBSIMNS"UnilateralConstraint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(GeneralizedForceLaw, RegularizedBilateralConstraint, MBSIMNS"RegularizedBilateralConstraint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(GeneralizedForceLaw, RegularizedUnilateralConstraint, MBSIMNS"RegularizedUnilateralConstraint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(GeneralizedForceLaw, MaxwellUnilateralConstraint, MBSIMNS"MaxwellUnilateralConstraint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(GeneralizedImpactLaw, BilateralImpact, MBSIMNS"BilateralImpact")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(GeneralizedImpactLaw, UnilateralNewtonImpact, MBSIMNS"UnilateralNewtonImpact")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(FrictionForceLaw, SpatialCoulombFriction, MBSIMNS"SpatialCoulombFriction")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(FrictionForceLaw, PlanarCoulombFriction, MBSIMNS"PlanarCoulombFriction")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(FrictionForceLaw, RegularizedPlanarFriction, MBSIMNS"RegularizedPlanarFriction")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(FrictionForceLaw, RegularizedSpatialFriction, MBSIMNS"RegularizedSpatialFriction")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(FrictionImpactLaw, SpatialCoulombImpact, MBSIMNS"SpatialCoulombImpact")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(FrictionImpactLaw, PlanarCoulombImpact, MBSIMNS"PlanarCoulombImpact")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, CircleHollow, MBSIMNS"CircleHollow")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, CircleSolid, MBSIMNS"CircleSolid")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Frustum, MBSIMNS"Frustum")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Line, MBSIMNS"Line")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, LineSegment, MBSIMNS"LineSegment")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Plane, MBSIMNS"Plane")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, PlaneWithFrustum, MBSIMNS"PlaneWithFrustum")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Point, MBSIMNS"Point")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Sphere, MBSIMNS"Sphere")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Element, Contour1sAnalytical, MBSIMNS"Contour1sAnalytical")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORYASSINGLETON(Environment, MBSimEnvironment, MBSIMNS"MBSimEnvironment")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Mat3xV COMMA Vec COMMA double>, ConstantJacobian, MBSIMNS"ConstantJacobian")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, ConstantFunction1<double COMMA double>, MBSIMNS"ConstantFunction1_SS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, Function1_SS_from_VS, MBSIMNS"Function1_SS_from_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, Polynom1_SS, MBSIMNS"Polynom1_SS")
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<double COMMA double>, SymbolicFunction1<double COMMA double>, MBSIMNS"SymbolicFunction1_SS")
#endif
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec COMMA double>, ConstantFunction1<Vec COMMA double>, MBSIMNS"ConstantFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec COMMA double>, PPolynom<Ref COMMA Ref>, MBSIMNS"PiecewisePolynom1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec COMMA double>, QuadraticFunction1_VS<Ref>, MBSIMNS"QuadraticFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec COMMA double>, SinusFunction1_VS<Ref>, MBSIMNS"SinusFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec COMMA double>, PositiveSinusFunction1_VS, MBSIMNS"PositiveSinusFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec COMMA double>, StepFunction1_VS, MBSIMNS"StepFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec COMMA double>, TabularFunction1_VS<Ref COMMA Ref>, MBSIMNS"TabularFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec COMMA double>, PeriodicTabularFunction1_VS, MBSIMNS"PeriodicTabularFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec COMMA double>, SummationFunction1_VS, MBSIMNS"SummationFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec COMMA double>, Function1_VS_from_SS<Ref>, MBSIMNS"Function1_VS_from_SS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<VecV COMMA double>, ConstantFunction1<VecV COMMA double>, MBSIMNS"ConstantFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<VecV COMMA double>, QuadraticFunction1_VS<Var>, MBSIMNS"QuadraticFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<VecV COMMA double>, SinusFunction1_VS<Var>, MBSIMNS"SinusFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<VecV COMMA double>, TabularFunction1_VS<Var COMMA Var>, MBSIMNS"TabularFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<VecV COMMA double>, PPolynom<Var COMMA Var>, MBSIMNS"PiecewisePolynom1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<VecV COMMA double>, Function1_VS_from_SS<Var>, MBSIMNS"Function1_VS_from_SS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec3 COMMA double>, ConstantFunction1<Vec3 COMMA double>, MBSIMNS"ConstantFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec3 COMMA double>, QuadraticFunction1_VS<Fixed<3> >, MBSIMNS"QuadraticFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec3 COMMA double>, SinusFunction1_VS<Fixed<3> >, MBSIMNS"SinusFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec3 COMMA double>, TabularFunction1_VS<Var COMMA Fixed<3> >, MBSIMNS"TabularFunction1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec3 COMMA double>, PPolynom<Var COMMA Fixed<3> >, MBSIMNS"PiecewisePolynom1_VS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec3 COMMA double>, Function1_VS_from_SS<Fixed<3> >, MBSIMNS"Function1_VS_from_SS")
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec3 COMMA double>, SymbolicFunction1<Vec3 COMMA double>, MBSIMNS"SymbolicFunction1_VS")
#endif
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<double COMMA double COMMA double>, ConstantFunction2<double COMMA double COMMA double>, MBSIMNS"ConstantFunction2_SSS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<double COMMA double COMMA double>, TabularFunction2_SSS, MBSIMNS"TabularFunction2_SSS")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<double COMMA double COMMA double>, LinearSpringDamperForce, MBSIMNS"LinearSpringDamperForce")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<double COMMA double COMMA double>, NonlinearSpringDamperForce, MBSIMNS"NonlinearSpringDamperForce")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<double COMMA double COMMA double>, LinearRegularizedUnilateralConstraint, MBSIMNS"LinearRegularizedUnilateralConstraint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<double COMMA double COMMA double>, LinearRegularizedBilateralConstraint, MBSIMNS"LinearRegularizedBilateralConstraint")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec COMMA Vec COMMA double>, LinearRegularizedCoulombFriction, MBSIMNS"LinearRegularizedCoulombFriction")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec COMMA Vec COMMA double>, LinearRegularizedStribeckFriction, MBSIMNS"LinearRegularizedStribeckFriction")
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<Vec3 COMMA Vec COMMA double>, SymbolicFunction2<Vec3 COMMA Vec COMMA double>, MBSIMNS"SymbolicFunction2_V3VS")
#endif
#ifdef HAVE_CASADI_SYMBOLIC_SX_SX_HPP
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function1<Vec3 COMMA Vec>, SymbolicFunction1<Vec3 COMMA Vec>, MBSIMNS"SymbolicFunction1_V3V")
#endif
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<double COMMA Vec2 COMMA Vec2>, FlexibilityInfluenceFunction, MBSIMNS"FlexibilityInfluenceFunction")
MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(Function2<double COMMA Vec2 COMMA Vec2>, ConstantInfluenceFunction, MBSIMNS"ConstantInfluenceFunction")
/*
 * DO NOT ADD ANYTHING IN THIS FILE!!! (see also the comment on top of this file)
 */
