#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/spring_damper.h"
#include "mbsim/environment.h"
#include "mbsim/contours/sphere.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"

#include "mbsim/functions/basic_functions.h"
#include "mbsim/functions/symbolic_functions.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/functions/kinetic_functions.h"
#include "mbsim/functions/nested_functions.h"
#include <casadi/symbolic/sx/sx_tools.hpp>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/coilspring.h"
#include <openmbvcppinterface/cuboid.h>
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;
using namespace CasADi;

//double omg = 10.;
//double tend = 5.;

//class Omega : public Function1<double, double> {
//  public:
//    double operator()(const double& t, const void*) {
//      double ang;
//      if (t <= tend)
//        ang = omg * t / tend;
//      else
//        ang = omg;
//      return ang;
//    }
//};
//
//class AngleOverTime : public DifferentiableFunction1<double> {
//  public:
//
//    AngleOverTime() {
//      addDerivative(new Omega());
//    }
//    double operator()(const double& t, const void*) {
//      double ang;
//      if (t <= tend)
//        ang = 1. / 2. * omg * t * t / tend;
//      else
//        ang = (1. / 2. * omg) * tend + omg * (t - tend);
//      return ang;
//    }
//
//};

System::System(const string &projectName) :
    DynamicSystemSolver(projectName) {
  // acceleration of gravity
  Vec grav(3);
//  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  double length_crank = 0.2;
  double length_spring = 0.1;
  double radius_mass = length_crank / 10.;
  double mass_crank = 1; //0.038; // m1
  double mass_mass1 = 2;
  // crank
  RigidBody *crank = new RigidBody("Crank");
  this->addObject(crank);
  // kinematics
  crank->setFrameOfReference(getFrameI());
  Vec3 kinematicsFrameCrankPos;
  kinematicsFrameCrankPos(0) = -length_crank / 2.;
  FixedRelativeFrame * kinematicsFrameCrank = new FixedRelativeFrame("LoadFrame", kinematicsFrameCrankPos);
  FixedRelativeFrame * crankToSpring = new FixedRelativeFrame("crankToSpring", -kinematicsFrameCrankPos);
  crank->addFrame(kinematicsFrameCrank);
  crank->addFrame(crankToSpring);
  crank->setFrameForKinematics(kinematicsFrameCrank);
  kinematicsFrameCrank->enableOpenMBV(0.5e-1);
  crankToSpring->enableOpenMBV(0.5e-1);
  crank->getFrameC()->enableOpenMBV(0.7e-1);

  SX t=SX::sym("t");
  SX fexp2 = log(cosh(t));
  SXFunction foo2(t, fexp2);

  SymbolicFunction<double(double)> *f2 = new SymbolicFunction<double(double)>(foo2);
  crank->setRotation(new NestedFunction<RotMat3(double(double))>(new RotationAboutFixedAxis<double>("[0;1;0]"), f2));
  crank->setTranslation(new TranslationAlongAxesXYZ<VecV>());
  Joint * fix = new Joint("Fix");
  fix->connect(getFrameI(), kinematicsFrameCrank);
  fix->setForceDirection(Mat3x3(EYE));   // crank2 has to have six DOFs
  fix->setForceLaw(new BilateralConstraint);
  addLink(fix);

  crank->setMass(mass_crank);
  SymMat inertia_crank(3, INIT, 0.);
  inertia_crank(0, 0) = 1.; // DUMMY
  inertia_crank(1, 1) = 1.; // DUMMY
  inertia_crank(2, 2) = 1; // J1
  crank->setInertiaTensor(inertia_crank);

  // visualisation
#ifdef HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::Cuboid> openMBVCrank = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVCrank->setLength(length_crank, 0.05, 0.05);
  openMBVCrank->setDiffuseColor(0.5, 0, 0);
  openMBVCrank->setTransparency(0.5);
  crank->setOpenMBVRigidBody(openMBVCrank);
#endif

//  Vec3 pos;
//  pos(0) = length_spring + 0.5 * length_crank;
//  FixedRelativeFrame * refMass = new FixedRelativeFrame("Mass Frame", pos);
//  refMass->enableOpenMBV();
//  refMass->setFrameOfReference(crank->getFrameC());
//  crank->addFrame(refMass);

  // bodies
  RigidBody *mass1 = new RigidBody("Mass1");
  this->addObject(mass1);

  // attributes
  mass1->setMass(mass_mass1);
  mass1->setInertiaTensor(SymMat(3, EYE));
  mass1->setTranslation(new TranslationAlongXAxis<VecV>());
  mass1->setFrameOfReference(crankToSpring);
  mass1->setInitialGeneralizedPosition(length_spring);
//  mass1->setInitialGeneralizedVelocity(Vec("[0; 1]"));

  // spring
  SpringDamper *spring1 = new SpringDamper("Spring1");
  spring1->setForceFunction(new LinearSpringDamperForce(1000, 0, length_spring));
  spring1->connect(mass1->getFrame("C"), crankToSpring);

  // add spring to dynamical system
  this->addLink(spring1);

  // contour
  Sphere *sphere1 = new Sphere("Sphere1");
  sphere1->setRadius(radius_mass);
#ifdef HAVE_OPENMBVCPPINTERFACE
  sphere1->enableOpenMBV();
#endif
  mass1->addContour(sphere1);

  // visualisation
#ifdef HAVE_OPENMBVCPPINTERFACE
  spring1->enableOpenMBVCoilSpring(_springRadius = 0.002, _crossSectionRadius = 0.01, _numberOfCoils = 5);
#endif
}

