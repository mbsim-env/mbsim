#include "slider_crank_system.h"
#include "circlesolid_circlehollow_ehd.h"
#include "ehd_force_law.h"

#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/circle_solid.h"
#include "mbsim/contours/circle_hollow.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/environment.h"
#include "mbsim/functions/symbolic_functions.h"
#include "mbsim/functions/basic_functions.h"
#include "mbsim/functions/kinematic_functions.h"
#include <mbsim/functions/kinetic_functions.h>
// Beginning Contact
#include "mbsim/rigid_body.h"
#include "mbsim/contour.h"
#include "mbsim/constitutive_laws.h"
// End Contact
#include "mbsim/environment.h"

#include "mbsim/kinetic_excitation.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h" // ball
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/polygonpoint.h>
#include <openmbvcppinterface/arrow.h> // Contact
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace casadi;
using namespace std;
using namespace boost;

class Moment : public MBSim::Function<fmatvec::VecV(double)> {
  public:
    fmatvec::VecV operator()(const double& t) {
      double t0 = 0.00;
      double t1 = 0.1;
      double M0 = 200;
      VecV M(1);
      if (t < t0)
        M(0) = t * M0 / t0;
      else if (t < t1)
        M(0) = M0 - (t - t0) / (t1 - t0) * M0;
      else
        M(0) = 0;

      M(0) = M0;
      return M;
    }
};

SliderCrankSystem::SliderCrankSystem(const string &projectName) :
    DynamicSystemSolver(projectName) {

  Vec grav(3, INIT, 0.);
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // geometrical characteristics
  //ROD
  double rodBigRadius = 22.5e-3;
//  double rodSmallRadius = 11.25e-3;
  double length_rod = 140e-3;
  double thicknessRod = 22e-3;
  double mass_rod = 2; //0.038; // m1
  SymMat inertia_rod(3, INIT, 0.);
  inertia_rod(0, 0) = 1.; // DUMMY
  inertia_rod(1, 1) = 1.; // DUMMY
  inertia_rod(2, 2) = mass_rod / 12. * length_rod * length_rod; // J1

  //CRANK
  double width_crank = 0.5 * thicknessRod;
  double thickness_crank = 0.5 * thicknessRod;
  double length_crank = length_rod / 2.; // l1
  double mass_crank = 1; //0.038; // m1
  SymMat inertia_crank(3, INIT, 0.);
  inertia_crank(0, 0) = 1.; // DUMMY
  inertia_crank(1, 1) = 1.; // DUMMY
  inertia_crank(2, 2) = mass_crank / 12. * length_crank * length_crank; // J1

  //PISTON
  double width_piston = 2. * thicknessRod; // 2b
  double thickness_piston = thicknessRod;
  double length_piston = 2. * 0.05; // 2a
  double mass_piston = 0.076; // m3
  SymMat inertia_piston(3, INIT, 0.);
  inertia_piston(0, 0) = 1.; // DUMMY
  inertia_piston(1, 1) = 1.; // DUMMY
  inertia_piston(2, 2) = 2.7e-6; // J3

  double clearence = 1e-4;

  double innerRadius = rodBigRadius - clearence;

  /* CRANK */
  RigidBody *crank = new RigidBody("Crank");
  this->addObject(crank);
  // kinematics
  crank->setFrameOfReference(getFrameI());

  Vec3 kinematicsFrameCrankPos;
  kinematicsFrameCrankPos(0) = -length_crank / 2.;
  FixedRelativeFrame * kinematicsFrameCrank = new FixedRelativeFrame("LoadFrame", kinematicsFrameCrankPos);
  crank->addFrame(kinematicsFrameCrank);
  crank->setFrameForKinematics(kinematicsFrameCrank);
  kinematicsFrameCrank->enableOpenMBV(0.5e-1);
  crank->getFrameC()->enableOpenMBV(0.7e-1);


  crank->setMass(mass_crank);
  crank->setInertiaTensor(inertia_crank);

  // visualisation crank
#ifdef HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::Cuboid> openMBVCrank = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVCrank->setLength(length_crank, width_crank, thickness_crank);
  openMBVCrank->setDiffuseColor(0.5, 0, 0);
  openMBVCrank->setTransparency(0.5);
  crank->setOpenMBVRigidBody(openMBVCrank);
#endif

  //Add contour to crank
  Vec3 cylinderCenterPos;
  cylinderCenterPos(0) = length_crank / 2;
  FixedRelativeFrame * circleCrankRef = new FixedRelativeFrame("CylinderRef", cylinderCenterPos);
  circleCrankRef->enableOpenMBV(0.3e-1);
  crank->addFrame(circleCrankRef);

  MBSim::CircleHollow * circleCrankContour = new MBSim::CircleHollow("CircleCrank");
  crank->addContour(circleCrankContour);
  circleCrankContour->setFrameOfReference(circleCrankRef);
  circleCrankContour->setRadius(innerRadius);
  circleCrankContour->enableOpenMBV(_diffuseColor = "[0.6;0.3;0.6]", _transparency = 0.3);

  /* ROD */

  Vec3 rodInitPos;
  rodInitPos(0) = length_crank + length_rod / 2.;
  FixedRelativeFrame * rodRefEnd = new FixedRelativeFrame("RodRefEnd", rodInitPos);
  rodRefEnd->enableOpenMBV(1e-1);
  addFrame(rodRefEnd);

  RigidBody *rod = new RigidBody("rod");
  this->addObject(rod);
  rod->setMass(1);
  rod->setInertiaTensor(inertia_rod);
  rod->setTranslation(new TranslationAlongAxesXY<VecV>());
  rod->setRotation(new RotationAboutZAxis<VecV>());
  rod->setFrameOfReference(rodRefEnd);

  Vec3 rodCircleCenterDistance;
  rodCircleCenterDistance(0) = -length_rod / 2;
  FixedRelativeFrame * rodRefCrankEnd = new FixedRelativeFrame("RodRefCrankEnd", rodCircleCenterDistance);
  rodRefCrankEnd->setFrameOfReference(rod->getFrameC());
  rod->addFrame(rodRefCrankEnd);
  rodRefCrankEnd->enableOpenMBV(1e-2);

  FixedRelativeFrame * rodRefPistonEnd = new FixedRelativeFrame("RodRefPistonEnd", -rodCircleCenterDistance);
  rodRefPistonEnd->setFrameOfReference(rod->getFrameC());
  rod->addFrame(rodRefPistonEnd);
  rodRefPistonEnd->enableOpenMBV(1e-2);

  // set a initial angle for the body reference

  // add neutral contour to the rod
  CircleSolid* circleRodContour = new CircleSolid("Buchse");
  rod->addContour(circleRodContour);
  circleRodContour->setFrameOfReference(rodRefCrankEnd);
  circleRodContour->setRadius(rodBigRadius);
  circleRodContour->enableOpenMBV(_diffuseColor = "[0.2;0.3;0.6]", _transparency = 0.8);

  // visualisation crank
#ifdef HAVE_OPENMBVCPPINTERFACE
  {
    boost::shared_ptr<OpenMBV::Cuboid> openMBVRod = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
    openMBVRod->setLength(length_rod, width_crank, thickness_crank);
    openMBVRod->setDiffuseColor(0.2, 0, 0);
    openMBVRod->setTransparency(0.5);
    rod->setOpenMBVRigidBody(openMBVRod);
  }
#endif

  /* PISTON */
  RigidBody *piston = new RigidBody("Piston");
  addObject(piston);

  Vec3 pistonInitPos;
  pistonInitPos(0) = length_crank + length_rod + width_piston / 2.;
  FixedRelativeFrame * pistonRef = new FixedRelativeFrame("PistonRef", pistonInitPos);
  pistonRef->enableOpenMBV(0.5e-1);
  addFrame(pistonRef);

  // Generalized coordinates
  piston->setFrameOfReference(pistonRef);
  piston->setTranslation(new TranslationAlongXAxis<VecV>());
  // inertial properties
  piston->setMass(mass_piston);
  piston->setInertiaTensor(inertia_piston);
  // add frame to fix
  Vec3 pistonFixPos;
  pistonFixPos(0) = -width_piston / 2.;
  FixedRelativeFrame * pistonFixRef = new FixedRelativeFrame("PistonFixRef", pistonFixPos);
  pistonFixRef->enableOpenMBV(1e-2);
  pistonFixRef->setFrameOfReference(piston->getFrameC());
  piston->addFrame(pistonFixRef);

#ifdef HAVE_OPENMBVCPPINTERFACE
  // Visualization of the piston
  boost::shared_ptr<OpenMBV::Cuboid> openMBVPiston = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVPiston->setLength(width_piston, thickness_piston, length_piston);
  openMBVPiston->setDiffuseColor(1, 0, 0);
  openMBVPiston->setTransparency(0.5);
  piston->setOpenMBVRigidBody(openMBVPiston);
#endif

  // LINKS
  /* Excitation of the crank */
  if (1) {
    // Define kinematically the angle (omega is the angular velocity)
    double omega = 3.141592 * 100;
    SX t=SX::sym("t"); // symbolic variable for time
    SX angleExp = omega*t; // expression for the angle
    SXFunction angleFuncSX(t,angleExp); // SXfunction for the angle
    SymbolicFunction<double(double)> *angleFunc = new SymbolicFunction<double(double)>(angleFuncSX); // function for the angle
    crank->setRotation(new NestedFunction<RotMat3(double(double))>(new RotationAboutFixedAxis<double>("[0;0;1]"), angleFunc));
  }
  else {
    // Alternatively: define a load that is applied to the crank
    crank->setRotation(new RotationAboutZAxis<VecV>());
    crank->setInitialGeneralizedVelocity(0.);
    KineticExcitation *load = new KineticExcitation("Motor");
    addLink(load);
    load->setMomentFunction(new Moment);
    load->setMomentDirection(Mat("[0;0;1]"));
    load->connect(kinematicsFrameCrank);
  }

  // ---------- LINK-DEFINTIION
  // Contact Between rod and crank (EHD-Contact)
  Contact * contactCrankRod = new Contact("CrankRodContact");
  addLink(contactCrankRod);
  EHDForceLaw *fL = new EHDForceLaw();
  contactCrankRod->setNormalForceLaw(fL);
  contactCrankRod->enableOpenMBVContactPoints(1e-5);
  contactCrankRod->enableOpenMBVNormalForce();
  ContactKinematicsCircleSolidCircleHollowEHD *cK = new ContactKinematicsCircleSolidCircleHollowEHD();
  contactCrankRod->connect(circleCrankContour, circleRodContour, cK);

  // Fix piston to rod (bilaterally)
  Joint *joint_rod_piston = new Joint("Joint_Rod_Piston");
  addLink(joint_rod_piston);
  joint_rod_piston->setForceDirection("[1,0;0,1;0,0]");
  joint_rod_piston->setForceLaw(new BilateralConstraint());
//  joint_rod_piston->setMomentDirection("[1,0;0,1;0,0]");
//  joint_rod_piston->setMomentLaw(new BilateralConstraint());
  joint_rod_piston->connect(pistonFixRef, rodRefPistonEnd);

}

