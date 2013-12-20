#include "system.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/environment.h"
#include "mbsimFlexibleBody/contours/neutral_contour/contour_2s_neutral_linear_external_FFR.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
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
#include <openmbvcppinterface/polygonpoint.h>
#include <openmbvcppinterface/arrow.h> // Contact
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

class Moment : public Function1<fmatvec::Vec, double> {
  public:
    fmatvec::Vec operator()(const double& t, const void * = NULL) {
      double t0 = 1;
      double t1 = 1.5;
      double M0 = 50;
      Vec M(1);
      if (t < t0)
        M(0) = t * M0 / t0;
      else if (t < t1)
        M(0) = M0 - (t - t0) / (t1 - t0) * M0;
      else
        M(0) = 0;
      return M;
    }
};

System::System(const string &projectName, const std::string & inputFilesPath): DynamicSystemSolver(projectName) {
  Vec grav(3, INIT, 0.);
  grav(1) = -9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // geometrical characteristics
    //ROD
  double rodBigRadius = 22.5e-3;
//  double rodSmallRadius = 11.25e-3;
  double centerPointDistance = 140e-3;
  double thicknessRod = 22e-3;

  //CRANK
  double width_crank = 1.5 * thicknessRod;
  double thickness_crank = thicknessRod;
  double length_crank = centerPointDistance / 2.; // l1

  //PISTON
  double width_piston = 2. * thicknessRod; // 2b
  double thickness_piston = thicknessRod;
  double length_piston = 2. * 0.05; // 2a
  double mass_piston = 0.076; // m3

  double clearence = 1e-5;

  // rod
  FlexibleBodyLinearExternalFFR *rod = new FlexibleBodyLinearExternalFFR("rod", false);
  rod->readFEMData(inputFilesPath, true);
  int nf = rod->getNumberModes();

  Vec3 rodInitPos;
  rodInitPos(0) = length_crank - rodBigRadius;
  rodInitPos(2) = thicknessRod / 2;
  SqrMat3 rodInitRot(EYE);
  rodInitRot = BasicRotAIKy(M_PI_2) * BasicRotAIKx(M_PI_2);
  FixedRelativeFrame * rodRefBigEnd = new FixedRelativeFrame("RodRefBigEnd", rodInitPos, rodInitRot);
  addFrame(rodRefBigEnd);

  Vec3 rodCircleCenterDistance;
  rodCircleCenterDistance(0) = thicknessRod / 2.;
  rodCircleCenterDistance(1) = centerPointDistance;
  FixedRelativeFrame * rodRefSmallEnd = new FixedRelativeFrame("RodRefSmallEnd", rodCircleCenterDistance);
  rod->addFrame(rodRefSmallEnd);
  rodRefSmallEnd->enableOpenMBV(1e-2);

  rod->setFrameOfReference(rodRefBigEnd);
  // set a initial angle for the body reference
  Vec q0 = Vec(nf + 6, INIT, 0.0);
  Vec u0Rod = Vec(nf + 6, INIT, 0.0);

  rod->setq0(q0);
  rod->setu0(u0Rod);

  this->addObject(rod);

  // add neutral contour to the rod
  int numOfTransNodesU = 24;
  int numOfTransNodesV = 5;
  Vec startingIndex1("[625; 473; 321; 169; 17]");
  Vec startingIndex2("[637; 485; 333; 181; 29]");
  Vec startingIndex3("[636; 484; 332; 180; 28]");
  Mat transNodes(numOfTransNodesU, numOfTransNodesV, NONINIT);
  for (int j = 0; j < numOfTransNodesV; j++)
    for (int i = 0; i < 11; i++)
      transNodes(i, j) = startingIndex1(j) + i;

  for (int j = 0; j < numOfTransNodesV; j++)
    for (int i = 0; i < 12; i++)
      transNodes(11 + i, j) = startingIndex2(j) + i;

  for (int j = 0; j < numOfTransNodesV; j++)
    transNodes(23, j) = startingIndex3(j);

  cout << transNodes << endl << endl;

  int degU = 3;
  int degV = 3;
  bool openStructure = false;
  double nodeOffset = 0.;

  Contour2sNeutralLinearExternalFFR* ncc = new Contour2sNeutralLinearExternalFFR("neutralFibre", rod, transNodes, nodeOffset, degU, degV, openStructure);
  ncc->setFrameOfReference(rod->getFrameOfReference());
  ncc->setAlphaStart(Vec(2, INIT, 0));
  ncc->setAlphaEnd(Vec(2, INIT, 1));

  // bodies
  //CRANK
  RigidBody *crank = new RigidBody("Crank");
  this->addObject(crank);
  // kinematics
  crank->setFrameOfReference(getFrameI());

  Vec3 kinematicsFrameCrankPos;
  kinematicsFrameCrankPos(0) = - length_crank / 2.;
  FixedRelativeFrame * kinematicsFrameCrank = new FixedRelativeFrame("LoadFrame", kinematicsFrameCrankPos);
  crank->addFrame(kinematicsFrameCrank);
  crank->setFrameForKinematics(kinematicsFrameCrank);

  crank->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  crank->setInitialGeneralizedVelocity(0.);


  double mass_crank = 2; //0.038; // m1
  crank->setMass(mass_crank);

  SymMat inertia_crank(3, INIT, 0.);
  inertia_crank(0, 0) = 1.; // DUMMY
  inertia_crank(1, 1) = 1.; // DUMMY
  inertia_crank(2, 2) = mass_crank / 12. * length_crank * length_crank; // J1
  crank->setInertiaTensor(inertia_crank);

  KineticExcitation *load = new KineticExcitation("Motor");
  addLink(load);
  load->setMoment("[0;0;1]", new Moment);
  load->connect(kinematicsFrameCrank);

  RigidBody *piston = new RigidBody("Piston");
  this->addObject(piston);

  Vec3 pistonInitPos;
  pistonInitPos(0) = thicknessRod / 2.;
  pistonInitPos(1) = centerPointDistance;
  FixedRelativeFrame * pistonRef = new FixedRelativeFrame("PistonRef", pistonInitPos, SqrMat3(EYE), rodRefBigEnd);
  addFrame(pistonRef);

  // generalised coordinates
  piston->setFrameOfReference(pistonRef);
//  piston->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  piston->setTranslation(new LinearTranslation(Mat("[0;1;0]")));
  piston->setMass(mass_piston);

  // inertial properties

  SymMat inertia_piston(3, INIT, 0.);
  inertia_piston(0, 0) = 1.; // DUMMY
  inertia_piston(1, 1) = 1.; // DUMMY
  inertia_piston(2, 2) = 2.7e-6; // J3
  piston->setInertiaTensor(inertia_piston);

  Contact * contactCrankRod = new Contact("CrankRodContact");
  if (0) {
    contactCrankRod->setContactForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e7, 0.)));
  }
  else {
    contactCrankRod->setContactForceLaw(new UnilateralConstraint());
    contactCrankRod->setContactImpactLaw(new UnilateralNewtonImpact(0.));
  }
  contactCrankRod->enableOpenMBVContactPoints(1e-2);
  addLink(contactCrankRod);

  //Add contours to crank
  Vec3 cylinderCenterPos;
  cylinderCenterPos(0) = length_crank / 2 - rodBigRadius;
  FixedRelativeFrame * cylinderRef = new FixedRelativeFrame("CylinderRef", cylinderCenterPos);
  cylinderRef->enableOpenMBV(1e-2);
  crank->addFrame(cylinderRef);

  Vec3 pointRelPos;
  double innerRadius = rodBigRadius - clearence;
  int numberOfPoints = 10;
  for (int i = 0; i < numberOfPoints; i++) {
    string pointName = "Point" + numtostr(i);
    double phi = i * 2 * M_PI / numberOfPoints;
    pointRelPos(0) = innerRadius * cos(phi);
    pointRelPos(1) = innerRadius * sin(phi);
    FixedRelativeFrame * pointRef = new FixedRelativeFrame(pointName + "Ref", pointRelPos, SqrMat3(EYE), cylinderRef);
    crank->addFrame(pointRef);
    pointRef->enableOpenMBV(5e-4);
    MBSim::Point * pnt = new MBSim::Point("CrankCylinder" + pointName);
    pnt->setFrameOfReference(pointRef);
    crank->addContour(pnt);

    contactCrankRod->connect(pnt, ncc);
  }

  Joint *joint_rod_piston = new Joint("Joint_Rod_Piston");
  addLink(joint_rod_piston);
  joint_rod_piston->setForceDirection("[1,0,0;0,1,0;0,0,1]");
  joint_rod_piston->setForceLaw(new BilateralConstraint());
  joint_rod_piston->setImpactForceLaw(new BilateralImpact());
  joint_rod_piston->setMomentDirection("[0,0;1,0;0,1]");
  joint_rod_piston->setMomentLaw(new BilateralConstraint());
  joint_rod_piston->setImpactMomentLaw(new BilateralImpact());
  joint_rod_piston->connect(rodRefSmallEnd, piston->getFrameC());



  // visualisation
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cuboid *openMBVCrank = new OpenMBV::Cuboid();
  openMBVCrank->setLength(length_crank, width_crank, thickness_crank);
  openMBVCrank->setDiffuseColor(0.5,0,0);
  openMBVCrank->setTransparency(0.5);
  crank->setOpenMBVRigidBody(openMBVCrank);


  OpenMBV::Cuboid *openMBVPiston = new OpenMBV::Cuboid();
  openMBVPiston->setLength(length_piston, width_piston, thickness_piston);
  openMBVPiston->setDiffuseColor(1,0,0);
  openMBVPiston->setTransparency(0.5);
  piston->setOpenMBVRigidBody(openMBVPiston);
#endif


}

