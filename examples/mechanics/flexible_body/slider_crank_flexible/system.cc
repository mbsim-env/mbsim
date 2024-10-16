#include "system.h"
#include "mbsim/links/joint.h"
#include "mbsim/links/contact.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/plane.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/environment.h"
#include "mbsimFlexibleBody/contours/nc/2s_neutral_linear_external_FFR.h"
#include "mbsimFlexibleBody/contours/flexible_band.h"
#include <mbsim/functions/kinematics/kinematics.h>
#include <mbsim/functions/kinetics/kinetics.h>
// Beginning Contact
#include "mbsim/objects/rigid_body.h"
#include "mbsim/contours/contour.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
// End Contact
#include "mbsim/environment.h"
#include "mbsim/contact_kinematics/point_spatialcontour.h"

#include "mbsim/links/kinetic_excitation.h"
#include "mbsim/observers/contact_observer.h"

#include <openmbvcppinterface/spineextrusion.h>
#include "openmbvcppinterface/sphere.h" // ball
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/frustum.h>
#include <openmbvcppinterface/polygonpoint.h>
#include <openmbvcppinterface/arrow.h> // Contact

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

class Moment : public MBSim::Function<fmatvec::VecV(double)> {
  public:
    std::pair<int, int> getRetSize() const { return std::make_pair(1,1); }
    fmatvec::VecV operator()(const double& t) {
      double t0 = 0.05;
      double t1 = 0.1;
      double M0 = 20;
      VecV M(1);
      if (t < t0)
        M(0) = t * M0 / t0;
      else if (t < t1)
        M(0) = M0 - (t - t0) / (t1 - t0) * M0;
      else
        M(0) = 0;
      return M;
    }
};

FlexibleSliderCrankSystem::FlexibleSliderCrankSystem(const string &projectName) :
    DynamicSystemSolver(projectName) {

  Vec grav(3, INIT, 0.);
  grav(1) = -9.81;
  getMBSimEnvironment()->setAccelerationOfGravity(grav);

  // geometrical characteristics
  //ROD
  double rodBigRadius = 22.5e-3;
//  double rodSmallRadius = 11.25e-3;
  double centerPointDistance = 140e-3;
  double thicknessRod = 22e-3;

  //CRANK
  double width_crank = 0.5 * thicknessRod;
  double thickness_crank = 0.5 * thicknessRod;
  double length_crank = centerPointDistance / 2.; // l1

  //PISTON
  double width_piston = 2. * thicknessRod; // 2b
  double thickness_piston = thicknessRod;
  double length_piston = 2. * 0.05; // 2a
  double mass_piston = 0.076; // m3

  double clearence = 1e-5;

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
  crank->setRotation(new RotationAboutZAxis<VecV>());
  crank->setGeneralizedInitialVelocity(0.);

  double mass_crank = 2; //0.038; // m1
  crank->setMass(mass_crank);

  SymMat inertia_crank(3, INIT, 0.);
  inertia_crank(0, 0) = 1.; // DUMMY
  inertia_crank(1, 1) = 1.; // DUMMY
  inertia_crank(2, 2) = mass_crank / 12. * length_crank * length_crank; // J1
  crank->setInertiaTensor(inertia_crank);

  /* rod */
  FlexibleBodyLinearExternalFFR *rod = new FlexibleBodyLinearExternalFFR("rod");
  rod->readFEMData("rod", true);

  const int nf = rod->getNumberModes();

  Vec3 rodInitPos;
  rodInitPos(0) = length_crank;
  rodInitPos(2) = thicknessRod / 2;
  SqrMat3 rodInitRot(EYE);
  rodInitRot = BasicRotAIKy(M_PI_2) * BasicRotAIKx(M_PI_2);
  FixedRelativeFrame * rodRefBigEnd = new FixedRelativeFrame("RodRefBigEnd", rodInitPos, rodInitRot);
  rodRefBigEnd->enableOpenMBV(1e-1);
  addFrame(rodRefBigEnd);

  Vec3 rodCircleCenterDistance;
  rodCircleCenterDistance(0) = thicknessRod / 2.;
  rodCircleCenterDistance(1) = centerPointDistance;
  FixedRelativeFrame * rodRefSmallEnd = new FixedRelativeFrame("RodRefSmallEnd", rodCircleCenterDistance, Cardan2AIK(-M_PI/2,-M_PI/2,0));
  rodRefSmallEnd->setFrameOfReference(rod->getFloatingFrameOfReference());
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
  Contour2sNeutralLinearExternalFFR* ncc = new Contour2sNeutralLinearExternalFFR("InnerRing");
  rod->addContour(ncc);
  vector<double> nodes(11);
  for(size_t i=0; i<11; i++)
    nodes[i] = i/10.;
  ncc->setEtaNodes(nodes);
  ncc->setXiNodes(nodes);
  ncc->readTransNodes("rod/InnerRing.txt");
  ncc->setOpenStructure(false);

  if (1) {
    // enable plotting of all nodal frames
    rod->enableFramePlot(1e-3);
  }

  if (0) {
    // Enable plotting of the nodes on the rod that are in contact
    MatVI transNodes(ncc->getTransNodes());
    for (int i = 0; i < transNodes.rows(); i++)
      for (int j = 0; j < transNodes.cols(); j++) {
        int nodeNumber = transNodes(i, j);
        NodeFrame * refFrame = new NodeFrame("ContourFrame" + toString(nodeNumber), nodeNumber);
        rod->addFrame(refFrame);
        refFrame->enableOpenMBV(1e-3);
      }
  }

  /* PISTON */
  RigidBody *piston = new RigidBody("Piston");
  addObject(piston);

  Vec3 pistonInitPos;
  pistonInitPos(0) = rodInitPos(0) + centerPointDistance;
  FixedRelativeFrame * pistonRef = new FixedRelativeFrame("PistonRef", pistonInitPos);
  pistonRef->enableOpenMBV(0.5e-1);
  addFrame(pistonRef);

  // Generalized coordinates
  piston->setFrameOfReference(pistonRef);
//  piston->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  piston->setTranslation(new TranslationAlongXAxis<VecV>());
  piston->setMass(mass_piston);

  // inertial properties

  SymMat inertia_piston(3, INIT, 0.);
  inertia_piston(0, 0) = 1.; // DUMMY
  inertia_piston(1, 1) = 1.; // DUMMY
  inertia_piston(2, 2) = 2.7e-6; // J3
  piston->setInertiaTensor(inertia_piston);

  std::shared_ptr<OpenMBV::Cuboid> openMBVPiston = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVPiston->setLength(width_piston, thickness_piston, length_piston);
  openMBVPiston->setDiffuseColor(1, 0, 0);
  openMBVPiston->setTransparency(0.5);
  piston->setOpenMBVRigidBody(openMBVPiston);

  // LINKS
  /* Excitation of the crank */
  KineticExcitation *load = new KineticExcitation("Motor");
  addLink(load);
  load->setMomentFunction(new Moment);
  load->setMomentDirection(Mat("[0;0;1]"));
  load->connect(kinematicsFrameCrank);

  //Add contact to crank
  Vec3 cylinderCenterPos;
  cylinderCenterPos(0) = length_crank / 2;
  FixedRelativeFrame * cylinderRef = new FixedRelativeFrame("CylinderRef", cylinderCenterPos);
  cylinderRef->enableOpenMBV(0.3e-1);
  crank->addFrame(cylinderRef);

  MBSim::RigidBody * cylinderContour = new MBSim::RigidBody("CylinderContour");
  addObject(cylinderContour);
  cylinderContour->setFrameOfReference(cylinderRef);

  std::shared_ptr<OpenMBV::Frustum> frustum = OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  frustum->setHeight(thicknessRod);
  frustum->setBaseRadius(innerRadius);
  frustum->setTopRadius(innerRadius);
  frustum->setInitialTranslation(0, 0, thicknessRod / 2.);
  frustum->setTransparency(0.7);
  frustum->setDiffuseColor(0.36, 1, 0.4);
  cylinderContour->setOpenMBVRigidBody(frustum);

  Vec3 pointRelPos;
  int numberOfPoints = 10;
  for (int i = 0; i < numberOfPoints; i++) {
    string pointName = "Point" + toString(i);
    double phi = i * 2 * M_PI / numberOfPoints;
    pointRelPos(0) = innerRadius * cos(phi);
    pointRelPos(1) = innerRadius * sin(phi);
    FixedRelativeFrame * pointRef = new FixedRelativeFrame(pointName + "Ref", pointRelPos, SqrMat3(EYE), cylinderRef);
    crank->addFrame(pointRef);
    MBSim::Point * pnt = new MBSim::Point("CrankCylinder" + pointName);
    pnt->enableOpenMBV(_pointSize=4,_diffuseColor=Vec3("[1;0;0]"));
    pnt->setFrameOfReference(pointRef);
    crank->addContour(pnt);
    Contact * contactCrankRod = new Contact("CrankRodContact_"+to_string(i));
    addLink(contactCrankRod);
    if (0) {
      contactCrankRod->setNormalForceLaw(new RegularizedUnilateralConstraint(new LinearRegularizedUnilateralConstraint(1e7, 0.)));
    }
    else {
      contactCrankRod->setNormalForceLaw(new UnilateralConstraint());
      contactCrankRod->setNormalImpactLaw(new UnilateralNewtonImpact(0.));
    }
    contactCrankRod->setInitialGlobalSearch(true);
    contactCrankRod->connect(pnt, ncc);
    contactCrankRod->setContactKinematics(new ContactKinematicsPointSpatialContour);

    ContactObserver *observer = new ContactObserver(contactCrankRod->getName()+"_Observer_"+to_string(i));
    addObserver(observer);
    observer->setContact(contactCrankRod);
    observer->enableOpenMBVContactPoints(1e-5);
    observer->enableOpenMBVNormalForce(_colorRepresentation=OpenMBVArrow::absoluteValue);
  }

  Joint *joint_rod_piston = new Joint("Joint_Rod_Piston");
  addLink(joint_rod_piston);
  joint_rod_piston->setForceDirection("[1,0,0;0,1,0;0,0,1]");
  joint_rod_piston->setForceLaw(new BilateralConstraint());
  joint_rod_piston->setMomentDirection("[1,0;0,1;0,0]");
  joint_rod_piston->setMomentLaw(new BilateralConstraint());
  joint_rod_piston->connect(piston->getFrameC(), rodRefSmallEnd);

  // The Rod could also just be fixed at two relative frames
  if (0) {
    Joint *joint_crank_rod = new Joint("Joint_Crank_Rod");
    addLink(joint_crank_rod);
    joint_crank_rod->setForceDirection("[1,0,0;0,1,0;0,0,1]");
    joint_crank_rod->setForceLaw(new BilateralConstraint());
    joint_crank_rod->setMomentDirection("[0,0;1,0;0,1]");
    joint_crank_rod->setMomentLaw(new BilateralConstraint());
    joint_crank_rod->connect(rod->getFloatingFrameOfReference(), cylinderRef);
  }

  // visualisation
  std::shared_ptr<OpenMBV::Cuboid> openMBVCrank = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVCrank->setLength(length_crank, width_crank, thickness_crank);
  openMBVCrank->setDiffuseColor(0.5, 0, 0);
  openMBVCrank->setTransparency(0.5);
  crank->setOpenMBVRigidBody(openMBVCrank);

  setPlotFeatureRecursive(generalizedPosition, true);
  setPlotFeatureRecursive(generalizedVelocity, true);
  setPlotFeatureRecursive(generalizedRelativePosition, true);
  setPlotFeatureRecursive(generalizedRelativeVelocity, true);
  setPlotFeatureRecursive(generalizedForce, true);
}

