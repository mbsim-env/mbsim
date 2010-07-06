#include "system.h"

#include "mbsimFlexibleBody/flexible_body/flexible_body_2s_13_disk.h"
#include "mbsim/rigid_body.h"
#include "mbsim/joint.h"
#include "mbsim/contact.h"
#include "mbsim/contours/circle.h"
#include "mbsimFlexibleBody/contact_kinematics/circle_nurbsdisk2s.h" 
#include "mbsim/constitutive_laws.h"
#include "mbsim/environment.h"
#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/frustum.h>
#endif

using namespace MBSimFlexibleBody;
using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  /* preliminaries */
  Vec WrOK(3,INIT,0);
  Vec KrKS(3,INIT,0);
  Vec KrKP(3,INIT,0);
  SymMat Theta(3,INIT,0);
  SqrMat AWK(3,EYE);

  /* gravity */
  Vec grav(3,INIT,0.);
  grav(1) = -10.;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  /* axis */
  RigidBody *axis = new RigidBody("Axis");

  double l_axis = 0.1;
  double r_axis = 0.025;
  double m_axis = 1.;
  double Jyy_axis = 0.5 * m_axis * r_axis * r_axis;
  Theta(1,1) = Jyy_axis;
  
  Vec JR_axis(3,INIT,0.);
  JR_axis(1) = 1.;

  KrKS(1) = 0.5*l_axis;

  axis->addFrame("R",-KrKS,SqrMat(3,EYE));
  axis->setFrameOfReference(this->getFrame("I"));
  axis->setFrameForKinematics(axis->getFrame("R"));
  axis->setRotation(new RotationAboutFixedAxis(JR_axis));
  axis->setMass(m_axis);
  axis->setInertiaTensor(Theta);
  axis->setInitialGeneralizedVelocity(Vec(1,INIT,2.));
  this->addObject(axis);

  /* pole */
  RigidBody *pole = new RigidBody("Pole");

  double l_pole = 0.25;
  double r_pole = 0.025;
  double m_pole = 1.;
  double delta = 20 / 180. * M_PI; 
  double Jzz_pole = 1./12. * m_pole *(3*r_pole*r_pole + l_pole*l_pole);
  Theta(2,2) = Jzz_pole;
  
  Vec JR_pole(3,INIT,0.);
  JR_pole(2) = 1.;

  AWK(0,0) = cos(delta); AWK(0,1) = -sin(delta);
  AWK(1,0) = sin(delta); AWK(1,1) = cos(delta);
  WrOK(1) = l_axis;
  KrKS(0) = 0.5*l_pole; KrKS(1) = 0.;

  axis->addFrame("P",WrOK,AWK,axis->getFrame("R"));
  pole->addFrame("R",-KrKS,SqrMat(3,EYE));
  pole->setFrameOfReference(axis->getFrame("P"));
  pole->setFrameForKinematics(pole->getFrame("R"));
  pole->setRotation(new RotationAboutFixedAxis(JR_pole));
  pole->setMass(m_pole);
  pole->setInertiaTensor(Theta);
  this->addObject(pole);

  /* muller */
  RigidBody* muller = new RigidBody("Muller");

  double l_muller = 0.025;
  double r_muller = 0.1;
  double m_muller = 1.;
  double Jxx_muller = 0.5 * m_muller * r_muller * r_muller;
  Theta(0,0) = Jxx_muller;
  
  Vec JR_muller(3,INIT,0.);
  JR_muller(0) = 1.;

  WrOK(0) = l_pole; WrOK(1) = 0.;
  KrKS(0) = 0.5*l_muller;

  pole->addFrame("P",WrOK,SqrMat(3,EYE),pole->getFrame("R"));
  muller->addFrame("R",-KrKS,SqrMat(3,EYE));
  muller->setFrameOfReference(pole->getFrame("P"));
  muller->setFrameForKinematics(muller->getFrame("R"));
  muller->setRotation(new RotationAboutFixedAxis(JR_muller));	
  muller->setMass(m_muller);
  muller->setInertiaTensor(Theta);
  this->addObject(muller);

  /* contour of muller */
  Circle* disk = new Circle("Disk");
  AWK = SqrMat(3,EYE);
  AWK(0,0) = cos(M_PI*0.5); AWK(0,2) = sin(M_PI*0.5);
  AWK(2,0) = -sin(M_PI*0.5); AWK(2,2) = cos(M_PI*0.5);
  disk->setOutCont(true);
  disk->setRadius(r_muller);
  disk->enableOpenMBV();
  muller->addContour(disk,Vec(3,INIT,0.),AWK,muller->getFrame("C"));

  /* disk */
  // body
  FlexibleBody2s13Disk *nurbsdisk = new FlexibleBody2s13Disk("NurbsDisk");
  double E = 5.e10; // Young's modulus  
  double rho = 2.7e3; // density
  double nu = 0.3; // poisson ratio
  double rI = r_axis; // inner radius
  double rO = (l_pole+r_muller)*cos(delta) + 0.05; // outer radius

  Vec d(3,INIT,0.); 
  d(0) =l_axis; // thickness at r==0
  d(1) = 2.*tan(delta);
  d(2) = 0.; // linear

  int nr = 3; // radial number of elements
  int nj = 6; // azimuthal number of elements

  SqrMat AWK_disk(3,EYE);
  AWK_disk(1,1) = cos(M_PI/2); AWK_disk(1,2) = sin(M_PI/2);
  AWK_disk(2,1) = -sin(M_PI/2); AWK_disk(2,2) = cos(M_PI/2);

  WrOK = Vec(3,INIT,0);
  WrOK(1) = - d(0)/2 + l_axis - r_muller/cos(delta);
  this->addFrame("D",WrOK,AWK_disk);

  nurbsdisk->setEModul(E);
  nurbsdisk->setDensity(rho);
  nurbsdisk->setPoissonRatio(nu);
  nurbsdisk->setRadius(rI,rO);
  nurbsdisk->setThickness(d);
  nurbsdisk->setFrameOfReference(this->getFrame("D")); // location of disk
  nurbsdisk->setLockType(innerring); // inner ring has no elastic dof
  nurbsdisk->setReferenceInertia(1.,SymMat(3,EYE)); // inertia of the reference frame
  nurbsdisk->setNumberElements(nr,nj);

  Vec q0 = Vec(2+nr*nj*3,INIT,0.); // initial position
  nurbsdisk->setq0(q0);
  Vec u0 = Vec(2+nr*nj*3,INIT,0.); // initial velocity
  nurbsdisk->setu0(u0);
  this->addObject(nurbsdisk);

  // bearing
  Joint *joint = new Joint("Clamping");
  joint->setForceDirection(Mat("[0;0;1]"));
  //joint->setMomentDirection(Mat("[0;0;1]")); // TODO
  joint->connect(nurbsdisk->getFrame("COG"),this->getFrame("I"));
  joint->setForceLaw(new BilateralConstraint());
  //joint->setMomentLaw(new BilateralConstraint()); // TODO
  joint->setImpactForceLaw(new BilateralImpact());
  //joint->setImpactMomentLaw(new BilateralImpact()); // TODO
  this->addLink(joint);

  /* contact */
  Contact *contact = new Contact("Contact");
  contact->connect(nurbsdisk->getContour("SurfaceContour"),muller->getContour("Disk"));
  contact->setContactForceLaw(new UnilateralConstraint);
  contact->setContactImpactLaw(new UnilateralNewtonImpact);
  contact->setFrictionForceLaw(new SpatialCoulombFriction(0.2));
  contact->setFrictionImpactLaw(new SpatialCoulombImpact(0.2));
  contact->enableOpenMBVContactPoints(); // shows the frames in openmbv
  this->addLink(contact);

  /* OpenMBV */
#ifdef HAVE_OPENMBVCPPINTERFACE
  /* axis */
  OpenMBV::Frustum *obj1 = new OpenMBV::Frustum;
  obj1->setBaseRadius(r_axis);
  obj1->setTopRadius(r_axis);
  obj1->setHeight(l_axis);
  obj1->setInitialRotation(-M_PI/2,0.,0.);
  obj1->setInitialTranslation(0.,l_axis*0.5,0.);
  obj1->setStaticColor(0.75);
  axis->setOpenMBVRigidBody(obj1);

  /* pole */
  OpenMBV::Frustum *obj2 = new OpenMBV::Frustum;
  obj2->setBaseRadius(r_pole);
  obj2->setTopRadius(r_pole);
  obj2->setHeight(l_pole);
  obj2->setInitialRotation(0.,M_PI/2,0.);
  obj2->setInitialTranslation(l_pole*0.5,0.,0.);
  pole->setOpenMBVRigidBody(obj2);

#endif
} 

void System::init(InitStage stage) {
  if(stage==preInit) {
    DynamicSystemSolver::init(stage);
    static_cast<ContactKinematicsCircleNurbsDisk2s*>(static_cast<Contact*>(this->getLink("Contact"))->getContactKinematics())->setLocalSearch(true);
  }
  else DynamicSystemSolver::init(stage);
}

