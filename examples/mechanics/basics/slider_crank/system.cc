#include "system.h"
#include "mbsim/environment.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/line.h"
#include "mbsim/links/contact.h"
#include "mbsim/constitutive_laws/constitutive_laws.h"
#include "mbsim/functions/kinematics/kinematics.h"

#include "openmbvcppinterface/cuboid.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  // geometrical characteristics
  double width_crank = 0.05;
  double thickness_crank = 0.05;
  double length_crank = 0.153; // l1

  double width_rod = 0.05;
  double thickness_rod = 0.05;
  double length_rod = 0.306; // l2

  double width_piston = 2.*0.025; // 2b
  double thickness_piston = 0.05;
  double length_piston = 2.*0.05; // 2a

  double clearance = 0.001; // c
  
  // inertial properties
  double mass_crank = 0.038; // m1
  SymMat inertia_crank(3,INIT,0.);
  inertia_crank(0,0) = 1.; // DUMMY
  inertia_crank(1,1) = 1.; // DUMMY
  inertia_crank(2,2) = 7.4e-5; // J1

  double mass_rod = 0.038; // m2
  SymMat inertia_rod(3,INIT,0.);
  inertia_rod(0,0) = 1.; // DUMMY
  inertia_rod(1,1) = 1.; // DUMMY
  inertia_rod(2,2) = 5.9e-4; // J2

  double mass_piston = 0.076; // m3
  SymMat inertia_piston(3,INIT,0.);
  inertia_piston(0,0) = 1.; // DUMMY
  inertia_piston(1,1) = 1.; // DUMMY
  inertia_piston(2,2) = 2.7e-6; // J3

  // force elements
  Vec grav(3,INIT,0.);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  // contact parameters
  double epsN = 0.4;
  double mu = 0.01;

  // initial conditions
  double q_crank = 0.;
  double v_crank = 150.;
  double q_rod = 0.;
  double v_rod = -75.;
  double q_piston = 0.;
  double v_piston = 0.;
  //------------------------------------------------------------------------------

  // bodies
  RigidBody *crank = new RigidBody("Crank");
  crank->setFrameOfReference(getFrame("I"));
  Vec crank_KrCP1(3,INIT,0.);
  crank_KrCP1(0) = -0.5*length_crank;
  crank->addFrame(new FixedRelativeFrame("P1",crank_KrCP1,SqrMat(3,EYE)));
  crank->setFrameForKinematics(crank->getFrame("P1"));
  crank->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  Vec crank_KrCP2(3,INIT,0.);
  crank_KrCP2(0) = 0.5*length_crank;
  crank->addFrame(new FixedRelativeFrame("P2",crank_KrCP2,SqrMat(3,EYE)));
  crank->setMass(mass_crank);
  crank->setInertiaTensor(inertia_crank);
  crank->setGeneralizedInitialPosition(q_crank);
  crank->setGeneralizedInitialVelocity(v_crank);
  this->addObject(crank);	

  RigidBody *rod = new RigidBody("Rod");
  rod->setFrameOfReference(crank->getFrame("P2"));
  Vec rod_KrCP1(3,INIT,0.);
  rod_KrCP1(0) = -0.5*length_rod;
  rod->addFrame(new FixedRelativeFrame("P1",rod_KrCP1,SqrMat(3,EYE)));
  rod->setFrameForKinematics(rod->getFrame("P1"));
  rod->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  Vec rod_KrCP2(3,INIT,0.);
  rod_KrCP2(0) = 0.5*length_rod;
  rod->addFrame(new FixedRelativeFrame("P2",rod_KrCP2,SqrMat(3,EYE)));
  rod->setMass(mass_rod);
  rod->setInertiaTensor(inertia_rod);
  rod->setGeneralizedInitialPosition(q_rod);
  rod->setGeneralizedInitialVelocity(v_rod);
  this->addObject(rod);

  RigidBody *piston = new RigidBody("Piston");
  piston->setFrameOfReference(rod->getFrame("P2"));
  piston->setFrameForKinematics(piston->getFrame("C"));
  piston->setRotation(new RotationAboutFixedAxis<VecV>(Vec("[0;0;1]")));
  piston->setMass(mass_piston);
  piston->setInertiaTensor(inertia_piston);
  piston->setGeneralizedInitialPosition(q_piston);
  piston->setGeneralizedInitialVelocity(v_piston);
  this->addObject(piston);
  //------------------------------------------------------------------------------

  // contours
  // points on piston
  Point *point_piston_1 = new Point("Point_Piston_1");
  Vec piston_KrCP1(3,INIT,0.);
  piston_KrCP1(0) = -0.5*length_piston;
  piston_KrCP1(1) = 0.5*width_piston;
  piston->addFrame(new FixedRelativeFrame("P1",piston_KrCP1,SqrMat(3,EYE)));
  point_piston_1->setFrameOfReference(piston->getFrame("P1"));
  piston->addContour(point_piston_1);

  Point *point_piston_2 = new Point("Point_Piston_2");
  Vec piston_KrCP2(3,INIT,0.);
  piston_KrCP2(0) = 0.5*length_piston;
  piston_KrCP2(1) = 0.5*width_piston;
  piston->addFrame(new FixedRelativeFrame("P2",piston_KrCP2,SqrMat(3,EYE)));
  point_piston_2->setFrameOfReference(piston->getFrame("P2"));
  piston->addContour(point_piston_2);

  Point *point_piston_3 = new Point("Point_Piston_3");
  Vec piston_KrCP3(3,INIT,0.);
  piston_KrCP3(0) = -0.5*length_piston;
  piston_KrCP3(1) = -0.5*width_piston;
  piston->addFrame(new FixedRelativeFrame("P3",piston_KrCP3,SqrMat(3,EYE)));
  point_piston_3->setFrameOfReference(piston->getFrame("P3"));
  piston->addContour(point_piston_3);

  Point *point_piston_4 = new Point("Point_Piston_4");
  Vec piston_KrCP4(3,INIT,0.);
  piston_KrCP4(0) = 0.5*length_piston;
  piston_KrCP4(1) = -0.5*width_piston;
  piston->addFrame(new FixedRelativeFrame("P4",piston_KrCP4,SqrMat(3,EYE)));
  point_piston_4->setFrameOfReference(piston->getFrame("P4"));
  piston->addContour(point_piston_4);

  // bottom plane
  Line *bottom = new Line("Bottom");
  Vec bottom_IrIB(3,INIT,0.);
  bottom_IrIB(1) = -clearance-0.5*width_piston;
  SqrMat bottom_A(3,INIT,0.);
  bottom_A(0,1) = -1; bottom_A(1,0) = 1; bottom_A(2,2) = 1;
  addFrame(new FixedRelativeFrame("B",bottom_IrIB,bottom_A));
  bottom->setFrameOfReference(getFrame("B"));
  bottom->enableOpenMBV();
  this->addContour(bottom);

  // top plane
  Line *top = new Line("Top");
  Vec top_IrIT(3,INIT,0.);
  top_IrIT(1) = clearance+0.5*width_piston;
  SqrMat top_A(3,INIT,0.);
  top_A(0,1) = 1; top_A(1,0) = -1; top_A(2,2) = 1;
  addFrame(new FixedRelativeFrame("T",top_IrIT,top_A));
  top->setFrameOfReference(getFrame("T"));
  top->enableOpenMBV();
  this->addContour(top);
  //---------------------------------------------------------------------------

  // contacts
  Contact *contact_point_piston_1_top = new Contact("Contact_Point_Piston_1_Top");
  contact_point_piston_1_top->connect(point_piston_1,top);
  contact_point_piston_1_top->setTangentialForceLaw(new PlanarCoulombFriction(mu)); // mu1
  contact_point_piston_1_top->setTangentialImpactLaw(new PlanarCoulombImpact(mu));
  contact_point_piston_1_top->setNormalForceLaw(new UnilateralConstraint());
  contact_point_piston_1_top->setNormalImpactLaw(new UnilateralNewtonImpact(epsN)); // epsN1
  contact_point_piston_1_top->enableOpenMBVContactPoints();
  this->addLink(contact_point_piston_1_top);

  Contact *contact_point_piston_2_top = new Contact("Contact_Point_Piston_2_Top");
  contact_point_piston_2_top->connect(point_piston_2,top);
  contact_point_piston_2_top->setTangentialForceLaw(new PlanarCoulombFriction(0.01)); // mu2
  contact_point_piston_2_top->setTangentialImpactLaw(new PlanarCoulombImpact(0.01));
  contact_point_piston_2_top->setNormalForceLaw(new UnilateralConstraint());
  contact_point_piston_2_top->setNormalImpactLaw(new UnilateralNewtonImpact(0.4)); // epsN2
  contact_point_piston_2_top->enableOpenMBVContactPoints();
  this->addLink(contact_point_piston_2_top);

  Contact *contact_point_piston_3_bottom = new Contact("Contact_Point_Piston_3_Bottom");
  contact_point_piston_3_bottom->connect(point_piston_3,bottom);
  contact_point_piston_3_bottom->setTangentialForceLaw(new PlanarCoulombFriction(0.01)); // mu3
  contact_point_piston_3_bottom->setTangentialImpactLaw(new PlanarCoulombImpact(0.01));
  contact_point_piston_3_bottom->setNormalForceLaw(new UnilateralConstraint());
  contact_point_piston_3_bottom->setNormalImpactLaw(new UnilateralNewtonImpact(0.4)); // epsN3
  contact_point_piston_3_bottom->enableOpenMBVContactPoints();
  this->addLink(contact_point_piston_3_bottom);
  
  Contact *contact_point_piston_4_bottom = new Contact("Contact_Point_Piston_4_Bottom");
  contact_point_piston_4_bottom->connect(point_piston_4,bottom);
  contact_point_piston_4_bottom->setTangentialForceLaw(new PlanarCoulombFriction(0.01)); // mu4
  contact_point_piston_4_bottom->setTangentialImpactLaw(new PlanarCoulombImpact(0.01));
  contact_point_piston_4_bottom->setNormalForceLaw(new UnilateralConstraint());
  contact_point_piston_4_bottom->setNormalImpactLaw(new UnilateralNewtonImpact(0.4)); // epsN4
  contact_point_piston_4_bottom->enableOpenMBVContactPoints();
  this->addLink(contact_point_piston_4_bottom);
  //---------------------------------------------------------------------------

  // visualisation
  std::shared_ptr<OpenMBV::Cuboid> openMBVCrank = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVCrank->setLength(length_crank,width_crank,thickness_crank);
  openMBVCrank->setDiffuseColor(180./360.,1,1);
  crank->setOpenMBVRigidBody(openMBVCrank);

  std::shared_ptr<OpenMBV::Cuboid> openMBVRod = OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVRod->setLength(length_rod,width_rod,thickness_rod);
  openMBVRod->setDiffuseColor(180./360.,1,1);
  rod->setOpenMBVRigidBody(openMBVRod);

  std::shared_ptr<OpenMBV::Cuboid> openMBVPiston=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVPiston->setLength(length_piston,width_piston,thickness_piston);
  openMBVPiston->setDiffuseColor(240./360.,1,1);
  piston->setOpenMBVRigidBody(openMBVPiston);

}

