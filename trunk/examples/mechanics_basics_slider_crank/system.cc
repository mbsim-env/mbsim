#include "system.h"
#include "mbsim/environment.h"
#include "mbsim/rigid_body.h"
#include "mbsim/joint.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/line.h"
#include "mbsim/contact.h"
#include "mbsim/constitutive_laws.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/cuboid.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  // gravitation
  Vec grav(3,INIT,0.);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

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
  
  // bodies
  RigidBody *crank = new RigidBody("Crank");
  this->addObject(crank);	

  RigidBody *rod = new RigidBody("Rod");
  this->addObject(rod);

  RigidBody *piston = new RigidBody("Piston");
  this->addObject(piston);

  // generalised coordinates
  crank->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  rod->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  rod->setTranslation(new LinearTranslation(Mat("[1,0;0,1;0,0]")));
  piston->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
  piston->setTranslation(new LinearTranslation(Mat("[1,0;0,1;0,0]")));

  // inertial properties
  double mass_crank = 0.038; // m1
  crank->setMass(mass_crank);

  SymMat inertia_crank(3,INIT,0.);
  inertia_crank(0,0) = 1.; // DUMMY
  inertia_crank(1,1) = 1.; // DUMMY
  inertia_crank(2,2) = 7.4e-5; // J1
  crank->setInertiaTensor(inertia_crank);

  double mass_rod = 0.038; // m2
  rod->setMass(mass_rod);
  
  SymMat inertia_rod(3,INIT,0.);
  inertia_rod(0,0) = 1.; // DUMMY
  inertia_rod(1,1) = 1.; // DUMMY
  inertia_rod(2,2) = 5.9e-4; // J2
  rod->setInertiaTensor(inertia_rod);

  double mass_piston = 0.076; // m3
  piston->setMass(mass_piston);

  SymMat inertia_piston(3,INIT,0.);
  inertia_piston(0,0) = 1.; // DUMMY
  inertia_piston(1,1) = 1.; // DUMMY
  inertia_piston(2,2) = 2.7e-6; // J3
  piston->setInertiaTensor(inertia_piston);

  // kinematics 
  crank->setFrameOfReference(getFrame("I"));
  
  Vec crank_KrCP(3,INIT,0.);
  crank_KrCP(0) = -0.5*length_crank;
  crank->addFrame("P",crank_KrCP,SqrMat(3,EYE));
  crank->setFrameForKinematics(crank->getFrame("P"));
  
  rod->setFrameOfReference(getFrame("I"));
  rod->setFrameForKinematics(rod->getFrame("C"));

  Joint *joint_crank_rod = new Joint("Joint_Crank_Rod");
  addLink(joint_crank_rod);
  Vec crank_KrCS(3,INIT,0.);
  crank_KrCS(0) = 0.5*length_crank;
  crank->addFrame("S",crank_KrCS,SqrMat(3,EYE));
  Vec rod_KrCP(3,INIT,0.);
  rod_KrCP(0) = -0.5*length_rod;
  rod->addFrame("P",rod_KrCP,SqrMat(3,EYE));
  joint_crank_rod->setForceDirection("[1,0;0,1;0,0]");
  joint_crank_rod->setForceLaw( new BilateralConstraint());
  joint_crank_rod->setImpactForceLaw( new BilateralImpact());
  joint_crank_rod->connect(crank->getFrame("S"),rod->getFrame("P"));

  piston->setFrameOfReference(getFrame("I"));
  piston->setFrameForKinematics(piston->getFrame("C"));
  
  Joint *joint_rod_piston = new Joint("Joint_Rod_Piston");
  addLink(joint_rod_piston);
  Vec rod_KrCS(3,INIT,0.);
  rod_KrCS(0) = 0.5*length_rod;
  rod->addFrame("S",rod_KrCS,SqrMat(3,EYE));
  joint_rod_piston->setForceDirection("[1,0;0,1;0,0]");
  joint_rod_piston->setForceLaw( new BilateralConstraint());
  joint_rod_piston->setImpactForceLaw( new BilateralImpact());
  joint_rod_piston->connect(rod->getFrame("S"),piston->getFrame("C"));

  // initial conditions
  crank->setInitialGeneralizedVelocity(150.);
  Vec q0_rod(3,INIT,0.);
  q0_rod(0) = length_crank+0.5*length_rod;
  rod->setInitialGeneralizedPosition(q0_rod);
  Vec u0_rod(3,INIT,0.);
  u0_rod(2) = -75;
  rod->setInitialGeneralizedVelocity(u0_rod);
  Vec q0_piston(3,INIT,0.);
  q0_piston(0) = length_crank+length_rod+0.5*length_piston;
  piston->setInitialGeneralizedPosition(q0_piston);
  //------------------------------------------------------------------------------

  // contours
  // points on piston
  Point *point_piston_1 = new Point("Point_Piston_1");
  Vec piston_KrCP1(3,INIT,0.);
  piston_KrCP1(0) = -0.5*length_piston;
  piston_KrCP1(1) = 0.5*width_piston;
  rod->addFrame("P1",piston_KrCP1,SqrMat(3,EYE));
  piston->addContour(point_piston_1,piston_KrCP1,SqrMat(3,EYE));

  Point *point_piston_2 = new Point("Point_Piston_2");
  Vec piston_KrCP2(3,INIT,0.);
  piston_KrCP2(0) = 0.5*length_piston;
  piston_KrCP2(1) = 0.5*width_piston;
  rod->addFrame("P2",piston_KrCP2,SqrMat(3,EYE));
  piston->addContour(point_piston_2,piston_KrCP2,SqrMat(3,EYE));

  Point *point_piston_3 = new Point("Point_Piston_3");
  Vec piston_KrCP3(3,INIT,0.);
  piston_KrCP3(0) = -0.5*length_piston;
  piston_KrCP3(1) = -0.5*width_piston;
  rod->addFrame("P3",piston_KrCP3,SqrMat(3,EYE));
  piston->addContour(point_piston_3,piston_KrCP3,SqrMat(3,EYE));

  Point *point_piston_4 = new Point("Point_Piston_4");
  Vec piston_KrCP4(3,INIT,0.);
  piston_KrCP4(0) = 0.5*length_piston;
  piston_KrCP4(1) = -0.5*width_piston;
  rod->addFrame("P4",piston_KrCP4,SqrMat(3,EYE));
  piston->addContour(point_piston_4,piston_KrCP4,SqrMat(3,EYE));

  // bottom plane
  Line *bottom = new Line("Bottom");
  Vec bottom_IrIB(3,INIT,0.);
  bottom_IrIB(1) = -clearance-0.5*width_piston;
  SqrMat bottom_A(3,INIT,0.);
  bottom_A(0,1) = -1; bottom_A(1,0) = 1; bottom_A(2,2) = 1;
  this->addContour(bottom,bottom_IrIB,bottom_A);

  // top plane
  Line *top = new Line("Top");
  Vec top_IrIT(3,INIT,0.);
  top_IrIT(1) = clearance+0.5*width_piston;
  SqrMat top_A(3,INIT,0.);
  top_A(0,1) = 1; top_A(1,0) = -1; top_A(2,2) = 1;
  this->addContour(top,top_IrIT,top_A);
  //---------------------------------------------------------------------------

  // contacts
  Contact *contact_point_piston_1_top = new Contact("Contact_Point_Piston_1_Top");
  contact_point_piston_1_top->connect(point_piston_1,top);
  contact_point_piston_1_top->setFrictionForceLaw(new PlanarCoulombFriction(0.01)); // mu1
  contact_point_piston_1_top->setFrictionImpactLaw(new PlanarCoulombImpact(0.01));
  contact_point_piston_1_top->setContactForceLaw(new UnilateralConstraint());
  contact_point_piston_1_top->setContactImpactLaw(new UnilateralNewtonImpact(0.4)); // epsN1
  contact_point_piston_1_top->enableOpenMBVContactPoints();
  this->addLink(contact_point_piston_1_top);

  Contact *contact_point_piston_2_top = new Contact("Contact_Point_Piston_2_Top");
  contact_point_piston_2_top->connect(point_piston_2,top);
  contact_point_piston_2_top->setFrictionForceLaw(new PlanarCoulombFriction(0.01)); // mu2
  contact_point_piston_2_top->setFrictionImpactLaw(new PlanarCoulombImpact(0.01));
  contact_point_piston_2_top->setContactForceLaw(new UnilateralConstraint());
  contact_point_piston_2_top->setContactImpactLaw(new UnilateralNewtonImpact(0.4)); // epsN2
  contact_point_piston_2_top->enableOpenMBVContactPoints();
  this->addLink(contact_point_piston_2_top);

  Contact *contact_point_piston_3_bottom = new Contact("Contact_Point_Piston_3_Bottom");
  contact_point_piston_3_bottom->connect(point_piston_3,bottom);
  contact_point_piston_3_bottom->setFrictionForceLaw(new PlanarCoulombFriction(0.01)); // mu3
  contact_point_piston_3_bottom->setFrictionImpactLaw(new PlanarCoulombImpact(0.01));
  contact_point_piston_3_bottom->setContactForceLaw(new UnilateralConstraint());
  contact_point_piston_3_bottom->setContactImpactLaw(new UnilateralNewtonImpact(0.4)); // epsN3
  contact_point_piston_3_bottom->enableOpenMBVContactPoints();
  this->addLink(contact_point_piston_3_bottom);
  
  Contact *contact_point_piston_4_bottom = new Contact("Contact_Point_Piston_4_Bottom");
  contact_point_piston_4_bottom->connect(point_piston_4,bottom);
  contact_point_piston_4_bottom->setFrictionForceLaw(new PlanarCoulombFriction(0.01)); // mu4
  contact_point_piston_4_bottom->setFrictionImpactLaw(new PlanarCoulombImpact(0.01));
  contact_point_piston_4_bottom->setContactForceLaw(new UnilateralConstraint());
  contact_point_piston_4_bottom->setContactImpactLaw(new UnilateralNewtonImpact(0.4)); // epsN4
  contact_point_piston_4_bottom->enableOpenMBVContactPoints();
  this->addLink(contact_point_piston_4_bottom);
  //---------------------------------------------------------------------------

  // visualisation
#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cuboid *openMBVCrank = new OpenMBV::Cuboid();
  openMBVCrank->setLength(length_crank,width_crank,thickness_crank);
  openMBVCrank->setStaticColor(0.5);
  crank->setOpenMBVRigidBody(openMBVCrank);

  OpenMBV::Cuboid *openMBVRod = new OpenMBV::Cuboid();
  openMBVRod->setLength(length_rod,width_rod,thickness_rod);
  openMBVRod->setStaticColor(0.5);
  rod->setOpenMBVRigidBody(openMBVRod);

  OpenMBV::Cuboid *openMBVPiston=new OpenMBV::Cuboid();
  openMBVPiston->setLength(length_piston,width_piston,thickness_piston);
  openMBVPiston->setStaticColor(0.8);
  piston->setOpenMBVRigidBody(openMBVPiston);
#endif

}

