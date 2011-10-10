#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/spring_damper.h"
#include "mbsim/environment.h"
#include "mbsim/contours/point.h"
#include "mbsim/contours/line_segment.h"
#include "mbsim/contours/line.h"
#include "mbsim/contact.h"
#include "mbsim/joint.h"
#include "mbsim/kinetic_excitation.h"
#include "mbsim/utils/function.h"
#include "mbsim/constitutive_laws.h"

#include <cmath>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/coilspring.h"
#include "openmbvcppinterface/cuboid.h"
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

//  // gravitation
//  Vec grav(3,INIT,0.);
//  grav(1)=-9.81;
//  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);
//
//  // bodies
//  RigidBody *crank = new RigidBody("Crank");
//  this->addObject(Crank);	
//
//  RigidBody *rod = new RigidBody("Rod");
//  this->addObject(Rod);
//
//  RigidBody *piston = new RigidBody("Piston");
//  this->addObject(Piston);
//
//  // geometrical characteristics
//  double width_crank = 0.05;
//  double thickness_crank = 0.05;
//  double length_crank = 0.153; // l1
//
//  double width_rod = 0.05;
//  double thickness_rod = 0.05;
//  double length_rod = 0.306; // l2
//
//  double length_piston = 2.*0.05; // 2a
//
//  // inertial properties
//  double mass_crank = 0.038; // m1
//  double mass_rod = 0.038; // m2
//  double mass_piston = 0.076; // m3
//
//  SymMat inertia_crank(3,INIT,0.);
//  inertia_crank(0,0) = 1.; // DUMMY
//  inertia_crank(1,1) = 1.; // DUMMY
//  inertia_crank(2,2) = 7.4e-5; // J1
//
//  SymMat inertia_rod(3,INIT,0.);
//  inertia_rod(0,0) = 1.; // DUMMY
//  inertia_rod(1,1) = 1.; // DUMMY
//  inertia_rod(2,2) = 5.9e-4; // J2
//
//  SymMat inertia_piston(3,INIT,0.);
//  inertia_piston(0,0) = 1.; // DUMMY
//  inertia_piston(1,1) = 1.; // DUMMY
//  inertia_piston(2,2) = 2.7e-6; // J3
//
//  crank->setMass(mass_crank);
//  rod->setMass(mass_rod);
//  piston->setMass(mass_piston);
//  crank->setInertiaTensor(inertia_crank);
//  rod->setInertiaTensor(inertia_rod);
//  piston->setInertiaTensor(inertia_piston);
//
//  // generalised coordinates
//  crank->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
//  rod->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));
//  piston->setTranslation(new LinearTranslation("[1,0;0,1;0,0]"));
//  //------------------------------------------------------------------------------
//
//  // Körper Crank: Setzen des Frame of Reference
//  Crank->setFrameOfReference(getFrame("I"));
//
//  // Körper Crank: Setzen des Frame for Kinematics
//  Vec Crank_KrCK(3,INIT,0.);
//  Crank_KrCK(0) = -Laenge_Crank/2.0;
//  Crank->addFrame("K",Crank_KrCK,SqrMat(3,EYE));
//  Crank->setFrameForKinematics(Crank->getFrame("K"));
//
//  // Körper Crank: Erstellung des P-Frames
//  Vec Crank_KrCP(3,INIT,0.);
//  Crank_KrCP(0) = Laenge_Crank/2.0;
//  Crank->addFrame("P",Crank_KrCP,SqrMat(3,EYE));
//
//  // Körper Rod: Setzen des Frame of Reference
//  Rod->setFrameOfReference(Crank->getFrame("P"));
//
//  // Körper Rod: Setzen des Frame of Kinematics
//  Vec Rod_KrCK(3,INIT,0.);
//  Rod_KrCK(0) = -Laenge_Rod/2.0;
//  Rod->addFrame("K",Rod_KrCK,SqrMat(3,EYE));
//  Rod->setFrameForKinematics(Rod->getFrame("K"));
//
//  // Körper Rod: Erstellung des P-Frames
//  Vec Rod_KrCP(3,INIT,0.);
//  Rod_KrCP(0) = Laenge_Rod/2.0;
//  Rod->addFrame("P",Rod_KrCP,SqrMat(3,EYE));
//
//  // Körper Piston: Setzen des Frame of Reference
//  Piston->setFrameOfReference(getFrame("I"));
//
//  // Körper Piston: Setzen des Frame of Kinematics
//  Piston->setFrameForKinematics(Piston->getFrame("C"));
//
//  // Anfangsbedingungen für Körper Crank:
//  double crank_u0 = 150.;
//  crank->setInitialGeneralizedPosition(0.);
//  crank->setInitialGeneralizedVelocity(crank_u0);
//
//  //Anfangsbedingunen für Körper Rod:
//  double rod_u0 = -75.;
//  rod->setInitialGeneralizedPosition(0.);
//  rod->setInitialGeneralizedVelocity(rod_u0);
//
//  //Anfangsbedingunen für Körper Piston:
//  Vec Piston_q0(2,INIT,0.);
//  Piston_q0(0) = Laenge_Crank+Laenge_Rod;
//  Piston->setInitialGeneralizedPosition(Piston_q0);
//  //------------------------------------------------------------------------------
//
//  // Joints
//  Joint *joint_rod_piston = new Joint("Joint_Rod_Piston");
//  joint_rod_piston->setForceDirection("[1,0;0,1;0,0]");
//  joint_rod_piston->setForceLaw( new BilateralConstraint());
//  joint_rod_piston->setImpactForceLaw( new BilateralImpact());
//  joint_rod_piston->connect(rod->getFrame("P"), Piston->getFrameForKinematics() );
//  addLink(joint_rod_piston);
//
//  // Contours
//  // Point an Piston
//  Point *Point_Piston = new Point("Point_Piston");
//  Piston->addContour(Point_Piston,Vec(3,INIT,0.),SqrMat(3,EYE));
//
//  // Ebene der Welt von unten
//  Line *plane1 = new Line("plane1");
//  SqrMat A(3,INIT,0.);
//  A(0,1) = -1; A(1,0) = 1; A(2,2) = 1;
//  addContour(plane1,Vec("[0;0;0]"),A);
//
//  // Ebene der Welt von oben
//  Line *plane2 = new Line("plane2");
//  SqrMat B(3,INIT,0.);
//  B(0,1) = 1; B(1,0) = -1; B(2,2) = 1;
//  addContour(plane2,Vec("[0;0;0]"),B);
//  //---------------------------------------------------------------------------
//
//  // Contacts
//  // Contact Point an Piston und Ebene der Welt unten
//  Contact *contact_Point_Piston_Ebene = new Contact("Contact_Point_Piston_Plane1");
//  contact_Point_Piston_Ebene->connect(Point_Piston,plane1);
//
//  contact_point_piston_plane1->setFrictionForceLaw(new PlanarCoulombFriction(0.01));
//  contact_point_piston_plane1->setFrictionImpactLaw(new PlanarCoulombImpact(0.01));
//  contact_point_piston_plane1->setContactForceLaw(new UnilateralConstraint());
//  contact_point_piston_plane1->setContactImpactLaw(new UnilateralNewtonImpact(0.4));
//  this->addLink(contact_point_piston_plane1);
//
//  // Contact Point an Piston und Ebene der Welt oben
//  Contact *contact_Point_Piston_Ebene2 = new Contact("contact_Point_Piston_Ebene2");
//  contact_Point_Piston_Ebene2->connect(Point_Piston,plane2);
//
//  contact_Point_Piston_Ebene2->setFrictionForceLaw(new PlanarCoulombFriction(0.1) );
//  contact_Point_Piston_Ebene2->setFrictionImpactLaw(new PlanarCoulombImpact(0.1));
//  contact_Point_Piston_Ebene2->setContactForceLaw(new UnilateralConstraint());
//  contact_Point_Piston_Ebene2->setContactImpactLaw(new UnilateralNewtonImpact(0.2));
//  this->addLink(contact_Point_Piston_Ebene2);
//  //---------------------------------------------------------------------------
//
//  // visualisation
//#ifdef HAVE_OPENMBVCPPINTERFACE
//  OpenMBV::Cuboid *openMBVCrank = new OpenMBV::Cuboid();
//  openMBVCrank->setLength(length_crank, width_crank, thickness_crank);
//  openMBVCrank->setInitialTranslation(0, 0, 0.05);
//  openMBVCrank->setStaticColor(0.5);
//  crank->setOpenMBVRigidBody(openMBVCrank);
//
//  OpenMBV::Cuboid *openMBVRod = new OpenMBV::Cuboid();
//  openMBVRod->setLength(length_rod, width_rod, thickness_rod);
//  openMBVRod->setStaticColor(0.5);
//  rod->setOpenMBVRigidBody(openMBVRod);
//
//  OpenMBV::Cuboid *openMBVPiston=new OpenMBV::Cuboid();
//  openMBVPiston->setLength(length_piston, width_piston, thickness_piston);
//  openMBVPiston->setStaticColor(0.8);
//  piston->setOpenMBVRigidBody(openMBVPiston);
//#endif

}

