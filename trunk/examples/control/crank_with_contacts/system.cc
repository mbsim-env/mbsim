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
#include "fmatvec/function.h"
#include "mbsim/constitutive_laws.h"
#include "mbsim/functions/kinematic_functions.h"
#include "mbsim/functions/kinetic_functions.h"
#include "mbsim/functions/basic_functions.h"

#include "mbsimControl/actuator.h"
#include "mbsimControl/linear_transfer_system.h"
#include "mbsimControl/object_sensors.h"
#include "mbsimControl/signal_processing_system_sensor.h"
#include "mbsimControl/function_sensor.h"
#include "mbsimControl/signal_manipulation.h"
#include "mbsimControl/frame_sensors.h"

#include <cmath>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/coilspring.h"
#include "openmbvcppinterface/cuboid.h"
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;
using namespace MBSimControl;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  // Schwerkraft
  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  //------------------------------------------------------------------------------

  // Körper
  RigidBody *Crank = new RigidBody("Crank");
  RigidBody *Rod = new RigidBody("Rod");
  RigidBody *Piston = new RigidBody("Piston");
  RigidBody *Block = new RigidBody("Block");

  // Hinzufügen zum Weltsystem
  this->addObject(Crank);	
  this->addObject(Rod);
  this->addObject(Piston);
  this->addObject(Block);

  //------------------------------------------------------------------------------

  // Eigenschaften der Körper
  double Breite_Crank = 0.1;
  double Dicke_Crank = 0.1;
  double Laenge_Crank = 0.3;
  double Masse_Crank = 8.1;

  double Breite_Rod = 0.1;
  double Dicke_Rod = 0.1;
  double Laenge_Rod = 0.8;
  double Masse_Rod = 63.2;

  double Laenge_Piston = 0.2;
  double Masse_Piston = 10.8;

  double Breite_Block = 0.2;
  double Dicke_Block = 0.2;
  double Laenge_Block = 0.1;
  double Masse_Block = 5.4;

  Crank->setMass(Masse_Crank);
  Rod->setMass(Masse_Rod);
  Piston->setMass(Masse_Piston);
  Block->setMass(Masse_Block);

  SymMat InertiaTensor_Crank(3,INIT,0.);
  InertiaTensor_Crank(0,0) = 1.0/12.0*Masse_Crank*(Breite_Crank*Breite_Crank + Dicke_Crank*Dicke_Crank);
  InertiaTensor_Crank(1,1) = 1.0/12.0*Masse_Crank*(Dicke_Crank*Dicke_Crank + Laenge_Crank*Laenge_Crank);
  InertiaTensor_Crank(2,2) = 1.0/12.0*Masse_Crank*(Breite_Crank*Breite_Crank + Laenge_Crank*Laenge_Crank);

  SymMat InertiaTensor_Rod(3,INIT,0.);
  InertiaTensor_Rod(0,0) = 1.0/12.0*Masse_Rod*(Breite_Rod*Breite_Rod + Dicke_Rod*Dicke_Rod);
  InertiaTensor_Rod(1,1) = 1.0/12.0*Masse_Rod*(Dicke_Rod*Dicke_Rod + Laenge_Rod*Laenge_Rod);
  InertiaTensor_Rod(2,2) = 1.0/12.0*Masse_Rod*(Breite_Rod*Breite_Rod + Laenge_Rod*Laenge_Rod);

  Crank->setInertiaTensor(InertiaTensor_Crank);
  Rod->setInertiaTensor(InertiaTensor_Rod);
  //Dummy-Werte da sich Körper nicht drehen
  Piston->setInertiaTensor(SymMat(3,INIT,1.));
  Block->setInertiaTensor(SymMat(3,INIT,1.));

  //------------------------------------------------------------------------------

  // Generalisierte Koordinaten

  Crank->setRotation(new RotationAboutZAxis<VecV>);

  Rod->setRotation(new RotationAboutZAxis<VecV>);

  Piston->setTranslation(new TranslationAlongAxesXY<VecV>);

  Block->setTranslation(new TranslationAlongAxesXY<VecV>);

  //------------------------------------------------------------------------------

  // Körper Crank: Setzen des Frame of Reference
  Crank->setFrameOfReference(getFrame("I"));

  // Körper Crank: Setzen des Frame for Kinematics
  Vec Crank_KrCK(3,INIT,0.);
  Crank_KrCK(0) = -Laenge_Crank/2.0;
  Crank->addFrame(new FixedRelativeFrame("K",Crank_KrCK,SqrMat(3,EYE)));
  Crank->setFrameForKinematics(Crank->getFrame("K"));

  // Körper Crank: Erstellung des P-Frames
  Vec Crank_KrCP(3,INIT,0.);
  Crank_KrCP(0) = Laenge_Crank/2.0;
  Crank->addFrame(new FixedRelativeFrame("P",Crank_KrCP,SqrMat(3,EYE)));

  //-----------------------------------

  // Körper Rod: Setzen des Frame of Reference
  Rod->setFrameOfReference(Crank->getFrame("P"));

  // Körper Rod: Setzen des Frame of Kinematics
  Vec Rod_KrCK(3,INIT,0.);
  Rod_KrCK(0) = -Laenge_Rod/2.0;
  Rod->addFrame(new FixedRelativeFrame("K",Rod_KrCK,SqrMat(3,EYE)));
  Rod->setFrameForKinematics(Rod->getFrame("K"));

  // Körper Rod: Erstellung des P-Frames
  Vec Rod_KrCP(3,INIT,0.);
  Rod_KrCP(0) = Laenge_Rod/2.0;
  Rod->addFrame(new FixedRelativeFrame("P",Rod_KrCP,SqrMat(3,EYE)));

  //-----------------------------------

  // Körper Piston: Setzen des Frame of Reference
  Piston->setFrameOfReference(getFrame("I"));

  // Körper Piston: Setzen des Frame of Kinematics
  Piston->setFrameForKinematics(Piston->getFrame("C"));

  //-----------------------------------

  // Körper Block: Setzen des Frame of Reference
  Block->setFrameOfReference(getFrame("I"));

  // Körper Block: Setzen des Frame of Kinematics
  Block->setFrameForKinematics(Block->getFrame("C"));


  //------------------------------------------------------------------------------

  // Anfangsbedingungen für Körper Crank:
  double Crank_q0 = 0;
  double Crank_u0 = 0;
  Crank->setInitialGeneralizedPosition(Crank_q0);
  Crank->setInitialGeneralizedVelocity(Crank_u0);

  //Anfangsbedingunen für Körper Rod:
  double Rod_q0 = 0;
  double Rod_u0 = 0;
  Rod->setInitialGeneralizedPosition(Rod_q0);
  Rod->setInitialGeneralizedVelocity(Rod_u0);

  //Anfangsbedingunen für Körper Piston:
  Vec Piston_q0(2,INIT,0.);
  Piston_q0(0) = Laenge_Crank+Laenge_Rod;
  Piston->setInitialGeneralizedPosition(Piston_q0);

  //Anfangsbedingunen für Körper Block:
  Vec Block_q0(2,INIT,0.);
  Block_q0(0) = Laenge_Crank+Laenge_Rod+0.25*Laenge_Piston;
  Block->setInitialGeneralizedPosition(Block_q0);


  //------------------------------------------------------------------------------

  // Anregung
  KineticExcitation *MomentAnCrank = new KineticExcitation("MomentAnCrank");
  MomentAnCrank->setMomentDirection("[0;0;1]");
  MomentAnCrank->setMomentFunction(new ConstantFunction<VecV(double)>(109));
  MomentAnCrank->connect(Crank->getFrameForKinematics());

  addLink(MomentAnCrank);

  //------------------------------------------------------------------------------

  // Joints

  // Joint zwischen Rod und Piston
  Joint *joint_Rod_Piston = new Joint("Joint_Rod_Piston");
  joint_Rod_Piston->setForceDirection("[1,0;0,1;0,0]");
  joint_Rod_Piston->setForceLaw( new BilateralConstraint());
  joint_Rod_Piston->connect( Rod->getFrame("P") , Piston->getFrameForKinematics() );

  addLink(joint_Rod_Piston);


  //------------------------------------------------------------------------------

  // Contours

  // Point an Piston
  Piston->addContour(new Point("Point_Piston"))

  // Point an Block
  ;
  Block->addContour(new Point("Point_Block"));


  // Liniensegment an Block für Kontakt mit Piston
  SqrMat C(3,INIT,0.);
  C(0,0) = -1; C(1,1) = 1; C(2,2) = -1;
  Vec RrRC_line1(3,INIT,0.);
  RrRC_line1(0) = -(0.5*Laenge_Block+0.5*Laenge_Piston); //damit Kontakt an beiden Konturen auftritt
  Block->addFrame(new FixedRelativeFrame("line1",RrRC_line1,C));
  Block->addContour(new LineSegment("line1",2,Block->getFrame("line1")));


  // Ebene der Welt von unten
  SqrMat A(3,INIT,0.);
  A(0,1) = -1; A(1,0) = 1; A(2,2) = 1;
  addFrame(new FixedRelativeFrame("plane1",Vec("[0;0;0]"),A));
  addContour(new Line("plane1",getFrame("plane1")));

  // Ebene der Welt von oben
  SqrMat B(3,INIT,0.);
  B(0,1) = 1; B(1,0) = -1; B(2,2) = 1;
  addFrame(new FixedRelativeFrame("plane2",Vec("[0;0;0]"),B));
  addContour(new Line("plane2",getFrame("plane2")));


  // Ebene Vertikal rechts für Feder und Block
  Vec Plane3_q0(3,INIT,0.);
  Plane3_q0(0) = Laenge_Crank+Laenge_Rod+Laenge_Piston+Laenge_Block+0.2;
  addFrame(new FixedRelativeFrame("plane3",Plane3_q0,C));
  addContour(new Line("plane3",getFrame("plane3")));

  //---------------------------------------------------------------------------

  // Contacts

  // Contact Point an Piston und Ebene der Welt unten
  Contact *contact_Point_Piston_Ebene = new Contact("contact_Point_Piston_Ebene");
  contact_Point_Piston_Ebene->connect(Piston->getContour("Point_Piston"),getContour("plane1"));

  contact_Point_Piston_Ebene->setTangentialForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Piston_Ebene->setTangentialImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Piston_Ebene->setNormalForceLaw(new UnilateralConstraint());
  contact_Point_Piston_Ebene->setNormalImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Piston_Ebene);

  // Contact Point an Piston und Ebene der Welt oben
  Contact *contact_Point_Piston_Ebene2 = new Contact("contact_Point_Piston_Ebene2");
  contact_Point_Piston_Ebene2->connect(Piston->getContour("Point_Piston"),getContour("plane2"));

  contact_Point_Piston_Ebene2->setTangentialForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Piston_Ebene2->setTangentialImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Piston_Ebene2->setNormalForceLaw(new UnilateralConstraint());
  contact_Point_Piston_Ebene2->setNormalImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Piston_Ebene2);

  // Contact Point an Block und Ebene der Welt unten
  Contact *contact_Point_Block_Ebene = new Contact("contact_Point_Block_Ebene");
  contact_Point_Block_Ebene->connect(Block->getContour("Point_Block"),getContour("plane1"));

  contact_Point_Block_Ebene->setTangentialForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Block_Ebene->setTangentialImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Block_Ebene->setNormalForceLaw(new UnilateralConstraint());
  contact_Point_Block_Ebene->setNormalImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Block_Ebene);

  // Contact Point an Block und Ebene der Welt oben
  Contact *contact_Point_Block_Ebene2 = new Contact("contact_Point_Block_Ebene2");
  contact_Point_Block_Ebene2->connect(Block->getContour("Point_Block"),getContour("plane2"));

  contact_Point_Block_Ebene2->setTangentialForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Block_Ebene2->setTangentialImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Block_Ebene2->setNormalForceLaw(new UnilateralConstraint());
  contact_Point_Block_Ebene2->setNormalImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Block_Ebene2);

  // Contact Point an Piston und Line an Block
  Contact *contact_Point_Piston_Line_Block = new Contact("contact_Point_Piston_Line_Block");
  contact_Point_Piston_Line_Block->connect(Piston->getContour("Point_Piston"),Block->getContour("line1"));

  contact_Point_Piston_Line_Block->setTangentialForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Piston_Line_Block->setTangentialImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Piston_Line_Block->setNormalForceLaw(new UnilateralConstraint());
  contact_Point_Piston_Line_Block->setNormalImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Piston_Line_Block);

  // Contact Point an Block und Ebene Vertikal rechts
  Contact *contact_Point_Block_Ebene_Vertikal = new Contact("contact_Point_Block_Ebene_Vertikal");
  contact_Point_Block_Ebene_Vertikal->connect(Block->getContour("Point_Block"),getContour("plane3"));

  contact_Point_Block_Ebene_Vertikal->setTangentialForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Block_Ebene_Vertikal->setTangentialImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Block_Ebene_Vertikal->setNormalForceLaw(new UnilateralConstraint());
  contact_Point_Block_Ebene_Vertikal->setNormalImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Block_Ebene_Vertikal);


  //---------------------------------------------------------------------------

  // spring

  // frames on environment 
  this->addFrame(new FixedRelativeFrame("L",Plane3_q0,SqrMat(3,EYE)));

  SpringDamper *spring1 = new SpringDamper("spring1");
  spring1->setForceFunction(new LinearSpringDamperForce(30,1,0.5));
  spring1->connect(Block->getFrame("C"),this->getFrame("L"));

  // add spring to dynamical system
  this->addLink(spring1);

  //------------------------------------------------------------------------------

  // Regelung

  // Geschwindigkeitssensor an Crank
  AbsoluteAngularVelocitySensor *AbsVelSensorAnCrank = new AbsoluteAngularVelocitySensor("AbsVelSensorAnCrank");
  addLink(AbsVelSensorAnCrank);
  AbsVelSensorAnCrank->setFrame(Crank->getFrameForKinematics());
  AbsVelSensorAnCrank->setDirection("[0;0;1]");

  // Zusätzlicher Sensor
  GeneralizedVelocitySensor *GenVelSensorAnCrank = new GeneralizedVelocitySensor("GenVelSensorAnCrank");
  addLink(GenVelSensorAnCrank);
  GenVelSensorAnCrank->setObject(Crank);
  GenVelSensorAnCrank->setIndex(0);

  // Zusätzlicher Sensor
  GeneralizedVelocitySensor *GenVelSensorAnRod = new GeneralizedVelocitySensor("GenVelSensorAnRod");
  addLink(GenVelSensorAnRod);
  GenVelSensorAnRod->setObject(Rod);
  GenVelSensorAnRod->setIndex(0);

  // Zusätzlicher Sensor
  AbsoluteAngularVelocitySensor *AbsVelSensorAnRod = new AbsoluteAngularVelocitySensor("AbsVelSensorAnRod");
  addLink(AbsVelSensorAnRod);
  AbsVelSensorAnRod->setFrame(Rod->getFrameForKinematics());
  AbsVelSensorAnRod->setDirection("[0;0;1]");




  // Sollsignal
  FunctionSensor *VelSoll = new FunctionSensor("VelSoll");
  addLink(VelSoll);
  VelSoll->setFunction(new ConstantFunction<VecV(double)>(10));

  // Signal-Addition
  SignalAddition *Regelfehler = new SignalAddition("Regelfehler");
  addLink(Regelfehler);
  Regelfehler->addSignal(AbsVelSensorAnCrank,-1);
  Regelfehler->addSignal(VelSoll,1);

  // Regler
  LinearTransferSystem *Regler = new LinearTransferSystem("Regler");
  addLink(Regler);
  Regler->setInputSignal(Regelfehler);
  Regler->setPID(10000.,1000.,0.);

  // Konvertierung
  SignalProcessingSystemSensor *Reglerausgang = new SignalProcessingSystemSensor("Reglerausgang");
  addLink(Reglerausgang);
  Reglerausgang->setSignalProcessingSystem(Regler);

  // Aktuator
  Actuator *Antrieb = new Actuator("Antrieb");
  addLink(Antrieb);
  Antrieb->setSignal(Reglerausgang);
  Antrieb->setMomentDirection("[0;0;1]");
  Antrieb->connect(this->getFrame("I"),Crank->getFrameForKinematics());

  //---------------------------------------------------------------------------
  //Dummy Körper zur Visualisierung von Lager und Gelenk
  RigidBody *Lager = new RigidBody("Lager");
  this->addObject(Lager);
  Lager->setMass(0.);
  Lager->setInertiaTensor(SymMat(3,INIT,1.));
  Lager->setFrameOfReference(getFrame("I"));

  RigidBody *Gelenk = new RigidBody("Gelenk");
  this->addObject(Gelenk);
  Gelenk->setMass(0.);
  Gelenk->setInertiaTensor(SymMat(3,INIT,1.));
  Gelenk->setFrameOfReference(Crank->getFrame("P"));

  // Visualisierung mit openMBV
#ifdef HAVE_OPENMBVCPPINTERFACE
  boost::shared_ptr<OpenMBV::Cuboid> openMBVCrank=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVCrank->setLength(Laenge_Crank,Breite_Crank-0.03,Dicke_Crank-0.05);
  openMBVCrank->setInitialTranslation(0, 0, 0.05);
  openMBVCrank->setDiffuseColor(0.5,1,1);
  Crank->setOpenMBVRigidBody(openMBVCrank);

  boost::shared_ptr<OpenMBV::Cuboid> openMBVRod=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVRod->setLength(Laenge_Rod,Breite_Rod-0.04,Dicke_Rod-0.05);
  openMBVRod->setDiffuseColor(0.5,1,1);
  Rod->setOpenMBVRigidBody(openMBVRod);

  boost::shared_ptr<OpenMBV::Frustum> openMBVCylinder=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  openMBVCylinder->setBaseRadius(0.045);
  openMBVCylinder->setTopRadius(0.045);
  openMBVCylinder->setHeight(0.054);
  openMBVCylinder->setInitialTranslation(0, 0, 0.077);
  openMBVCylinder->setDiffuseColor(0.5,1,1);
  Lager->setOpenMBVRigidBody(openMBVCylinder);

  boost::shared_ptr<OpenMBV::Frustum> openMBVCylinder2=OpenMBV::ObjectFactory::create<OpenMBV::Frustum>();
  openMBVCylinder2->setBaseRadius(0.045);
  openMBVCylinder2->setTopRadius(0.045);
  openMBVCylinder2->setHeight(0.105);
  openMBVCylinder2->setInitialTranslation(0, 0, 0.076);
  openMBVCylinder2->setDiffuseColor(0.45,1,1);
  Gelenk->setOpenMBVRigidBody(openMBVCylinder2);

  boost::shared_ptr<OpenMBV::Cuboid> openMBVPiston=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVPiston->setLength(Laenge_Piston,Laenge_Piston,Laenge_Piston);
  openMBVPiston->setDiffuseColor(0.8,1,1);
  Piston->setOpenMBVRigidBody(openMBVPiston);

  boost::shared_ptr<OpenMBV::Cuboid> openMBVBlock=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  openMBVBlock->setLength(Laenge_Block,Breite_Block,Dicke_Block);
  Block->setOpenMBVRigidBody(openMBVBlock);

  spring1->enableOpenMBVCoilSpring(_springRadius=0.1,_crossSectionRadius=0.01,_numberOfCoils=5);
#endif

}
