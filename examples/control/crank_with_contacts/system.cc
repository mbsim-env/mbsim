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

  Crank->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  Rod->setRotation(new RotationAboutFixedAxis(Vec("[0;0;1]")));

  Piston->setTranslation(new LinearTranslation("[1,0;0,1;0,0]"));

  Block->setTranslation(new LinearTranslation("[1,0;0,1;0,0]"));

  //------------------------------------------------------------------------------

  // Körper Crank: Setzen des Frame of Reference
  Crank->setFrameOfReference(getFrame("I"));

  // Körper Crank: Setzen des Frame for Kinematics
  Vec Crank_KrCK(3,INIT,0.);
  Crank_KrCK(0) = -Laenge_Crank/2.0;
  Crank->addFrame("K",Crank_KrCK,SqrMat(3,EYE));
  Crank->setFrameForKinematics(Crank->getFrame("K"));

  // Körper Crank: Erstellung des P-Frames
  Vec Crank_KrCP(3,INIT,0.);
  Crank_KrCP(0) = Laenge_Crank/2.0;
  Crank->addFrame("P",Crank_KrCP,SqrMat(3,EYE));

  //-----------------------------------

  // Körper Rod: Setzen des Frame of Reference
  Rod->setFrameOfReference(Crank->getFrame("P"));

  // Körper Rod: Setzen des Frame of Kinematics
  Vec Rod_KrCK(3,INIT,0.);
  Rod_KrCK(0) = -Laenge_Rod/2.0;
  Rod->addFrame("K",Rod_KrCK,SqrMat(3,EYE));
  Rod->setFrameForKinematics(Rod->getFrame("K"));

  // Körper Rod: Erstellung des P-Frames
  Vec Rod_KrCP(3,INIT,0.);
  Rod_KrCP(0) = Laenge_Rod/2.0;
  Rod->addFrame("P",Rod_KrCP,SqrMat(3,EYE));

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
  MomentAnCrank->setMoment(Mat("[0;0;1]"), new ConstantFunction1<Vec, double>(Vec("[109]")) );
  MomentAnCrank->setFrameOfReference(Crank->getFrameForKinematics());
  MomentAnCrank->connect(Crank->getFrameForKinematics());

  addLink(MomentAnCrank);

  //------------------------------------------------------------------------------

  // Joints

  // Joint zwischen Rod und Piston
  Joint *joint_Rod_Piston = new Joint("Joint_Rod_Piston");
  joint_Rod_Piston->setForceDirection("[1,0;0,1;0,0]");
  joint_Rod_Piston->setForceLaw( new BilateralConstraint());
  joint_Rod_Piston->setImpactForceLaw( new BilateralImpact());
  joint_Rod_Piston->connect( Rod->getFrame("P") , Piston->getFrameForKinematics() );

  addLink(joint_Rod_Piston);


  //------------------------------------------------------------------------------

  // Contours

  // Point an Piston
  Point *Point_Piston = new Point("Point_Piston");
  Piston->addContour(Point_Piston,Vec(3,INIT,0.),SqrMat(3,EYE));

  // Point an Block
  Point *Point_Block = new Point("Point_Block");
  Block->addContour(Point_Block,Vec(3,INIT,0.),SqrMat(3,EYE));


  // Liniensegment an Block für Kontakt mit Piston
  LineSegment *line1 = new LineSegment("line1");
  SqrMat C(3,INIT,0.);
  C(0,0) = -1; C(1,1) = 1; C(2,2) = -1;
  Vec RrRC_line1(3,INIT,0.);
  RrRC_line1(0) = -(0.5*Laenge_Block+0.5*Laenge_Piston); //damit Kontakt an beiden Konturen auftritt
  line1->setLength(2);
  Block->addContour(line1,RrRC_line1,C);


  // Ebene der Welt von unten
  Line *plane1 = new Line("plane1");
  SqrMat A(3,INIT,0.);
  A(0,1) = -1; A(1,0) = 1; A(2,2) = 1;
  addContour(plane1,Vec("[0;0;0]"),A);

  // Ebene der Welt von oben
  Line *plane2 = new Line("plane2");
  SqrMat B(3,INIT,0.);
  B(0,1) = 1; B(1,0) = -1; B(2,2) = 1;
  addContour(plane2,Vec("[0;0;0]"),B);


  // Ebene Vertikal rechts für Feder und Block
  Line *plane3 = new Line("plane3");
  Vec Plane3_q0(3,INIT,0.);
  Plane3_q0(0) = Laenge_Crank+Laenge_Rod+Laenge_Piston+Laenge_Block+0.2;
  addContour(plane3,Plane3_q0,C);

  //---------------------------------------------------------------------------

  // Contacts

  // Contact Point an Piston und Ebene der Welt unten
  Contact *contact_Point_Piston_Ebene = new Contact("contact_Point_Piston_Ebene");
  contact_Point_Piston_Ebene->connect(Point_Piston,plane1);

  contact_Point_Piston_Ebene->setFrictionForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Piston_Ebene->setFrictionImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Piston_Ebene->setContactForceLaw(new UnilateralConstraint());
  contact_Point_Piston_Ebene->setContactImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Piston_Ebene);

  // Contact Point an Piston und Ebene der Welt oben
  Contact *contact_Point_Piston_Ebene2 = new Contact("contact_Point_Piston_Ebene2");
  contact_Point_Piston_Ebene2->connect(Point_Piston,plane2);

  contact_Point_Piston_Ebene2->setFrictionForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Piston_Ebene2->setFrictionImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Piston_Ebene2->setContactForceLaw(new UnilateralConstraint());
  contact_Point_Piston_Ebene2->setContactImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Piston_Ebene2);

  // Contact Point an Block und Ebene der Welt unten
  Contact *contact_Point_Block_Ebene = new Contact("contact_Point_Block_Ebene");
  contact_Point_Block_Ebene->connect(Point_Block,plane1);

  contact_Point_Block_Ebene->setFrictionForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Block_Ebene->setFrictionImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Block_Ebene->setContactForceLaw(new UnilateralConstraint());
  contact_Point_Block_Ebene->setContactImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Block_Ebene);

  // Contact Point an Block und Ebene der Welt oben
  Contact *contact_Point_Block_Ebene2 = new Contact("contact_Point_Block_Ebene2");
  contact_Point_Block_Ebene2->connect(Point_Block,plane2);

  contact_Point_Block_Ebene2->setFrictionForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Block_Ebene2->setFrictionImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Block_Ebene2->setContactForceLaw(new UnilateralConstraint());
  contact_Point_Block_Ebene2->setContactImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Block_Ebene2);

  // Contact Point an Piston und Line an Block
  Contact *contact_Point_Piston_Line_Block = new Contact("contact_Point_Piston_Line_Block");
  contact_Point_Piston_Line_Block->connect(Point_Piston,line1);

  contact_Point_Piston_Line_Block->setFrictionForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Piston_Line_Block->setFrictionImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Piston_Line_Block->setContactForceLaw(new UnilateralConstraint());
  contact_Point_Piston_Line_Block->setContactImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Piston_Line_Block);

  // Contact Point an Block und Ebene Vertikal rechts
  Contact *contact_Point_Block_Ebene_Vertikal = new Contact("contact_Point_Block_Ebene_Vertikal");
  contact_Point_Block_Ebene_Vertikal->connect(Point_Block,plane3);

  contact_Point_Block_Ebene_Vertikal->setFrictionForceLaw(new PlanarCoulombFriction(0.1) );
  contact_Point_Block_Ebene_Vertikal->setFrictionImpactLaw(new PlanarCoulombImpact(0.1));
  contact_Point_Block_Ebene_Vertikal->setContactForceLaw(new UnilateralConstraint());
  contact_Point_Block_Ebene_Vertikal->setContactImpactLaw(new UnilateralNewtonImpact(0.2));
  this->addLink(contact_Point_Block_Ebene_Vertikal);


  //---------------------------------------------------------------------------

  // spring

  // frames on environment 
  this->addFrame("L",Plane3_q0,SqrMat(3,EYE));

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
  VelSoll->setFunction( new ConstantFunction1<Vec, double>(Vec("10") ));

  // Signal-Addition
  SignalAddition *Regelfehler = new SignalAddition("Regelfehler");
  addLink(Regelfehler);
  Regelfehler->addSignal(AbsVelSensorAnCrank,-1);
  Regelfehler->addSignal(VelSoll,1);

  // Regler
  LinearTransferSystem *Regler = new LinearTransferSystem("Regler");
  addExtraDynamic(Regler);
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
  OpenMBV::Cuboid *openMBVCrank=new OpenMBV::Cuboid();
  openMBVCrank->setLength(Laenge_Crank,Breite_Crank-0.03,Dicke_Crank-0.05);
  openMBVCrank->setInitialTranslation(0, 0, 0.05);
  openMBVCrank->setStaticColor(0.5);
  Crank->setOpenMBVRigidBody(openMBVCrank);

  OpenMBV::Cuboid *openMBVRod=new OpenMBV::Cuboid();
  openMBVRod->setLength(Laenge_Rod,Breite_Rod-0.04,Dicke_Rod-0.05);
  openMBVRod->setStaticColor(0.5);
  Rod->setOpenMBVRigidBody(openMBVRod);

  OpenMBV::Frustum *openMBVCylinder=new OpenMBV::Frustum();
  openMBVCylinder->setBaseRadius(0.045);
  openMBVCylinder->setTopRadius(0.045);
  openMBVCylinder->setHeight(0.054);
  openMBVCylinder->setInitialTranslation(0, 0, 0.077);
  openMBVCylinder->setStaticColor(0.5);
  Lager->setOpenMBVRigidBody(openMBVCylinder);

  OpenMBV::Frustum *openMBVCylinder2=new OpenMBV::Frustum();
  openMBVCylinder2->setBaseRadius(0.045);
  openMBVCylinder2->setTopRadius(0.045);
  openMBVCylinder2->setHeight(0.105);
  openMBVCylinder2->setInitialTranslation(0, 0, 0.076);
  openMBVCylinder2->setStaticColor(0.45);
  Gelenk->setOpenMBVRigidBody(openMBVCylinder2);

  OpenMBV::Cuboid *openMBVPiston=new OpenMBV::Cuboid();
  openMBVPiston->setLength(Laenge_Piston,Laenge_Piston,Laenge_Piston);
  openMBVPiston->setStaticColor(0.8);
  Piston->setOpenMBVRigidBody(openMBVPiston);

  OpenMBV::Cuboid *openMBVBlock=new OpenMBV::Cuboid();
  openMBVBlock->setLength(Laenge_Block,Breite_Block,Dicke_Block);
  Block->setOpenMBVRigidBody(openMBVBlock);

  OpenMBV::CoilSpring *openMBVspring1=new OpenMBV::CoilSpring;
  openMBVspring1->setSpringRadius(0.1);
  openMBVspring1->setCrossSectionRadius(0.01);
  openMBVspring1->setNumberOfCoils(5);
  spring1->setOpenMBVSpring(openMBVspring1);
#endif

}
