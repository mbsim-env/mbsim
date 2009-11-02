#include "system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/userfunction.h"
#include "mbsim/spring_damper.h"
#include "mbsim/load.h"
#include "mbsim/environment.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/cube.h"
#include "openmbvcppinterface/coilspring.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  // Erdbeschleungigung definieren
  Vec g(3);
  g(1)=-9.81*0;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(g);

  // Parameter der Körper
  double m1 = 5;
  double m2 = 2;
  SymMat Theta1(3,EYE);
  SymMat Theta2(3,EYE);
  double h1 = 0.5;
  double h2 = 0.5;

  // Parameter der Federn
  double c1 = 1e3;
  double c2 = 1e2;
  double d1 = 0;
  double d2 = 0;
  double l01 = 0.5;
  double l02 = 0.5;

  // ----------------------- Definition des 1. Körpers --------------------  
  RigidBody *box1 = new RigidBody("Box1");
  addObject(box1);

  // Masse und Trägheit definieren
  box1->setMass(m1);
  box1->setInertiaTensor(Theta1);

  // Kinematik: Bewegung des Schwerpunktes (Center of Gravity COG) 
  // entlang der y-Richtung ausgehend vom I-System (Ursprung O)
  box1->setTranslation(new LinearTranslation("[0; 1; 0]"));
  box1->setFrameOfReference(getFrame("I"));
  box1->setFrameForKinematics(box1->getFrame("C"));


  // ----------------------- Definition des 2. Körpers --------------------  
  RigidBody *box2 = new RigidBody("Box2");
  addObject(box2);

  // Masse und Trägheit definieren
  box2->setMass(m2);
  box2->setInertiaTensor(Theta2);

  // Kinematik: Bewegung des Schwerpunktes (Center of Gravity COG) 
  // entlang der y-Richtung ausgehend vom I-System (Ursprung O)
  box2->setTranslation(new LinearTranslation("[0; 1; 0]"));
  box2->setFrameOfReference(getFrame("I"));
  box2->setFrameForKinematics(box2->getFrame("C"));

  // ----------------------- Anschlusspunkte der Federn --------------------  
  Vec SrSP(3);
  SqrMat ASP(3,EYE);

  // Federanschlusspunkte P1 und P2 auf Körper 1 definieren
  SrSP(1) = h1/2.;
  box1->addFrame("P1",-SrSP,ASP); 
  box1->addFrame("P2",SrSP,ASP);

  // Federanschlusspunkt P1 auf Körper 2 definieren
  SrSP(1) = h2/2.;
  box2->addFrame("P1",-SrSP,ASP);

  // ----------------------- Definition der 1. Feder --------------------  
  SpringDamper *spring1 = new SpringDamper("Feder1");
  addLink(spring1);
  spring1->setForceFunction(new LinearSpringDamperForce(c1,d1,l01));
  spring1->connect(box1->getFrame("P1"),getFrame("I"));

  // ----------------------- Definition der 2. Feder --------------------  
  SpringDamper *spring2 = new SpringDamper("Feder2");
  addLink(spring2);
  spring2->setForceFunction(new LinearSpringDamperForce(c2,d2,l02));
  spring2->connect(box1->getFrame("P2"),box2->getFrame("P1"));

  // ----------------------- Anfangsbedingungen der Körper -------------------  
  box1->setInitialGeneralizedPosition(Vec(1,INIT,l01 + h1/2 + 0.2));
  box2->setInitialGeneralizedPosition(Vec(1,INIT,l01 + l02 + h1 + h2/2));

#ifdef HAVE_OPENMBVCPPINTERFACE
  // ----------------------- Visualisierung in OpenMBV --------------------  
  OpenMBV::Cube *cuboid=new OpenMBV::Cube;
  cuboid->setLength(h1);
  cuboid->setStaticColor(0.5);
  box1->setOpenMBVRigidBody(cuboid);

  cuboid=new OpenMBV::Cube;
  cuboid->setLength(h2);
  cuboid->setStaticColor(0.8);
  box2->setOpenMBVRigidBody(cuboid);

  OpenMBV::CoilSpring* openMBVspring1=new OpenMBV::CoilSpring;
  openMBVspring1->setSpringRadius(0.1);
  openMBVspring1->setCrossSectionRadius(0.01);
  openMBVspring1->setNumberOfCoils(5);
  openMBVspring1->setStaticColor(0.);
  spring1->setOpenMBVSpring(openMBVspring1);

  OpenMBV::CoilSpring* openMBVspring2=new OpenMBV::CoilSpring;
  openMBVspring2->setSpringRadius(0.1);
  openMBVspring2->setCrossSectionRadius(0.01);
  openMBVspring2->setNumberOfCoils(5);
  spring2->setOpenMBVSpring(openMBVspring2);
#endif

}

