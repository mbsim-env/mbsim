#include "group1.h"
#include "mbsim/rigid_body.h"
#include "mbsim/linear_spring_damper.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/coilspring.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

Group1::Group1(const string &name) : Group(name) {
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

  // Kinematik: Bewegung des Schwerpunktes (Center of Gravity C) 
  // entlang der y-Richtung ausgehend vom I-System (Ursprung O)
  box1->setTranslation(new LinearTranslation("[0; 1; 0]"));
  box1->setFrameOfReference(getFrame("I"));
  box1->setFrameForKinematics(box1->getFrame("C"));
  box1->setFrameOfReference(getFrame("I"));


  // ----------------------- Definition des 2. Körpers --------------------  
  RigidBody *box2 = new RigidBody("Box2");
  box2->setPlotFeature(stateDerivative, disabled);
  //box2->setPlotFeatureForChildren(plotRecursive, disabled);
  addObject(box2);

  // Masse und Trägheit definieren
  box2->setMass(m2);
  box2->setInertiaTensor(Theta2);

  // Kinematik: Bewegung des Schwerpunktes (Center of Gravity C) 
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
  LinearSpringDamper *spring1 = new LinearSpringDamper("Feder1");
  addLink(spring1);
  spring1->setStiffness(c1);
  spring1->setDamping(d1);
  spring1->setUnloadedLength(l01);
  spring1->connect(box1->getFrame("P1"),getFrame("I"));

  // ----------------------- Definition der 2. Feder --------------------  
  LinearSpringDamper *spring2 = new LinearSpringDamper("Feder2");
  addLink(spring2);
  spring2->setStiffness(c2);
  spring2->setDamping(d2);
  spring2->setUnloadedLength(l02);
  spring2->connect(box1->getFrame("P2"),box2->getFrame("P1"));

  // ----------------------- Anfangsbedingungen der Körper -------------------  
  box1->setq0(Vec(1,INIT,l01 + h1/2 + 0.2));
  box2->setq0(Vec(1,INIT,l01 + l02 + h1 + h2/2));

#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cuboid* body1=new OpenMBV::Cuboid;
  body1->setLength(Vec(3,INIT,1)*h1);
  box1->setOpenMBVRigidBody(body1);
  box1->getFrame("P1")->enableOpenMBV(0.5);

  OpenMBV::Cuboid* body2=new OpenMBV::Cuboid;
  body2->setLength(Vec(3,INIT,1)*h2);
  box2->setOpenMBVRigidBody(body2);
  box2->getFrame("P1")->enableOpenMBV(0.5);

  OpenMBV::CoilSpring* openMBVspring1=new OpenMBV::CoilSpring;
  openMBVspring1->setSpringRadius(0.1);
  openMBVspring1->setCrossSectionRadius(0.01);
  openMBVspring1->setNumberOfCoils(5);
  spring1->setOpenMBVSpring(openMBVspring1);

  OpenMBV::CoilSpring* openMBVspring2=new OpenMBV::CoilSpring;
  openMBVspring2->setSpringRadius(0.1);
  openMBVspring2->setCrossSectionRadius(0.01);
  openMBVspring2->setNumberOfCoils(5);
  spring2->setOpenMBVSpring(openMBVspring2);
#endif
}
