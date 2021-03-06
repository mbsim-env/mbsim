#include "group1.h"
#include "mbsim/objects/rigid_body.h"
#include "mbsim/links/spring_damper.h"
#include "mbsim/functions/kinematics/kinematics.h"
#include "mbsim/functions/kinetics/kinetics.h"
#include "mbsim/frames/fixed_relative_frame.h"
#include <openmbvcppinterface/cuboid.h>
#include <openmbvcppinterface/coilspring.h>

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
  box1->setTranslation(new LinearTranslation<VecV>("[0; 1; 0]"));
  box1->setFrameOfReference(getFrame("I"));
  box1->setFrameForKinematics(box1->getFrame("C"));
  box1->setFrameOfReference(getFrame("I"));


  // ----------------------- Definition des 2. Körpers --------------------  
  RigidBody *box2 = new RigidBody("Box2");
  box2->setPlotFeature(derivativeOfGeneralizedPosition, false);
  box2->setPlotFeature(generalizedAcceleration, false);
  //box2->setPlotFeatureForChildren(plotRecursive, false);
  addObject(box2);

  // Masse und Trägheit definieren
  box2->setMass(m2);
  box2->setInertiaTensor(Theta2);

  // Kinematik: Bewegung des Schwerpunktes (Center of Gravity C) 
  // entlang der y-Richtung ausgehend vom I-System (Ursprung O)
  box2->setTranslation(new LinearTranslation<VecV>("[0; 1; 0]"));
  box2->setFrameOfReference(getFrame("I"));
  box2->setFrameForKinematics(box2->getFrame("C"));

  // ----------------------- Anschlusspunkte der Federn --------------------  
  Vec SrSP(3);
  SqrMat ASP(3,EYE);

  // Federanschlusspunkte P1 und P2 auf Körper 1 definieren
  SrSP(1) = h1/2.;
  box1->addFrame(new FixedRelativeFrame("P1",-SrSP,ASP));
  box1->addFrame(new FixedRelativeFrame("P2",SrSP,ASP));

  // Federanschlusspunkt P1 auf Körper 2 definieren
  SrSP(1) = h2/2.;
  box2->addFrame(new FixedRelativeFrame("P1",-SrSP,ASP));

  // ----------------------- Definition der 1. Feder --------------------  
  SpringDamper *spring1 = new SpringDamper("Feder1");
  addLink(spring1);
  spring1->setForceFunction(new LinearSpringDamperForce(c1,d1));
  spring1->setUnloadedLength(l01);
  spring1->connect(box1->getFrame("P1"),getFrame("I"));

  // ----------------------- Definition der 2. Feder --------------------  
  SpringDamper *spring2=new SpringDamper("Feder2");
  addLink(spring2);
  spring2->setForceFunction(new LinearSpringDamperForce(c2,d2));
  spring2->setUnloadedLength(l02);
  spring2->connect(box1->getFrame("P2"),box2->getFrame("P1"));

  // ----------------------- Anfangsbedingungen der Körper -------------------  
  box1->setGeneralizedInitialPosition(Vec(1,INIT,l01 + h1/2 + 0.2));
  box2->setGeneralizedInitialPosition(Vec(1,INIT,l01 + l02 + h1 + h2/2));

  std::shared_ptr<OpenMBV::Cuboid> body1=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  body1->setLength(vector<double>(3,h1));
  body1->setDiffuseColor(240./360.,1,1);
  box1->setOpenMBVRigidBody(body1);
  box1->getFrame("P1")->enableOpenMBV(0.5);

  std::shared_ptr<OpenMBV::Cuboid> body2=OpenMBV::ObjectFactory::create<OpenMBV::Cuboid>();
  body2->setLength(vector<double>(3,h2));
  body2->setDiffuseColor(360./360.,1,1);
  box2->setOpenMBVRigidBody(body2);
  box2->getFrame("P1")->enableOpenMBV(0.5);

  spring1->enableOpenMBV(_colorRepresentation=OpenMBVCoilSpring::absoluteForce,_springRadius=0.1,_crossSectionRadius=0.01,_numberOfCoils=5);

  spring2->enableOpenMBV(_colorRepresentation=OpenMBVCoilSpring::absoluteForce,_springRadius=0.1,_crossSectionRadius=0.01,_numberOfCoils=5);
}
