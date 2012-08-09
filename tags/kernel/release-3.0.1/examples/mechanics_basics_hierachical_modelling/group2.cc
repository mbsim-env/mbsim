#include "group2.h"
#include "group1.h"
#include "mbsim/rigid_body.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/frame.h"
#include <openmbvcppinterface/cuboid.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

Group2::Group2(const string &name) : Group(name) {
  // Parameter der Körper
  double m1 = 5;
  SymMat Theta1(3,EYE);
  SymMat Theta2(3,EYE);
#ifdef HAVE_OPENMBVCPPINTERFACE
  double h1 = 0.5;
#endif

  // ----------------------- Definition des 1. Körpers --------------------  
  RigidBody *box1 = new RigidBody("Box1");
  box1->setPlotFeature(globalPosition, enabled);
  addObject(box1);

  // Masse und Trägheit definieren
  box1->setMass(m1);
  box1->setInertiaTensor(Theta1);

  // Kinematik: Bewegung des Schwerpunktes (Center of Gravity C) 
  // entlang der y-Richtung ausgehend vom I-System (Ursprung O)
  box1->setTranslation(new LinearTranslation("[0; 1; 0]"));
  box1->setFrameOfReference(getFrame("I"));
  box1->setFrameForKinematics(box1->getFrame("C"));
  box1->setInitialGeneralizedVelocity("[0.1]");

  Group1 *group = new Group1("Untergruppe");
  Vec r(3);
  r(0) = 1;
  SqrMat A(3);
  double a = M_PI/4;
  A(0,0) = cos(a);
  A(1,1) = cos(a);
  A(2,2) = 1;
  A(0,1) = sin(a);
  A(1,0) = -sin(a);
  group->setPosition(r);
  group->setOrientation(A);
  addGroup(group);


#ifdef HAVE_OPENMBVCPPINTERFACE
  OpenMBV::Cuboid* body1=new OpenMBV::Cuboid;
  body1->setLength(Vec(3,INIT,1)*h1);
  box1->setOpenMBVRigidBody(body1);
  box1->getFrame("C")->enableOpenMBV();
#endif


}
