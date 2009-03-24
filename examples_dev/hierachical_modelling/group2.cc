#include "group2.h"
#include "group1.h"
#include "mbsim/rigid_body.h"
#include "springs.h"
#ifdef HAVE_AMVIS
#include "cube.h"
#include "coilspring.h"

using namespace AMVis;
#endif
#ifdef HAVE_AMVISCPPINTERFACE
#include <amviscppinterface/cuboid.h>
#endif

Group2::Group2(const string &name) : Group(name) {
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
  RigidBody *box1 = new RigidBody(name+"_Box1");
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
  box1->setu0("[0.1]");

  Group1 *group = new Group1(name+"_Untergruppe");
  Vec r(3);
  r(0) = 1;
  SqrMat A(3);
  double a = M_PI/4;
  A(0,0) = cos(a);
  A(1,1) = cos(a);
  A(2,2) = 1;
  A(0,1) = sin(a);
  A(1,0) = -sin(a);
  addSubsystem(group,r,A);


#ifdef HAVE_AMVIS
  {
  ostringstream os;
  os <<name<< "." << box1->getName();
  Cube * cuboid = new Cube(os.str(),1,false);
  cuboid->setLength(h1);
  cuboid->setColor(0.5);
  box1->setAMVisBody(cuboid);
  }
#endif
#ifdef HAVE_AMVISCPPINTERFACE
  AMVis::Cuboid* body1=new AMVis::Cuboid;
  body1->setLength(Vec(3,INIT,1)*h1);
  box1->setAMVisRigidBody(body1);
  box1->getFrame("C")->enableAMVis();
#endif


}
