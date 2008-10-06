#include "system.h"
#include "group2.h"
#include "load.h"
#include "coordinate_system.h"

using namespace AMVis;


System::System(const string &projectName) : MultiBodySystem(projectName) {
  setProjectDirectory("plot");

  // Erdbeschleungigung definieren
  Vec g(3);
  g(1)=-9.81*0;
  setAccelerationOfGravity(g);

  Group2 *group1 = new Group2("Hauptgruppe1");
  addObject(group1);
  //group1->setKinematicsCoordinateSystem(group1->getCoordinateSystem("O"));
  //group1->setReferenceCoordinateSystem(getCoordinateSystem("O"));

  Group2 *group2 = new Group2("Hauptgruppe2");
  Vec r(3);
  r(0) = 2;
  SqrMat A(3);
  double a = M_PI/4;
  A(0,0) = cos(a);
  A(1,1) = cos(a);
  A(2,2) = 1;
  A(0,1) = sin(a);
  A(1,0) = -sin(a);
  addObject(group2);
  //group2->setKinematicsCoordinateSystem(group2->getCoordinateSystem("O"));
  //group2->setReferenceCoordinateSystem(getCoordinateSystem("O"));
  group2->setTranslation(r);
  group2->setRotation(A);

  //cout << findCoordinateSystem("TS.Hauptgruppe2.Hauptgruppe2_Untergruppe.Box1.P2")->getName();

}
