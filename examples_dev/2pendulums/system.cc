#include "system.h"
#include "body_rigid.h"
#include "objobject.h"
#include "pendulum.h"

using namespace AMVis;

System::System(const string &projectName) : MultiBodySystem(projectName) {

  setProjectDirectory("plot");

  Vec grav(3);
  grav(1)=-9.81;
  setAccelerationOfGravity(grav);

  Tree* tree = new Tree("Master");
  addObject(tree);

  Pendulum *pendel1 = new Pendulum("Pendel1"); 
  tree->addObject(pendel1);

  pendel1->getRod1()->setParentCoordinateSystem(getCoordinateSystem("O"));
  Vec x(3);
  x(0) = 0.15;

  SqrMat A(3);
  for(int i=0; i<3; i++)
    A(i,i) = 1;

  pendel1->getRod2()->addCoordinateSystem("P",x,A,pendel1->getRod2()->getCoordinateSystem("R"));

  Pendulum *pendel2 = new Pendulum("Pendel2"); 
  tree->addObject(pendel2);
  pendel2->getRod1()->setParentCoordinateSystem(pendel1->getRod2()->getCoordinateSystem("P"));
}

