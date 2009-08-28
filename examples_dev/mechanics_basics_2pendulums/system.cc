#include "system.h"
#include "mbsim/rigid_body.h"
#include "pendulum.h"
#include "mbsim/environment.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  Vec grav(3);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  Tree* tree = new Tree("Master");
  addGroup(tree);

  Pendulum *pendel1 = new Pendulum("Pendel1"); 
  Node* node = tree->addTree(0,pendel1);

  Vec x(3);
  x(0) = 0.15;

  SqrMat A(3);
  for(int i=0; i<3; i++)
    A(i,i) = 1;

  pendel1->getRod2()->addFrame("P",x,A,pendel1->getRod2()->getFrame("R"));

  Pendulum *pendel2 = new Pendulum("Pendel2"); 
  tree->addTree(node,pendel2);
  pendel2->getRod1()->setFrameOfReference(pendel1->getRod2()->getFrame("P"));
}

