#include "system.h"
#include "pendulum.h"
#include "mbsim/environment.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  Vec grav(3,INIT,0.);
  grav(1)=-9.81;
  MBSimEnvironment::getInstance()->setAccelerationOfGravity(grav);

  Pendulum *pendel1 = new Pendulum("Pendel1"); 
  addGroup(pendel1);
  Vec x(3,INIT,0.);
  x(0) = 0.15;
  SqrMat A(3,EYE);
  pendel1->getRod2()->addFrame("P",x,A,pendel1->getRod2()->getFrame("R"));

  Pendulum *pendel2 = new Pendulum("Pendel2"); 
  addGroup(pendel2);
  pendel2->getRod1()->setFrameOfReference(pendel1->getRod2()->getFrame("P"));
}

