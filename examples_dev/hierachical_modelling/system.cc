#include "system.h"
#include "group2.h"
#include "mbsim/load.h"
#include "mbsim/frame.h"

using namespace MBSim;
using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  setProjectDirectory("plot");

  // Erdbeschleungigung definieren
  Vec g(3);
  g(1)=-9.81*0;
  setAccelerationOfGravity(g);

  Group2 *group1 = new Group2("Hauptgruppe1");
  addDynamicSystem(group1,Vec(3),SqrMat(3,EYE));

  Group2 *group2 = new Group2("Hauptgruppe2");
  group2->setPlotFeatureRecursive(stateDerivative, enabled);
  group2->setPlotFeature(separateFilePerDynamicSystem, enabled);
  Vec r(3);
  r(0) = 2;
  SqrMat A(3);
  double a = M_PI/4;
  A(0,0) = cos(a);
  A(1,1) = cos(a);
  A(2,2) = 1;
  A(0,1) = sin(a);
  A(1,0) = -sin(a);
  addDynamicSystem(group2,r,A);

  //cout << getSubsystem("Hauptgruppe2")->getSubsystem("Hauptgruppe2_Untergruppe")->getName()<<endl;;
  //cout << findFrame("TS.Hauptgruppe2.Hauptgruppe2_Untergruppe.Box1.P2")->getName()<<endl;

}
