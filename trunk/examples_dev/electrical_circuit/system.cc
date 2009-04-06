#include "system.h"
#include "my_circuit.h"

using namespace AMVis;
using namespace fmatvec;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
  setProjectDirectory("plot");

  // Erdbeschleungigung definieren
  Vec g(3);
  g(1)=-9.81*0;
  setAccelerationOfGravity(g);

  MyCircuit *elnet = new MyCircuit("ElectricalCircuit");
  addDynamicSystem(elnet,Vec(3),SqrMat(3));

}
