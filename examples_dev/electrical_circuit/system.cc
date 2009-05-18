#include "system.h"
#include "my_circuit.h"

using namespace fmatvec;
using namespace std;

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  MyCircuit *elnet = new MyCircuit("ElectricalCircuit");
  addDynamicSystem(elnet,Vec(3),SqrMat(3));

}

