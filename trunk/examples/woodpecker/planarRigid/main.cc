#include "system.h"
#include "integrators.h"

using namespace std;

using namespace MBSim;

int main (int argc, char* argv[]) {

  System sys("WP");
//  sys.setSolver(GaussSeidel);
  sys.setSolver(RootFinding);
//  sys.setLinAlg(PseudoInverse);
  sys.init();

  TimeSteppingIntegrator integrator;
    // integrator.setHighIter(20);
//  integrator.setDecreaseLevel(Vector<int>(1,INIT,5000));
  integrator.setdt(5.e-5);

  integrator.settEnd(5.0);
  integrator.setdtPlot(1.e-3);
    // integrator.setWarnLevel(0); defautl: 0
    // integrator.setOutput(false); default: true

  integrator.integrate(sys);

  return 0;
}

