#include "system.h"
#include <mbsim/integrators/integrators.h>

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace MBSim;
using namespace std;

int main(int argc, char* argv[]) {

  DynamicSystemSolver *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->setConstraintSolver(RootFinding);
  sys->setImpactSolver(RootFinding);
  sys->setLinAlg(PseudoInverse);
  sys->initialize();

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(1e-3);
  integrator.setStepSize(1e-6);
  integrator.setPlotStepSize(1e-4);

#ifdef _OPENMP
  double start = omp_get_wtime();
#endif
  integrator.integrate(*sys);
#ifdef _OPENMP
  double elapsed = omp_get_wtime() -start;
  cout << "CPU-Time = " << elapsed << endl;
#endif

  sys->closePlot();

  cout << "finished"<<endl;

  delete sys;

  return 0;

}

