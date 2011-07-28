#include "system.h"
#include <mbsim/integrators/integrators.h>

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace MBSim;
using namespace std;

int main(int argc, char* argv[]) {
#ifdef _OPENMP 
    omp_set_num_threads(1);
#endif

  System *sys = new System("MBS");

  sys->setStopIfNoConvergence(true,true);
  sys->setMaxIter(100000); // set up to 100000 because of "No Convergence" in only ONE step
  sys->initialize();

  TimeSteppingIntegrator integrator;

  integrator.setEndTime(5e-4);
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

