#include <mbsim/integrators/integrators.h>
#include <mbsim/dynamic_system_solver.h>
#include "mbsim/utils/rotarymatrices.h"

#include "mbsimHydraulics/environment.h"
#include "system.h"

#include <time.h>

using namespace std;
using namespace MBSim;
using namespace fmatvec;

int main (int argc, char* argv[]) {

  for (int i=0; i<2; i++) {
    string name;
    if (i==0)
      name = "RIGID";
    else
      name = "ELASTIC";

    DynamicSystemSolver * dss = new DynamicSystemSolver(name);
    dss->addDynamicSystem(new System("HS", (i==0)));

    HydraulicEnvironment::getInstance()->setProperties(800, 2e11, 12e-6, 1.3);

    dss->setConstraintSolver(GaussSeidel);
    dss->setImpactSolver(GaussSeidel);
    dss->setgdTol(1e-9);
    dss->init();

    double tEnd=.2;
    double dtPlot=1e-4;
    clock_t start, end;
    if (i==0) {
      TimeSteppingIntegrator integrator; integrator.setStepSize(5e-5);
      integrator.setEndTime(tEnd);
      integrator.setPlotStepSize(dtPlot);
      integrator.setOutput(true);
      start=clock();
      integrator.integrate(*dss);
      end=clock();
      cout << "Integration Time TimeStepping: " << double(end - start)/CLOCKS_PER_SEC << " s." << endl;
    }
    else {
      RADAU5Integrator integrator;
      integrator.setEndTime(tEnd);
      integrator.setPlotStepSize(dtPlot);
      integrator.setOutput(true);
      integrator.setMaximalStepSize(dtPlot);
      start=clock();
      integrator.integrate(*dss);
      end=clock();
      cout << "Integration Time Radau5: " << double(end - start)/CLOCKS_PER_SEC << " s." << endl;
    }

    dss->closePlot();
    delete dss;
  }

  return 0;
}


