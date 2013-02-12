#include "system.h"
#include <mbsim/integrators/integrators.h>
#include <mbsim/utils/stopwatch.h>

using namespace std;
using namespace MBSim;

int main (int argc, char* argv[]) {

  StopWatch sw;

  //firstRun is a attempt to avoid the time addition that comes with first time accesses on memory and stuff
  //TODO: attempt does not work properly...
  bool firstRun = true;

  for(int i = 0; i <= 2; i++) {
    sw.start();

    stringstream ss;
    ss << "TS";
    switch(i) {
      case 0:
        ss << "Maxwell";
        break;
      case 1:
        ss << "Regularized";
        break;
      default:
        ss << "Rigid";
        break;
    }

    System *sys;
    if(firstRun)
      sys = new System("FirstRun", i, 1);
    else
      sys = new System(ss.str(), i, 3);

    //  sys->setImpactSolver(RootFinding);
    //  sys->setConstraintSolver(RootFinding);
    //  sys->setLinAlg(PseudoInverse);
    sys->setNumJacProj(true);
    sys->initialize();

    TimeSteppingIntegrator integrator;
    integrator.setStepSize(1e-4);

    integrator.setEndTime(.39);
    integrator.setPlotStepSize(1e-3);

    integrator.integrate(*sys);


    if(firstRun) {
      firstRun = false;
      i--;
    }


    ofstream filestream;
    if(i == 0)
      filestream.open("Info.txt", ios::out);
    else
      filestream.open("Info.txt", ios::app);

    filestream << "*********************" << endl;
    filestream << "Example: " << ss.str() << endl;
    filestream << " Time = " << sw.stop() << " s " << endl;
    filestream << "*********************" << endl;
    filestream << "finished"<<endl;

    filestream.close();

    delete sys;
  }

  return 0;
}

