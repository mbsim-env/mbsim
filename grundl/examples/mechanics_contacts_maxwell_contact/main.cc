#include <mbsim/dynamic_system_solver.h>
#include <mbsim/integrators/time_stepping_integrator.h>

#include "system.h"

#include <boost/timer.hpp>

using namespace MBSim;
using namespace fmatvec;
using namespace std;

int main(int argc, char *argv[]) {

  DynamicSystemSolver *sys;
  TimeSteppingIntegrator *integrator;
  double stepSize = 1e-5;
  double plotStepSize = 1 * stepSize;
  double endTime = 5e-2;

  for (int contactType =0; contactType <= 0; contactType++) {
    for (int contactNums = 1; contactNums <= 1; contactNums+=20) {
      stringstream MBSName;
      MBSName << "MBS_";

      switch (contactType) {
        case 0:
          MBSName << "Maxwell";
          break;
        case 1 :
          MBSName << "Regularized";
          break;
        case 2 :
          MBSName << "Unilateral";
          break;
        default:
          throw MBSimError("No valid contactType chosen");
      }

      MBSName << "_CircNo=" << contactNums;

      //stream output to a file
//      freopen((MBSName.str()+"_Output.txt").c_str(), "w+", stdout);

      Vec shift(3,INIT,0.);

      switch(contactType) {
        case 0:
          break;
        case 1:
          shift(2) = 0.2;
          break;
        case 2:
          shift(2) = 0.4;
          break;
        default:
          throw MBSimError("No valid contactType chosen");
      }

      sys = new System(MBSName.str(), contactType, 0, contactNums, shift);

      sys->setLinAlg(PseudoInverse);
      sys->initialize();

      integrator = new TimeSteppingIntegrator;

      integrator->setStepSize(stepSize);
      integrator->setPlotStepSize(plotStepSize);
      integrator->setEndTime(endTime);

      boost::timer timer;

      integrator->integrate(*sys);

      double elapsedIntegrationTime = timer.elapsed();

      sys->closePlot();

      delete sys;
      delete integrator;

      cout << "******** Results of speed are:****************" << endl;
      cout << "*  Time elapsed: " << elapsedIntegrationTime << endl;
      cout << "**********************************************" << endl;

      if(contactNums == 1)
        contactNums--;
    }
  }

  cout << "finished" << endl;
  return 0;

}
