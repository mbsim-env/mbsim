#include <mbsim/dynamic_system_solver.h>
#include <mbsim/integrators/time_stepping_integrator.h>

#include "system.h"

#include <boost/timer.hpp>

using namespace MBSim;
using namespace std;

int main(int argc, char *argv[]) {

  DynamicSystemSolver *sys;
  TimeSteppingIntegrator *integrator;
  double stepSize = 1e-5;
  double plotStepSize = 1e-5;//1000 * stepSize;
  double endTime = 5e-2;

  for (int contactType = 0; contactType <= 0; contactType++) {
    for (int circleNums = 6; circleNums <= 66; circleNums+=5) {
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

      MBSName << "_CircNo=" << circleNums;

      //stream output to a file
      //freopen((MBSName.str()+"_Output.txt").c_str(), "w+", stdout);

      sys = new System(MBSName.str(), contactType, circleNums);

      sys->setLinAlg(PseudoInverse);
      sys->initialize();

      integrator = new TimeSteppingIntegrator;

      integrator->setStepSize(stepSize);
      integrator->setPlotStepSize(plotStepSize);
      integrator->setEndTime(endTime);

      boost::timer timer;

      integrator->integrate(*sys);

      double elapsedIntegrationTime = timer.elapsed();

      cout << "finished" << endl;
      delete sys;
      delete integrator;

      cout << "******** Results of speed are:****************" << endl;
      cout << "*  That makes a difference of: " << elapsedIntegrationTime << endl;
      cout << "**********************************************" << endl;

      if(circleNums == 1)
        circleNums--;
    }
  }

  cout << "finished" << endl;
  return 0;

}
