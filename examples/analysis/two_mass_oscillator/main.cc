#include "system.h"
#include <mbsimControl/linear_system_analyzer.h>
#include "mbsim/utils/eps.h"

using namespace std;
using namespace MBSim;
using namespace MBSimControl;
using namespace fmatvec;

int main (int argc, char* argv[]) {

  System *sys = new System("TS");

  sys->setDetermineEquilibriumState(true);
  sys->initialize();

  LinearSystemAnalyzer analyzer;
  analyzer.visualizeNormalModes();
  analyzer.setSystem(sys);
  analyzer.execute();

  delete sys;

  return 0;
}

