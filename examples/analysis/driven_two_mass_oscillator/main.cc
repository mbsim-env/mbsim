#include "system.h"
#include "mbsim/functions/constant_function.h"
#include "mbsimControl/linear_system_analyzer.h"

using namespace std;
using namespace MBSim;
using namespace MBSimControl;
using namespace fmatvec;

double f = 2;

int main (int argc, char* argv[]) {

  System *sys = new System("TS");

  sys->setDetermineEquilibriumState(true);
  sys->initialize();

  LinearSystemAnalyzer analyzer;
  Vec z0(sys->getzSize());
  Vec freq(50);
  for(int i=0; i<freq.size(); i++)
    freq(i) = 0.1 + i*0.1;
  analyzer.setExcitationFrequencies(freq);
  analyzer.setExcitationAmplitudeFunction(new ConstantFunction<VecV(double)>(10));
  analyzer.visualizeFrequencyResponse(_frequencyRange="[1.5;2]");
  analyzer.setSystem(sys);
  analyzer.execute();

  delete sys;

  return 0;
}

