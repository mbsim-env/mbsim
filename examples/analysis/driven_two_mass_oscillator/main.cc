#include "system.h"
#include <mbsim/analyzers/harmonic_response_analyzer.h>
#include "mbsim/utils/eps.h"

using namespace std;
using namespace MBSim;
using namespace MBSimAnalyzer;
using namespace fmatvec;

double f = 2;

int main (int argc, char* argv[]) {

  System *sys = new System("TS");

  sys->initialize();

  HarmonicResponseAnalyzer analyzer;
  Vec z0(sys->getzSize());
  analyzer.setDetermineEquilibriumState(true);
  Vec freq(50);
  for(int i=0; i<freq.size(); i++)
    freq(i) = 0.1 + i*0.1;
  analyzer.setExcitationFrequencies(freq);
  analyzer.setSystemFrequencies(VecV(1,INIT,f));
  analyzer.setSystem(sys);
  analyzer.execute();

  delete sys;

  return 0;
}

