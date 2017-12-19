#include "system.h"
#include <mbsim/analyzers/eigenanalyzer.h>
#include "mbsim/utils/eps.h"
#include "mbsim/integrators/integrators.h"

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;
using namespace MBSimAnalyzer;
using namespace fmatvec;

int main (int argc, char* argv[]) {
  double g = 9.81;
  double a = 5;
  double theta0 = 20;
  double psid = sqrt(g/(a*cos(theta0/180*M_PI)));

  System *sys = new System("TS");

  sys->initialize();

  DOPRI5Integrator integrator;
  integrator.setEndTime(5.0);
  integrator.setPlotStepSize(1e-2);
  integrator.integrate(*sys);

  cout << "Analyse planar motion" << endl;
  Eigenanalyzer analyzer;
  analyzer.setOutputFileName("Eigenanalysis1.mat");
  analyzer.setEndTime(4.5);
  Vec z0(sys->getzSize());
  z0(1) = 1.0/180*M_PI;
  analyzer.setInitialDeviation(z0);
  Vec zEq(sys->getzSize());
  analyzer.setInitialState(zEq);
  analyzer.setAmplitude(0);
  analyzer.setSystem(sys);
  analyzer.execute();

  cout << "Eigenfrequency is " << analyzer.getEigenvalues()(0).imag();
  cout << " (should be " << sqrt(g/a) << ")" << endl;

  cout << "Analyse cone-pendel" << endl;
  analyzer.setOutputFileName("Eigenanalysis2.mat");
  z0.init(0);
  zEq.init(0);
  zEq(1) = theta0/180*M_PI;
  zEq(2) = psid;
  analyzer.setInitialDeviation(z0);
  analyzer.setInitialState(zEq);
  analyzer.setSystem(sys);
  analyzer.execute();

  cout << "Eigenfrequency is " << analyzer.getEigenvalues()(0).imag();
  cout << " (should be " << sqrt(g/a*(1./cos(theta0/180*M_PI)+3*cos(theta0/180*M_PI))) << ")" << endl;

  delete sys;

  return 0;
}

