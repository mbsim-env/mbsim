#include "system.h"
#include <mbsim/analysers/eigenanalyser.h>
#include "mbsim/utils/eps.h"
#include "mbsim/integrators/integrators.h"

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;
using namespace MBSimAnalyser;
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
  Eigenanalyser analyser;
  analyser.setOutputFileName("Eigenanalysis1.mat");
  analyser.setEndTime(4.5);
  Vec z0(sys->getzSize());
  z0(1) = 1.0/180*M_PI;
  analyser.setInitialDeviation(z0);
  Vec zEq(sys->getzSize());
  analyser.setInitialState(zEq);
  analyser.setAmplitude(0);
  analyser.setSystem(sys);
  analyser.execute();

  cout << "Eigenfrequency is " << analyser.getEigenvalues()(0).imag();
  cout << " (should be " << sqrt(g/a) << ")" << endl;

  cout << "Analyse cone-pendel" << endl;
  analyser.setOutputFileName("Eigenanalysis2.mat");
  z0.init(0);
  zEq.init(0);
  zEq(1) = theta0/180*M_PI;
  zEq(2) = psid;
  analyser.setInitialDeviation(z0);
  analyser.setInitialState(zEq);
  analyser.setSystem(sys);
  analyser.execute();

  cout << "Eigenfrequency is " << analyser.getEigenvalues()(0).imag();
  cout << " (should be " << sqrt(g/a*(1./cos(theta0/180*M_PI)+3*cos(theta0/180*M_PI))) << ")" << endl;

  delete sys;

  return 0;
}

