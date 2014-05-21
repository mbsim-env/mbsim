#include "system.h"
#include <mbsim/analysis/eigenanalysis.h>
#include "mbsim/utils/eps.h"
#include "mbsim/integrators/integrators.h"

using namespace std;
using namespace MBSim;
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
  Eigenanalysis analysis;
  analysis.setOutputFileName("Eigenanalysis1.mat");
  analysis.setEndTime(4.5);
  Vec z0(sys->getzSize());
  z0(1) = 1.0/180*M_PI;
  analysis.setInitialDeviation(z0);
  Vec zEq(sys->getzSize());
  analysis.setEquilibriumState(zEq);
  analysis.analyse(*sys);

  cout << "Eigenfrequency is " << analysis.getEigenvalues()(0).imag();
  cout << " (should be " << sqrt(g/a) << ")" << endl;

  cout << "Analyse cone-pendel" << endl;
  analysis.setOutputFileName("Eigenanalysis2.mat");
  z0.init(0);
  zEq.init(0);
  zEq(1) = theta0/180*M_PI;
  zEq(2) = psid;
  analysis.setInitialDeviation(z0);
  analysis.setEquilibriumState(zEq);
  analysis.analyse(*sys);

  cout << "Eigenfrequency is " << analysis.getEigenvalues()(0).imag();
  cout << " (should be " << sqrt(g/a*(1./cos(theta0/180*M_PI)+3*cos(theta0/180*M_PI))) << ")" << endl;

  delete sys;

  return 0;
}

