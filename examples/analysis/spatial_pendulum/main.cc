#include "system.h"
#include <mbsimControl/linear_system_analyzer.h>
#include "mbsim/integrators/integrators.h"
#include <boost/filesystem.hpp>

using namespace std;
using namespace MBSim;
using namespace MBSimControl;
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
  LinearSystemAnalyzer analyzer;
  Vec z0(sys->getzSize());
  z0(1) = 1.0/180*M_PI;
  Vec zEq(sys->getzSize());
  analyzer.setInitialState(zEq);
  analyzer.setSystem(sys);
  analyzer.execute();

  cout << "Eigenfrequency should be " << sqrt(g/a) << endl;
  boost::filesystem::rename("linear_system_analysis.h5","linear_system_analysis1.h5");

  cout << "Analyse cone-pendel" << endl;
  z0.init(0);
  zEq.init(0);
  zEq(1) = theta0/180*M_PI;
  zEq(2) = psid;
  analyzer.setInitialState(zEq);
  analyzer.setSystem(sys);
  analyzer.execute();

  cout << "Eigenfrequency should be " << sqrt(g/a*(1./cos(theta0/180*M_PI)+3*cos(theta0/180*M_PI))) << endl;
  boost::filesystem::rename("linear_system_analysis.h5","linear_system_analysis2.h5");

  delete sys;

  return 0;
}

