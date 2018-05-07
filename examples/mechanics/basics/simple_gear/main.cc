#include "system.h"
#include <boost/mpl/set/set30.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/order.hpp>
#include <mbsim/integrators/boost_odeint_integrator_predef.h>
#include <mbsim/integrators/daspk_integrator.h>
#include <mbsim/integrators/dop853_integrator.h>
#include <mbsim/integrators/dopri5_integrator.h>
#include <mbsim/integrators/explicit_euler_integrator.h>
#include <mbsim/integrators/hets2_integrator.h>
#include <mbsim/integrators/implicit_euler_integrator.h>
#include <mbsim/integrators/lsoda_integrator.h>
#include <mbsim/integrators/lsode_integrator.h>
#include <mbsim/integrators/lsodi_integrator.h>
#include <mbsim/integrators/odex_integrator.h>
#include <mbsim/integrators/radau5_integrator.h>
#include <mbsim/integrators/radau_integrator.h>
#include <mbsim/integrators/rksuite_integrator.h>
#include <mbsim/integrators/rodas_integrator.h>
#include <mbsim/integrators/seulex_integrator.h>
#include <mbsim/integrators/phem56_integrator.h>
#include <mbsim/integrators/theta_time_stepping_integrator.h>
#include <mbsim/integrators/time_stepping_integrator.h>
#include <mbsim/integrators/time_stepping_ssc_integrator.h>

using namespace std;
using namespace MBSim;
using namespace MBSimIntegrator;

class Integrate {
  public:
    template<typename Int>
    void operator()(Int& integrator);
};

typedef boost::mpl::set22<
  BoostOdeintDOS_RKDOPRI5,
  BoostOdeintDOS_BulirschStoer,
  BoostOdeintDOS_Rosenbrock4,
  DASPKIntegrator,
  DOP853Integrator,
  DOPRI5Integrator,
  ExplicitEulerIntegrator,
  HETS2Integrator,
  ImplicitEulerIntegrator,
  LSODAIntegrator,
  LSODEIntegrator,
  LSODIIntegrator,
  ODEXIntegrator,
  RADAU5Integrator,
  RADAUIntegrator,
  RKSuiteIntegrator,
  RODASIntegrator,
  SEULEXIntegrator,
  PHEM56Integrator,
  ThetaTimeSteppingIntegrator,
  TimeSteppingIntegrator,
  TimeSteppingSSCIntegrator
> Integrators;

int main (int argc, char* argv[]) {
  boost::mpl::for_each<Integrators>(Integrate());    

  return 0;

}

template<typename Int>
void Integrate::operator()(Int& integrator) {
  string typeStr(typeid(Int).name());
  int order=boost::mpl::order<Integrators,Int>::type::value;

  DynamicSystemSolver *sys = new System("MBS_"+to_string(order));
  sys->initialize();
  
  // Integration
  cout << "integrate using "<<typeStr<<" = MBS_"<<order<<endl;
  integrator.setEndTime(10.0);
  integrator.setPlotStepSize(1e-2);
  integrator.setSystem(sys);
  integrator.integrate();

  cout << endl;
  cout << "finished using "<<typeStr<<" = MBS_"<<order<<endl;

  delete sys;
}
