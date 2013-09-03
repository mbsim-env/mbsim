#include "system.h"
#include <mbsimInterface/interface_integrator.h>
#include <mbsimInterface/mbsim_server.h>
#include <fmatvec/fmatvec.h>
#include <string>

using namespace std;
using namespace MBSim;
using namespace fmatvec;

#include <sstream>

int main (int argc, char* argv[])
{
  // build single modules
  System *sys = new System("TS");

  // add modules to overall dynamical system 
  sys->initialize();

  MBSimInterface::InterfaceIntegrator integrator;
  integrator.setStartTime(-0.14e-2);
  integrator.setEndTime(10.0);
  integrator.setPlotStepSize(1e-3);
  MBSimInterface::MBSimTcpServer *m=new MBSimInterface::MBSimTcpServer(&integrator);
  m->setPort(4567);
  m->setOutputPrecision(18);
  integrator.setMBSimServer(m);

  integrator.integrate(*sys);
  cout << "finished"<<endl;

  delete sys;

  return 0;
}

