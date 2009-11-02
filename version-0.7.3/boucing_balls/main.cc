#include "bouncing_ball.h"
#include <integrators.h>
#include <sstream>

using namespace std;

int main (int argc, char* argv[]) {
  Vec mu("[0.01;0.02;0.04;0.08;0.16;0.32;0.64]");

  for(int i=0;i<mu.size();i++) {
    stringstream name;
    name << "Ball_" << mu(i);
    BouncingBall sys(name.str());
    sys.setProjectDirectory("plot");
    sys.setmu(mu(i));
    sys.init();

    TimeSteppingIntegrator integrator;
    integrator.setdt(5e-4);
    integrator.settEnd(3);
    integrator.setdtPlot(5e-4);

    integrator.integrate(sys);

  }
  cout << "finished"<<endl;
  return 0;
}

