#include "system.h"
#include "mbsimElectronics/simulation_classes.h"

using namespace fmatvec;
using namespace std;
using namespace MBSimElectronics;

using namespace std;
double mod(double x, double y) {
 return x - y * floor (x / y);
}

class SwitchSignal : public MBSim::UserFunction {
  protected:
    double T,u1,uref,K,uu;
    Resistor *resistor;
  public:
    SwitchSignal(Resistor *res) : resistor(res) {
      T = 4e-4;
      u1 = 3.8;
      uu = 8.2;
      uref = 11.3;
      K=8.2;
    }
    fmatvec::Vec operator()(double t) {
      fmatvec::Vec U(1);
      double UR = -resistor->computeU(t);
      double ug = u1+mod(t,T)/T*(uu-u1);
      double h = uref+1/K*ug;
      if(-UR <= h)
	U(0) = 0;
      else
	U(0) = 100;

      return U;
    }
};

// Just a test voltage signal
class Signal : public MBSim::UserFunction {
  public:
    fmatvec::Vec operator()(double t) {
      fmatvec::Vec U(1);
      U(0) = 35;
      return U;
    }
};

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  VoltageSource *voltageSource = new VoltageSource("VoltageSource");
  addModel(voltageSource);
  voltageSource->setVoltageSignal(new Signal);

  Inductor *inductorL = new Inductor("InductorL");
  inductorL->setInductance(0.0001);
  addModel(inductorL);

  Switch *eswitch = new Switch("Switch");
  addModel(eswitch);

  Diode *diode = new Diode("Diode");
  addModel(diode);

  connectTerminal(voltageSource->getTerminal("B"),inductorL->getTerminal("A"));
  connectTerminal(inductorL->getTerminal("B"),eswitch->getTerminal("A"));
  connectTerminal(eswitch->getTerminal("B"),diode->getTerminal("A"));
  connectTerminal(diode->getTerminal("B"),voltageSource->getTerminal("A"));

  Inductor* inductorM = new Inductor("InductorM");
  inductorM->setInductance(0.02);
  addModel(inductorM);

  Capacitor *capacitor = new Capacitor("Capacitor");
  addModel(capacitor);
  capacitor->setCapacity(0.000047);
 
  connectTerminal(eswitch->getTerminal("B"),inductorM->getTerminal("A"));
  connectTerminal(inductorM->getTerminal("B"),capacitor->getTerminal("A"));
  connectTerminal(capacitor->getTerminal("B"),diode->getTerminal("B"));

  Inductor* inductorR = new Inductor("InductorR");
  inductorR->setInductance(0.0001);
  addModel(inductorR);

  Resistor *resistor = new Resistor("Resistor");
  resistor->setResistance(22);
  addModel(resistor);

  connectTerminal(inductorM->getTerminal("B"),inductorR->getTerminal("A"));
  connectTerminal(inductorR->getTerminal("B"),resistor->getTerminal("A"));
  connectTerminal(resistor->getTerminal("B"),capacitor->getTerminal("B"));

  eswitch->setVoltageSignal(new SwitchSignal(resistor));
}


