#include "system.h"
#include "mbsim/userfunction.h"
#include "mbsimElectronics/simulation_classes.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimElectronics;
// Just a test voltage signal
class Signal : public MBSim::UserFunction {
  public:
    fmatvec::Vec operator()(double t) {
      fmatvec::Vec U(1);
      U(0) = 3;
      return U;
    }
};

System::System(const string &projectName) : DynamicSystemSolver(projectName) {
 Resistor *resistor = new Resistor("Resistor");
  addModel(resistor);
  Inductor *inductorL = new Inductor("InductorL");
  inductorL->setInductance(0.1);
  addModel(inductorL);
  VoltageSource *voltageSource = new VoltageSource("VoltageSource");
  addModel(voltageSource);
  voltageSource->setVoltageSignal(new Signal);

  connectTerminal(voltageSource->getTerminal("B"),inductorL->getTerminal("A"));
  connectTerminal(inductorL->getTerminal("B"),resistor->getTerminal("A"));
  connectTerminal(resistor->getTerminal("B"),voltageSource->getTerminal("A"));

  Inductor* inductorR = new Inductor("InductorR");
  inductorR->setInductance(0.2);
  addModel(inductorR);
  connectTerminal(inductorL->getTerminal("B"),inductorR->getTerminal("A"));

  Capacitor *capacitor = new Capacitor("Capacitor");
  addModel(capacitor);
  capacitor->setCapacity(1./10);

  connectTerminal(inductorR->getTerminal("B"),capacitor->getTerminal("A"));
  connectTerminal(capacitor->getTerminal("B"),resistor->getTerminal("B"));

}

