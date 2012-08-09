#include "system.h"
#include "mbsimElectronics/simulation_classes.h"
#include <mbsim/utils/function.h>
#include "mbsimControl/function_sensor.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBSimElectronics;
//using namespace MBSimControl;
// Just a test voltage signal
class Signal : public Function1<fmatvec::Vec,double> {
  public:
    fmatvec::Vec operator()(const double &t, const void * =NULL) {
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
  MBSimControl::FunctionSensor * sensorVoltageSource = new MBSimControl::FunctionSensor("SensorVoltageSource");
  addLink(sensorVoltageSource);
  sensorVoltageSource->setFunction(new Signal);
  voltageSource->setVoltageSignal(sensorVoltageSource);

  connectTerminal(voltageSource->getTerminal("A"),inductorL->getTerminal("A"));
  connectTerminal(inductorL->getTerminal("B"),resistor->getTerminal("A"));
  connectTerminal(resistor->getTerminal("B"),voltageSource->getTerminal("B"));

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

