#include "system.h"
#include "mbsimElectronics/simulation_classes.h"
#include "mbsimControl/signal_.h"
#include "mbsimControl/function_sensor.h"
#include "mbsimControl/sensor.h"
#include "mbsimControl/signal_function.h"

using namespace fmatvec;
using namespace std;
using namespace MBSimElectronics;
using namespace MBSimControl;
using namespace MBSim;

extern bool setValued;

using namespace std;
double mod(double x, double y) {
 return x - y * floor (x / y);
}

class VoltageSensor : public MBSimControl::Sensor {
  protected:
    MBSimElectronics::ElectronicLink *comp;
  public:
    void setComponent(MBSimElectronics::ElectronicLink *comp_) {comp = comp_;}
    VoltageSensor(const std::string &name) : Sensor(name) { }
    void updateSignal() {
      s = comp->evalGeneralizedForce();
      upds = false;
    }
    int getSignalSize() const { return comp->getGeneralizedForceSize(); }
};


class SwitchSignal : public MBSimControl::Signal {
  protected:
    double T,u1,uref,K,uu;
    Signal *inputSignal;
    double h;
  public:
    SwitchSignal(const std::string &name) : Signal(name), h(0) {
      T = 4e-4;
      u1 = 3.8;
      uu = 8.2;
      uref = 11.3;
      K=8.2;
    }
    void updateSignal() {
      double ug = u1+mod(getTime(),T)/T*(uu-u1);
      h = uref+1./K*ug;
      double UR = inputSignal->evalSignal()(0);
      s = VecV(1, INIT, (-UR <= h ? 0 : 100));
      upds = false;
    }
    void setInputSignal(Signal *input) {inputSignal = input;}
    int getSignalSize() const { return 1; }
};

class Signal : public MBSim::Function<fmatvec::VecV(double)> {
  public:
    std::pair<int, int> getRetSize() const { return std::make_pair(1,1); }
    fmatvec::VecV operator()(const double &t) {
      fmatvec::VecV U(1);
      U(0) = 35;
      return U;
    }
};

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  VoltageSource *voltageSource = new VoltageSource("VoltageSource");
  addModel(voltageSource);
  voltageSource->setVoltageSignal(new ::Signal);

  Inductor *inductorL = new Inductor("InductorL");
  inductorL->setInductance(0.0001);
  addModel(inductorL);

  Switch *eswitch = new Switch("Switch");
  eswitch->setSetValued(setValued);
  addModel(eswitch);

  Diode *diode = new Diode("Diode");
  diode->setSetValued(setValued);
  addModel(diode);

  connectTerminal(voltageSource->getTerminal("A"),inductorL->getTerminal("A"));
  connectTerminal(inductorL->getTerminal("B"),eswitch->getTerminal("A"));
  connectTerminal(eswitch->getTerminal("B"),diode->getTerminal("A"));
  connectTerminal(diode->getTerminal("B"),voltageSource->getTerminal("B"));

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

  VoltageSensor *vs = new VoltageSensor("VoltageSensor");
  vs->setComponent(resistor);
  addLink(vs);

  SwitchSignal *sw = new SwitchSignal("SwitchSignal");
  addLink(sw);
  sw->setInputSignal(vs);
  eswitch->setVoltageSignal(new SignalFunction<VecV(double)>(sw));

  setPlotFeatureRecursive(current, true);
  setPlotFeatureRecursive(charge, true);
  setPlotFeatureRecursive(voltage, true);
  setPlotFeatureRecursive(generalizedForce, true);
  setPlotFeatureRecursive(signal, true);
}


