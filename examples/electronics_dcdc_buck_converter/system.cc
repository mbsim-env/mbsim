#include "system.h"
#include "mbsimElectronics/simulation_classes.h"
#include "mbsimControl/signal_.h"
#include <mbsim/utils/function.h>
#include "mbsimControl/function_sensor.h"
#include "mbsimControl/sensor.h"

using namespace fmatvec;
using namespace std;
using namespace MBSimElectronics;
using namespace MBSim;

extern bool setValued;

using namespace std;
double mod(double x, double y) {
 return x - y * floor (x / y);
}

class VoltageSensor : public MBSimControl::Sensor {
  protected:
    MBSimElectronics::ElectronicLink * comp;
  public:
    void setComponent(MBSimElectronics::ElectronicLink *comp_) {comp = comp_;}
    VoltageSensor(const std::string &name) : Sensor(name) {}
    std::string getType() const { return "VoltageSensor"; }
    fmatvec::Vec getSignal(double t) {
      return Vec(1,INIT,comp->computeVoltage(t));
    }
};


class SwitchSignal : public MBSimControl::Signal {
  protected:
    double T,u1,uref,K,uu;
    Signal *inputSignal;
  public:
    SwitchSignal(const std::string &name) : Signal(name) {
      T = 4e-4;
      u1 = 3.8;
      uu = 8.2;
      uref = 11.3;
      K=8.2;
    }
    void setInputSignal(Signal *input) {inputSignal = input;}
    Vec getSignal(double t) {
      fmatvec::Vec U(1);
      double UR = inputSignal->getSignal(t)(0);
      double ug = u1+mod(t,T)/T*(uu-u1);
      double h = uref+1/K*ug;
      if(-UR <= h)
	U(0) = 0;
      else
	U(0) = 100;

      return U;
    }
};

class Signal : public Function1<fmatvec::Vec,double> {
  public:
    fmatvec::Vec operator()(const double &t, const void * =NULL) {
      fmatvec::Vec U(1);
      U(0) = 35;
      return U;
    }
};

System::System(const string &projectName) : DynamicSystemSolver(projectName) {

  VoltageSource *voltageSource = new VoltageSource("VoltageSource");
  addModel(voltageSource);
  //voltageSource->setVoltageSignal(new Signal);
  MBSimControl::FunctionSensor * fs = new MBSimControl::FunctionSensor("BasePositionSoll");
  addLink(fs);
  fs->setFunction(new Signal);
  voltageSource->setVoltageSignal(fs);

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
  eswitch->setVoltageSignal(sw);
}


