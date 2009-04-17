#include "my_circuit.h"
#include "modeling_classes.h"

using namespace std;

MyCircuit::MyCircuit(const string &name) : ElectricalCircuit(name) {

  Resistor *resistor = new Resistor("Resistor");
  addComponent(resistor);
  Inductor *inductorL = new Inductor("InductorL");
  inductorL->setInductance(0.1);
  addComponent(inductorL);
  VoltageSource *voltageSource = new VoltageSource("VoltageSource");
  addComponent(voltageSource);
  voltageSource->setVoltageSignal(new Signal);

  connectPin(voltageSource->getPin("B"),inductorL->getPin("A"));
  connectPin(inductorL->getPin("B"),resistor->getPin("A"));
  connectPin(resistor->getPin("B"),voltageSource->getPin("A"));

  Inductor* inductorR = new Inductor("InductorR");
  inductorR->setInductance(0.2);
  addComponent(inductorR);
  connectPin(inductorL->getPin("B"),inductorR->getPin("A"));

  Capacitor *capacitor = new Capacitor("Capacitor");
  addComponent(capacitor);
  capacitor->setCapacity(10);

  connectPin(inductorR->getPin("B"),capacitor->getPin("A"));
  connectPin(capacitor->getPin("B"),resistor->getPin("B"));
}
