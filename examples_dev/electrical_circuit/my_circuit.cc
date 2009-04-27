#include "my_circuit.h"
#include "modeling_classes.h"

using namespace std;

MyCircuit::MyCircuit(const string &name) : ElectricalCircuit(name) {

  Resistor *resistor = new Resistor("Resistor");
  addModell(resistor);
  Inductor *inductorL = new Inductor("InductorL");
  inductorL->setInductance(0.1);
  addModell(inductorL);
  VoltageSource *voltageSource = new VoltageSource("VoltageSource");
  addModell(voltageSource);
  voltageSource->setVoltageSignal(new Signal);

  connectTerminal(voltageSource->getTerminal("B"),inductorL->getTerminal("A"));
  connectTerminal(inductorL->getTerminal("B"),resistor->getTerminal("A"));
  connectTerminal(resistor->getTerminal("B"),voltageSource->getTerminal("A"));

  Inductor* inductorR = new Inductor("InductorR");
  inductorR->setInductance(0.2);
  addModell(inductorR);
  connectTerminal(inductorL->getTerminal("B"),inductorR->getTerminal("A"));

  Capacitor *capacitor = new Capacitor("Capacitor");
  addModell(capacitor);
  capacitor->setCapacity(10);

  connectTerminal(inductorR->getTerminal("B"),capacitor->getTerminal("A"));
  connectTerminal(capacitor->getTerminal("B"),resistor->getTerminal("B"));
}
