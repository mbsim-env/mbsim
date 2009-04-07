#include "my_circuit.h"
#include "modeling_classes.h"

using namespace std;

MyCircuit::MyCircuit(const string &name) : ElectricalCircuit(name) {

  CompResistor *resistor = new CompResistor("Resistor");
  addComponent(resistor);
  CompInductor *inductor = new CompInductor("Inductor");
  addComponent(inductor);
  CompVoltageSource *voltageSource = new CompVoltageSource("voltageSource");
  addComponent(voltageSource);

  connectPin(voltageSource->getPin("B"),inductor->getPin("A"));
  connectPin(inductor->getPin("B"),resistor->getPin("A"));
  connectPin(resistor->getPin("B"),voltageSource->getPin("A"));
}
