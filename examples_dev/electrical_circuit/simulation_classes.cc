#include "simulation_classes.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

void Mesh::init() {
  T(0,0) = 1;
}

void ElectricalCircuit::addPin(Pin *pin_) {
  if(getPin(pin_->getName(),false)) {
    cout << "Error: The Network " << name << " can only comprise one Object by the name " <<  pin_->getName() << "!" << endl;
    assert(getPin(pin_->getName(),false) == NULL); 
  }
  pin.push_back(pin_);
}

Pin* ElectricalCircuit::getPin(const string &name, bool check) {
  unsigned int i;
  for(i=0; i<pin.size(); i++) {
    if(pin[i]->getName() == name)
      return pin[i];
  }
  if(check){
    if(!(i<pin.size())) cout << "Error: The Network " << this->name <<" comprises no pin " << name << "!" << endl; 
    assert(i<pin.size());
  }
  return NULL;
}

void ElectricalCircuit::addComponent(Component *comp_) {
  if(getComponent(comp_->getName(),false)) {
    cout << "Error: The Network " << name << " can only comprise one Object by the name " <<  comp_->getName() << "!" << endl;
    assert(getComponent(comp_->getName(),false) == NULL); 
  }
  comp.push_back(comp_);
  //obj->setParent(this);
}

Component* ElectricalCircuit::getComponent(const string &name, bool check) {
  unsigned int i;
  for(i=0; i<comp.size(); i++) {
    if(comp[i]->getName() == name)
      return comp[i];
  }
  if(check){
    if(!(i<comp.size())) cout << "Error: The Network " << this->name <<" comprises no comp " << name << "!" << endl; 
    assert(i<comp.size());
  }
  return NULL;
}

void ElectricalCircuit::addPin(const string &str) {
  addPin(new Pin(str));
}

void ElectricalCircuit::preinit() {
  for(vector<Component*>::iterator i = comp.begin(); i != comp.end(); ++i) {
    (*i)->sethSize((*i)->getuSize());
    (*i)->sethInd((*i)->getuInd());
  }
  vector<Pin*> pinList;
  buildListOfPins(pinList,true);
  for(unsigned int i=0; i<pinList.size(); i++) {
    cout << pinList[i]->getName()<< " " << pinList[i]->getFlag() << endl;
  }
  pinList[0]->setFlag(2); // root
  pinList[0]->go(0);

  for(unsigned int i=0; i<pinList.size(); i++) {
    cout << pinList[i]->getName()<< " " << pinList[i]->getFlag() << endl;
  }

  Mesh *meshL = new Mesh("MeshL");
  Node* node = addObject(0,meshL);

  Mesh *meshR = new Mesh("MeshR");
  node = addObject(node,meshR);

  Wire *wireL = new Wire("WireL");
  node = addObject(node,wireL);
  wireL->connect(meshL);
 
  Wire* wireM = new Wire("WireM");
  node = addObject(node,wireM);
  wireM->connect(meshL);
  wireM->connect(meshR);

  Wire* wireR = new Wire("WireR");
  node = addObject(node,wireR);
  wireR->connect(meshR);

  Inductor *inductor = new Inductor("InductorL");
  node = addObject(node,inductor);
  inductor->setInductance(0.1);
  inductor->connect(wireL);

  Resistor *resistor = new Resistor("Resistor");
  addLink(resistor);
  resistor->connect(wireM);

  VoltageSource *voltageSource = new VoltageSource("VoltageSource");
  addLink(voltageSource);
  voltageSource->connect(wireL);
  voltageSource->setVoltageSignal(new Signal);

  inductor = new Inductor("InductorR");
  node = addObject(node,inductor);
  inductor->setInductance(0.2);
  inductor->connect(wireR);

  Capacitor *capacitor = new Capacitor("Capacitor");
  addLink(capacitor);
  capacitor->setCapacity(10);
  capacitor->connect(wireR);

  //  for(unsigned int i=0; i<pinList.size(); i++) {
  //    if(pinList[i]->getFlag())
  //      loop->addPin(pinList[i]);
  //  }
}

void ElectricalCircuit::facLLM() {
}

void ElectricalCircuit::init() {
  Tree::init();
  updateM(0);
  LLM = facLL(M);
}

void ElectricalCircuit::buildListOfPins(std::vector<Pin*> &pinList, bool recursive) {
  for(unsigned int i=0; i<comp.size(); i++)
    comp[i]->buildListOfPins(pinList,recursive);
  //if(recursive)
  //for(unsigned int i=0; i<dynamicsystem.size(); i++)
  //dynamicsystem[i]->buildListOfObjects(obj,recursive);
}

Inductor::Inductor(const string &name) : Object(name), L(1) {
}

void Inductor::updateM(double t) {
  M += L*JTJ(wire[0]->getJacobian());
  //cout << hSize[0] << endl;
}

void Wire::updateStateDependentVariables(double t) {
  Q = J*parent->getq();
  I = J*parent->getu();
}

Resistor::Resistor(const string &name) : ElectricalLink(name), R(1) {
}

void Resistor::updateh(double t) {
  h[0] -= trans(wire[0]->getJacobian())*R*wire[0]->getCurrent(); // TODO Vorzeichen?
}

Capacitor::Capacitor(const string &name) : ElectricalLink(name), C(1) {
}

void Capacitor::updateh(double t) {
  h[0] -= trans(wire[0]->getJacobian())*C*wire[0]->getCharge(); // TODO Vorzeichen?
}

VoltageSource::VoltageSource(const string &name) : ElectricalLink(name) {
}

void VoltageSource::updateh(double t) {
  h[0] += trans(wire[0]->getJacobian())*(*voltageSignal)(t);
}

ElectricalLink::ElectricalLink(const string &name) : Link(name) {
}

void ElectricalLink::init() {
  h.push_back(fmatvec::Vec(wire[0]->getJacobian().cols()));
}

void ElectricalLink::updatehRef(const fmatvec::Vec &hParent, int j) {
  fmatvec::Index I = fmatvec::Index(wire[0]->getParent()->gethInd(parent,j),wire[0]->getParent()->gethInd(parent,j)+wire[0]->getJacobian().cols()-1);
  h[0]>>hParent(I);
} 

void Wire::init() {
  J.resize(1,gethSize());
  for(int i=0; i<mesh.size(); i++)
    J(0,mesh[i]->getuInd()) = i==0?1:-1; 
}
