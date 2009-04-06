#include "modeling_classes.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

void connectPin(Pin *pin1, Pin *pin2) {
  pin1->addConnectedPin(pin2);
  pin2->addConnectedPin(pin1);
}

void Pin::addConnectedPin(Pin *pin) {
  connectedPin.push_back(pin);
}

void Pin::go(Pin* callingPin) {
  cout << "in " << name << " of parent "<<getParent()->getName() <<endl;
  for(unsigned int i=0; i<connectedPin.size(); i++)
    if(connectedPin[i] != callingPin) {
    if(connectedPin[i]->getFlag()==0) {
      connectedPin[i]->setFlag(1);
      cout << "   try pin " << connectedPin[i]->getName() << " of parent " << connectedPin[i]->getParent()->getName() << endl;
      connectedPin[i]->go(this);
    }
    else if(connectedPin[i]->getFlag()==2)
      cout << "found loop" << endl;
    }
}

void Component::addPin(Pin *pin_) {
  if(getPin(pin_->getName(),false)) {
    cout << "Error: The Component " << name << " can only comprise one Object by the name " <<  pin_->getName() << "!" << endl;
    assert(getPin(pin_->getName(),false) == NULL); 
  }
  pin.push_back(pin_);
  pin_->setParent(this);
}

void Component::addPin(const string &str) {
  addPin(new Pin(str));
}

Pin* Component::getPin(const string &name, bool check) {
  unsigned int i;
  for(i=0; i<pin.size(); i++) {
    if(pin[i]->getName() == name)
      return pin[i];
  }
  if(check){
    if(!(i<pin.size())) cout << "Error: The Component " << this->name <<" comprises no pin " << name << "!" << endl; 
    assert(i<pin.size());
  }
  return NULL;
}

CompInductor::CompInductor(const string &name) : Component(name) {
  addPin("A");
  addPin("B");
  connectPin(pin[0],pin[1]);
}

CompResistor::CompResistor(const string &name) : Component(name) {
  addPin("A");
  addPin("B");
  connectPin(pin[0],pin[1]);
}

CompCapacitor::CompCapacitor(const string &name) : Component(name) {
  addPin("A");
  addPin("B");
  connectPin(pin[0],pin[1]);
}

CompVoltageSource::CompVoltageSource(const string &name) : Component(name) {
  addPin("A");
  addPin("B");
  connectPin(pin[0],pin[1]);
}

void Component::buildListOfPins(std::vector<Pin*> &pinList, bool recursive) {
   for(unsigned int i=0; i<pin.size(); i++)
      pinList.push_back(pin[i]);
    //if(recursive)
      //for(unsigned int i=0; i<dynamicsystem.size(); i++)
	//dynamicsystem[i]->buildListOfObjects(obj,recursive);
}

