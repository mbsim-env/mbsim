#include "modeling_classes.h"
#include "simulation_classes.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

void connectPin(Pin *pin1, Pin *pin2) {
  pin1->addConnectedPin(pin2);
  pin2->addConnectedPin(pin1);
}

void connectBranch(Branch *branch1, Branch *branch2) {
  branch1->addConnectedBranch(branch2);
  branch2->addConnectedBranch(branch1);
}

void Pin::addConnectedPin(Pin *pin) {
  connectedPin.push_back(pin);
}

int Pin::searchForBranches(Pin* callingPin) {
  int k = 0;
  for(unsigned int i=0; i<connectedPin.size(); i++)
    if(connectedPin[i] != callingPin) {
      if(connectedPin[i]->getFlag()==0) {
	connectedPin[i]->setFlag(1);
	cout << "   try pin " << connectedPin[i]->getName() << " of parent " << connectedPin[i]->getParent()->getName() << endl;
	k += connectedPin[i]->searchForBranches(this);
      }
      else if(connectedPin[i]->getFlag()==2) {
	cout << "found branch" << endl;
	k++;
      }
    }
  return k;
}

vector<Branch*> Pin::buildBranches(Pin* callingPin, Branch* currentBranch) {
  vector<Branch*> branch;
  for(unsigned int i=0; i<connectedPin.size(); i++)
    if(connectedPin[i] != callingPin) {
      if(connectedPin[i]->getFlag()==0) {
	if(callingPin == 0) {
	  currentBranch = new Branch("Name");
	  branch.push_back(currentBranch);
	  currentBranch->setStartPin(this);
	  this->setBranch(currentBranch);
	}
	if(this->getParent() == connectedPin[i]->getParent()) {
	  cout << "connect " << this->getParent()->getName()<< " with branch "<< currentBranch<< endl;
	  this->getParent()->connect(currentBranch);
	}
	  
	connectedPin[i]->setFlag(1);
	connectedPin[i]->setBranch(currentBranch);
	connectedPin[i]->buildBranches(this,currentBranch);
      }
      else if(connectedPin[i]->getFlag()==2) {
	if(this->getParent() == connectedPin[i]->getParent()) {
	  cout << "connect " << this->getParent()->getName()<< " with branch "<< currentBranch<< endl;
	  this->getParent()->connect(currentBranch);
	}
	currentBranch->setEndPin(connectedPin[i]);
	cout << "found branch" << endl;
	connectedPin[i]->setBranch(currentBranch);
      }
    }
  return branch;
}


void Component::addPin(Pin *pin_) {
  if(getPin(pin_->getName(),false)) {
    cout << "Error: The Component " << getName() << " can only comprise one Object by the name " <<  pin_->getName() << "!" << endl;
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
    if(!(i<pin.size())) cout << "Error: The Component " << this->getName() <<" comprises no pin " << name << "!" << endl; 
    assert(i<pin.size());
  }
  return NULL;
}

void Component::buildListOfPins(std::vector<Pin*> &pinList, bool recursive) {
  for(unsigned int i=0; i<pin.size(); i++)
    pinList.push_back(pin[i]);
  //if(recursive)
  //for(unsigned int i=0; i<dynamicsystem.size(); i++)
  //dynamicsystem[i]->buildListOfObjects(obj,recursive);
}

