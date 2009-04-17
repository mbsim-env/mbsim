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
  vector<Pin*> pinList;
  buildListOfPins(pinList,true);
  vector<Pin*> nodeList;
  for(unsigned int i=0; i<pinList.size(); i++) {
    cout << pinList[i]->getName()<< " " << pinList[i]->getFlag() << " " << pinList[i]->getNumberOfConnectedPins() << endl;
    if(pinList[i]->getNumberOfConnectedPins() > 2) {
      nodeList.push_back(pinList[i]);
      pinList[i]->setFlag(2); // root
    }
  }
  cout << "Nodes:"<<endl;
  vector<Branch*> branchList;
  int k=0;
  for(unsigned int i=0; i<nodeList.size(); i++) {
    cout << nodeList[i]->getName()<< " " << nodeList[i]->getFlag() << " " << nodeList[i]->getNumberOfConnectedPins() << endl;
    //cout << "number of branches " << nodeList[i]->searchForBranches(0)<<endl;
    vector<Branch*> branchs_tmp = nodeList[i]->buildBranches(0,0);
    for(int j=0; j<branchs_tmp.size(); j++) {
      branchList.push_back(branchs_tmp[j]);
      stringstream str;
      str << "Branch" << k++;
      branchs_tmp[j]->setName(str.str());
    }
  }
  cout << "number of nodes " << nodeList.size() <<endl;
  cout << "number of branches " << branchList.size() <<endl;
  cout << "number of meshes " << branchList.size() - nodeList.size()+1 <<endl;
  for(unsigned int i=0; i<branchList.size(); i++) {
    for(unsigned int j=0; j<i; j++) {
      //if(i!=j)
	if((branchList[i]->getStartPin() == branchList[j]->getStartPin() && branchList[i]->getEndPin() == branchList[j]->getEndPin()) || (branchList[i]->getStartPin() == branchList[j]->getEndPin() && branchList[i]->getEndPin() == branchList[j]->getStartPin())) {
	  connectBranch(branchList[i],branchList[j]);
	  cout << "connect "<< branchList[i]->getName()<< " with "<< branchList[j]->getName() << endl;
	}
    }
  }
  vector<Branch*> treeBranch, linkBranch;
  unsigned int numberOfTreeBranches = nodeList.size() - 1;
  branchList[0]->buildTreeBranches(0, treeBranch, numberOfTreeBranches);
  for(unsigned int i=0; i<branchList.size(); i++) {
    bool flag = false;
    for(unsigned int j=0; j<treeBranch.size(); j++) {
      if(branchList[i]==treeBranch[j])
	flag = true;
    }
    if(!flag)
      linkBranch.push_back(branchList[i]);
  }
  for(unsigned int j=0; j<treeBranch.size(); j++) {
    cout << "treeBranch " << treeBranch[j]->getName() << endl;
    treeBranch[j]->setFlag(3);
  }
  for(unsigned int j=0; j<linkBranch.size(); j++) 
    cout << "linkBranch " << linkBranch[j]->getName() << endl;

  vector<Mesh*> meshList;
  k=0;
  for(unsigned int i=0; i<linkBranch.size(); i++) {
    bool flag = false;
    stringstream str;
    str << "Mesh" << k++;
    Mesh* mesh = new Mesh(str.str());
    linkBranch[i]->buildMeshes(0, mesh, flag);
    meshList.push_back(mesh);
  }
  Node* node = 0;
  for(unsigned int i=0; i<meshList.size(); i++) {
    cout << "mesh " <<  meshList[i]->getName() << endl;
    node = addObject(node,meshList[i]);
  }
  for(unsigned int i=0; i<branchList.size(); i++) {
    cout << "branch " <<  branchList[i]->getName();
    cout << " connected with mesh " <<endl;
    for(int j=0; j<branchList[i]->getNumberOfConnectedMeshes(); j++) {
      cout <<" - "<< branchList[i]->getMesh(j)->getName() << endl;
    }
    node = addObject(node,branchList[i]);
  }
  for(unsigned int i=0; i<pinList.size(); i++) {
    cout << "pin " <<  pinList[i]->getName();
    cout << " flag " <<  pinList[i]->getFlag();
    cout << " connected with branch " <<endl;
    if(pinList[i]->getBranch())
    cout <<" - "<< pinList[i]->getBranch()->getName() << endl;
    else
    cout <<" - "<< "not connected" << endl;
  }

  for(unsigned int i=0; i<comp.size(); i++) {
    cout << "comp " <<  comp[i]->getName();
    cout << " connected with branch " <<endl;
    if(comp[i]->getBranch())
    cout <<" - "<< comp[i]->getBranch()->getName() << endl;
    else
    cout <<" - "<< "not connected" << endl;
    Object* objectcomp = dynamic_cast<Object*>(comp[i]);
    Link* linkcomp = dynamic_cast<Link*>(comp[i]);
    if(objectcomp) {
      cout << "is Object" << endl;
      node = addObject(node,objectcomp);
    }
    else if(linkcomp) {
      cout << "is Link" << endl;
      addLink(linkcomp);
    }
    else {
      cout << "Fehler" << endl;
      throw 5;
    }
  }
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

void Inductor::updateM(double t) {
  M += L*JTJ(branch[0]->getJacobian());
  //cout << hSize[0] << endl;
}

void Branch::updateStateDependentVariables(double t) {
  Q = J*parent->getq();
  I = J*parent->getu();
}

Resistor::Resistor(const string &name) : ElectricalLink(name), R(1) {
}

void Resistor::updateh(double t) {
  h[0] -= trans(branch[0]->getJacobian())*R*branch[0]->getCurrent(); // TODO Vorzeichen?
}

Capacitor::Capacitor(const string &name) : ElectricalLink(name), C(1) {
}

void Capacitor::updateh(double t) {
  h[0] -= trans(branch[0]->getJacobian())*C*branch[0]->getCharge(); // TODO Vorzeichen?
}

VoltageSource::VoltageSource(const string &name) : ElectricalLink(name) {
}

void VoltageSource::updateh(double t) {
  h[0] += trans(branch[0]->getJacobian())*(*voltageSignal)(t);
}

void ElectricalLink::init() {
  h.push_back(fmatvec::Vec(branch[0]->getJacobian().cols()));
}

void ElectricalLink::updatehRef(const fmatvec::Vec &hParent, int j) {
  fmatvec::Index I = fmatvec::Index(branch[0]->getParent()->gethInd(parent,j),branch[0]->getParent()->gethInd(parent,j)+branch[0]->getJacobian().cols()-1);
  h[0]>>hParent(I);
} 

void Branch::init() {
  J.resize(1,gethSize());
  for(int i=0; i<mesh.size(); i++)
    J(0,mesh[i]->getuInd()) = i==0?1:-1; 
}

void Branch::addConnectedBranch(Branch *branch) {
  connectedBranch.push_back(branch);
}

void Branch::buildTreeBranches(Branch* callingBranch, vector<Branch*> &treeBranch, unsigned int nmax) {
  cout << "   insert " << this->getName() << endl;
  treeBranch.push_back(this);
  for(unsigned int i=0; i<connectedBranch.size(); i++)
    if(connectedBranch[i] != callingBranch) {
      if(connectedBranch[i]->getFlag()==0) {
	connectedBranch[i]->setFlag(1);
	//treeBranch.push_back(connectedBranch[i]);
	//cout << "   insert " << connectedBranch[i]->getName() << endl;
	cout << nmax << endl;
	cout << treeBranch.size() << endl;

	if(treeBranch.size() < nmax)
	  connectedBranch[i]->buildTreeBranches(this,treeBranch,nmax);
      }
    }
}

void Branch::buildMeshes(Branch* callingBranch, Mesh* currentMesh, bool &foundMesh) {
  //cout << "begin " << getName() << endl;
  if(callingBranch == 0) {
    callingBranch = this;
  } 
  for(unsigned int i=0; i<connectedBranch.size(); i++) {
    //cout << " treeBranch? " << (connectedBranch[i]->getFlag()==3) <<endl;
    //cout << " found? " << (connectedBranch[i] == callingBranch) <<endl;
    //cout << " sondern: " << connectedBranch[i]->getName() <<endl;
    if(callingBranch)
      //cout << " und: " << callingBranch->getName() <<endl;
      if(connectedBranch[i]->getFlag() == 3)
	connectedBranch[i]->buildMeshes(callingBranch,currentMesh,foundMesh);
      else if(connectedBranch[i] == callingBranch)
	foundMesh = true;
    if(foundMesh) {
      //cout << "connect " << name << " with " << currentMesh->getName() << endl;
      connect(currentMesh);
      return;
    }
  }
  //cout << " end " << getName() << endl;
}

void Branch::initPlot() {
  //updatePlotFeatures(parent);

  //if(getPlotFeature(plotRecursive)==enabled) {
  //if(getPlotFeature(globalPosition)==enabled) {
  plotColumns.push_back("Current");
  //}

  Object::initPlot();
  //}
}

void Branch::plot(double t, double dt) {
  //if(getPlotFeature(plotRecursive)==enabled) {
  //if(getPlotFeature(globalPosition)==enabled) {
  plotVector.push_back(I(0));
  //}

  Object::plot(t,dt);
  //}
}

//vector<Mesh*> Branch::buildMeshes(Branch* callingBranch, Mesh* currentMesh, bool &foundMesh) {
//  vector<Mesh*> mesh;
//  if(callingBranch == 0) {
//    currentMesh = new Mesh("Name");
//    //this->connect(currentMesh);
//    mesh.push_back(currentMesh);
//  }
//  for(unsigned int i=0; i<connectedBranch.size(); i++) {
//    if(!foundMesh) {
//      if(connectedBranch[i] != callingBranch) {
//	if(connectedBranch[i]->getFlag()==3) {
//	  connectedBranch[i]->buildMeshes(this,currentMesh,foundMesh);
//	}
//	else if(connectedBranch[i]->getFlag()==2) {
//	  foundMesh = true;
//	  cout << "found mesh" << endl;
//	}
//      }
//    }
//  }
//  if(foundMesh)
//    connect(currentMesh);
//  return mesh;
//}

