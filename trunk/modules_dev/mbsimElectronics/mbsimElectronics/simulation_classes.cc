#include "simulation_classes.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

void Mesh::init() {
  T(0,0) = 1;
}

//void ElectricalCircuit::addModell(ModellingInterface *modell_) {
//  if(getModell(modell_->getName(),false)) {
//    cout << "Error: The Network " << name << " can only comprise one Object by the name " <<  modell_->getName() << "!" << endl;
//    assert(getModell(modell_->getName(),false) == NULL); 
//  }
//  modell.push_back(modell_);
//  //obj->setParent(this);
//}
//
//ModellingInterface* ElectricalCircuit::getModell(const string &name, bool check) {
//  unsigned int i;
//  for(i=0; i<modell.size(); i++) {
//    if(modell[i]->getName() == name)
//      return modell[i];
//  }
//  if(check){
//    if(!(i<modell.size())) cout << "Error: The Network " << this->name <<" modell no modell " << name << "!" << endl; 
//    assert(i<modell.size());
//  }
//  return NULL;
//}

//void ElectricalCircuit::preinit() {
//
// // vector<ModellingInterface*> modellList;
// // buildListOfModels(modellList,true);
// // vector<Object*> objectList;
// // vector<Link*> linkList;
// // if(modellList.size())
// //   do {
// //     modellList[0]->processModellList(modellList,objectList,linkList);
// //   } while(modellList.size());
// // Node* node = 0;
// // for(unsigned int i=0; i<objectList.size(); i++) {
// //   addObject(objectList[i]);
// //   //node = addObject(node,objectList[i]);
// // }
// // for(unsigned int i=0; i<linkList.size(); i++) {
// //   addLink(linkList[i]);
// // }
//  SpecialGroup::preinit();
//}

/*void ElectricalCircuit::facLLM() {
}

void ElectricalCircuit::init() {
  Tree::init();
  updateM(0);
  LLM = facLL(M);
}

void ElectricalCircuit::buildListOfModells(std::vector<ModellingInterface*> &modellList, bool recursive) {
  for(unsigned int i=0; i<modell.size(); i++)
    modellList.push_back(modell[i]);
  //if(recursive) TODO: comment in, wenn in dynamic_system.cc
  //  for(unsigned int i=0; i<dynamicsystem.size(); i++)
  //    dynamicsystem[i]->buildListOfModells(modellList,recursive);
}
*/

Inductor::Inductor(const string &name) : ElectronicObject(name), L(1) {
  addTerminal("A");
  addTerminal("B");
  connectTerminal(terminal[0],terminal[1]);
}

void Inductor::updateM(double t) {
  M += L*JTJ(branch->getJacobian());
  //cout << hSize[0] << endl;
}

void Branch::updateStateDependentVariables(double t) {
  Q = J*parent->getq();
  I = J*parent->getu();
}

Diode::Diode(const string &name) : ElectronicLink(name) {
  addTerminal("A");
  addTerminal("B");
  connectTerminal(terminal[0],terminal[1]);
}

void Diode::updateh(double t) {
  double slope=1e4;
  double Is = (branch->getCurrent()(0) >= 0) ? 0 : slope*branch->getCurrent()(0);
  h[0] -= trans(branch->getJacobian())*Is; // TODO Vorzeichen?
  hLink[0] -= trans(branch->getJacobian())*Is; // TODO Vorzeichen?
}

Switch::Switch(const string &name) : ElectronicLink(name) {
  addTerminal("A");
  addTerminal("B");
  connectTerminal(terminal[0],terminal[1]);
}

void Switch::updateh(double t) {
  double Is = branch->getCurrent()(0);
  double gdLim = 0.01;
  double U0 = (*voltageSignal)(t)(0);
  double U;
  if(fabs(Is) < gdLim)
    U = -U0*Is/gdLim;
  else
    U = Is>0?-U0:U0;
  U*=-1;
  h[0] -= trans(branch->getJacobian())*U; // TODO Vorzeichen?
  hLink[0] -= trans(branch->getJacobian())*U; // TODO Vorzeichen?
}

Resistor::Resistor(const string &name) : ElectronicLink(name), R(1) {
  addTerminal("A");
  addTerminal("B");
  connectTerminal(terminal[0],terminal[1]);
}

void Resistor::updateh(double t) {
  h[0] -= trans(branch->getJacobian())*R*branch->getCurrent(); // TODO Vorzeichen?
  hLink[0] -= trans(branch->getJacobian())*R*branch->getCurrent(); // TODO Vorzeichen?
}

double Resistor::computeU(double t) {
  return -R*branch->getCurrent()(0); // TODO Vorzeichen?
}

Capacitor::Capacitor(const string &name) : ElectronicLink(name), C(1) {
  addTerminal("A");
  addTerminal("B");
  connectTerminal(terminal[0],terminal[1]);
}

void Capacitor::updateh(double t) {
  h[0] -= trans(branch->getJacobian())*branch->getCharge()/C; // TODO Vorzeichen?
  hLink[0] -= trans(branch->getJacobian())*branch->getCharge()/C; // TODO Vorzeichen?
}

VoltageSource::VoltageSource(const string &name) : ElectronicLink(name) {
  addTerminal("A");
  addTerminal("B");
  connectTerminal(terminal[0],terminal[1]);
}

void VoltageSource::updateh(double t) {
  h[0] -= trans(branch->getJacobian())*(*voltageSignal)(t);
  hLink[0] -= trans(branch->getJacobian())*(*voltageSignal)(t);
}

void ElectronicLink::init() {
  h.push_back(fmatvec::Vec(branch->getJacobian().cols()));
  hLink.push_back(fmatvec::Vec(branch->getJacobian().cols()));
}

void ElectronicLink::updatehRef(const fmatvec::Vec &hParent, const fmatvec::Vec &hLinkParent, int j) {
  fmatvec::Index I = fmatvec::Index(branch->getParent()->gethInd(parent,j),branch->getParent()->gethInd(parent,j)+branch->getJacobian().cols()-1);
  h[0]>>hParent(I);
  hLink[0]>>hLinkParent(I);
} 

void Branch::init() {
  if(J.cols() == 0) {
  J.resize(1,gethSize());
  for(int i=0; i<mesh.size(); i++)
    J(0,mesh[i]->getuInd()) = i==0?1:1; 
  }
  cout << name << " " << J << endl;
}

void Branch::addConnectedBranch(Branch *branch) {
  connectedBranch.push_back(branch);
}

void Branch::buildTreeBranches(Branch* callingBranch, vector<Branch*> &treeBranch, unsigned int nmax) {
  //cout << "   insert " << this->getName() << endl;
  treeBranch.push_back(this);
  for(unsigned int i=0; i<connectedBranch.size(); i++)
    if(connectedBranch[i] != callingBranch) {
      if(connectedBranch[i]->getFlag()==0) {
	connectedBranch[i]->setFlag(1);
	//treeBranch.push_back(connectedBranch[i]);
	//cout << "   insert " << connectedBranch[i]->getName() << endl;
	//cout << nmax << endl;
	//cout << treeBranch.size() << endl;

	if(treeBranch.size() < nmax)
	  connectedBranch[i]->buildTreeBranches(this,treeBranch,nmax);
      }
    }
}


//void Branch::buildMeshes(Branch* callingBranch, Mesh* currentMesh, bool &foundMesh) {
//  cout << "begin " << getName() << endl;
//  if(callingBranch == 0) {
//    callingBranch = this;
//  } 
//  if(getFlag() == 3)
//    setFlag(4);
//  for(unsigned int i=0; i<connectedBranch.size(); i++) {
//    cout << i << endl;
//   // cout << " treeBranch? " << (connectedBranch[i]->getFlag()==3) <<endl;
//   // cout << " found? " << (connectedBranch[i] == callingBranch) <<endl;
//   // cout << " sondern: " << connectedBranch[i]->getName() <<endl;
//    if(callingBranch)
//      //cout << " und: " << callingBranch->getName() <<endl;
//      if(connectedBranch[i]->getFlag() == 3)
//	connectedBranch[i]->buildMeshes(callingBranch,currentMesh,foundMesh);
//      else if(connectedBranch[i] == callingBranch)
//	foundMesh = true;
//    if(foundMesh) {
//      if(getFlag()==4)
//	setFlag(3);
//      cout << "connect " << name << " with " << currentMesh->getName() << endl;
//      connect(currentMesh);
//      return;
//    }
//  }
//  //cout << " end " << getName() << endl;
//}
//
void Branch::buildMeshes(Terminal *callingTerminal, Branch* callingBranch, Mesh* currentMesh, bool &foundMesh) {
  cout << "begin " << getName() << endl;
  if(callingBranch == 0) {
    callingBranch = this;
  } 
  if(getFlag() == 3)
    setFlag(4);
  Terminal *sT;
  sT = callingTerminal == getEndTerminal() ? getStartTerminal() : getEndTerminal();
  for(unsigned int i=0; i<sT->getNumberOfConnectedBranches(); i++) {
    if(sT->getBranch(i) != this) { // nur vorwärts laufen
      if(sT->getBranch(i)->getFlag() == 3) // Branch ist TreeBranch
	sT->getBranch(i)->buildMeshes(sT, callingBranch, currentMesh, foundMesh);
      else if(sT->getBranch(i) == callingBranch)
	foundMesh = true;
      if(foundMesh) {
	if(getFlag()==4)
	  setFlag(3);
	cout << "connect " << name << " with " << currentMesh->getName() << endl;
	connect(currentMesh);
	return;
      }
    }
  }
  cout << " end " << getName() << endl;
}

void Branch::initPlot() {
  //updatePlotFeatures(parent);

  //if(getPlotFeature(plotRecursive)==enabled) {
  //if(getPlotFeature(globalPosition)==enabled) {
  plotColumns.push_back("Charge");
  plotColumns.push_back("Current");
  //}

  Object::initPlot();
  //}
}

void Branch::plot(double t, double dt) {
  //if(getPlotFeature(plotRecursive)==enabled) {
  //if(getPlotFeature(globalPosition)==enabled) {
  plotVector.push_back(Q(0));
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

void Resistor::initPlot() {
  plotColumns.push_back("VoltageDrop");

  Link::initPlot();
}

void Resistor::plot(double t, double dt) {
  plotVector.push_back(R*branch->getCurrent()(0));

  Link::plot(t,dt);
}

