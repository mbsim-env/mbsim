#include "modeling_classes.h"
#include "simulation_classes.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;

void connectTerminal(Terminal *terminal1, Terminal *terminal2) {
  terminal1->addConnectedTerminal(terminal2);
  terminal2->addConnectedTerminal(terminal1);
}

void connectBranch(Branch *branch1, Branch *branch2) {
  branch1->addConnectedBranch(branch2);
  branch2->addConnectedBranch(branch1);
}

void Terminal::addConnectedTerminal(Terminal *terminal) {
  connectedTerminal.push_back(terminal);
}

void Terminal::addConnectedBranch(Branch *branch) {
  connectedBranch.push_back(branch);
}

int Terminal::searchForBranches(Terminal* callingTerminal) {
  int k = 0;
  for(unsigned int i=0; i<connectedTerminal.size(); i++)
    if(connectedTerminal[i] != callingTerminal) {
      if(connectedTerminal[i]->getFlag()==0) {
	connectedTerminal[i]->setFlag(1);
	//cout << "   try terminal " << connectedTerminal[i]->getName() << " of parent " << connectedTerminal[i]->getParent()->getName() << endl;
	k += connectedTerminal[i]->searchForBranches(this);
      }
      else if(connectedTerminal[i]->getFlag()==2) {
	//cout << "found branch" << endl;
	k++;
      }
    }
  return k;
}

void Terminal::findEndOfBranch(Terminal* callingTerminal, Branch* currentBranch) {
  cout << "starte findEndOfBranch of " << getName() << " of " << getParent()->getName() << " with flag " << getFlag()<< endl;
  for(unsigned int i=0; i<connectedTerminal.size(); i++) {
    if(callingTerminal != connectedTerminal[i]) {
      if(connectedTerminal[i]->getFlag() == 0)
	connectedTerminal[i]->findEndOfBranch(this,currentBranch);
      else if(connectedTerminal[i]->getFlag() == 2) {
	currentBranch->setEndTerminal(connectedTerminal[i]);
	cout << "set end terminal with flag" << connectedTerminal[i]->getFlag() << endl;
	cout << "\t found branch " << endl;
      }
      else {
	cout << "Fehler wegen flag = "<< (connectedTerminal[i]->getFlag()) << endl;
	throw 5;
      }

      if(this->getParent() == connectedTerminal[i]->getParent()) {
	this->getParent()->connect(currentBranch);
      }
    }
  }
  setFlag(1);
  cout << "end of findEndOfBranch of " << getName() << " of " << getParent()->getName() << " with flag " << getFlag()<< endl;
}

vector<Branch*> Terminal::buildBranches(Branch* currentBranch) {
  vector<Branch*> branch;
  cout << "starte buildBranches of " << getName() << " of " << getParent()->getName() << " with flag " << getFlag()<< endl;
  for(unsigned int i=0; i<connectedTerminal.size(); i++) {
    //cout << "\tcalling Terminal " << callingTerminal << endl;
    cout <<"\t"<<i <<" " << connectedTerminal[i]->getName() << " " << connectedTerminal[i]->getParent()->getName() <<" " << connectedTerminal[i]->getFlag()<<endl;
    if(connectedTerminal[i]->getFlag()==0) {
      cout << "\tbuild new Branch" << endl;
      currentBranch = new Branch("Name");
      branch.push_back(currentBranch);
      currentBranch->setStartTerminal(this);
      cout << "set start terminal with flag" << this->getFlag() << endl;
      if(getParent() == connectedTerminal[i]->getParent()) {
	getParent()->connect(currentBranch);
      }
      connectedTerminal[i]->findEndOfBranch(this,currentBranch);
    }
    else if(connectedTerminal[i]->getFlag()==2) {
      cout << "\tbuild new Branch" << endl;
      currentBranch = new Branch("Name");
      branch.push_back(currentBranch);
      currentBranch->setStartTerminal(this);
      cout << "set start terminal with flag" << this->getFlag() << endl;
      currentBranch->setEndTerminal(connectedTerminal[i]);
      cout << "set end terminal with flag" << connectedTerminal[i]->getFlag() << endl;
      if(getParent() == connectedTerminal[i]->getParent()) {
	getParent()->connect(currentBranch);
      }
      cout << "\t found branch " << endl;
    }

  }
  setFlag(4);
  cout << "end buildBranches of " << getName() << " of " << getParent()->getName() << " with flag " << getFlag()<< endl;
  return branch;
}


void ElectronicComponent::addTerminal(Terminal *terminal_) {
  if(getTerminal(terminal_->getName(),false)) {
    cout << "Error: The Component " << getName() << " can only comprise one Object by the name " <<  terminal_->getName() << "!" << endl;
    assert(getTerminal(terminal_->getName(),false) == NULL); 
  }
  terminal.push_back(terminal_);
  terminal_->setParent(this);
}

void ElectronicComponent::addTerminal(const string &str) {
  addTerminal(new Terminal(str));
}

Terminal* ElectronicComponent::getTerminal(const string &name, bool check) {
  unsigned int i;
  for(i=0; i<terminal.size(); i++) {
    if(terminal[i]->getName() == name)
      return terminal[i];
  }
  if(check){
    if(!(i<terminal.size())) cout << "Error: The Component " << this->getName() <<" comprises no terminal " << name << "!" << endl; 
    assert(i<terminal.size());
  }
  return NULL;
}

void ElectronicComponent::buildListOfTerminals(std::vector<Terminal*> &terminalList) {
  for(unsigned int i=0; i<terminal.size(); i++)
    terminalList.push_back(terminal[i]);
}

void ElectronicComponent::processModellList(vector<ModellingInterface*> &modellList, vector<Object*> &objectList, vector<Link*> &linkList) {

  vector<ElectronicComponent*> compList;
  vector<ModellingInterface*> remainingModells;
  for(unsigned int i=0; i<modellList.size(); i++) {
    ElectronicComponent *comp = dynamic_cast<ElectronicComponent*>(modellList[i]);
    if(comp)
      compList.push_back(comp);
    else
      remainingModells.push_back(modellList[i]);
  }

  cout << "Electronic Components: " << endl;
  for(unsigned int i=0; i<compList.size(); i++) {
    cout<< compList[i]->getName() << endl;
  }
  modellList.clear();
  for(unsigned int i=0; i<remainingModells.size(); i++) {
    modellList.push_back(remainingModells[i]);
  }

  vector<Terminal*> terminalList;
  for(unsigned int i=0; i<compList.size(); i++)
    compList[i]->buildListOfTerminals(terminalList);

  vector<Terminal*> nodeList;
  for(unsigned int i=0; i<terminalList.size(); i++) {
    //cout << terminalList[i]->getName()<< " " << terminalList[i]->getFlag() << " " << terminalList[i]->getNumberOfConnectedTerminals() << endl;
    if(terminalList[i]->getNumberOfConnectedTerminals() > 2) {
      nodeList.push_back(terminalList[i]);
      terminalList[i]->setFlag(2); // root
    }
  }
  cout << "number of nodes " << nodeList.size() <<endl;
  vector<Branch*> branchList;
  int k=0;
  for(unsigned int i=0; i<nodeList.size(); i++) {
    cout << nodeList[i]->getName()<< " " << nodeList[i]->getFlag() << " " << nodeList[i]->getNumberOfConnectedTerminals() << " " <<nodeList[i]->getParent()->getName() << endl;
    //cout << "number of branches " << nodeList[i]->searchForBranches(0)<<endl;
    vector<Branch*> branchs_tmp = nodeList[i]->buildBranches(0);
    for(int j=0; j<branchs_tmp.size(); j++) {
      branchList.push_back(branchs_tmp[j]);
      stringstream str;
      str << "Branch" << k++;
      branchs_tmp[j]->setName(str.str());
    }
  }
  cout << "number of branches " << branchList.size() <<endl;
  cout << "number of meshes " << branchList.size() - nodeList.size()+1 <<endl;

  for(unsigned int i=0; i<branchList.size(); i++) {
    cout <<branchList[i]->getName()<< " with " << branchList[i]->getStartTerminal()->getName() <<"  of "<< branchList[i]->getStartTerminal()->getParent()->getName() << endl;
    cout <<" and with " << branchList[i]->getEndTerminal()->getName() <<"  of "<< branchList[i]->getEndTerminal()->getParent()->getName() << endl;
    for(unsigned int j=0; j<i; j++) {
      //if(i!=j)
      if((branchList[i]->getStartTerminal() == branchList[j]->getStartTerminal() || branchList[i]->getEndTerminal() == branchList[j]->getEndTerminal()) || (branchList[i]->getStartTerminal() == branchList[j]->getEndTerminal() || branchList[i]->getEndTerminal() == branchList[j]->getStartTerminal())) {
	connectBranch(branchList[i],branchList[j]);
	cout << "connect "<< branchList[i]->getName()<< " with "<< branchList[j]->getName() << endl;
      }
    }
  }
  vector<Branch*> treeBranch, linkBranch;
  unsigned int numberOfTreeBranches = nodeList.size() - 1;
  cout<< "numberOfTreeBranches " << numberOfTreeBranches << endl;
  // TODO 0->4
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
    linkBranch[i]->buildMeshes(0, 0, mesh, flag);
    meshList.push_back(mesh);
  }

  for(unsigned int i=0; i<meshList.size(); i++) {
    objectList.push_back(meshList[i]);
    if(i>0)
      meshList[i]->setPrecessor(meshList[i-1]);
  }

  for(unsigned int i=0; i<branchList.size(); i++) {
    objectList.push_back(branchList[i]);
    if(i>0)
      branchList[i]->setPrecessor(branchList[i-1]);
    else
      branchList[i]->setPrecessor(meshList[meshList.size()-1]);
  }

  for(unsigned int i=0; i<compList.size(); i++) {
    //cout << "comp " <<  compList[i]->getName();
    //cout << " connected with branch " <<endl;
    //if(compList[i]->getBranch())
    //cout <<" - "<< compList[i]->getBranch()->getName() << endl;
    //else
    //cout <<" - "<< "not connected" << endl;
    Object* objectcomp = dynamic_cast<Object*>(compList[i]);
    Link* linkcomp = dynamic_cast<Link*>(compList[i]);
    if(objectcomp) {
      //cout << "is Object" << endl;
      objectList.push_back(objectcomp);
      //node = addObject(node,objectcomp);
    }
    else if(linkcomp) {
      //cout << "is Link" << endl;
      linkList.push_back(linkcomp);
      //addLink(linkcomp);
    }
    else {
      cout << "Fehler" << endl;
      throw 5;
    }
  }
  int mode = 2;
  if(mode == 0) {
  branchList[0]->clearMeshList();
  branchList[0]->connect(meshList[0]);

  branchList[1]->clearMeshList();
  branchList[1]->connect(meshList[0]);
  branchList[1]->connect(meshList[1]);

  branchList[2]->clearMeshList();
  branchList[2]->connect(meshList[1]);

  branchList[3]->clearMeshList();
  branchList[3]->connect(meshList[1]);

  branchList[4]->clearMeshList();
  branchList[4]->connect(meshList[1]);
  branchList[4]->connect(meshList[2]);

  branchList[5]->clearMeshList();
  branchList[5]->connect(meshList[2]);
  } else if(mode ==1) {

  branchList[0]->clearMeshList();
  branchList[0]->connect(meshList[0]);

  branchList[1]->clearMeshList();
  //branchList[1]->connect(meshList[0]);
  branchList[1]->connect(meshList[1]);

  branchList[2]->clearMeshList();
  branchList[2]->connect(meshList[0]);
  branchList[2]->connect(meshList[1]);

  branchList[3]->clearMeshList();
  branchList[3]->connect(meshList[0]);
  branchList[3]->connect(meshList[1]);

  branchList[4]->clearMeshList();
  branchList[4]->connect(meshList[0]);
  branchList[4]->connect(meshList[1]);
  branchList[4]->connect(meshList[2]);

  branchList[5]->clearMeshList();
  branchList[5]->connect(meshList[2]);
  } else if(mode == 2) {
  //branchList[2]->clearMeshList();
  //branchList[2]->connect(meshList[1]);
  }

  if(mode<=1) {
  compList[0]->connect(branchList[0]);
  compList[1]->connect(branchList[0]);
  compList[2]->connect(branchList[0]);
  compList[3]->connect(branchList[1]);
  compList[4]->connect(branchList[2]);
  compList[5]->connect(branchList[4]);
  compList[6]->connect(branchList[5]);
  compList[7]->connect(branchList[5]);
  }

  //branchList[0]->getJacobian() = Mat("[1,0,0]");
  //branchList[1]->getJacobian() = Mat("[0,1,0]");
  //branchList[2]->getJacobian() = Mat("[1,1,0]");
  //branchList[3]->getJacobian() = Mat("[1,1,0]");
  //branchList[4]->getJacobian() = Mat("[1,1,-1]");

}
