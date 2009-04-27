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

vector<Branch*> Terminal::buildBranches(Terminal* callingTerminal, Branch* currentBranch) {
  vector<Branch*> branch;
  for(unsigned int i=0; i<connectedTerminal.size(); i++)
    if(connectedTerminal[i] != callingTerminal) {
      if(connectedTerminal[i]->getFlag()==0) {
	if(callingTerminal == 0) {
	  currentBranch = new Branch("Name");
	  branch.push_back(currentBranch);
	  currentBranch->setStartTerminal(this);
	}
	if(this->getParent() == connectedTerminal[i]->getParent()) {
	  //cout << "connect " << this->getParent()->getName()<< " with branch "<< currentBranch<< endl;
	  this->getParent()->connect(currentBranch);
	}

	connectedTerminal[i]->setFlag(1);
	connectedTerminal[i]->buildBranches(this,currentBranch);
      }
      else if(connectedTerminal[i]->getFlag()==2) {
	if(this->getParent() == connectedTerminal[i]->getParent()) {
	  //cout << "connect " << this->getParent()->getName()<< " with branch "<< currentBranch<< endl;
	  this->getParent()->connect(currentBranch);
	}
	currentBranch->setEndTerminal(connectedTerminal[i]);
	//cout << "found branch" << endl;
      }
    }
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

  //cout << "Electronic Components: " << endl;
  for(unsigned int i=0; i<compList.size(); i++) {
    //cout<< compList[i]->getName() << endl;
  }
  //cout << endl;
  modellList.clear();
  //cout << "Non-electronic Components: " << endl;
  for(unsigned int i=0; i<remainingModells.size(); i++) {
    //cout<< remainingModells[i]->getName() << endl;
    modellList.push_back(remainingModells[i]);
  }
  //cout << endl;

  vector<Terminal*> terminalList;
  for(unsigned int i=0; i<compList.size(); i++)
    compList[i]->buildListOfTerminals(terminalList);

  //cout << "Terminals: " << endl;
  //for(unsigned int i=0; i<terminalList.size(); i++)
  //cout<< terminalList[i]->getName() << endl;
  //cout << endl;

  vector<Terminal*> nodeList;
  for(unsigned int i=0; i<terminalList.size(); i++) {
    //cout << terminalList[i]->getName()<< " " << terminalList[i]->getFlag() << " " << terminalList[i]->getNumberOfConnectedTerminals() << endl;
    if(terminalList[i]->getNumberOfConnectedTerminals() > 2) {
      nodeList.push_back(terminalList[i]);
      terminalList[i]->setFlag(2); // root
    }
  }
  //cout << "Nodes:"<<endl;
  vector<Branch*> branchList;
  int k=0;
  for(unsigned int i=0; i<nodeList.size(); i++) {
    //cout << nodeList[i]->getName()<< " " << nodeList[i]->getFlag() << " " << nodeList[i]->getNumberOfConnectedTerminals() << endl;
    //cout << "number of branches " << nodeList[i]->searchForBranches(0)<<endl;
    vector<Branch*> branchs_tmp = nodeList[i]->buildBranches(0,0);
    for(unsigned int j=0; j<branchs_tmp.size(); j++) {
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
      if((branchList[i]->getStartTerminal() == branchList[j]->getStartTerminal() && branchList[i]->getEndTerminal() == branchList[j]->getEndTerminal()) || (branchList[i]->getStartTerminal() == branchList[j]->getEndTerminal() && branchList[i]->getEndTerminal() == branchList[j]->getStartTerminal())) {
	connectBranch(branchList[i],branchList[j]);
	//cout << "connect "<< branchList[i]->getName()<< " with "<< branchList[j]->getName() << endl;
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
    //cout << "treeBranch " << treeBranch[j]->getName() << endl;
    treeBranch[j]->setFlag(3);
  }
  //for(unsigned int j=0; j<linkBranch.size(); j++) 
  //cout << "linkBranch " << linkBranch[j]->getName() << endl;

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

  for(unsigned int i=0; i<meshList.size(); i++) 
    objectList.push_back(meshList[i]);

  for(unsigned int i=0; i<branchList.size(); i++) {
    objectList.push_back(branchList[i]);
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
}
