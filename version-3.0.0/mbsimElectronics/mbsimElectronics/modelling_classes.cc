#include "modelling_classes.h"
#include "simulation_classes.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;


namespace MBSimElectronics {

  Branch tmpbranch("Dummy");
  
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

  void Terminal::findEndOfBranch(Terminal* callingTerminal, Branch* currentBranch) {
    for(unsigned int i=0; i<connectedTerminal.size(); i++) {
      if(callingTerminal != connectedTerminal[i]) {
        if(connectedTerminal[i]->getFlag() == 0)
          connectedTerminal[i]->findEndOfBranch(this,currentBranch);
        else if(connectedTerminal[i]->getFlag() == 2) {
          currentBranch->setEndTerminal(connectedTerminal[i]);
        }
        else {
          cout << "Fehler wegen flag = "<< (connectedTerminal[i]->getFlag()) << endl;
          throw 5;
        }

        if(getParent() == connectedTerminal[i]->getParent()) {
     	  int vz = name=="A" ? 1 : -1;
	  getParent()->connect(currentBranch,vz);
        }
      }
    }
    setFlag(1);
  }

  vector<Branch*> Terminal::buildBranches(Branch* currentBranch) {
    vector<Branch*> branch;
    for(unsigned int i=0; i<connectedTerminal.size(); i++) {
      if(connectedTerminal[i]->getFlag()==0) {
        currentBranch = new Branch("Name");
        branch.push_back(currentBranch);
        currentBranch->setStartTerminal(this);
        if(getParent() == connectedTerminal[i]->getParent()) {
	  int vz = name=="A" ? 1 : -1;
          getParent()->connect(currentBranch,vz);
        }
        connectedTerminal[i]->findEndOfBranch(this,currentBranch);
      }
      else if(connectedTerminal[i]->getFlag()==2) {
        currentBranch = new Branch("Name");
        branch.push_back(currentBranch);
        currentBranch->setStartTerminal(this);
        currentBranch->setEndTerminal(connectedTerminal[i]);
        if(getParent() == connectedTerminal[i]->getParent()) {
	  int vz = name=="A" ? 1 : -1;
          getParent()->connect(currentBranch,vz);
        }
      }

    }
    setFlag(4);
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

    modellList.clear();
    for(unsigned int i=0; i<remainingModells.size(); i++) {
      modellList.push_back(remainingModells[i]);
    }

    vector<Terminal*> terminalList;
    for(unsigned int i=0; i<compList.size(); i++)
      compList[i]->buildListOfTerminals(terminalList);

    vector<Terminal*> nodeList;
    for(unsigned int i=0; i<terminalList.size(); i++) {
      if(terminalList[i]->getNumberOfConnectedTerminals() > 2) {
        nodeList.push_back(terminalList[i]);
        terminalList[i]->setFlag(2); // root
      }
    }
    vector<Branch*> branchList;
    int k=0;
    for(unsigned int i=0; i<nodeList.size(); i++) {
      vector<Branch*> branchs_tmp = nodeList[i]->buildBranches(0);
      for(int j=0; j<branchs_tmp.size(); j++) {
        branchList.push_back(branchs_tmp[j]);
        stringstream str;
        str << "Branch" << k++;
        branchs_tmp[j]->setName(str.str());
      }
    }

    for(unsigned int i=0; i<branchList.size(); i++) {
      for(unsigned int j=0; j<i; j++) {
        //if(i!=j)
        if((branchList[i]->getStartTerminal() == branchList[j]->getStartTerminal() || branchList[i]->getEndTerminal() == branchList[j]->getEndTerminal()) || (branchList[i]->getStartTerminal() == branchList[j]->getEndTerminal() || branchList[i]->getEndTerminal() == branchList[j]->getStartTerminal())) {
          connectBranch(branchList[i],branchList[j]);
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
      treeBranch[j]->setFlag(3);
    }

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
      meshList[i]->getBranch(0)->setvz(1,meshList[i]);
      for(unsigned int j=1; j<meshList[i]->getNumberOfBranches(); j++) {
	if(meshList[i]->getBranch(j)->getEndTerminal() == meshList[i]->getBranch(j-1)->getStartTerminal())
	  meshList[i]->getBranch(j)->setvz(1,meshList[i]);
	else if(meshList[i]->getBranch(j)->getStartTerminal() == meshList[i]->getBranch(j-1)->getStartTerminal())
	  meshList[i]->getBranch(j)->setvz(-1,meshList[i]);
	else if(meshList[i]->getBranch(j)->getStartTerminal() == meshList[i]->getBranch(j-1)->getEndTerminal())
	  meshList[i]->getBranch(j)->setvz(1,meshList[i]);
	else if(meshList[i]->getBranch(j)->getEndTerminal() == meshList[i]->getBranch(j-1)->getEndTerminal())
	  meshList[i]->getBranch(j)->setvz(-1,meshList[i]);
	else
	  throw 5;
      }
      objectList.push_back(meshList[i]);
    }

    for(unsigned int i=0; i<branchList.size(); i++) {
      objectList.push_back(branchList[i]);
      for(unsigned j=0; j<branchList[i]->getNumberOfConnectedMeshes(); j++)
	branchList[i]->addDependency(branchList[i]->getMesh(j));
    }

    for(unsigned int i=0; i<compList.size(); i++) {
      Object* objectcomp = dynamic_cast<Object*>(compList[i]);
      Link* linkcomp = dynamic_cast<Link*>(compList[i]);
      if(objectcomp) {
	objectList.push_back(objectcomp);
	objectcomp->addDependency(compList[i]->getBranch());
      }
      else if(linkcomp) {
	linkList.push_back(linkcomp);
      }
      else {
	cout << "Fehler" << endl;
	throw 5;
      }
    }
  }

  ElectronicComponent::ElectronicComponent() : branch(0), vz(0), Q(0), I(0) {
    branch = &tmpbranch;
  }

  void ElectronicComponent::connect(Branch *branch_, int vz_) {
    vz = vz_;
    branch=branch_;
  }
}

