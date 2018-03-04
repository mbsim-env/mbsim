#include <config.h>
#include <iostream>
#include "modelling_classes.h"
#include "simulation_classes.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;


namespace MBSimElectronics {

  const PlotFeatureEnum charge;
  const PlotFeatureEnum current;
  const PlotFeatureEnum voltage;

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
    for(auto & i : connectedTerminal) {
      if(callingTerminal != i) {
        if(i->getFlag() == 0)
          i->findEndOfBranch(this,currentBranch);
        else if(i->getFlag() == 2) {
          currentBranch->setEndTerminal(i);
        }
        else {
          stringstream error;
          error << "Fehler wegen flag = "<< (i->getFlag()) << endl;
          throw runtime_error(error.str());
        }

        if(getParent() == i->getParent()) {
     	  int vz = name=="A" ? 1 : -1;
	  dynamic_cast<ElectronicComponent*>(getParent())->connect(currentBranch,vz);
        }
      }
    }
    setFlag(1);
  }

  vector<Branch*> Terminal::buildBranches(Branch* currentBranch) {
    vector<Branch*> branch;
    for(auto & i : connectedTerminal) {
      if(i->getFlag()==0) {
        currentBranch = new Branch("Name");
        branch.push_back(currentBranch);
        currentBranch->setStartTerminal(this);
        if(getParent() == i->getParent()) {
	  int vz = name=="A" ? 1 : -1;
          dynamic_cast<ElectronicComponent*>(getParent())->connect(currentBranch,vz);
        }
        i->findEndOfBranch(this,currentBranch);
      }
      else if(i->getFlag()==2) {
        currentBranch = new Branch("Name");
        branch.push_back(currentBranch);
        currentBranch->setStartTerminal(this);
        currentBranch->setEndTerminal(i);
        if(getParent() == i->getParent()) {
	  int vz = name=="A" ? 1 : -1;
          dynamic_cast<ElectronicComponent*>(getParent())->connect(currentBranch,vz);
        }
      }

    }
    setFlag(4);
    return branch;
  }


  void ElectronicComponent::addTerminal(Terminal *terminal_) {
    if(getTerminal(terminal_->getName(),false)) {
      msg(Error) << "The Component " << getName() << " can only comprise one Object by the name " <<  terminal_->getName() << "!" << endl;
      assert(getTerminal(terminal_->getName(),false) == nullptr); 
    }
    terminal.push_back(terminal_);
    //terminal_->setParent(this);
    terminal_->setParent(dynamic_cast<Element*>(this));
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
      if(!(i<terminal.size())) msg(Error) << "The Component " << this->getName() <<" comprises no terminal " << name << "!" << endl; 
      assert(i<terminal.size());
    }
    return nullptr;
  }

  void ElectronicComponent::buildListOfTerminals(std::vector<Terminal*> &terminalList) {
    for(auto i : terminal)
      terminalList.push_back(i);
  }

  void ElectronicComponent::processModellList(vector<ModellingInterface*> &modellList, vector<Object*> &objectList, vector<Link*> &linkList) {

    vector<ElectronicComponent*> compList;
    vector<ModellingInterface*> remainingModells;
    for(auto & i : modellList) {
      auto *comp = dynamic_cast<ElectronicComponent*>(i);
      if(comp)
        compList.push_back(comp);
      else
        remainingModells.push_back(i);
    }

    modellList.clear();
    for(auto remainingModell : remainingModells) {
      modellList.push_back(remainingModell);
    }

    vector<Terminal*> terminalList;
    for(auto & i : compList)
      i->buildListOfTerminals(terminalList);

    vector<Terminal*> nodeList;
    for(auto & i : terminalList) {
      if(i->getNumberOfConnectedTerminals() > 2) {
        nodeList.push_back(i);
        i->setFlag(2); // root
      }
    }
    vector<Branch*> branchList;
    int k=0;
    for(auto & i : nodeList) {
      vector<Branch*> branchs_tmp = i->buildBranches(nullptr);
      for(auto & j : branchs_tmp) {
        branchList.push_back(j);
        stringstream str;
        str << "Branch" << k++;
        j->setName(str.str());
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
    branchList[0]->buildTreeBranches(nullptr, treeBranch, numberOfTreeBranches);
    for(auto & i : branchList) {
      bool flag = false;
      for(auto & j : treeBranch) {
        if(i==j)
          flag = true;
      }
      if(!flag)
        linkBranch.push_back(i);
    }
    for(auto & j : treeBranch) {
      j->setFlag(3);
    }

    vector<Mesh*> meshList;
    k=0;
    for(auto & i : linkBranch) {
      bool flag = false;
      stringstream str;
      str << "Mesh" << k++;
      Mesh* mesh = new Mesh(str.str());
      i->buildMeshes(nullptr, nullptr, mesh, flag);
      meshList.push_back(mesh);
    }

    for(auto & i : meshList) {
      i->getBranch(0)->setvz(1,i);
      for(int j=1; j<i->getNumberOfBranches(); j++) {
	if(i->getBranch(j)->getEndTerminal() == i->getBranch(j-1)->getStartTerminal())
	  i->getBranch(j)->setvz(1,i);
	else if(i->getBranch(j)->getStartTerminal() == i->getBranch(j-1)->getStartTerminal())
	  i->getBranch(j)->setvz(-1,i);
	else if(i->getBranch(j)->getStartTerminal() == i->getBranch(j-1)->getEndTerminal())
	  i->getBranch(j)->setvz(1,i);
	else if(i->getBranch(j)->getEndTerminal() == i->getBranch(j-1)->getEndTerminal())
	  i->getBranch(j)->setvz(-1,i);
	else
	  throw runtime_error("Error 2 in ElectronicComponent::processModellList");
      }
      objectList.push_back(i);
      // we need to set the path here to a dummy path since their is no path defined by the user (this element is created)
      i->setPath("[created_by_ModellingInterface_"+toString(objectList.size()-1)+"]");
    }

    for(auto & i : branchList) {
      objectList.push_back(i);
      // we need to set the path here to a dummy path since their is no path defined by the user (this element is created)
      i->setPath("[created_by_ModellingInterface_"+toString(objectList.size()-1)+"]");
      for(int j=0; j<i->getNumberOfConnectedMeshes(); j++)
	i->addDependency(i->getMesh(j));
    }

    for(auto & i : compList) {
      auto* objectcomp = dynamic_cast<Object*>(i);
      auto* linkcomp = dynamic_cast<Link*>(i);
      if(objectcomp) {
	objectList.push_back(objectcomp);
        // we need to set the path here to a dummy path since their is no path defined by the user (this element is created)
        objectcomp->setPath("[created_by_ModellingInterface_"+toString(objectList.size()-1)+"]");
	objectcomp->addDependency(i->getBranch());
      }
      else if(linkcomp) {
	linkList.push_back(linkcomp);
        // we need to set the path here to a dummy path since their is no path defined by the user (this element is created)
        linkcomp->setPath("[created_by_ModellingInterface_"+toString(linkList.size()-1)+"]");
      }
      else {
	throw runtime_error("Error 2 in ElectronicComponent::processModellList");
      }
    }
  }

  ElectronicComponent::ElectronicComponent()  {
    branch = &tmpbranch;
  }

  ElectronicComponent::~ElectronicComponent() { 
    for(auto & i : terminal)
      delete i;
  }

  void ElectronicComponent::connect(Branch *branch_, double vz_) {
    vz = vz_;
    branch=branch_;
  }

  void ElectronicComponent::updateCharge() {
    Q = branch->evalCharge()(0)*vz;
    updQ = false;
  }

  void ElectronicComponent::updateCurrent() {
    I = branch->evalCurrent()(0)*vz;
    updI = false;
  }

}
