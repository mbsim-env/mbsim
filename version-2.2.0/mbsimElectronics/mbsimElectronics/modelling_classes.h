#ifndef _MODELING_CLASSES_H
#define _MODELING_CLASSES_H

#include "mbsim/element.h"
#include "mbsim/modelling_interface.h"
#include <string>
#include <vector>

namespace MBSim {
  class Object;
  class Link;
}

namespace MBSimElectronics {

  class ElectronicComponent;
  class Branch;

  class Terminal : public MBSim::Element {
    std::vector<Terminal*> connectedTerminal;
    std::vector<Branch*> connectedBranch;
    int flag;
    ElectronicComponent* parent;
    public:
    Terminal(const std::string &name) : Element(name), flag(0), parent(0) {}
    void addConnectedTerminal(Terminal* terminal);
    void addConnectedBranch(Branch* branch);
    void setFlag(int f) { flag = f; }
    int getFlag() const { return flag; }
    void setParent(ElectronicComponent* p) { parent = p; }
    ElectronicComponent* getParent() const { return parent; }
    int getNumberOfConnectedTerminals() const {return connectedTerminal.size();}
    int getNumberOfConnectedBranches() const {return connectedBranch.size();}
    Branch* getBranch(int i) {return connectedBranch[i];}
    std::vector<Branch*> buildBranches(Branch* branch);
    void findEndOfBranch(Terminal* callingTerminal, Branch* currentBranch);
  };

  void connectTerminal(Terminal *terminal1, Terminal *terminal2);
  void connectBranch(Branch *branch1, Branch *branch2);

  class ElectronicComponent : public MBSim::ModellingInterface {
    protected:
      std::vector<Terminal*> terminal;
      Branch* branch;
      int vz;
      double Q,I;
    public:
      ElectronicComponent();
      void addTerminal(Terminal *terminal);
      void addTerminal(const std::string &str);
      Terminal* getTerminal(const std::string &name, bool check=true);
      void buildListOfTerminals(std::vector<Terminal*> &terminal);
      void connect(Branch *branch_,int vz=0);
      Branch* getBranch() {return branch;}
      void processModellList(std::vector<ModellingInterface*> &modellList, std::vector<MBSim::Object*> &objectList, std::vector<MBSim::Link*> &linkList);
      int getvz() const { return vz;}
  };

}

#endif

