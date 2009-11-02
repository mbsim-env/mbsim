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
    int searchForBranches(Terminal* callingTerminal);
    std::vector<Branch*> buildBranches(Branch* branch);
    void findEndOfBranch(Terminal* callingTerminal, Branch* currentBranch);
  };

  void connectTerminal(Terminal *terminal1, Terminal *terminal2);
  void connectBranch(Branch *branch1, Branch *branch2);

  /*class ModellingInterface {
    public:
    virtual std::string getName() const = 0;
    virtual void setName(std::string name) = 0;
    virtual void processModellList(std::vector<ModellingInterface*> &modellList, std::vector<MBSim::Object*> &objectList, std::vector<MBSim::Link*> &linkList) = 0;
    };
    */

  class ElectronicComponent : public MBSim::ModellingInterface {
    protected:
      std::vector<Terminal*> terminal;
      Branch* branch;
    public:
      ElectronicComponent() : branch(0) {}
      void addTerminal(Terminal *terminal);
      void addTerminal(const std::string &str);
      Terminal* getTerminal(const std::string &name, bool check=true);
      void buildListOfTerminals(std::vector<Terminal*> &terminal);
      void connect(Branch *branch_) {branch=branch_;}
      Branch* getBranch() {return branch;}
      void processModellList(std::vector<ModellingInterface*> &modellList, std::vector<MBSim::Object*> &objectList, std::vector<MBSim::Link*> &linkList);
  };

}

#endif

