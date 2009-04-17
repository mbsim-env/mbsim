#ifndef _MODELING_CLASSES_H
#define _MODELING_CLASSES_H

#include "mbsim/element.h"
#include <string>
#include <vector>

class Component;
class Branch;

class Pin : public MBSim::Element {
  std::vector<Pin*> connectedPin;
  int flag;
  Component* parent;
  Branch* branch;
  public:
    Pin(const std::string &name) : Element(name), flag(0), parent(0), branch(0) {}
    void addConnectedPin(Pin* pin);
    void setFlag(int f) { flag = f; }
    int getFlag() const { return flag; }
    void setParent(Component* p) { parent = p; }
    Component* getParent() const { return parent; }
    int getNumberOfConnectedPins() const {return connectedPin.size();}
    int searchForBranches(Pin* callingPin);
    vector<Branch*> buildBranches(Pin* callingPin, Branch* branch);
    void setBranch(Branch *branch_) {branch = branch_;}
    Branch* getBranch() {return branch;}
};

void connectPin(Pin *pin1, Pin *pin2);
void connectBranch(Branch *branch1, Branch *branch2);

class Component {
  protected:
    std::vector<Pin*> pin;
    std::vector<Branch*> branch;
  public:
    Component() : branch(0) {
      addPin("A");
      addPin("B");
     connectPin(pin[0],pin[1]);
    }
    virtual std::string getName() const = 0;
    virtual void setName(std::string name) = 0;
    void addPin(Pin *pin);
    void addPin(const std::string &str);
    Pin* getPin(const std::string &name, bool check=true);
    void buildListOfPins(std::vector<Pin*> &pin, bool recursive = true);
    void connect(Branch *branch_) {branch.push_back(branch_);}
    Branch* getBranch() {return branch[0];}
};

#endif

