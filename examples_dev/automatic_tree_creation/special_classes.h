#ifndef _SPECIAL_GROUP_H
#define _SPECIAL_GROUP_H

#include "mbsim/multi_body_system.h"
#include "mbsim/rigid_body.h"
#include "mbsim/tree.h"
#include <string>


using namespace std;
using namespace MBSim;

// Test-RigidBody mit kleinen Erweiterungen
class SpecialRigidBody : public RigidBody {

  public:
    SpecialRigidBody(const string &name) : RigidBody(name) {} 
    Frame* getFrameForKinematics() { return port[iRef]; };
    Frame* getFrameOfReference() { return frameParent; };
};

// unsichtbarer Test-Tree 
class SpecialTree : public Tree {
  public:
    SpecialTree(const string &name) : Tree(name) {}
    ~SpecialTree() {};

    //void setFullName(const string &str) {
    //  Element::setFullName(str);
    //  for(unsigned i=0; i<subsystem.size(); i++) {
    //    cout << getFullName() << endl;
    //    cout << parent->getFullName() << endl;
    //    subsystem[i]->setFullName(parent->getFullName() + "." + subsystem[i]->getName());
    //  }
    //  for(unsigned i=0; i<object.size(); i++) {
    //    cout << getFullName() << endl;
    //    cout << parent->getFullName() << endl;
    //    object[i]->setFullName(parent->getFullName() + "." + object[i]->getName());
    //    cout << object[i]->getFullName()<<endl;
    //  }
};

// Test-Gruppe mit Intelligenz zum Anlegen von "unsichtbaren Trees"
class SpecialGroup : public Group {

  //protected:
    //vector<SpecialTree*> specialTree;
    
  public:
    SpecialGroup(const string &name); 

    void preinit();

    //  void updateKinematics(double t) {
    //    Group::updateKinematics(t);
    //    for(unsigned i=0; i<specialTree.size(); i++)
    //      specialTree[i]->updateKinematics(t);
    //  }

};


/// --------------------- ALT ---------------------------

//namespace MBSim {
//class Node;
//}

//  class SpecialTree : public Object {
//
//    protected:
//      Node* root;
//
//    public:
//
//    SpecialTree(const string &projectName);
//    ~SpecialTree();
//
//    void calcqSize();
//    void calcuSize(int j=0);
//    void sethSize(int h, int j=0);
//
//    void facLLM(); 
//    void updatezd(double t);
//    void updatedu(double t, double dt);
//    void updateKinematics(double t);
//    void updateJacobians(double t);
//    void updateSecondJacobians(double t);
//
//    Node* addObject(Node* node, Object* obj);
//    Node* addSubsystem(Node* node, Subsystem* sys);
//
//    virtual string getType() const {return "SpecialTree";}
//  };

#endif
