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

// Test-Gruppe mit Intelligenz zum Anlegen von "unsichtbaren Trees"
class SpecialGroup : public Group {

  public:
    SpecialGroup(const string &name); 

    void preinit();

    void addToTree(Tree* tree, Node* node, SqrMat &A, int i, vector<Object*> &objList);
};

#endif
