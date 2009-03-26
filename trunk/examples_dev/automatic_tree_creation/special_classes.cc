#include "special_classes.h"
//#include "mbsim/tree.h"
#include "mbsim/object.h"
#include "mbsim/frame.h"
#include "mbsim/tree.h"

SpecialGroup::SpecialGroup(const string &name) : Group(name) {
}

void SpecialGroup::preinit() {
  Group::preinit();

  cout << "Special group preinit():" << endl;

  // Matrix anlegen, zeigt die Abhängigkeiten der Körper
  SqrMat A(object.size() + subsystem.size());
  for(unsigned int i=0; i<object.size(); i++) {

    SpecialRigidBody *body = static_cast<SpecialRigidBody*>(object[i]);
    Frame* frame =  body->getFrameOfReference();
    Subsystem* parentSys = dynamic_cast<Subsystem*>(frame->getParent());
    Body* parentBody = dynamic_cast<Body*>(frame->getParent());

    if(parentSys) { // Körper hat Absolutkinematik
    }
    else if(parentBody) { // Körper hat Relativkinematik
      unsigned int j;
      for(j=0; j<object.size(); j++)
	if(object[j]->getName() == parentBody->getName())
	  break;
      A(i,j) = 2; // 2 bedeuted Vorgänger
      A(j,i) = 1; // 1 bedeuted Nachfolger
    }
    else { // sollte nicht vorkommen
      cout << "unknown type" << endl;
      throw 5;
    }
  }

  // TODO Abhängigkeiten von Subsystem noch zu implementieren
  for(unsigned int i=object.size(); i<A.size(); i++) {
    SpecialGroup *sys = static_cast<SpecialGroup*>(subsystem[i]);
  }

  // Matrix der Abhängigkeiten
  cout << "A=" << A << endl;

  // Lege unsichtbaren Tree an
  SpecialTree *tree = new SpecialTree("InvisibleTree");
  vector<Node*> node;
  vector<Object*> buf;
  for(unsigned int i=0; i<object.size(); i++) {
    node.push_back(0);
    double a = max(trans(A).col(i));
    if(a==1) { // Root einer Relativkinematik
      node[i] = tree->addObject(0,object[i]);
    } else if(a==0) // Absolutkinematik
      buf.push_back(object[i]);
  }
  // TODO AUfpassen, Reihenfolge kann variieren!!! Anderes Konzept überlegen
  for(unsigned int i=0; i<object.size(); i++) {
    for(int j=0; j<A.cols(); j++)
      if(A(i,j) == 2) {
	node[i] = tree->addObject(node[j],object[i]);
    }
  }

  object.clear(); // Alte Object-Liste löschen
  object = buf; // Neue Object-Liste enthält nur noch Körper mit Absolutkinematik

  addSubsystem(tree,Vec(3),SqrMat(3));
  //specialTree.push_back(tree);
  
 // for(unsigned int i=0; i<object.size(); i++) {
 //   cout << "Objekt " << object[i]->getName() << endl;
 // }

  cout << "End of special group preinit()" << endl;
}


// ---------------------- ALT -----------------------
//
//  SpecialTree::SpecialTree(const string &projectName) : Object(projectName) {
//  }
//
//  SpecialTree::~SpecialTree() {
//  }
//
//  Node* SpecialTree::addObject(Node* tree, Object* obj) {
//
//    Node *node = new Node(obj);
//    if(tree)
//      tree->addChild(node);
//    else
//      root = node;
//    return node;
//  }
//
//  Node* SpecialTree::addSubsystem(Node* tree, Subsystem *sys) {
//
//    Node *node = new Node(sys);
//    if(tree)
//      tree->addChild(node);
//    else
//      root = node;
//    return node;
//  }
//
//  void SpecialTree::calcqSize() {
//    qSize = 0;
//    root->calcqSize(qSize);
//  }
//
//  void SpecialTree::calcuSize(int j) {
//    uSize[j] = 0;
//    root->calcuSize(uSize[j],j);
//  }
//
//  void SpecialTree::sethSize(int hSize_, int j) {
//
//    hSize[j] = hSize_;
//    root->sethSize(hSize_,j);
//  } 
//
//  void SpecialTree::updateKinematics(double t) {
//    root->updateKinematics(t);
//  }
//
//  void SpecialTree::updateJacobians(double t) {
//    root->updateJacobians(t);
//  }
//
//  void SpecialTree::updateSecondJacobians(double t) {
//    root->updateSecondJacobians(t);
//  }
//
//  void SpecialTree::updatedu(double t, double dt) {
//
//    ud = slvLLFac(LLM, h*dt+r);
//  }
//
//  void SpecialTree::updatezd(double t) {
//
//    qd = T*u;
//    ud =  slvLLFac(LLM, h+r);
//  }
//
//  void SpecialTree::facLLM() {
//    LLM = facLL(M); 
//  }
