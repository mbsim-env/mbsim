#include "special_classes.h"
//#include "mbsim/tree.h"
#include "mbsim/object.h"
#include "mbsim/frame.h"
#include "mbsim/tree.h"

SpecialGroup::SpecialGroup(const string &name) : Group(name) {
}

void SpecialGroup::preinit() {
  Group::preinit();

  if(dynamic_cast<MultiBodySystem*>(parent)) {
    cout <<name << " (special group) preinit():" << endl;


    vector<Object*>  objList;
    buildListOfObjects(objList,true);

    for(unsigned int i=0; i<objList.size(); i++) {
      cout << objList[i]->getName() << endl;
    }
    // Matrix anlegen, zeigt die Abhängigkeiten der Körper
    SqrMat A(objList.size());
    cout << A.size() << endl;
    for(unsigned int i=0; i<objList.size(); i++) {

      RigidBody *body = static_cast<RigidBody*>(objList[i]);
      Frame* frame =  body->getFrameOfReference();
      Subsystem* parentSys = dynamic_cast<Subsystem*>(frame->getParent());
      Body* parentBody = dynamic_cast<Body*>(frame->getParent());

      if(parentSys) { // Körper hat Absolutkinematik
      }
      else if(parentBody) { // Körper hat Relativkinematik
	unsigned int j=0;
	bool foundBody = false;
	for(unsigned int k=0; k<objList.size(); k++, j++) {
	  if(objList[k] == parentBody) {
	    foundBody = true;
	    break;
	  }
	}

	if(foundBody) {
	  A(i,j) = 2; // 2 bedeuted Vorgänger
	  A(j,i) = 1; // 1 bedeuted Nachfolger
	}
      }
      else { // sollte nicht vorkommen
	cout << "unknown type" << endl;
	throw 5;
      }
    }
    // Matrix der Abhängigkeiten
    cout << "A=" << A << endl;
    // Tree Liste
    vector<Tree*> bufTree;
    int nt = 0;
    // Lege unsichtbare Group an 
    Group* group = new Group("InvisibleGroup");
    object.clear(); // Alte Object-Liste löschen
    subsystem.clear(); // Alte Subsystem-Liste löschen
    // Starte Aufbau
    for(unsigned int i=0; i<A.size(); i++) {
      double a = max(trans(A).col(i));
      if(a==1) { // Root einer Relativkinematik
	stringstream str;
	str << "InvisibleTree" << nt++;
	// Lege unsichtbaren Tree an 
	Tree *tree = new Tree(str.str());
	bufTree.push_back(tree);
	//addSubsystem(tree,Vec(3),SqrMat(3));
	addToTree(tree, 0, A, i, objList);
      } 
      else if(a==0) // Absolutkinematik
	group->addObject(objList[i]);
    }


    addSubsystem(group,Vec(3),SqrMat(3));
    for(unsigned int i=0; i<bufTree.size(); i++) {
      addSubsystem(bufTree[i],Vec(3),SqrMat(3));
    }
    cout << "End of special group preinit()" << endl;
  }
}


void SpecialGroup::addToTree(Tree* tree, Node* node, SqrMat &A, int i, vector<Object*>& objList) {

  Node *nextNode;
  stringstream str;
  str << objList[i]->getName() << i;
  objList[i]->setName(str.str());
  nextNode = tree->addObject(node,objList[i]);

  for(int j=0; j<A.cols(); j++)
    if(A(i,j) == 1) // Child node of object i
      addToTree(tree, nextNode, A, j,objList);
}

