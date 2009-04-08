/* Copyright (C) 2004-2009 MBSim Development Team
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: mfoerg@users.berlios.de
 */

#include <config.h>
#include "mbsim/special_classes.h"
#include "mbsim/object.h"
#include "mbsim/link.h"
#include "mbsim/order_one_dynamics.h"
#include "mbsim/tree.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {
  SpecialGroup::SpecialGroup(const string &name) : Group(name) {
  }

  void SpecialGroup::preinit() {
    Group::preinit();

    if(dynamic_cast<DynamicSystemSolver*>(parent)) {
      cout <<name << " (special group) preinit():" << endl;

      vector<Object*>  objList;
      vector<Link*>  lnkList;
      vector<OrderOneDynamics*>  oodList;
      buildListOfObjects(objList,true);
      buildListOfLinks(lnkList,true);
      buildListOfOrderOneDynamics(oodList,true);

      link.clear(); // Alte link-Liste löschen
      orderOneDynamics.clear(); // Alte ood-Liste löschen
      cout << "object List:" << endl;
      for(unsigned int i=0; i<objList.size(); i++) {
        cout << objList[i]->getName() << endl;
        stringstream str;
        str << objList[i]->getName() << "#" << i;
        objList[i]->setName(str.str());
      }
      cout << "link List:" << endl;
      for(unsigned int i=0; i<lnkList.size(); i++) {
        cout << lnkList[i]->getName() << endl;
        stringstream str;
        str << lnkList[i]->getName() << "#" << i;
        lnkList[i]->setName(str.str());
        addLink(lnkList[i]);
      }
      cout << "ood List:" << endl;
      for(unsigned int i=0; i<oodList.size(); i++) {
        cout << oodList[i]->getName() << endl;
        stringstream str;
        str << oodList[i]->getName() << "#" << i;
        oodList[i]->setName(str.str());
        addOrderOneDynamics(oodList[i]);
      }
      // Matrix anlegen, zeigt die Abhängigkeiten der Körper
      SqrMat A(objList.size());
      cout << A.size() << endl;
      for(unsigned int i=0; i<objList.size(); i++) {

        RigidBody *body = static_cast<RigidBody*>(objList[i]);
        FrameInterface* frame =  body->getFrameOfReference();
        DynamicSystem* parentSys = dynamic_cast<DynamicSystem*>(frame->getParent());
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
      dynamicsystem.clear(); // Alte DynamicSystem-Liste löschen
      // Starte Aufbau
      for(int i=0; i<A.size(); i++) {
        double a = max(trans(A).col(i));
        if(a==1) { // Root einer Relativkinematik
          stringstream str;
          str << "InvisibleTree" << nt++;
          // Lege unsichtbaren Tree an 
          Tree *tree = new Tree(str.str());
          bufTree.push_back(tree);
          addToTree(tree, 0, A, i, objList);
        } 
        else if(a==0) // Absolutkinematik
          addObject(objList[i]);
        //group->addObject(objList[i]);
      }

      addDynamicSystem(group,Vec(3),SqrMat(3));

      for(unsigned int i=0; i<bufTree.size(); i++) {
        addDynamicSystem(bufTree[i],Vec(3),SqrMat(3));
      }

      cout << "End of special group preinit()" << endl;
    }
  }


  void SpecialGroup::addToTree(Tree* tree, Node* node, SqrMat &A, int i, vector<Object*>& objList) {

    Node *nextNode = tree->addObject(node,objList[i]);

    for(int j=0; j<A.cols(); j++)
      if(A(i,j) == 1) // Child node of object i
        addToTree(tree, nextNode, A, j,objList);
  }
}

