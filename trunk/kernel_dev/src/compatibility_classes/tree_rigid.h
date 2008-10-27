#ifndef _TREE_RIGID_H_
#define _TREE_RIGID_H_

#include "tree.h"
#include "body_rigid_rel.h"

namespace MBSim {

  class TreeRigid : public Tree {

    protected:
      Subsystem* parent;

    public:
      TreeRigid(const string &name) : Tree(name) { }
      void setParent(Subsystem *sys) {parent = sys;}
      void setRoot(BodyRigidRel *body) { addObject(body); }
  };

}

#endif
