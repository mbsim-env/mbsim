#ifndef _BODY_RIGID_REL_H_
#define _BODY_RIGID_REL_H_

#include "body_rigid.h"

namespace MBSim {

  class BodyRigidRel : public BodyRigid {

    private:
      Vec PrPK0;
      SqrMat APK0;
      BodyRigidRel *predecessor;

    public:
      BodyRigidRel(const string &name) : BodyRigid(name), PrPK0(3), APK0(3,EYE), predecessor(0) { }
     
      void setAPK0(const SqrMat &APK0_) { APK0 = APK0_; }
      void setPrPK0(const Vec &PrPK0_) { PrPK0 = PrPK0_; }
      void addChild(BodyRigidRel* body) {  parent->addObject(body); body->setPredecessor(this); }
      void setPredecessor(BodyRigidRel* predecessor_) { predecessor = predecessor_;}

      void calcqSize() {

	string cosyname = string("B_") + name;
	if(predecessor) {
	  predecessor->addFrame(cosyname,PrPK0,APK0,predecessor->getFrame("B"));
	  setFrameOfReference(predecessor->getFrame(cosyname));
	} else {
	  parent->addFrame(cosyname,PrPK0,APK0);
	  setFrameOfReference(parent->getFrame(cosyname));
	}

	BodyRigid::calcqSize();
      }

  };

}

#endif
