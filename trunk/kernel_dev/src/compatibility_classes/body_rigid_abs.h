#ifndef _BODY_RIGID_ABS_H_
#define _BODY_RIGID_ABS_H_

#include "body_rigid.h"

namespace MBSim {

  class BodyRigidAbs : public BodyRigid {

    protected:
      Vec WrOK0;
      SqrMat AWK0;

    public:
      BodyRigidAbs(const string &name) : BodyRigid(name), WrOK0(3), AWK0(3,EYE) { }
     
      void setAWK0(const SqrMat &AWK0_) { AWK0 = AWK0_; }
      void setWrOK0(const Vec &WrOK0_) { WrOK0 = WrOK0_; }

      void calcSize() {

	string cosyname = string("B_") + name;
	parent->addCoordinateSystem(cosyname,WrOK0,AWK0);
	setFrameOfReference(parent->getCoordinateSystem(cosyname));

	BodyRigid::calcSize();
      }

  };

}

#endif
