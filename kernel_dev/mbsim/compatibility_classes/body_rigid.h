#ifndef _BODY_RIGID_H_
#define _BODY_RIGID_H_

#include "rigid_body.h"

#define setGrav(x) setAccelerationOfGravity(x) 

namespace MBSim {

  enum Rot {cardanAngles, eulerParameters};

  class BodyRigid : public RigidBody {

    protected:
      fmatvec::SymMat I;
      fmatvec::Mat JT, JR;
      fmatvec::Vec KrKS;
      Rot rot;
      bool I_COG;
      Group *parent;

    public:
      BodyRigid(const std::string &name) : RigidBody(name), I(3), KrKS(3), rot(cardanAngles), I_COG(false), parent(0) { }
     
      void setParent(Group *sys) {parent = sys;}
      void setRotationalParameters(Rot rot_) {rot = rot_;}
      void setInertia(const fmatvec::SymMat &I_, bool I_COG_ = false) { I = I_; I_COG = I_COG_;}
      void setJT(const fmatvec::Mat &JT_) { JT = JT_; }
      void setJR(const fmatvec::Mat &JR_) { JR = JR_; }
      void setKrKS(const fmatvec::Vec &KrKS_) { KrKS = KrKS_; }

      Frame* getPort(const std::string &str) { return RigidBody::getFrame(str); }
      void addPort(const std::string &str, const fmatvec::Vec &r) { RigidBody::addFrame(str,r,fmatvec::SqrMat(3,fmatvec::EYE)); }
      void addContour(Contour* contour, const fmatvec::Vec &r) { RigidBody::addContour(contour,r,fmatvec::SqrMat(3,fmatvec::EYE)); }

      //std::string getFullName() const {return mbs ? RigidBody::getFullName() : name;}
      //void setFullName(const std::string &str) {Element::setFullName(str);}

      void calcqSize() {

	if(JT.cols() > 0)
	  RigidBody::setTranslation(new LinearTranslation(JT)); 

	if(JR.cols() == 1)
	  RigidBody::setRotation(new RotationAboutFixedAxis(fmatvec::Vec(JR)));
	else if(JR.cols() == 3) {
	  if(rot == cardanAngles)
	    RigidBody::setRotation(new CardanAngles());
	  else if(rot == eulerParameters) {
        std::cout << "TODO: Euler Parameters" << std::endl;
	    throw 5;
	  } else {
        std::cout << "Error: Unkown Rotation" << std::endl;
	    throw 5;
	  }
	}

	addFrame("B",-KrKS,fmatvec::SqrMat(3,fmatvec::EYE));
	setFrameForKinematics(getFrame("B"));

	if(I_COG == true)
	  RigidBody::setInertiaTensor(I);
	else
	  RigidBody::setInertiaTensor(I,getFrame("B"));

	RigidBody::calcqSize();
      }

     };

}

#endif
