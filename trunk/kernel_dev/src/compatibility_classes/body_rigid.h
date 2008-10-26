#ifndef _BODY_RIGID_H_
#define _BODY_RIGID_H_

#include "rigid_body.h"

#define setGrav(x) setAccelerationOfGravity(x) 

namespace MBSim {

  enum Rot {cardanAngles, eulerParameters};

  class BodyRigid : public RigidBody {

    protected:
      SymMat I;
      Mat JT, JR;
      Vec KrKS;
      Rot rot;
      bool I_COG;
      Subsystem *parent;

    public:
      BodyRigid(const string &name) : RigidBody(name), I(3), KrKS(3), rot(cardanAngles), I_COG(false), parent(0) { }
     
      void setParent(Subsystem *sys) {parent = sys;}
      void setRotationalParameters(Rot rot_) {rot = rot_;}
      void setInertia(const SymMat &I_, bool I_COG_ = false) { I = I_; I_COG = I_COG_;}
      void setJT(const Mat &JT_) { JT = JT_; }
      void setJR(const Mat &JR_) { JR = JR_; }
      void setKrKS(const Vec &KrKS_) { KrKS = KrKS_; }

      CoordinateSystem* getPort(const string &str) { return RigidBody::getCoordinateSystem(str); }
      void addPort(const string &str, const Vec &r) { RigidBody::addCoordinateSystem(str,r,SqrMat(3,EYE)); }
      void addContour(Contour* contour, const Vec &r) { RigidBody::addContour(contour,r,SqrMat(3,EYE)); }

      //string getFullName() const {return mbs ? RigidBody::getFullName() : name;}
      //void setFullName(const string &str) {Element::setFullName(str);}

      void calcqSize() {

	if(JT.cols() > 0)
	  RigidBody::setTranslation(new LinearTranslation(JT)); 

	if(JR.cols() == 1)
	  RigidBody::setRotation(new RotationAboutFixedAxis(Vec(JR)));
	else if(JR.cols() == 3) {
	  if(rot == cardanAngles)
	    RigidBody::setRotation(new CardanAngles());
	  else if(rot == eulerParameters) {
	    cout << "TODO: Euler Parameters" << endl;
	    throw 5;
	  } else {
	    cout << "Error: Unkown Rotation" << endl;
	    throw 5;
	  }
	}

	addCoordinateSystem("B",-KrKS,SqrMat(3,EYE));
	setCoordinateSystemForKinematics(getCoordinateSystem("B"));

	if(I_COG == true)
	  RigidBody::setInertiaTensor(I);
	else
	  RigidBody::setInertiaTensor(I,getCoordinateSystem("B"));

	RigidBody::calcqSize();
      }

     };

}

#endif
