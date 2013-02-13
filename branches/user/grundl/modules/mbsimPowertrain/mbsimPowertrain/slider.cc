#include "slider.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimPowertrain {

  void Slider::init(InitStage stage) {
    if(stage==unknownStage) {
      RigidBodyWith1DRelativeMotion::init(stage);
      //    OpenMBV::Frustum *cylinder=new OpenMBV::Frustum;
      //    cylinder->setTopRadius(radius);
      //    cylinder->setBaseRadius(radius);
      //    cylinder->setHeight(length);
      //    cylinder->setStaticColor(color);
      //    setOpenMBVRigidBody(cylinder);
      //    cylinder->setInitialTranslation(0,0,length/2);
    }
    else
      RigidBodyWith1DRelativeMotion::init(stage);
  }

  void Slider::updateKinematicsForSelectedFrame(double t) {

    RigidBodyWith1DRelativeMotion::updateKinematicsForSelectedFrame(t);

    if(body.size()) {
      frame[iKinematics]->setOrientation(frameOfReference->getOrientation());
      frame[iKinematics]->setAngularVelocity(frameOfReference->getAngularVelocity());
      frame[iKinematics]->setPosition(frameOfReference->getPosition() + a*sRel);
      frame[iKinematics]->setVelocity(frameOfReference->getVelocity() + a*sdRel + crossProduct(frameOfReference->getAngularVelocity(),a*sRel));
    }
  }

  void Slider::updateJacobiansForSelectedFrame(double t) {

    if(body.size()) {

      frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation());
      frame[iKinematics]->setGyroscopicAccelerationOfRotation(frameOfReference->getGyroscopicAccelerationOfRotation());

      Vec WrPK = a*sRel;
      Vec WvPKrel = a*sdRel;
      SqrMat tWrPK = tilde(WrPK);
      frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation() - tWrPK*frameOfReference->getGyroscopicAccelerationOfRotation() + crossProduct(frameOfReference->getAngularVelocity(), 2*WvPKrel+crossProduct(frameOfReference->getAngularVelocity(),WrPK)));

      frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(0,frameOfReference->getJacobianOfTranslation().cols()-1)) = frameOfReference->getJacobianOfTranslation() - tWrPK*frameOfReference->getJacobianOfRotation();
      frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(0,frameOfReference->getJacobianOfRotation().cols()-1)) = frameOfReference->getJacobianOfRotation();

      for(int i=0; i<I.size(); i++) {
	if(I(i) != 0) {
	  frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),i) = a/I(i);
	  frame[iKinematics]->getJacobianOfRotation()(Index(0,2),i).init(0); 
	}
      }
    }
    else {
      RigidBodyWith1DRelativeMotion::updateJacobiansForSelectedFrame(t);
    }
  }

}

