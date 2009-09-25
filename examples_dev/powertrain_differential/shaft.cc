#include "shaft.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/frustum.h"
#endif

using namespace MBSim;
using namespace fmatvec;
using namespace std;

void RigidBodyWith1DRelativeMotion::init(InitStage stage) {
  if(stage==unknownStage) {
    RigidBody::init(stage);
    I.resize(gethSize());
    if(body.size()) {
      for(unsigned int i=0; i<body.size(); i++) {
	Vec Ip = body[i]->getI()*ratio[i];
	I(0,Ip.size()-1) += Ip;
      }
    }
    else {
      if(qSize) {
	assert(qSize == 1);
	I(gethSize()-1) = 1;
      }
      else {
	RigidBodyWith1DRelativeMotion* parent = dynamic_cast<RigidBodyWith1DRelativeMotion*>(frameOfReference->getParent());
	if(parent) {
	  I = parent->getI();
	}
      }
    }
  }
  else
    RigidBody::init(stage);
}

void RigidBodyWith1DRelativeMotion::updateKinematicsForSelectedFrame(double t) {
  if(body.size()) {
    sRel = 0;
    sdRel = 0;
    for(unsigned int i=0; i<body.size(); i++) {
      sRel += body[i]->getsRel()/ratio[i];
      sdRel += body[i]->getsdRel()/ratio[i];
    }
  }
  else {
    RigidBody::updateKinematicsForSelectedFrame(t);
    if(qSize) {
      assert(qSize == 1);
      sRel = q(0);
      sdRel = u(0);
    }
    else {
      RigidBodyWith1DRelativeMotion* parent = dynamic_cast<RigidBodyWith1DRelativeMotion*>(frameOfReference->getParent());
      if(parent) {
	sRel = parent->getsRel();
	sdRel = parent->getsdRel();
      }
    }
  }

  a = frameOfReference->getOrientation()*Ka;
}

void Shaft::init(InitStage stage) {
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

void Shaft::updateKinematicsForSelectedFrame(double t) {

  RigidBodyWith1DRelativeMotion::updateKinematicsForSelectedFrame(t);

  if(body.size()) {
    SqrMat A(3);
    A(0,0) = cos(sRel);
    A(0,1) = -sin(sRel);
    A(1,0) = sin(sRel);
    A(1,1) = cos(sRel);
    A(2,2) = 1;

    frame[iKinematics]->setOrientation(frameOfReference->getOrientation()*A);
    frame[iKinematics]->setAngularVelocity(frameOfReference->getAngularVelocity() + a*sdRel);
    frame[iKinematics]->setPosition(frameOfReference->getPosition());
    frame[iKinematics]->setVelocity(frameOfReference->getVelocity());
  }
}

void Shaft::updateJacobiansForSelectedFrame(double t) {

  if(body.size()) {

    frame[iKinematics]->setGyroscopicAccelerationOfTranslation(frameOfReference->getGyroscopicAccelerationOfTranslation());
    frame[iKinematics]->setGyroscopicAccelerationOfRotation(frameOfReference->getGyroscopicAccelerationOfRotation() + crossProduct(frameOfReference->getAngularVelocity(), a*sdRel));

    frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),Index(0,frameOfReference->getJacobianOfTranslation().cols()-1)) = frameOfReference->getJacobianOfTranslation();
    frame[iKinematics]->getJacobianOfRotation()(Index(0,2),Index(0,frameOfReference->getJacobianOfRotation().cols()-1)) = frameOfReference->getJacobianOfRotation();
    for(int i=0; i<I.size(); i++) {
      if(I(i) != 0) {
	frame[iKinematics]->getJacobianOfTranslation()(Index(0,2),i).init(0);
	frame[iKinematics]->getJacobianOfRotation()(Index(0,2),i) = a/I(i);
      }
    }
  }
  else {
    RigidBodyWith1DRelativeMotion::updateJacobiansForSelectedFrame(t);
  }
}

