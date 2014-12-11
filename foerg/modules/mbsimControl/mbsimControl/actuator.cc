/* Copyright (C) 2004-2009 MBSim Development Team
 *
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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsimControl/actuator.h"
#include "mbsimControl/signal_.h"
#include "mbsim/frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimControl {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(Actuator, MBSIMCONTROL%"Actuator")

  Actuator::Actuator(const string &name) : LinkMechanics(name), signal(0), KOSYID(1), refFrame(0) {
  }

  void Actuator::updateh(double t, int j) {
    Vec3 WrP0P1 = frame[1]->getPosition()-frame[0]->getPosition();
    Mat3x3 tWrP0P1 = tilde(WrP0P1);

    C.setOrientation(frame[0]->getOrientation());
    C.setPosition(frame[0]->getPosition() + WrP0P1);
    C.setAngularVelocity(frame[0]->getAngularVelocity());
    C.setVelocity(frame[0]->getVelocity() + crossProduct(frame[0]->getAngularVelocity(),WrP0P1));
    C.setJacobianOfTranslation(frame[0]->getJacobianOfTranslation(j) - tWrP0P1*frame[0]->getJacobianOfRotation(j),j);
    C.setJacobianOfRotation(frame[0]->getJacobianOfRotation(j),j);
    C.setGyroscopicAccelerationOfTranslation(frame[0]->getGyroscopicAccelerationOfTranslation(j) - tWrP0P1*frame[0]->getGyroscopicAccelerationOfRotation(j) + crossProduct(frame[0]->getAngularVelocity(),crossProduct(frame[0]->getAngularVelocity(),WrP0P1)),j);
    C.setGyroscopicAccelerationOfRotation(frame[0]->getGyroscopicAccelerationOfRotation(j),j);

    la = signal->getSignal();
    WF[1] = refFrame->getOrientation()*forceDir*la(IT);
    WF[0] = -WF[1];
    WM[1] = refFrame->getOrientation()*momentDir*la(IR);
    WM[0] = -WM[1];

    h[j][0] += C.getJacobianOfTranslation(j).T()*WF[0] + C.getJacobianOfRotation(j).T()*WM[0];
    h[j][1] += frame[1]->getJacobianOfTranslation(j).T()*WF[1] + frame[1]->getJacobianOfRotation(j).T()*WM[1];
  }

  void Actuator::init(InitStage stage) {
    if (stage==resolveXMLPath) {
      if(saved_inputSignal!="")
        setSignal(getByPath<Signal>(saved_inputSignal));
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      if(not(frame.size()))
        THROW_MBSIMERROR("no connection given!");
      LinkMechanics::init(stage);
    }
    else if(stage==preInit) {
      LinkMechanics::init(stage);
      for(unsigned int i=0; i<frame.size(); i++)
        addDependencies(frame[i]->getDependencies());
      addDependency(signal);
    }
    else if (stage==resize) {
      LinkMechanics::init(stage);
      IT = Index(0,forceDir.cols()-1);
      IR = Index(forceDir.cols(),forceDir.cols()+momentDir.cols()-1);
    }
    else if(stage==unknownStage) {
      LinkMechanics::init(stage);
      refFrame = KOSYID==1?frame[0]:frame[1];
      C.getJacobianOfTranslation(0).resize(frame[0]->getJacobianOfTranslation(0).cols());
      C.getJacobianOfRotation(0).resize(frame[0]->getJacobianOfRotation(0).cols());
      C.getJacobianOfTranslation(1).resize(frame[0]->getJacobianOfTranslation(1).cols());
      C.getJacobianOfRotation(1).resize(frame[0]->getJacobianOfRotation(1).cols());
    }
    else
      LinkMechanics::init(stage);
  }

  void Actuator::calclaSize(int j) {
    LinkMechanics::calclaSize(j);
    laSize = forceDir.cols()+momentDir.cols(); // cols = columns
  }

  void Actuator::connect(Frame *frame0, Frame* frame1) {
    LinkMechanics::connect(frame0);
    LinkMechanics::connect(frame1);
  }

  void Actuator::setForceDirection(const Mat &fd) {
    assert(fd.rows() == 3);

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.set(i, forceDir.col(i)/nrm2(fd.col(i)));
  }

  void Actuator::setMomentDirection(const Mat &md) {
    assert(md.rows() == 3);

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.set(i, momentDir.col(i)/nrm2(md.col(i)));
  }
  
  void Actuator::initializeUsingXML(DOMElement *element) {
    LinkMechanics::initializeUsingXML(element);
    DOMElement *e;
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"forceDirection");
    if(e) setForceDirection(getMat(e,3,0));
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"momentDirection");
    if(e) setMomentDirection(getMat(e,3,0));
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"referenceFrame");
    if(e) setKOSY(getInt(e));
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"inputSignal");
    saved_inputSignal=E(e)->getAttribute("ref");
    e=E(element)->getFirstElementChildNamed(MBSIMCONTROL%"connect");
    saved_ref1=E(e)->getAttribute("ref1");
    saved_ref2=E(e)->getAttribute("ref2");

#ifdef HAVE_OPENMBVCPPINTERFACE
    e = E(element)->getFirstElementChildNamed(MBSIMCONTROL%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      std::vector<bool> which; which.resize(2, false);
      which[1]=true;
      LinkMechanics::setOpenMBVForceArrow(ombv.createOpenMBV(e), which);
    }

    e = E(element)->getFirstElementChildNamed(MBSIMCONTROL%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toDoubleHead,OpenMBV::Arrow::toPoint,1,1);
      std::vector<bool> which; which.resize(2, false);
      which[1]=true;
      LinkMechanics::setOpenMBVMomentArrow(ombv.createOpenMBV(e), which);
    }
#endif
  }

}

