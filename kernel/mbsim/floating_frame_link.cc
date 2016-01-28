/* Copyright (C) 2004-2014 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include "mbsim/floating_frame_link.h"
#include "mbsim/frame.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/arrow.h>
#include "openmbvcppinterface/objectfactory.h"
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  FloatingFrameLink::FloatingFrameLink(const std::string &name) : Link(name), updPos(true), updVel(true), updFD(true), updF(true), updM(true), updRMV(true), updlaF(true), updlaM(true), refFrame(NULL), refFrameID(0), C("F") {
    frame[0] = NULL;
    frame[1] = NULL;
    C.setParent(this);
  }

  FloatingFrameLink::~FloatingFrameLink() {}

  void FloatingFrameLink::resetUpToDate() { 
    Link::resetUpToDate(); 
    updPos = true; 
    updVel = true; 
    updFD = true; 
    updF = true; 
    updM = true; 
    updRMV = true; 
    updlaF = true;
    updlaM = true;
    C.resetUpToDate();  
  }

  void FloatingFrameLink::calclaSize(int j) {
    Link::calclaSize(j);
    laSize = forceDir.cols() + momentDir.cols();
  }

  void FloatingFrameLink::calcgSize(int j) {
    Link::calcgSize(j);
    gSize = forceDir.cols() + momentDir.cols();
  }

  void FloatingFrameLink::calcgdSize(int j) {
    Link::calcgdSize(j);
    gdSize = forceDir.cols() + momentDir.cols();
  }

  void FloatingFrameLink::calcrFactorSize(int j) {
    Link::calcrFactorSize(j);
    rFactorSize = isSetValued() ? forceDir.cols() + momentDir.cols() : 0;
  }

  void FloatingFrameLink::calccorrSize(int j) {
    Link::calccorrSize(j);
    corrSize = forceDir.cols() + momentDir.cols();
  }

  void FloatingFrameLink::updatedhdz(double t) {
    THROW_MBSIMERROR("Internal error");
  }

  void FloatingFrameLink::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(openMBVArrowF) {
        vector<double> data;
        data.push_back(t); 
        Vec3 toPoint=frame[1]->getPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WF = getForce(t);
        data.push_back(WF(0));
        data.push_back(WF(1));
        data.push_back(WF(2));
        data.push_back(nrm2(WF));
        openMBVArrowF->append(data);
      }
      if(openMBVArrowM) {
        vector<double> data;
        data.push_back(t); 
        Vec3 toPoint=frame[1]->getPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WM = getMoment(t);
        data.push_back(WM(0));
        data.push_back(WM(1));
        data.push_back(WM(2));
        data.push_back(nrm2(WM));
        openMBVArrowM->append(data);
      }
#endif
      Link::plot(t,dt);
    }
  }

  void FloatingFrameLink::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Link::closePlot();
    }
  }

  void FloatingFrameLink::updatewb(double t) {
//    Mat3xV WJT = refFrame->getOrientation(t) * JT;
//    VecV sdT = WJT.T() * (getGlobalRelativeVelocity(t));
//
//    wb(0, DF.cols() - 1) += getGlobalForceDirection(t).T() * (frame[1]->getGyroscopicAccelerationOfTranslation(t) - C.getGyroscopicAccelerationOfTranslation(t) - crossProduct(C.getAngularVelocity(t), getGlobalRelativeVelocity(t) + WJT * sdT));
//    wb(DF.cols(), DM.cols() + DF.cols() - 1) += getGlobalMomentDirection(t).T() * (frame[1]->getGyroscopicAccelerationOfRotation(t) - C.getGyroscopicAccelerationOfRotation(t) - crossProduct(C.getAngularVelocity(t), getGlobalRelativeAngularVelocity(t)));
  }

  void FloatingFrameLink::updateW(double t, int j) {
    W[j][0] -= C.getJacobianOfTranslation(t,j).T() * getSetValuedForceDirection(t) + C.getJacobianOfRotation(t,j).T() * getSetValuedMomentDirection(t);
    W[j][1] += frame[1]->getJacobianOfTranslation(t,j).T() * getSetValuedForceDirection(t) + frame[1]->getJacobianOfRotation(t,j).T() * getSetValuedMomentDirection(t);
  }

  void FloatingFrameLink::updateh(double t, int j) {
    h[j][0] -= C.getJacobianOfTranslation(t,j).T() * getForce(t) + C.getJacobianOfRotation(t,j).T() * getMoment(t);
    h[j][1] += frame[1]->getJacobianOfTranslation(t,j).T() * getForce(t) + frame[1]->getJacobianOfRotation(t,j).T() * getMoment(t);
  }

  void FloatingFrameLink::updatePositions(double t) {
    WrP0P1 = frame[1]->getPosition(t) - frame[0]->getPosition(t);
    C.setPosition(frame[1]->getPosition());
    C.setOrientation(frame[0]->getOrientation());
    updPos = false;
  }

  void FloatingFrameLink::updateVelocities(double t) {
    WvP0P1 = frame[1]->getVelocity(t) - C.getVelocity(t);
    WomP0P1 = frame[1]->getAngularVelocity(t) - C.getAngularVelocity(t);
    updVel = false;
  }

  void FloatingFrameLink::updateGeneralizedPositions(double t) {
    rrel.set(iF, getGlobalForceDirection(t).T() * getGlobalRelativePosition(t));
    rrel.set(iM, x);
    updrrel = false;
  }

  void FloatingFrameLink::updateGeneralizedVelocities(double t) {
    vrel.set(iF, getGlobalForceDirection(t).T() * getGlobalRelativeVelocity(t));
    vrel.set(iM, getGlobalMomentDirection(t).T() * getGlobalRelativeAngularVelocity(t));
    updvrel = false;
  }

  void FloatingFrameLink::updateGeneralizedForce(double t) {
    laSV.set(iF, getGeneralizedForceForce(t));
    laSV.set(iM, getGeneralizedMomentForce(t));
    updlaSV = false;
  }

  void FloatingFrameLink::updateForce(double t) {
    F = getGlobalForceDirection(t)*getGeneralizedForce(t)(iF);
    updF = false;
  }

  void FloatingFrameLink::updateMoment(double t) {
    M = getGlobalMomentDirection(t)*getGeneralizedForce(t)(iM);
    updM = false;
  }

  void FloatingFrameLink::updateForceDirections(double t) {
    DF = refFrame->getOrientation(t) * forceDir;
    DM = refFrame->getOrientation(t) * momentDir;
    updFD = false;
  }

  void FloatingFrameLink::updateWRef(const Mat& WParent, int j) {
    for(unsigned i=0; i<2; i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1); // TODO Prüfen ob hSize
      W[j][i]>>WParent(I,J);
    }
  } 

  void FloatingFrameLink::updateVRef(const Mat& VParent, int j) {
    for(unsigned i=0; i<2; i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1);
      V[j][i]>>VParent(I,J);
    }
  } 

  void FloatingFrameLink::updatehRef(const Vec &hParent, int j) {
    for(unsigned i=0; i<2; i++) {
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1);
      h[j][i]>>hParent(I);
    }
  } 

  void FloatingFrameLink::updatedhdqRef(const fmatvec::Mat& dhdqParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void FloatingFrameLink::updatedhduRef(const fmatvec::SqrMat& dhduParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void FloatingFrameLink::updatedhdtRef(const fmatvec::Vec& dhdtParent, int j) {
    for(unsigned i=0; i<2; i++) {
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1);
      dhdt[i]>>dhdtParent(I);
    }
  }

  void FloatingFrameLink::updaterRef(const Vec &rParent, int j) {
    for(unsigned i=0; i<2; i++) {
      int hInd =  frame[i]->gethInd(j);
      Index I = Index(hInd,hInd+frame[i]->gethSize(j)-1);
      r[j][i]>>rParent(I);
    }
  } 

  void FloatingFrameLink::updateSetValuedForceDirections(double t) { 
    RF.set(Index(0,2), Index(iF), getGlobalForceDirection(t));
    RM.set(Index(0,2), Index(iM), getGlobalMomentDirection(t));
    updRMV = false;
  }

  void FloatingFrameLink::updateg(double t) {
    g(iF) = getGeneralizedRelativePosition(t)(iF);
    g(iM) = rrel(iM);;
  }

  void FloatingFrameLink::updategd(double t) {
    gd(iF) = getGeneralizedRelativeVelocity(t)(iF);
    gd(iM) = vrel(iM);
  }

  void FloatingFrameLink::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      Link::init(stage);
    }
    else if(stage==resize) {
      Link::init(stage);
      int size = forceDir.cols() + momentDir.cols();
      iF = Index(0, forceDir.cols() - 1);
      iM = Index(forceDir.cols(), forceDir.cols() + momentDir.cols() - 1);
      rrel.resize(size);
      vrel.resize(size);
      if(isSetValued()) {
        g.resize(size);
        gd.resize(size);
        RF.resize(size);
        RM.resize(size);
        la.resize(size);
        laMV.resize(size);
      }
      laSV.resize(size);
      lambdaF.resize(forceDir.cols());
      lambdaM.resize(momentDir.cols());
      for(unsigned int i=0; i<2; i++) {
        W[i].resize(2);
        V[i].resize(2);
        h[i].resize(2);
        r[i].resize(2);
      }
    }
    else if(stage==unknownStage) {
      Link::init(stage);

      if(frame[0]==NULL or frame[1]==NULL)
        THROW_MBSIMERROR("Not all connections are given!");

      if (forceDir.cols()) DF = forceDir;
      if (momentDir.cols()) DM = momentDir;

      refFrame = refFrameID ? frame[1] : frame[0];
      C.setFrameOfReference(frame[0]);
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        openMBVForceGrp=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
        openMBVForceGrp->setExpand(false);
        openMBVForceGrp->setName(name+"_ArrowGroup");
        parent->getOpenMBVGrp()->addObject(openMBVForceGrp);
        if(openMBVArrowF) {
          openMBVArrowF->setName("Force");
          openMBVForceGrp->addObject(openMBVArrowF);
        }
        if(openMBVArrowM) {
          openMBVArrowM->setName("Moment");
          openMBVForceGrp->addObject(openMBVArrowM);
        }
#endif
        Link::init(stage);
      }
    }
    else
      Link::init(stage);
  }

  void FloatingFrameLink::initializeUsingXML(DOMElement *element) {
    Link::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"frameOfReferenceID");
    if (e)
      refFrameID = getInt(e);
    e = E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1 = E(e)->getAttribute("ref1");
    saved_ref2 = E(e)->getAttribute("ref2");
#ifdef HAVE_OPENMBVCPPINTERFACE
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toHead, OpenMBV::Arrow::toPoint, 1, 1);
      setOpenMBVForce(ombv.createOpenMBV(e));
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVMoment");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]", 0, OpenMBV::Arrow::toDoubleHead, OpenMBV::Arrow::toPoint, 1, 1);
      setOpenMBVMoment(ombv.createOpenMBV(e));
    }
#endif
  }

}

