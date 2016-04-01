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
#include "mbsim/links/floating_frame_link.h"
#include "mbsim/frames/frame.h"
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

  FloatingFrameLink::FloatingFrameLink(const std::string &name) : Link(name), frame(2), updPos(true), updVel(true), updFD(true), updF(true), updM(true), updRMV(true), updlaF(true), updlaM(true), refFrame(NULL), refFrameID(0), C("F") {
    C.setParent(this);
  }

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

  void FloatingFrameLink::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(openMBVArrowF) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=frame[1]->getPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WF = evalForce();
        data.push_back(WF(0));
        data.push_back(WF(1));
        data.push_back(WF(2));
        data.push_back(nrm2(WF));
        openMBVArrowF->append(data);
      }
      if(openMBVArrowM) {
        vector<double> data;
        data.push_back(getTime());
        Vec3 toPoint=frame[1]->getPosition();
        data.push_back(toPoint(0));
        data.push_back(toPoint(1));
        data.push_back(toPoint(2));
        Vec3 WM = evalMoment();
        data.push_back(WM(0));
        data.push_back(WM(1));
        data.push_back(WM(2));
        data.push_back(nrm2(WM));
        openMBVArrowM->append(data);
      }
#endif
      Link::plot();
    }
  }

  void FloatingFrameLink::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Link::closePlot();
    }
  }

  void FloatingFrameLink::updateW(double t, int j) {
    W[j][0] -= C.evalJacobianOfTranslation(j).T() * evalRF() + C.evalJacobianOfRotation(j).T() * evalRM();
    W[j][1] += frame[1]->evalJacobianOfTranslation(j).T() * evalRF() + frame[1]->evalJacobianOfRotation(j).T() * evalRM();
  }

  void FloatingFrameLink::updateh(double t, int j) {
    h[j][0] -= C.evalJacobianOfTranslation(j).T() * evalForce() + C.evalJacobianOfRotation(j).T() * evalMoment();
    h[j][1] += frame[1]->evalJacobianOfTranslation(j).T() * evalForce() + frame[1]->evalJacobianOfRotation(j).T() * evalMoment();
  }

  void FloatingFrameLink::updatePositions(double t) {
    WrP0P1 = frame[1]->evalPosition() - frame[0]->evalPosition();
    updPos = false;
  }

  void FloatingFrameLink::updatePositions(double t, Frame *frame_) {
    frame_->setPosition(frame[1]->evalPosition());
    frame_->setOrientation(frame[0]->evalOrientation());
  }

  void FloatingFrameLink::updateVelocities(double t) {
    WvP0P1 = frame[1]->evalVelocity() - C.evalVelocity();
    WomP0P1 = frame[1]->evalAngularVelocity() - C.evalAngularVelocity();
    updVel = false;
  }

  void FloatingFrameLink::updateGeneralizedPositions(double t) {
    rrel.set(iF, evalGlobalForceDirection().T() * evalGlobalRelativePosition());
    rrel.set(iM, x);
    updrrel = false;
  }

  void FloatingFrameLink::updateGeneralizedVelocities(double t) {
    vrel.set(iF, evalGlobalForceDirection().T() * evalGlobalRelativeVelocity());
    vrel.set(iM, evalGlobalMomentDirection().T() * evalGlobalRelativeAngularVelocity());
    updvrel = false;
  }

  void FloatingFrameLink::updateGeneralizedForces(double t) {
    lambda.set(iF, evallaF());
    lambda.set(iM, evallaM());
    updla = false;
  }

  void FloatingFrameLink::updateForce(double t) {
    F = evalGlobalForceDirection()*evalGeneralizedForce()(iF);
    updF = false;
  }

  void FloatingFrameLink::updateMoment(double t) {
    M = evalGlobalMomentDirection()*evalGeneralizedForce()(iM);
    updM = false;
  }

  void FloatingFrameLink::updateForceDirections(double t) {
    DF = refFrame->evalOrientation() * forceDir;
    DM = refFrame->evalOrientation() * momentDir;
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

  void FloatingFrameLink::updateR(double t) {
    RF.set(Index(0,2), Index(iF), evalGlobalForceDirection());
    RM.set(Index(0,2), Index(iM), evalGlobalMomentDirection());
    updRMV = false;
  }

  void FloatingFrameLink::updateg(double t) {
    g(iF) = evalGeneralizedRelativePosition()(iF);
    g(iM) = rrel(iM);;
  }

  void FloatingFrameLink::updategd(double t) {
    gd(iF) = evalGeneralizedRelativeVelocity()(iF);
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
      }
      lambda.resize(size);
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

