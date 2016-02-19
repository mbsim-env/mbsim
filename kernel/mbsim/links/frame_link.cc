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
#include "mbsim/links/frame_link.h"
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

  FrameLink::FrameLink(const std::string &name) : Link(name), updPos(true), updVel(true), updFD(true), updF(true), updRMV(true), updlaF(true), updlaM(true) {
    frame[0] = NULL;
    frame[1] = NULL;
  }

  void FrameLink::resetUpToDate() { 
    Link::resetUpToDate();
    updPos = true;
    updVel = true;
    updFD = true;
    updF = true;
    updRMV = true;
    updlaF = true;
    updlaM = true;
  }

  void FrameLink::updatedhdz(double t) {
    THROW_MBSIMERROR("Internal error");
  }

  void FrameLink::plot(double t, double dt) {
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
#endif
      Link::plot(t,dt);
    }
  }

  void FrameLink::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Link::closePlot();
    }
  }

  void FrameLink::updateWRef(const Mat& WParent, int j) {
    for(unsigned i=0; i<2; i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1); // TODO Prüfen ob hSize
      W[j][i]>>WParent(I,J);
    }
  } 

  void FrameLink::updateVRef(const Mat& VParent, int j) {
    for(unsigned i=0; i<2; i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1);
      V[j][i]>>VParent(I,J);
    }
  } 

  void FrameLink::updatehRef(const Vec &hParent, int j) {
    for(unsigned i=0; i<2; i++) {
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1);
      h[j][i]>>hParent(I);
    }
  } 

  void FrameLink::updatedhdqRef(const fmatvec::Mat& dhdqParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void FrameLink::updatedhduRef(const fmatvec::SqrMat& dhduParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void FrameLink::updatedhdtRef(const fmatvec::Vec& dhdtParent, int j) {
    for(unsigned i=0; i<2; i++) {
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1);
      dhdt[i]>>dhdtParent(I);
    }
  }

  void FrameLink::updaterRef(const Vec &rParent, int j) {
    for(unsigned i=0; i<2; i++) {
      int hInd =  frame[i]->gethInd(j);
      Index I = Index(hInd,hInd+frame[i]->gethSize(j)-1);
      r[j][i]>>rParent(I);
    }
  } 

  void FrameLink::updateGeneralizedForces(double t) {
    lambda = getlaF(t);
    updla = false;
  }

  void FrameLink::updateForce(double t) {
    F = getGlobalForceDirection(t)*getGeneralizedForce(t)(iF);
    updF = false;
  }

  void FrameLink::updateForceDirections(double t) {
    if(getGeneralizedRelativePosition(t)(0)>epsroot())
      DF=getGlobalRelativePosition(t)/rrel(0);
    else
      DF.init(0);
    updFD = false;
  }

  void FrameLink::updateh(double t, int j) {
    h[j][0]-=frame[0]->getJacobianOfTranslation(t,j).T()*getForce(t);
    h[j][1]+=frame[1]->getJacobianOfTranslation(t,j).T()*getForce(t);
  }
  
  void FrameLink::updatePositions(double t) {
    WrP0P1=frame[1]->getPosition(t) - frame[0]->getPosition(t);
    updPos = false;
  }

  void FrameLink::updateVelocities(double t) {
    WvP0P1=frame[1]->getVelocity(t) - frame[0]->getVelocity(t);
    updVel = false;
  }

  void FrameLink::updateGeneralizedPositions(double t) {
    rrel(0)=nrm2(getGlobalRelativePosition(t));
    updrrel = false;
  }

  void FrameLink::updateGeneralizedVelocities(double t) {
    vrel=getGlobalForceDirection(t).T()*getGlobalRelativeVelocity(t);
    updvrel = false;
  }

  void FrameLink::updateR(double t) {
    RF.set(Index(0,2), Index(iF), getGlobalForceDirection(t));
    updRMV = false;
  }

  void FrameLink::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      Link::init(stage);
    }
    else if(stage==resize) {
      Link::init(stage);
      iF = Index(0, 0);
      rrel.resize(1);
      vrel.resize(1);
      if(isSetValued()) {
        g.resize(1);
        gd.resize(1);
        RF.resize(1);
        la.resize(1);
      }
      lambda.resize(1);
      lambdaF.resize(1);
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

  void FrameLink::initializeUsingXML(DOMElement *element) {
    Link::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1=E(e)->getAttribute("ref1");
    saved_ref2=E(e)->getAttribute("ref2");
    e = E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBVForce");
    if (e) {
      OpenMBVArrow ombv("[-1;1;1]",0,OpenMBV::Arrow::toHead,OpenMBV::Arrow::toPoint,1,1);
      setOpenMBVForce(ombv.createOpenMBV(e));
    }
  }

}

