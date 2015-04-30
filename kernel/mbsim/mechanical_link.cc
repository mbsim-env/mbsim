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
#include "mbsim/mechanical_link.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/contour.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/utils.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/arrow.h>
#endif

using namespace fmatvec;
using namespace std;
using namespace boost;

namespace MBSim {

  MechanicalLink::MechanicalLink(const std::string &name) : Link(name) {
  }

  MechanicalLink::~MechanicalLink() {}

  void MechanicalLink::updatedhdz(double t) {
    THROW_MBSIMERROR("Internal error");
  }

  void MechanicalLink::plot(double t, double dt) {
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

  void MechanicalLink::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Link::closePlot();
    }
  }

  void MechanicalLink::updateWRef(const Mat& WParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->getJacobianOfTranslation(j).cols()-1); // TODO Prüfen ob hSize
      W[j][i]>>WParent(I,J);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->gethSize(j)-1);
      W[j][i]>>WParent(I,J);
    }
  } 

  void MechanicalLink::updateVRef(const Mat& VParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->getJacobianOfTranslation(j).cols()-1);
      V[j][i]>>VParent(I,J);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index J = Index(laInd,laInd+laSize-1);
      Index I = Index(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->getReferenceJacobianOfTranslation(j).cols()-1);
      V[j][i]>>VParent(I,J);
    }
  } 

  void MechanicalLink::updatehRef(const Vec &hParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->getJacobianOfTranslation(j).cols()-1);
      h[j][i]>>hParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->getReferenceJacobianOfTranslation(j).cols()-1);
      h[j][i]>>hParent(I);
    }
  } 

  void MechanicalLink::updatedhdqRef(const fmatvec::Mat& dhdqParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void MechanicalLink::updatedhduRef(const fmatvec::SqrMat& dhduParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void MechanicalLink::updatedhdtRef(const fmatvec::Vec& dhdtParent, int j) {
    for(unsigned i=0; i<frame.size(); i++) {
      Index I = Index(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->getJacobianOfTranslation(j).cols()-1);
      dhdt[i]>>dhdtParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      Index I = Index(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->getReferenceJacobianOfTranslation(j).cols()-1);
      dhdt[i]>>dhdtParent(I);
    }
  }

  void MechanicalLink::updaterRef(const Vec &rParent, int j) {
  for(unsigned i=0; i<frame.size(); i++) {
      int hInd =  frame[i]->gethInd(j);
      Index I = Index(hInd,hInd+frame[i]->getJacobianOfTranslation(j).cols()-1);
      r[j][i]>>rParent(I);
    }
    for(unsigned i=0; i<contour.size(); i++) {
      int hInd =  contour[i]->gethInd(j);
      Index I = Index(hInd,hInd+contour[i]->getReferenceJacobianOfTranslation(j).cols()-1);
      r[j][i]>>rParent(I);
    }
  } 

  void MechanicalLink::updateSingleValuedForces(double t) { 
    F = getGlobalForceDirection(t)*getSingleValuedGeneralizedForce(t)(iF);
    M = getGlobalMomentDirection(t)*getSingleValuedGeneralizedForce(t)(iM);
    updFSV = false;
  }
  void MechanicalLink::updateSetValuedForces(double t) { 
    F = getGlobalForceDirection(t)*getSetValuedGeneralizedForce(t)(iF); 
    M = getGlobalMomentDirection(t)*getSetValuedGeneralizedForce(t)(iM);
    updFMV = false;
  }

  void MechanicalLink::updateSetValuedForceDirections(double t) { 
    RF.set(Index(0, 2), Index(iF), getGlobalForceDirection(t));
    RM.set(Index(0, 2), Index(iM), getGlobalMomentDirection(t));
    updRMV = false;
  }

  void MechanicalLink::init(InitStage stage) {
    if(stage==resize) {
      Link::init(stage);
      RF.resize(laSize);
      RM.resize(laSize);
    }
    else if(stage==unknownStage) {
      Link::init(stage);

      for(unsigned int i=0; i<frame.size(); i++) {
        W[0].push_back(Mat(0,0,NONINIT));
        V[0].push_back(Mat(0,0,NONINIT));
        h[0].push_back(Vec(0,NONINIT));
        W[1].push_back(Mat(0,0,NONINIT));
        V[1].push_back(Mat(0,0,NONINIT));
        h[1].push_back(Vec(0,NONINIT));
        r[0].push_back(Vec(0,NONINIT));
        r[1].push_back(Vec(0,NONINIT));
      }
#ifdef HAVE_OPENMBVCPPINTERFACE
      assert(openMBVArrowF.size()==0 || openMBVArrowF.size()==frame.size());
      assert(openMBVArrowM.size()==0 || openMBVArrowM.size()==frame.size());
#endif

      for(unsigned int i=0; i<contour.size(); i++) {
        W[0].push_back(Mat(0,0,NONINIT));
        V[0].push_back(Mat(0,0,NONINIT));
        h[0].push_back(Vec(0,NONINIT));
        W[1].push_back(Mat(0,0,NONINIT));
        V[1].push_back(Mat(0,0,NONINIT));
        h[1].push_back(Vec(0,NONINIT));
        r[0].push_back(Vec(0,NONINIT));
        r[1].push_back(Vec(0,NONINIT));
      }
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

  void MechanicalLink::connect(Frame *frame_) {
    frame.push_back(frame_);
  }

  void MechanicalLink::connect(Contour *contour_) {
    contour.push_back(contour_);
  }

}

