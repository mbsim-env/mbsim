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
#include "mbsim/contour_to_contour_link.h"
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

  ContourToContourLink::ContourToContourLink(const std::string &name) : Link(name), updPos(true), updVel(true), updFD(true), updFSV(true), updFMV(true), updRMV(true) {
  }

  ContourToContourLink::~ContourToContourLink() {}

  void ContourToContourLink::updatedhdz(double t) {
    THROW_MBSIMERROR("Internal error");
  }

  void ContourToContourLink::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(openMBVArrowF) {
        vector<double> data;
        data.push_back(t); 
        Vec3 toPoint=contour[1]->getFrame()->getPosition(t);
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
        Vec3 toPoint=contour[1]->getFrame()->getPosition(t);
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

  void ContourToContourLink::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Link::closePlot();
    }
  }

  void ContourToContourLink::updateWRef(const Mat& WParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      Index I = Index(contour[i]->gethInd(j), contour[i]->gethInd(j) + contour[i]->gethSize(j) - 1);
      Index J = Index(laInd, laInd + laSize - 1);
      W[j][i] >> WParent(I, J);
    }
  } 

  void ContourToContourLink::updateVRef(const Mat& VParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      Index I = Index(contour[i]->gethInd(j), contour[i]->gethInd(j) + contour[i]->gethSize(j) - 1);
      Index J = Index(laInd, laInd + laSize - 1);
      V[j][i] >> VParent(I, J);
    }
  } 

  void ContourToContourLink::updatehRef(const Vec &hParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      Index I = Index(contour[i]->gethInd(j), contour[i]->gethInd(j) + contour[i]->gethSize(j) - 1);
      h[j][i] >> hParent(I);
    }
  } 

  void ContourToContourLink::updatedhdqRef(const fmatvec::Mat& dhdqParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void ContourToContourLink::updatedhduRef(const fmatvec::SqrMat& dhduParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void ContourToContourLink::updatedhdtRef(const fmatvec::Vec& dhdtParent, int j) {
    for(unsigned i=0; i<2; i++) {
      Index I = Index(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->getFrame()->gethSize(j)-1);
      dhdt[i]>>dhdtParent(I);
    }
  }

  void ContourToContourLink::updaterRef(const Vec &rParent, int j) {
    for(unsigned i=0; i<2; i++) {
      int hInd =  contour[i]->gethInd(j);
      Index I = Index(hInd,hInd+contour[i]->getFrame()->gethSize(j)-1);
      r[j][i]>>rParent(I);
    }
  } 

  void ContourToContourLink::updateh(double t, int j) {
    h[j][0] -= cpData[0].getFrameOfReference().getJacobianOfTranslation(t,j).T() * getSingleValuedForce(t);
    h[j][1] += cpData[1].getFrameOfReference().getJacobianOfTranslation(t,j).T() * getSingleValuedForce(t);
  }

  void ContourToContourLink::updateW(double t, int j) {
    W[j][0] -= cpData[0].getFrameOfReference().getJacobianOfTranslation(t,j).T() * getSetValuedForceDirection(t)(Index(0,2),Index(0,laSize-1));
    W[j][1] += cpData[1].getFrameOfReference().getJacobianOfTranslation(t,j).T() * getSetValuedForceDirection(t)(Index(0,2),Index(0,laSize-1));
  }

  void ContourToContourLink::updateForceDirections(double t) {
    DF.set(0,cpData[0].getFrameOfReference().getOrientation(t).col(0));
    if (DF.cols()>1) {
      DF.set(1, cpData[0].getFrameOfReference().getOrientation().col(1));
      if (DF.cols()>2)
        DF.set(2, cpData[0].getFrameOfReference().getOrientation().col(2));
    }
    updFD = false;
  }

  void ContourToContourLink::updateSingleValuedForces(double t) {
    F = getGlobalForceDirection(t)*getGeneralizedSingleValuedForce(t)(iF);
    M = getGlobalMomentDirection(t)*getGeneralizedSingleValuedForce(t)(iM);
    updFSV = false;
  }

  void ContourToContourLink::updateSetValuedForces(double t) {
    F = getGlobalForceDirection(t)*getGeneralizedSetValuedForce(t)(iF);
    M = getGlobalMomentDirection(t)*getGeneralizedSetValuedForce(t)(iM);
    updFMV = false;
  }

  void ContourToContourLink::updateSetValuedForceDirections(double t) {
    RF.set(Index(0,2), Index(iF), getGlobalForceDirection(t));
    RM.set(Index(0,2), Index(iM), getGlobalMomentDirection(t));
    updRMV = false;
  }

  void ContourToContourLink::init(InitStage stage) {
    if(stage==resize) {
      Link::init(stage);
      RF.resize(laSize);
      RM.resize(laSize);
      cpData[0].getFrameOfReference().setParent(this);
      cpData[1].getFrameOfReference().setParent(this);
      cpData[0].getFrameOfReference().setFrameOfReference(contour[0]->getFrameOfReference());
      cpData[1].getFrameOfReference().setFrameOfReference(contour[1]->getFrameOfReference());

      cpData[0].getFrameOfReference().setName("0");
      cpData[1].getFrameOfReference().setName("1");

      cpData[0].getFrameOfReference().sethSize(contour[0]->gethSize(0), 0);
      cpData[0].getFrameOfReference().sethSize(contour[0]->gethSize(1), 1);
      cpData[1].getFrameOfReference().sethSize(contour[1]->gethSize(0), 0);
      cpData[1].getFrameOfReference().sethSize(contour[1]->gethSize(1), 1);

      cpData[0].getFrameOfReference().init(stage);
      cpData[1].getFrameOfReference().init(stage);
    }
    else if(stage==unknownStage) {
      Link::init(stage);

      for(unsigned int i=0; i<2; i++) {
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

}

