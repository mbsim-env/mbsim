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
#include "mbsim/links/contour_link.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/contour.h"
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

  ContourLink::ContourLink(const std::string &name) : Link(name), contour(2), cFrame(2), updPos(true), updVel(true), updFD(true), updF(true), updM(true), updR(true) {
  }

  void ContourLink::updatedhdz(double t) {
    THROW_MBSIMERROR("Internal error");
  }

  void ContourLink::plot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Link::plot();
    }
  }

  void ContourLink::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Link::closePlot();
    }
  }

  void ContourLink::updateWRef(const Mat& WParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      Index I = Index(contour[i]->gethInd(j), contour[i]->gethInd(j) + contour[i]->gethSize(j) - 1);
      Index J = Index(laInd, laInd + laSize - 1);
      W[j][i] >> WParent(I, J);
    }
  } 

  void ContourLink::updateVRef(const Mat& VParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      Index I = Index(contour[i]->gethInd(j), contour[i]->gethInd(j) + contour[i]->gethSize(j) - 1);
      Index J = Index(laInd, laInd + laSize - 1);
      V[j][i] >> VParent(I, J);
    }
  } 

  void ContourLink::updatehRef(const Vec &hParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      Index I = Index(contour[i]->gethInd(j), contour[i]->gethInd(j) + contour[i]->gethSize(j) - 1);
      h[j][i] >> hParent(I);
    }
  } 

  void ContourLink::updatedhdqRef(const fmatvec::Mat& dhdqParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void ContourLink::updatedhduRef(const fmatvec::SqrMat& dhduParent, int k) {
    THROW_MBSIMERROR("Internal error");
  }

  void ContourLink::updatedhdtRef(const fmatvec::Vec& dhdtParent, int j) {
    for(unsigned i=0; i<2; i++) {
      Index I = Index(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->gethSize(j)-1);
      dhdt[i]>>dhdtParent(I);
    }
  }

  void ContourLink::updaterRef(const Vec &rParent, int j) {
    for(unsigned i=0; i<2; i++) {
      int hInd =  contour[i]->gethInd(j);
      Index I = Index(hInd,hInd+contour[i]->gethSize(j)-1);
      r[j][i]>>rParent(I);
    }
  } 

  void ContourLink::updateh(double t, int j) {
//    h[j][0] -= cpData[0].getFrameOfReference().evalJacobianOfTranslation(j).T() * getSingleValuedForce(t);
//    h[j][1] += cpData[1].getFrameOfReference().evalJacobianOfTranslation(j).T() * getSingleValuedForce(t);
  }

  void ContourLink::updateW(double t, int j) {
//    W[j][0] -= cpData[0].getFrameOfReference().evalJacobianOfTranslation(j).T() * evalRF()(Index(0,2),Index(0,laSize-1));
//    W[j][1] += cpData[1].getFrameOfReference().evalJacobianOfTranslation(j).T() * evalRF()(Index(0,2),Index(0,laSize-1));
  }

  void ContourLink::updateForceDirections() {
    DF.set(0,cFrame[0]->evalOrientation().col(0));
    if (DF.cols()>1) {
      DF.set(1, cFrame[0]->getOrientation().col(1));
      if (DF.cols()>2)
        DF.set(2, cFrame[0]->getOrientation().col(2));
    }
    updFD = false;
  }

  void ContourLink::updateForce() {
    F = evalGlobalForceDirection()*evalGeneralizedForce()(iF);
    updF = false;
  }

  void ContourLink::updateMoment() {
    M = evalGlobalMomentDirection()*evalGeneralizedForce()(iM);
    updM = false;
  }

//  void ContourLink::updateR() {
//    RF.set(Index(0,2), Index(iF), evalGlobalForceDirection());
//    RM.set(Index(0,2), Index(iM), evalGlobalMomentDirection());
//    updR = false;
//  }

  void ContourLink::init(InitStage stage) {
    if(stage==preInit) {
      Link::init(stage);

      cFrame[0] = contour[0]->createContourFrame("P0");
      cFrame[1] = contour[1]->createContourFrame("P1");
      cFrame[0]->setParent(this);
      cFrame[1]->setParent(this);
    }
    else if(stage==resize) {
      Link::init(stage);

      cFrame[0]->sethSize(contour[0]->gethSize(0), 0);
      cFrame[0]->sethSize(contour[0]->gethSize(1), 1);
      cFrame[1]->sethSize(contour[1]->gethSize(0), 0);
      cFrame[1]->sethSize(contour[1]->gethSize(1), 1);
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

  void ContourLink::resetUpToDate() {
    Link::resetUpToDate();
    updPos = true;
    updVel = true;
    updFD = true;
    updF = true;
    updM = true;
    updR = true;
    cFrame[0]->resetUpToDate();
    cFrame[1]->resetUpToDate();
  }

}
