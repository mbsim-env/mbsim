/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "mbsim/utils/utils.h"
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/arrow.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  ContourLink::ContourLink(const std::string &name) : MechanicalLink(name), contour(2), cFrame(2), updPos(true), updVel(true), updDF(true) {
  }

  ContourLink::~ContourLink() {
    delete cFrame[0];
    delete cFrame[1];
  }

  void ContourLink::updateWRef(const Mat& WParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      RangeV I = RangeV(contour[i]->gethInd(j), contour[i]->gethInd(j) + contour[i]->gethSize(j) - 1);
      RangeV J = RangeV(laInd, laInd + laSize - 1);
      W[j][i] >> WParent(I, J);
    }
  } 

  void ContourLink::updateVRef(const Mat& VParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      RangeV I = RangeV(contour[i]->gethInd(j), contour[i]->gethInd(j) + contour[i]->gethSize(j) - 1);
      RangeV J = RangeV(laInd, laInd + laSize - 1);
      V[j][i] >> VParent(I, J);
    }
  } 

  void ContourLink::updatehRef(const Vec &hParent, int j) {
    for (unsigned i = 0; i < 2; i++) { //only two contours for one contactKinematic
      RangeV I = RangeV(contour[i]->gethInd(j), contour[i]->gethInd(j) + contour[i]->gethSize(j) - 1);
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
      RangeV I = RangeV(contour[i]->gethInd(j),contour[i]->gethInd(j)+contour[i]->gethSize(j)-1);
      dhdt[i]>>dhdtParent(I);
    }
  }

  void ContourLink::updaterRef(const Vec &rParent, int j) {
    for(unsigned i=0; i<2; i++) {
      int hInd =  contour[i]->gethInd(j);
      RangeV I = RangeV(hInd,hInd+contour[i]->gethSize(j)-1);
      r[j][i]>>rParent(I);
    }
  } 

  void ContourLink::updateForceDirections() {
    DF.set(0,cFrame[0]->evalOrientation().col(0));
    if (DF.cols()>1) {
      DF.set(1, cFrame[0]->getOrientation().col(1));
      if (DF.cols()>2)
        DF.set(2, cFrame[0]->getOrientation().col(2));
    }
    updDF = false;
  }

  void ContourLink::updateForce() {
    F[1] = evalGlobalForceDirection()*evalGeneralizedForce()(iF);
    F[0] = -F[1];
    updF = false;
  }

  void ContourLink::updateMoment() {
    M[1] = evalGlobalMomentDirection()*evalGeneralizedForce()(iM);
    M[0] = -M[1];
    updM = false;
  }

  void ContourLink::init(InitStage stage) {
    if(stage == resolveXMLPath) {
      if(saved_ref1 != "" && saved_ref2 != "")
        connect(getByPath<Contour>(saved_ref1), getByPath<Contour>(saved_ref2));
      MechanicalLink::init(stage);
    }
    else if(stage==preInit) {
      MechanicalLink::init(stage);

      cFrame[0] = contour[0]->createContourFrame("P0");
      cFrame[1] = contour[1]->createContourFrame("P1");
      cFrame[0]->setParent(this);
      cFrame[1]->setParent(this);
    }
    else if(stage==resize) {
      MechanicalLink::init(stage);

      cFrame[0]->sethSize(contour[0]->gethSize(0), 0);
      cFrame[0]->sethSize(contour[0]->gethSize(1), 1);
      cFrame[1]->sethSize(contour[1]->gethSize(0), 0);
      cFrame[1]->sethSize(contour[1]->gethSize(1), 1);
     }
    else if(stage==unknownStage) {
      MechanicalLink::init(stage);

      K = cFrame[1];

      if(contour[0]==NULL or contour[1]==NULL)
        THROW_MBSIMERROR("Not all connections are given!");

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
    else
      MechanicalLink::init(stage);
  }

  void ContourLink::resetUpToDate() {
    MechanicalLink::resetUpToDate();
    updPos = true;
    updVel = true;
    updDF = true;
    cFrame[0]->resetUpToDate();
    cFrame[1]->resetUpToDate();
  }

  void ContourLink::initializeUsingXML(DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e;
    //Save contour names for initialization
    e = E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1 = E(e)->getAttribute("ref1");
    saved_ref2 = E(e)->getAttribute("ref2");
  }

  void ContourLink::connect(Contour *contour0, Contour* contour1) {
    contour[0] = contour0;
    contour[1] = contour1;
  }

}
