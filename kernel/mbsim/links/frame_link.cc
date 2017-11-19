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
#include "mbsim/links/frame_link.h"
#include "mbsim/frames/frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  FrameLink::FrameLink(const std::string &name) : MechanicalLink(name), frame(2), updPos(true), updVel(true) {
    P.resize(2);
    F.resize(2);
    M.resize(2);
    RF.resize(2);
    RM.resize(2);
  }

  void FrameLink::resetUpToDate() {
    MechanicalLink::resetUpToDate();
    updPos = true;
    updVel = true;
  }

  void FrameLink::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(saved_ref1!="" && saved_ref2!="")
        connect(getByPath<Frame>(saved_ref1), getByPath<Frame>(saved_ref2));
      if(frame[0]==nullptr or frame[1]==nullptr)
        THROW_MBSIMERROR("Not all connections are given!");
    }
    MechanicalLink::init(stage, config);
  }

  void FrameLink::updateWRef(const Mat& WParent, int j) {
    for(unsigned i=0; i<2; i++) {
      RangeV J = RangeV(laInd,laInd+laSize-1);
      RangeV I = RangeV(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1); // TODO Prüfen ob hSize
      W[j][i]>>WParent(I,J);
    }
  }

  void FrameLink::updateVRef(const Mat& VParent, int j) {
    for(unsigned i=0; i<2; i++) {
      RangeV J = RangeV(laInd,laInd+laSize-1);
      RangeV I = RangeV(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1);
      V[j][i]>>VParent(I,J);
    }
  }

  void FrameLink::updatehRef(const Vec &hParent, int j) {
    for(unsigned i=0; i<2; i++) {
      RangeV I = RangeV(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1);
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
      RangeV I = RangeV(frame[i]->gethInd(j),frame[i]->gethInd(j)+frame[i]->gethSize(j)-1);
      dhdt[i]>>dhdtParent(I);
    }
  }

  void FrameLink::updaterRef(const Vec &rParent, int j) {
    for(unsigned i=0; i<2; i++) {
      int hInd =  frame[i]->gethInd(j);
      RangeV I = RangeV(hInd,hInd+frame[i]->gethSize(j)-1);
      r[j][i]>>rParent(I);
    }
  }

  void FrameLink::initializeUsingXML(DOMElement *element) {
    MechanicalLink::initializeUsingXML(element);
    DOMElement *e = E(element)->getFirstElementChildNamed(MBSIM%"connect");
    saved_ref1 = E(e)->getAttribute("ref1");
    saved_ref2 = E(e)->getAttribute("ref2");
  }

  void FrameLink::connect(Frame *frame0, Frame* frame1) {
    frame[0] = frame0;
    frame[1] = frame1;
  }

}
