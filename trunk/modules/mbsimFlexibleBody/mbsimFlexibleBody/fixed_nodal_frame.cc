/* Copyright (C) 2004-2015 MBSim Development Team
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
#include "fixed_nodal_frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void FixedNodalFrame::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<FixedNodalFrame>(saved_frameOfReference));
      Frame::init(stage);
    }
    if(stage==resize) {
      WJD[0].resize(nq,hSize[0]);
      WJD[1].resize(nq,hSize[1]);
      if(not(Phi.cols()))
        Phi.resize(nq);
      if(not(Psi.cols()))
        Psi.resize(nq);
      q.resize(nq);
      qd.resize(nq);
      Frame::init(stage);
    }
    else
      Frame::init(stage);
  }

  void FixedNodalFrame::initializeUsingXML(DOMElement *element) {
    Frame::initializeUsingXML(element);
//    DOMElement *ec=element->getFirstElementChild();
//    ec=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
//    if(ec) setFrameOfReference(E(ec)->getAttribute("ref"));
//    ec=E(element)->getFirstElementChildNamed(MBSIM%"relativePosition");
//    if(ec) setRelativePosition(getVec3(ec));
//    ec=E(element)->getFirstElementChildNamed(MBSIM%"relativeOrientation");
//    if(ec) setRelativeOrientation(getSqrMat3(ec));
  }

  DOMElement* FixedNodalFrame::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Frame::writeXMLFile(parent);
//     if(getFrameOfReference()) {
//        DOMElement *ele1 = new DOMElement( MBSIM%"frameOfReference" );
//        string str = string("../Frame[") + getFrameOfReference()->getName() + "]";
//        ele1->SetAttribute("ref", str);
//        ele0->LinkEndChild(ele1);
//      }
//     addElementText(ele0,MBSIM%"relativePosition",getRelativePosition());
//     addElementText(ele0,MBSIM%"relativeOrientation",getRelativeOrientation());
   return ele0;
  }

}

