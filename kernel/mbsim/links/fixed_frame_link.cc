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
#include "mbsim/links/fixed_frame_link.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/eps.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  FixedFrameLink::FixedFrameLink(const std::string &name) : FrameLink(name) {
  }

  void FixedFrameLink::updateGeneralizedForces() {
    lambda = evallaF();
    updla = false;
  }

  void FixedFrameLink::updateForceDirections() {
    if(evalGeneralizedRelativePosition()(0)>epsroot())
      DF=evalGlobalRelativePosition()/rrel(0);
    else
      DF.init(0);
    updFD = false;
  }

  void FixedFrameLink::updateh(int j) {
    h[j][0]-=frame[0]->evalJacobianOfTranslation(j).T()*evalForce();
    h[j][1]+=frame[1]->evalJacobianOfTranslation(j).T()*evalForce();
  }
  
  void FixedFrameLink::updatePositions() {
    WrP0P1=frame[1]->evalPosition() - frame[0]->evalPosition();
    updPos = false;
  }

  void FixedFrameLink::updateVelocities() {
    WvP0P1=frame[1]->evalVelocity() - frame[0]->evalVelocity();
    updVel = false;
  }

  void FixedFrameLink::updateGeneralizedPositions() {
    rrel(0)=nrm2(evalGlobalRelativePosition());
    updrrel = false;
  }

  void FixedFrameLink::updateGeneralizedVelocities() {
    vrel=evalGlobalForceDirection().T()*evalGlobalRelativeVelocity();
    updvrel = false;
  }

  void FixedFrameLink::updateR() {
    RF.set(RangeV(0,2), RangeV(iF), evalGlobalForceDirection());
    updRMV = false;
  }

  void FixedFrameLink::init(InitStage stage) {
    if(stage==resize) {
      FrameLink::init(stage);
      iF = RangeV(0, 0);
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
    else
      FrameLink::init(stage);
  }

}
