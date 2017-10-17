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

  FixedFrameLink::FixedFrameLink(const std::string &name) : FrameLink(name), DF(1), updDF(true) {
  }

  void FixedFrameLink::calcSize() {
    ng = 1;
    ngd = 1;
    nla = 1;
    updSize = false;
  }

  void FixedFrameLink::resetUpToDate() {
    FrameLink::resetUpToDate();
    updDF = true;
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

  void FixedFrameLink::updateForce() {
    F[1] = evalGlobalForceDirection(0)*evalGeneralizedForce()(iF);
    F[0] = -F[1];
    updF = false;
  }

  void FixedFrameLink::updateMoment() {
    M[1] = evalGlobalMomentDirection()*evalGeneralizedForce()(iM);
    M[0] = -M[1];
    updM = false;
  }

  void FixedFrameLink::updateR() {
    RF[1].set(RangeV(0,2), RangeV(iF), evalGlobalForceDirection());
    RM[1].set(RangeV(0,2), RangeV(iM), evalGlobalMomentDirection());
    RF[0] = -RF[1];
    RM[0] = -RM[1];
    updRMV = false;
  }

  void FixedFrameLink::updateForceDirections() {
    if(evalGeneralizedRelativePosition()(0)>epsroot())
      DF=evalGlobalRelativePosition()/rrel(0);
    else
      DF.init(0);
    updDF = false;
  }

  void FixedFrameLink::updateh(int j) {
    for(unsigned int i=0; i<h[j].size(); i++)
      h[j][i]+=frame[i]->evalJacobianOfTranslation(j).T()*evalForce(i);
  }

  void FixedFrameLink::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      iF = RangeV(0, 0);
      iM = RangeV(0, -1);
      if(isSetValued()) {
        g.resize(1);
        gd.resize(1);
        RF[0].resize(1);
        RF[1].resize(1);
        la.resize(1);
      }
      lambdaF.resize(1);
      for(unsigned int i=0; i<2; i++) {
        W[i].resize(2);
        V[i].resize(2);
        h[i].resize(2);
        r[i].resize(2);
      }
    }
    else if(stage==unknownStage) {
      P[0] = frame[0];
      P[1] = frame[1];
    }
    FrameLink::init(stage, config);
  }

}
