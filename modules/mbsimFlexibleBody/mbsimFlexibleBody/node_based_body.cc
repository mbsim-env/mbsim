/* Copyright (C) 2004-2016 MBSim Development Team
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
#include "node_based_body.h"
#include "mbsimFlexibleBody/frames/node_based_frame.h"
#include <mbsim/mbsim_event.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  void NodeBasedBody::resetUpToDate() {
    Body::resetUpToDate();
    for(unsigned int i=0; i<updNodalPos.size(); i++) {
      updNodalPos[i] = true;
      updNodalVel[i] = true;
      updNodalAcc[i] = true;
      updNodalJac[0][i] = true;
      updNodalJac[1][i] = true;
      updNodalGA[i] = true;
      updNodalStress[i] = true;
    }
  }

  void NodeBasedBody::updatePositions(int i) {
    throwError("(NodeBasedBody::updatePositions): Not implemented.");
  }

  void NodeBasedBody::updateVelocities(int i) {
    throwError("(NodeBasedBody::updateVelocities): Not implemented.");
  }

  void NodeBasedBody::updateAccelerations(int i) {
    throwError("(NodeBasedBody::updateAccelerations): Not implemented.");
  }

  void NodeBasedBody::updateJacobians(int i, int j) {
    throwError("(NodeBasedBody::updateJacobians): Not implemented.");
  }

  void NodeBasedBody::updateGyroscopicAccelerations(int i) {
    throwError("(NodeBasedBody::updateGyroscopicAccelerations): Not implemented.");
  }

  void NodeBasedBody::updateStresses(int j) {
    throwError("(NodeBasedBody::updateStresses): Not implemented.");
  }

  void NodeBasedBody::addFrame(NodeBasedFrame *frame) {
    Body::addFrame(frame);
  }

  void NodeBasedBody::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(nodeNumbers.empty()) {
	nodeNumbers.resize(getNumberOfNodes());
	nodeMap.resize(getNumberOfNodes()+1);
        for(int i=0; i<getNumberOfNodes(); i++) {
	  nodeNumbers[i] = i+1;
          nodeMap[i+1] = i;
	}
      }
      else {
	int nmax = 0;
	for(size_t i=0; i<nodeNumbers.size(); i++) {
	  if(nodeNumbers[i]>nmax)
	    nmax = nodeNumbers[i];
	}
	nodeMap.resize(nmax+1);
	for(size_t i=0; i<nodeNumbers.size(); i++)
	  nodeMap[nodeNumbers[i]] = i;
      }

      updNodalPos.resize(nn,true);
      updNodalVel.resize(nn,true);
      updNodalAcc.resize(nn,true);
      updNodalJac[0].resize(nn,true);
      updNodalJac[1].resize(nn,true);
      updNodalGA.resize(nn,true);
      updNodalStress.resize(nn,true);
      WrOP.resize(nn);
      disp.resize(nn);
      Wom.resize(nn);
      WvP.resize(nn);
      WaP.resize(nn);
      Wpsi.resize(nn);
      WjP.resize(nn);
      WjR.resize(nn);
      AWK.resize(nn);
      sigma.resize(nn);
    }
    else if(stage==unknownStage) {
      WJP[0].resize(nn,Mat3xV(gethSize(0),NONINIT));
      WJR[0].resize(nn,Mat3xV(gethSize(0),NONINIT));
      WJP[1].resize(nn,Mat3xV(gethSize(1),NONINIT));
      WJR[1].resize(nn,Mat3xV(gethSize(1),NONINIT));
    }
    Body::init(stage,config);
  }

}
