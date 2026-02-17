/* Copyright (C) 2004-2022 MBSim Development Team
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
#include "mbsimFlexibleBody/contours/nodes_contour.h"
#include "mbsimFlexibleBody/namespace.h"
#include "mbsimFlexibleBody/node_based_body.h"
#include <mbsim/utils/openmbv_utils.h>
#include <openmbvcppinterface/dynamicpointset.h>
#include <openmbvcppinterface/group.h>
#include "mbsim/frames/floating_contour_frame.h"

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, NodesContour)

  const Vec3& NodesContour::evalPosition(int i) {
    return static_cast<NodeBasedBody*>(parent)->evalNodalPosition(nodes(i));
  }

  void NodesContour::updatePositions(Frame *frame) {
    //frame->setVelocity(static_cast<NodeBasedBody*>(parent)->evalNodalPosition(nodes(frameMap[frame])));
    throwError("(NodesContour::updatePositions): not implemented");
  }

  void NodesContour::updateVelocities(Frame *frame) {
    auto contourFrame = static_cast<ContourFrame*>(frame);
    assert(dynamic_cast<ContourFrame*>(frame));
    contourFrame->setVelocity(static_cast<NodeBasedBody*>(parent)->evalNodalVelocity(nodes(frameMap[contourFrame])));
  }

  void NodesContour::updateAccelerations(Frame *frame) {
    //frame->setVelocity(static_cast<NodeBasedBody*>(parent)->evalNodalAcceleration(nodes(frameMap[frame])));
    throwError("(NodesContour::updateAccelerations): not implemented");
  }

  void NodesContour::updateJacobians(Frame *frame, int j) {
    auto contourFrame = static_cast<ContourFrame*>(frame);
    assert(dynamic_cast<ContourFrame*>(frame));
    contourFrame->setJacobianOfTranslation(static_cast<NodeBasedBody*>(parent)->evalNodalJacobianOfTranslation(nodes(frameMap[contourFrame])),j);
  }

  void NodesContour::updateGyroscopicAccelerations(Frame *frame) {
    auto contourFrame = static_cast<ContourFrame*>(frame);
    assert(dynamic_cast<ContourFrame*>(frame));
    contourFrame->setGyroscopicAccelerationOfTranslation(static_cast<NodeBasedBody*>(parent)->evalNodalGyroscopicAccelerationOfTranslation(nodes(frameMap[contourFrame])));
  }

  void NodesContour::init(InitStage stage, const MBSim::InitConfigSet &config) {
    if (stage == preInit) {
      for(int i=0; i<nodes.size(); i++)
	nodes(i) = static_cast<NodeBasedBody*>(parent)->getNodeIndex(nodes(i));
    }
    else if(stage==plotting) {
      if(plotFeature[MBSim::openMBV] && openMBVBody) {
	openMBVBody->setName(name);
        openMBVBody->setNumberOfVertexPositions(nodes.size());
        parent->getOpenMBVGrp()->addObject(openMBVBody);
      }
    }
    Contour::init(stage, config);
  }

  void NodesContour::plot() {
    if(plotFeature[openMBV] and openMBVBody) {
      vector<OpenMBV::Float> data;
      data.push_back(getTime());
      for(int i=0; i<openMBVBody->getNumberOfVertexPositions(); i++) {
        const Vec3 &WrOP = static_cast<NodeBasedBody*>(parent)->evalNodalPosition(nodes(i));
        for(int j=0; j<3; j++)
          data.push_back(WrOP(j));
	data.push_back(0);
     }
      openMBVBody->append(data);
    }
    Contour::plot();
  }

  void NodesContour::initializeUsingXML(DOMElement *element) {
    Contour::initializeUsingXML(element);
    DOMElement *e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"nodeNumbers");
    setNodeNumbers(E(e)->getText<VecVI>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(e) {
      OpenMBVColoredBody ombv;
      ombv.initializeUsingXML(e);
      openMBVBody=ombv.createOpenMBV<OpenMBV::DynamicPointSet>();
    }
  }

  ContourFrame* NodesContour::createContourFrame(const string &name) {
    FloatingContourFrame *frame = new FloatingContourFrame(name);
    frame->setContourOfReference(this);
    frameMap[frame] = i++;
    return frame;
  }

}
