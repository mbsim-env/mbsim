/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "mbsim/contours/rigid_contour.h"
#include "mbsim/frames/floating_relative_frame.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/utils/contact_utils.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/rigidbody.h>
#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  void RigidContour::init(InitStage stage) {
    if(stage==resolveXMLPath) {
      if(saved_frameOfReference!="")
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
      Contour::init(stage);
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
          openMBVRigidBody->setName(name);
          parent->getOpenMBVGrp()->addObject(openMBVRigidBody);
        }
#endif
        Contour::init(stage);
      }
    }
    else
      Contour::init(stage);
  }
  
  Frame* RigidContour::createContourFrame(const string &name) {
    FloatingRelativeFrame *frame = new FloatingRelativeFrame(name);      
    frame->setFrameOfReference(R);
    return frame;
  }

  Vec3 RigidContour::getPosition(double t, const Vec2 &zeta) {
    return R->getPosition(t) + getWrPS(t,zeta);
  }

  Vec3 RigidContour::getParDer1Wn(double t, const Vec2 &zeta) {
    return R->getOrientation(t)*getParDer1Kn(zeta);
  }

  Vec3 RigidContour::getParDer2Wn(double t, const Vec2 &zeta) {
    return R->getOrientation(t)*getParDer2Kn(zeta);
  }

  Vec3 RigidContour::getParDer1Wu(double t, const Vec2 &zeta) {
    return R->getOrientation(t)*getParDer1Ku(zeta);
  }

  Vec3 RigidContour::getParDer2Wu(double t, const Vec2 &zeta) {
    return R->getOrientation(t)*getParDer2Ku(zeta);
  }

  Vec3 RigidContour::getParDer1Wv(double t, const Vec2 &zeta) {
    return R->getOrientation(t)*getParDer1Kv(zeta);
  }

  Vec3 RigidContour::getParDer2Wv(double t, const Vec2 &zeta) {
    return R->getOrientation(t)*getParDer2Kv(zeta);
  }

  Vec3 RigidContour::getWrPS(double t, const Vec2 &zeta) {
    return R->getOrientation(t)*getKrPS(zeta);
  }

  Vec3 RigidContour::getWs(double t, const Vec2 &zeta) {
    return R->getOrientation(t)*getKs(zeta);
  }

  Vec3 RigidContour::getWt(double t, const Vec2 &zeta) {
    return R->getOrientation(t)*getKt(zeta);
  }

  void RigidContour::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
      if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
        vector<double> data;
        data.push_back(t);
        data.push_back(R->getPosition(t)(0));
        data.push_back(R->getPosition()(1));
        data.push_back(R->getPosition()(2));
        Vec3 cardan=AIK2Cardan(R->getOrientation(t));
        data.push_back(cardan(0));
        data.push_back(cardan(1));
        data.push_back(cardan(2));
        data.push_back(0);
        openMBVRigidBody->append(data);
      }
#endif
      Contour::plot(t,dt);
    }
  }

  ContactKinematics * RigidContour::findContactPairingWith(std::string type0, std::string type1) {
    return findContactPairingRigidRigid(type0.c_str(), type1.c_str());
  }

  void RigidContour::initializeUsingXML(DOMElement *element) {
    Contour::initializeUsingXML(element);
    DOMElement *ec=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(ec) setFrameOfReference(E(ec)->getAttribute("ref"));
  }

  DOMElement* RigidContour::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Contour::writeXMLFile(parent);
    return ele0;
  }

}
