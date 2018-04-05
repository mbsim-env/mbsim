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
#include "mbsim/frames/floating_relative_contour_frame.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/utils/contact_utils.h"

#include <openmbvcppinterface/group.h>
#include <openmbvcppinterface/rigidbody.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  void RigidContour::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      if(!saved_frameOfReference.empty())
        setFrameOfReference(getByPath<Frame>(saved_frameOfReference));
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        openMBVRigidBody->setName(name);
        parent->getOpenMBVGrp()->addObject(openMBVRigidBody);
      }
    }
    Contour::init(stage, config);
  }
  
  ContourFrame* RigidContour::createContourFrame(const string &name) {
    FloatingRelativeContourFrame *frame = new FloatingRelativeContourFrame(name);
    frame->setFrameOfReference(R);
    return frame;
  }

  Vec3 RigidContour::evalKrPS(const Vec2 &zeta) {
    throwError("(RigidContour::evalKrPS): Not implemented.");
    return 0;
  }

  Vec3 RigidContour::evalKs(const Vec2 &zeta) {
    throwError("(RigidContour::evalKs): Not implemented.");
    return 0;
  }

  Vec3 RigidContour::evalKt(const Vec2 &zeta) {
    throwError("(RigidContour::evalKt): Not implemented.");
    return 0;
  }
  Vec3 RigidContour::evalParDer1Ks(const Vec2 &zeta) {
    throwError("(RigidContour::evalParDer1Ks): Not implemented.");
    return 0;
  }

  Vec3 RigidContour::evalParDer2Ks(const Vec2 &zeta) {
    throwError("(RigidContour::evalParDer2Ks): Not implemented.");
    return 0;
  }

  Vec3 RigidContour::evalParDer1Kt(const Vec2 &zeta) {
    throwError("(RigidContour::evalParDer1Kt): Not implemented.");
    return 0;
  }

  Vec3 RigidContour::evalParDer2Kt(const Vec2 &zeta) {
    throwError("(RigidContour::evalParDer2Kt): Not implemented.");
    return 0;
  }

  Vec3 RigidContour::evalKu(const Vec2 &zeta) {
    Vec3 Ks=evalKs(zeta);
    return Ks/nrm2(Ks);
  }

  Vec3 RigidContour::evalKv(const Vec2 &zeta) {
    return crossProduct(evalKn(zeta),evalKu(zeta));;
  }

  Vec3 RigidContour::evalKn(const Vec2 &zeta) {
    Vec3 Kn=crossProduct(evalKs(zeta),evalKt(zeta));
    return Kn/nrm2(Kn);
  }

  Vec3 RigidContour::evalParDer1Ku(const Vec2 &zeta) {
    Vec3 Ks = evalKs(zeta);
    Vec3 parDer1Ks = evalParDer1Ks(zeta);
    return parDer1Ks/nrm2(Ks) - Ks*((Ks.T()*parDer1Ks)/pow(nrm2(Ks),3));
  }

  Vec3 RigidContour::evalParDer2Ku(const Vec2 &zeta) {
    Vec3 Ks = evalKs(zeta);
    Vec3 parDer2Ks = evalParDer2Ks(zeta);
    return parDer2Ks/nrm2(Ks) - Ks*((Ks.T()*parDer2Ks)/pow(nrm2(Ks),3));
  }

  Vec3 RigidContour::evalParDer1Kv(const Vec2 &zeta) {
    return crossProduct(evalParDer1Kn(zeta),evalKu(zeta)) + crossProduct(evalKn(zeta),evalParDer1Ku(zeta));
  }

  Vec3 RigidContour::evalParDer2Kv(const Vec2 &zeta) {
    return crossProduct(evalParDer2Kn(zeta),evalKu(zeta)) + crossProduct(evalKn(zeta),evalParDer2Ku(zeta));
  }

  Vec3 RigidContour::evalParDer1Kn(const Vec2 &zeta) {
    Vec3 Ksxt = crossProduct(evalKs(zeta),evalKt(zeta));
    Vec3 Ksxtd = crossProduct(evalParDer1Ks(zeta),evalKt(zeta)) + crossProduct(evalKs(zeta),evalParDer1Kt(zeta));
    return Ksxtd/nrm2(Ksxt) - Ksxt*((Ksxt.T()*Ksxtd)/pow(nrm2(Ksxt),3));
  }

  Vec3 RigidContour::evalParDer2Kn(const Vec2 &zeta) {
    Vec3 Ksxt = crossProduct(evalKs(zeta),evalKt(zeta));
    Vec3 Ksxtd = crossProduct(evalParDer2Ks(zeta),evalKt(zeta)) + crossProduct(evalKs(zeta),evalParDer2Kt(zeta));
    return Ksxtd/nrm2(Ksxt) - Ksxt*((Ksxt.T()*Ksxtd)/pow(nrm2(Ksxt),3));
  }

  Vec3 RigidContour::evalPosition(const Vec2 &zeta) {
    return R->evalPosition() + evalWrPS(zeta);
  }

  Vec3 RigidContour::evalParDer1Wn(const Vec2 &zeta) {
    return R->evalOrientation()*evalParDer1Kn(zeta);
  }

  Vec3 RigidContour::evalParDer2Wn(const Vec2 &zeta) {
    return R->evalOrientation()*evalParDer2Kn(zeta);
  }

  Vec3 RigidContour::evalParDer1Wu(const Vec2 &zeta) {
    return R->evalOrientation()*evalParDer1Ku(zeta);
  }

  Vec3 RigidContour::evalParDer2Wu(const Vec2 &zeta) {
    return R->evalOrientation()*evalParDer2Ku(zeta);
  }

  Vec3 RigidContour::evalParDer1Wv(const Vec2 &zeta) {
    return R->evalOrientation()*evalParDer1Kv(zeta);
  }

  Vec3 RigidContour::evalParDer2Wv(const Vec2 &zeta) {
    return R->evalOrientation()*evalParDer2Kv(zeta);
  }

  Vec3 RigidContour::evalWrPS(const Vec2 &zeta) {
    return R->evalOrientation()*evalKrPS(zeta);
  }

  Vec3 RigidContour::evalWs(const Vec2 &zeta) {
    return R->evalOrientation()*evalKs(zeta);
  }

  Vec3 RigidContour::evalWt(const Vec2 &zeta) {
    return R->evalOrientation()*evalKt(zeta);
  }

  void RigidContour::plot() {
    if(plotFeature[openMBV] && openMBVRigidBody) {
      vector<double> data;
      data.push_back(getTime());
      data.push_back(R->evalPosition()(0));
      data.push_back(R->getPosition()(1));
      data.push_back(R->getPosition()(2));
      Vec3 cardan=AIK2Cardan(R->evalOrientation());
      data.push_back(cardan(0));
      data.push_back(cardan(1));
      data.push_back(cardan(2));
      data.push_back(0);
      openMBVRigidBody->append(data);
    }
    Contour::plot();
  }

  ContactKinematics * RigidContour::findContactPairingWith(const std::type_info &type0, const std::type_info &type1) {
    return findContactPairingRigidRigid(type0, type1);
  }

  void RigidContour::initializeUsingXML(DOMElement *element) {
    Contour::initializeUsingXML(element);
    DOMElement *ec=E(element)->getFirstElementChildNamed(MBSIM%"frameOfReference");
    if(ec) setFrameOfReference(E(ec)->getAttribute("ref"));
  }

}
