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
#include "mbsim/contours/spatial_contour.h"
#include "mbsim/functions/function.h"
#include "mbsim/utils/contact_utils.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(SpatialContour, MBSIM%"SpatialContour")

  SpatialContour::~SpatialContour() {
     if (funcCrPC) 
       delete funcCrPC;
     funcCrPC=NULL;
  }

  Vec3 SpatialContour::evalKrPS(const Vec2 &zeta) {
    return (*funcCrPC)(zeta);
  }

  Vec3 SpatialContour::evalKs(const Vec2 &zeta) {
    return funcCrPC->parDer(zeta).col(0);
  }

  Vec3 SpatialContour::evalKt(const Vec2 &zeta) {
    return funcCrPC->parDer(zeta).col(1);
  }

  Vec3 SpatialContour::evalParDer1Ks(const Vec2 &zeta) {
    static Vec2 dir("[1;0]");
    return funcCrPC->parDerDirDer(dir,zeta).col(0);
  }

  void SpatialContour::init(InitStage stage) {
    if (stage == preInit) {
      RigidContour::init(stage);
//      if (etaNodes.size() < 2)
//        THROW_MBSIMERROR("(SpatialContour::init): Size of etaNodes must be greater than 1.");
    }
    else if(stage==plotting) {
      updatePlotFeatures();
  
      if(getPlotFeature(plotRecursive)==enabled) {
  #ifdef HAVE_OPENMBVCPPINTERFACE
//        if(getPlotFeature(openMBV)==enabled && openMBVRigidBody) {
//          shared_ptr<vector<shared_ptr<OpenMBV::PolygonPoint> > > vpp = make_shared<vector<shared_ptr<OpenMBV::PolygonPoint> > >();
//          if(not(ombvNodes.size())) {
//            ombvNodes.resize(101);
//            for(int i=0; i<101; i++)
//              ombvNodes[i] = etaNodes[0] + (etaNodes[etaNodes.size()-1]-etaNodes[0])*i/100.;
//          }
//          for (unsigned int i=0; i<ombvNodes.size(); i++) {
//            const Vec3 CrPC=(*funcCrPC)(ombvNodes[i]);
//            vpp->push_back(OpenMBV::PolygonPoint::create(CrPC(0), CrPC(1), 0));
//          }
//          static_pointer_cast<OpenMBV::Extrusion>(openMBVRigidBody)->setHeight(0);
//          static_pointer_cast<OpenMBV::Extrusion>(openMBVRigidBody)->addContour(vpp);
//        }
  #endif
        RigidContour::init(stage);
      }
    }
    else
      RigidContour::init(stage);

    funcCrPC->init(stage);
  }

  void SpatialContour::setContourFunction(Function<Vec3(Vec2)> *func) {
    funcCrPC=func;
    funcCrPC->setParent(this);
    funcCrPC->setName("Contour");
  }

  double SpatialContour::getCurvature(const Vec2 &zeta) {
    throw;
//    const Vec3 rs = funcCrPC->parDer(zeta(0));
//    return nrm2(crossProduct(rs,funcCrPC->parDerParDer(zeta(0))))/pow(nrm2(rs),3);
  }

  void SpatialContour::initializeUsingXML(DOMElement * element) {
    RigidContour::initializeUsingXML(element);
    DOMElement * e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"nodes");
    etaNodes=getVec(e);
    e=E(element)->getFirstElementChildNamed(MBSIM%"contourFunction");
    setContourFunction(ObjectFactory::createAndInit<Function<Vec3(Vec2)> >(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"open");
    if(e) setOpen(Element::getBool(e));
#ifdef HAVE_OPENMBVCPPINTERFACE
//    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
//    if(e) {
//      DOMElement *ee=E(e)->getFirstElementChildNamed(MBSIM%"nodes");
//      if(ee) ombvNodes=getVec(ee);
//      OpenMBVExtrusion ombv;
//      openMBVRigidBody=ombv.createOpenMBV(e); 
//    }
#endif
  }

  DOMElement* SpatialContour::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = RigidContour::writeXMLFile(parent);
//    addElementText(ele0,MBSIM%"alphaStart",as);
//    addElementText(ele0,MBSIM%"alphaEnd",ae);
//    addElementText(ele0,MBSIM%"nodes",nodes);
//    addElementText(ele0,MBSIM%"diameter",diameter);
//    DOMElement *ele1 = new DOMElement(MBSIM%"contourFunction");
//    funcCrPC->writeXMLFile(ele1);
//    ele0->LinkEndChild(ele1);
    return ele0;
  }

}
