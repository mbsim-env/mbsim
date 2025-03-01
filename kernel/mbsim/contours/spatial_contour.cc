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
#include <openmbvcppinterface/indexedfaceset.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, SpatialContour)

  SpatialContour::~SpatialContour() {
    delete funcCrPC;
    funcCrPC=nullptr;
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
    static Vec2 dir1("[1;0]");
    return funcCrPC->parDerDirDer(dir1,zeta).col(0);
  }

  Vec3 SpatialContour::evalParDer2Ks(const Vec2 &zeta) {
    static Vec2 dir2("[0;1]");
    return funcCrPC->parDerDirDer(dir2,zeta).col(0);
  }

  Vec3 SpatialContour::evalParDer1Kt(const Vec2 &zeta) {
    static Vec2 dir1("[1;0]");
    return funcCrPC->parDerDirDer(dir1,zeta).col(1);
  }

  Vec3 SpatialContour::evalParDer2Kt(const Vec2 &zeta) {
    static Vec2 dir2("[0;1]");
    return funcCrPC->parDerDirDer(dir2,zeta).col(1);
  }

  void SpatialContour::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit) {
      if (etaNodes.size() < 2)
        throwError("(SpatialContour::init): Size of etaNodes must be greater than 1.");
      if (xiNodes.size() < 2)
        throwError("(SpatialContour::init): Size of xiNodes must be greater than 1.");
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        vector<double> ombvEtaNodes = ombv->getEtaNodes();
        vector<double> ombvXiNodes = ombv->getXiNodes();
        if(not(ombvEtaNodes.size())) {
          ombvEtaNodes.resize(51);
          for(unsigned int i=0; i<ombvEtaNodes.size(); i++)
            ombvEtaNodes[i] = etaNodes[0] + (etaNodes[etaNodes.size()-1]-etaNodes[0])*i/50.;
        }
        if(not(ombvXiNodes.size())) {
          ombvXiNodes.resize(51);
          for(unsigned int i=0; i<ombvXiNodes.size(); i++)
            ombvXiNodes[i] = xiNodes[0] + (xiNodes[xiNodes.size()-1]-xiNodes[0])*i/50.;
        }
        vector<vector<double>> vp(ombvEtaNodes.size()*ombvXiNodes.size());
        Vec2 zeta(NONINIT);
        int n = ombvXiNodes.size();
        for (unsigned int i=0; i<ombvEtaNodes.size(); i++) {
          zeta(0) = ombvEtaNodes[i];
          for (unsigned int j=0; j<ombvXiNodes.size(); j++) {
            zeta(1) = ombvXiNodes[j];
            const Vec3 CrPC=(*funcCrPC)(zeta);
            vp[i*n+j].push_back(CrPC(0));
            vp[i*n+j].push_back(CrPC(1));
            vp[i*n+j].push_back(CrPC(2));
          }
        }
        vector<int> indices(5*(ombvEtaNodes.size()-1)*(ombvXiNodes.size()-1));
        int k=0;
        for(unsigned int i=0; i<ombvEtaNodes.size()-1; i++) {
          for(unsigned int j=0; j<ombvXiNodes.size()-1; j++) {
            indices[k+2] = i*ombvXiNodes.size()+j;
            indices[k+1] = i*ombvXiNodes.size()+j+1;
            indices[k+3] = (i+1)*ombvXiNodes.size()+j;
            indices[k] = (i+1)*ombvXiNodes.size()+j+1;
            indices[k+4] = -1;
            k+=5;
          }
        }
        static_pointer_cast<OpenMBV::IndexedFaceSet>(openMBVRigidBody)->setVertexPositions(vp);
        static_pointer_cast<OpenMBV::IndexedFaceSet>(openMBVRigidBody)->setIndices(indices);
      }
    }
    RigidContour::init(stage, config);

    funcCrPC->init(stage, config);
  }

  void SpatialContour::setContourFunction(Function<Vec3(Vec2)> *func) {
    funcCrPC=func;
    funcCrPC->setParent(this);
    funcCrPC->setName("Contour");
  }

  double SpatialContour::getCurvature(const Vec2 &zeta) {
    throwError("(SpatialContour::getCurvature): not implemented");
  }

  void SpatialContour::initializeUsingXML(DOMElement * element) {
    RigidContour::initializeUsingXML(element);
    DOMElement * e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"etaNodes");
    setEtaNodes(E(e)->getText<vector<double>>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"xiNodes");
    setXiNodes(E(e)->getText<vector<double>>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"contourFunction");
    setContourFunction(ObjectFactory::createAndInit<Function<Vec3(Vec2)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"openEta");
    if(e) setOpenEta(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"openXi");
    if(e) setOpenXi(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      ombv = shared_ptr<OpenMBVSpatialContour>(new OpenMBVSpatialContour);
      ombv->initializeUsingXML(e);
      openMBVRigidBody=ombv->createOpenMBV();
    }
  }

  bool SpatialContour::isZetaOutside(const fmatvec::Vec2 &zeta) {
    if(openEta and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]))
      return true;
    if(openXi and (zeta(1) < xiNodes[0] or zeta(1) > xiNodes[xiNodes.size()-1]))
      return true;
    return false;
  }

}
