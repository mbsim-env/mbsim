/* Copyright (C) 2004-2025 MBSim Development Team
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
#include "revolution.h"
#include "mbsim/frames/frame.h"
#include <openmbvcppinterface/indexedfaceset.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Revolution)

  Revolution::Revolution(const string& name, Frame *R) : RigidContour(name,R) {
    etaNodes = {0,2*M_PI};
    xiNodes = {0,1};
  }

  void Revolution::setProfileFunction(Function<double(double)> *fz_) {
    fz=fz_;
    fz->setParent(this);
    fz->setName("ProfileFunction");
  }

  Vec3 Revolution::evalKrPS(const Vec2 &zeta) {
    static Vec3 Kr(NONINIT);
    double phi = zeta(0);
    double z = r0(1)+(*fz)(zeta(1));
    Kr(0) = z*sin(phi);
    Kr(1) = r0(0)+zeta(1);
    Kr(2) = z*cos(phi);
    return Kr;
  }

  Vec3 Revolution::evalKs(const Vec2 &zeta) {
    static Vec3 Ks;
    double phi = zeta(0);
    double z = r0(1)+(*fz)(zeta(1));
    Ks(0) = z*cos(phi);
    Ks(2) = -z*sin(phi);
    return Ks;
  }

  Vec3 Revolution::evalKt(const Vec2 &zeta) {
    static Vec3 Kt(NONINIT);
    double phi = zeta(0);
    double zs = fz->parDer(zeta(1));
    Kt(0) = zs*sin(phi);
    Kt(1) = 1;
    Kt(2) = zs*cos(phi);
    return Kt;
  }

  Vec3 Revolution::evalParDer1Ks(const Vec2 &zeta) {
    static Vec3 parDer1Ks;
    double phi = zeta(0);
    double z = r0(1)+(*fz)(zeta(1));
    parDer1Ks(0) = -z*sin(phi);
    parDer1Ks(2) = -z*cos(phi);
    return parDer1Ks;
  }

  Vec3 Revolution::evalParDer2Ks(const Vec2 &zeta) {
    static Vec3 parDer2Ks;
    double phi = zeta(0);
    double zs = fz->parDer(zeta(1));
    parDer2Ks(0) = zs*cos(phi);
    parDer2Ks(2) = -zs*sin(phi);
    return parDer2Ks;
  }

  Vec3 Revolution::evalParDer1Kt(const Vec2 &zeta) {
    static Vec3 parDer1Kt;
    double phi = zeta(0);
    double zs = fz->parDer(zeta(1));
    parDer1Kt(0) = zs*cos(phi);
    parDer1Kt(2) = -zs*sin(phi);
    return parDer1Kt;
  }

  Vec3 Revolution::evalParDer2Kt(const Vec2 &zeta) {
    static Vec3 parDer2Kt;
    double phi = zeta(0);
    double zss = fz->parDerDirDer(1,zeta(1));
    parDer2Kt(0) = zss*sin(phi);
    parDer2Kt(2) = zss*cos(phi);
    return parDer2Kt;
  }

  bool Revolution::isZetaOutside(const fmatvec::Vec2 &zeta) {
    if(zeta(1)<xiNodes[0] or zeta(1)>xiNodes[xiNodes.size()-1])
      return true;
    return false;
  }

  void Revolution::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(xiNodes.size() < 2)
        throwError("(Revolution::init): Size of xiNodes must be greater than 1.");
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
	vector<vector<double>> vp;
        vp.resize(51*51);
        for (int i=0; i<51; i++) {
          double eta = 2*M_PI*i/50.;
          for (int j=0; j<51; j++) {
            double xi = xiNodes[0] + (xiNodes[xiNodes.size()-1]-xiNodes[0])*j/50.;
            double zi = r0(1)+(*fz)(xi);
            vp[i*51+j].push_back(zi*sin(eta));
            vp[i*51+j].push_back(r0(0)+xi);
            vp[i*51+j].push_back(zi*cos(eta));
          }
        }
	vector<int> indices(5*50*50);
	int k=0;
	for(int i=0; i<50; i++) {
	  for(int j=0; j<50; j++) {
	    indices[k+2] = i*51+j;
	    indices[k+1] = i*51+j+1;
	    indices[k+3] = (i+1)*51+j;
	    indices[k] = (i+1)*51+j+1;
	    indices[k+4] = -1;
	    k+=5;
	  }
	}
	static_pointer_cast<OpenMBV::IndexedFaceSet>(openMBVRigidBody)->setVertexPositions(vp);
	static_pointer_cast<OpenMBV::IndexedFaceSet>(openMBVRigidBody)->setIndices(indices);
      }
    }
    if(fz) fz->init(stage, config);
    RigidContour::init(stage, config);
  }

  void Revolution::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"positionOfReferencePoint");
    if(e) setPositionOfReferencePoint(E(e)->getText<Vec2>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"nodes");
    setNodes(E(e)->getText<vector<double>>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"profileFunction");
    setProfileFunction(ObjectFactory::createAndInit<Function<double(double)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      auto ombv = shared_ptr<OpenMBVSpatialContour>(new OpenMBVSpatialContour);
      ombv->initializeUsingXML(e);
      openMBVRigidBody=ombv->createOpenMBV();
    }
  }

}
