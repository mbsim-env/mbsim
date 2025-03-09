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
#include "extrusion.h"
#include "mbsim/frames/frame.h"
#include <openmbvcppinterface/indexedfaceset.h>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, Extrusion)

  Extrusion::Extrusion(const string& name, Frame *R) : RigidContour(name,R), A(EYE) {
    etaNodes = {0,1};
    xiNodes = {0,2*M_PI};
  }

  void Extrusion::setPositionFunction(Function<Vec3(double)> *fr_) {
    fr=fr_;
    fr->setParent(this);
    fr->setName("PositionFunction");
  }

  void Extrusion::setOrientationFunction(Function<RotMat3(double)> *fA_) {
    fA=fA_;
    fA->setParent(this);
    fA->setName("OrientationFunction");
  }

  void Extrusion::setProfileFunction(Function<Vec2(double)> *fyz_) {
    fyz=fyz_;
    fyz->setParent(this);
    fyz->setName("ProfileFunction");
  }

  Vec3 Extrusion::evalKrPS(const Vec2 &zeta) {
    if(fr) rOQ = (*fr)(zeta(0));
    if(fA) A = (*fA)(zeta(0));
    auto yz = (*fyz)(zeta(1));
    rQP(1) = yz(0);
    rQP(2) = yz(1);
    return rOQ + A*rQP;
  }

  Vec3 Extrusion::evalKs(const Vec2 &zeta) {
    if(fr) rOQp = fr->parDer(zeta(0));
    // We cannot determine A' directly, so we calculate it from tilde(omega):
    // tilde(parDer(A)) = tilde(omega) = A'*trans(A)
    // A' = tilde(omega)*A
    if(fA) Ap = tilde(fA->parDer(zeta(0)))*(*fA)(zeta(0));
    auto yz = (*fyz)(zeta(1));
    rQP(1) = yz(0);
    rQP(2) = yz(1);
    return rOQp + Ap*rQP;
  }

  Vec3 Extrusion::evalKt(const Vec2 &zeta) {
    if(fA) A = (*fA)(zeta(0));
    auto yzp = fyz->parDer(zeta(1));
    rQPp(1) = yzp(0);
    rQPp(2) = yzp(1);
    return A*rQPp;
  }

  Vec3 Extrusion::evalParDer1Ks(const Vec2 &zeta) {
    if(fr) rOQpp = fr->parDerDirDer(1,zeta(0));
    // We cannot determine A'' directly, so we calculate it from tilde(omega)':
    // parDerDirDer(1,A) = tilde(omega)' = (A'*trans(A))' = A''*trans(A) + A'*trans(A)' = 
    //                                = A''*trans(A) + A'*trans(A') 
    //                                = A''*trans(A) + tilde(omega)*A*trans(tilde(omega)*A)
    //                                = A''*trans(A) + tilde(omega)*A*trans(A)*trans(tilde(omega)
    //                                = A''*trans(A) - tilde(omega)*tilde(omega)
    // A'' = (tilde(omega)' + tilde(omega)*tilde(omega))*A
    if(fA) {
      auto omt = tilde(fA->parDer(zeta(0)));
      App = (tilde(fA->parDerDirDer(1,zeta(0)))+omt*omt)*(*fA)(zeta(0));
    }
    auto yz = (*fyz)(zeta(1));
    rQP(1) = yz(0);
    rQP(2) = yz(1);
    return rOQpp + App*rQP;
  }

  Vec3 Extrusion::evalParDer2Ks(const Vec2 &zeta) {
    if(fr) rOQp = fr->parDer(zeta(0));
    if(fA) Ap = tilde(fA->parDer(zeta(0)))*(*fA)(zeta(0));
    auto yzp = fyz->parDer(zeta(1));
    rQPp(1) = yzp(0);
    rQPp(2) = yzp(1);
    return rOQp + Ap*rQPp;
  }

  Vec3 Extrusion::evalParDer1Kt(const Vec2 &zeta) {
    if(fA) Ap = tilde(fA->parDer(zeta(0)))*(*fA)(zeta(0));
    auto yzp = fyz->parDer(zeta(1));
    rQPp(1) = yzp(0);
    rQPp(2) = yzp(1);
    return Ap*rQPp;
  }

  Vec3 Extrusion::evalParDer2Kt(const Vec2 &zeta) {
    if(fA) A = (*fA)(zeta(0));
    auto yzpp = fyz->parDerDirDer(1,zeta(1));
    rQPpp(1) = yzpp(0);
    rQPpp(2) = yzpp(1);
    return A*rQPpp;
  }

  bool Extrusion::isZetaOutside(const fmatvec::Vec2 &zeta) {
    if(openEta and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]))
      return true;
    if(openXi and (zeta(1) < xiNodes[0] or zeta(1) > xiNodes[xiNodes.size()-1]))
      return true;
    return false;
  }

  void Extrusion::init(InitStage stage, const InitConfigSet &config) {
    if(stage==preInit) {
      if(xiNodes.size() < 2)
        throwError("(Extrusion::init): Size of xiNodes must be greater than 1.");
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
	vector<vector<double>> vp;
        vp.resize(51*51);
        for (int i=0; i<51; i++) {
          double eta = etaNodes[0] + (etaNodes[etaNodes.size()-1]-etaNodes[0])*i/50.;
          if(fr) rOQ = (*fr)(eta);
          if(fA) A = (*fA)(eta);
          for (int j=0; j<51; j++) {
            double xi = xiNodes[0] + (xiNodes[xiNodes.size()-1]-xiNodes[0])*j/50.;
            auto yz = (*fyz)(xi);
            rQP(1) = yz(0);
            rQP(2) = yz(1);
            auto rOP = rOQ + A*rQP;
            vp[i*51+j].push_back(rOP(0));
            vp[i*51+j].push_back(rOP(1));
            vp[i*51+j].push_back(rOP(2));
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
    if(fr) fr->init(stage, config);
    if(fA) fA->init(stage, config);
    if(fyz) fyz->init(stage, config);
    RigidContour::init(stage, config);
  }

  void Extrusion::initializeUsingXML(DOMElement *element) {
    RigidContour::initializeUsingXML(element);
    DOMElement* e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"etaNodes");
    setEtaNodes(E(e)->getText<vector<double>>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"xiNodes");
    if(e) setXiNodes(E(e)->getText<vector<double>>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"positionFunction");
    if(e) setPositionFunction(ObjectFactory::createAndInit<Function<Vec3(double)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"orientationFunction");
    if(e) setOrientationFunction(ObjectFactory::createAndInit<Function<RotMat3(double)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"profileFunction");
    setProfileFunction(ObjectFactory::createAndInit<Function<Vec2(double)>>(e->getFirstElementChild()));
    e=E(element)->getFirstElementChildNamed(MBSIM%"openEta");
    if(e) setOpenEta(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"openXi");
    if(e) setOpenXi(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      auto ombv = shared_ptr<OpenMBVSpatialContour>(new OpenMBVSpatialContour);
      ombv->initializeUsingXML(e);
      openMBVRigidBody=ombv->createOpenMBV();
    }
  }

}
