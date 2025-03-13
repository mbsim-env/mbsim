/* Copyright (C) 2004-2018 MBSim Development Team
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
#include "mbsim/contours/spatial_nurbs_contour.h"
#include "mbsim/utils/utils.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, SpatialNurbsContour)

  void SpatialNurbsContour::updateHessianMatrix(const Vec2 &zeta_) {
    Vec2 zeta(NONINIT);
    if(openEta)
      zeta = zeta_;
    else if(openXi) {
      zeta(0) = mod(zeta_(0)-etaNodes[0],etaNodes[1]-etaNodes[0])+etaNodes[0];
      zeta(1) = zeta_(1);
    }
    else {
      if(mod(zeta_(1)-xiNodes[0],2.*(xiNodes[1]-xiNodes[0]))+xiNodes[0]>xiNodes[1]) {
        zeta(0) = mod(zeta_(0)+0.5*(etaNodes[1]-etaNodes[0])-etaNodes[0],etaNodes[1]-etaNodes[0])+etaNodes[0];
        zeta(1) = xiNodes[1]-mod(zeta_(1)-xiNodes[0],xiNodes[1]-xiNodes[0]);
      }
      else {
        zeta(0) = mod(zeta_(0)-etaNodes[0],etaNodes[1]-etaNodes[0])+etaNodes[0];
        zeta(1) = mod(zeta_(1)-xiNodes[0],xiNodes[1]-xiNodes[0])+xiNodes[0];
      }
    }
    srf.deriveAtH(zeta(0),zeta(1),2,hess);
    zetaOld = zeta_;
  }

  Vec3 SpatialNurbsContour::evalKrPS(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(0,0)(Range<Fixed<0>,Fixed<2>>());
  }

  Vec3 SpatialNurbsContour::evalKs(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(1,0)(Range<Fixed<0>,Fixed<2>>());
  }

  Vec3 SpatialNurbsContour::evalKt(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(0,1)(Range<Fixed<0>,Fixed<2>>());
  }

  Vec3 SpatialNurbsContour::evalParDer1Ks(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(2,0)(Range<Fixed<0>,Fixed<2>>());
  }

  Vec3 SpatialNurbsContour::evalParDer2Ks(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(1,1)(Range<Fixed<0>,Fixed<2>>());
  }

  Vec3 SpatialNurbsContour::evalParDer1Kt(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(1,1)(Range<Fixed<0>,Fixed<2>>());
  }

  Vec3 SpatialNurbsContour::evalParDer2Kt(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(0,2)(Range<Fixed<0>,Fixed<2>>());
  }

  void SpatialNurbsContour::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit) {
      if(cp.rows() != nu*nv)
        throwError("(SpatialNurbsContour::init): wrong number of control points");
      if(interpolation==unknown)
        throwError("(SpatialNurbsContour::init): interpolation unknown");
      GeneralMatrix<Vec4> cp(nu,nv);
      for(int i=0; i<nu; i++) {
        for(int j=0; j<nv; j++) {
          for(int k=0; k<4; k++)
            cp(i,j)(k) = this->cp(j*nu+i,k);
        }
    }
      if(interpolation==none) {
        srf.resize(nu,nv,uKnot.size()-nu-1,vKnot.size()-nv-1);
        srf.setDegreeU(uKnot.size()-nu-1);
        srf.setDegreeV(vKnot.size()-nv-1);
        srf.setKnotU(uKnot);
        srf.setKnotV(vKnot);
        srf.setCtrlPnts(cp);
      }
      else {
        if(openEta) {
          srf.globalInterpH(cp,etaDegree,xiDegree,NurbsSurface::Method(interpolation));
          if(not openXi)
            throwError("(SpatialNurbsContour::init): contour with open eta and closed xi not allowed");
        }
        else
          srf.globalInterpClosedUH(cp,etaDegree,xiDegree,NurbsSurface::Method(interpolation));
      }
      srf.deriveAtH(zetaOld(0),zetaOld(1),2,hess);
      etaNodes.resize(2);
      etaNodes[0] = srf.knotU()(srf.degreeU());
      etaNodes[1] = srf.knotU()(srf.knotU().size()-srf.degreeU()-1);
      xiNodes.resize(2);
      xiNodes[0] = srf.knotV()(srf.degreeV());
      xiNodes[1] = srf.knotV(srf.knotV().size()-srf.degreeV()-1);
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        vector<vector<double>> cp_(srf.ctrlPnts().rows()*srf.ctrlPnts().cols(),vector<double>(4));
        for(int i=0; i<srf.ctrlPnts().rows(); i++) {
          for(int j=0; j<srf.ctrlPnts().cols(); j++) {
            for(int k=0; k<4; k++)
              cp_[j*srf.ctrlPnts().rows()+i][k] = srf.ctrlPnts()(i,j)(k);
          }
        }
        static_pointer_cast<OpenMBV::NurbsSurface>(openMBVRigidBody)->setControlPoints(cp_);
        static_pointer_cast<OpenMBV::NurbsSurface>(openMBVRigidBody)->setUKnotVector((vector<double>)srf.knotU());
        static_pointer_cast<OpenMBV::NurbsSurface>(openMBVRigidBody)->setVKnotVector((vector<double>)srf.knotV());
        static_pointer_cast<OpenMBV::NurbsSurface>(openMBVRigidBody)->setNumberOfUControlPoints(srf.ctrlPnts().rows());
        static_pointer_cast<OpenMBV::NurbsSurface>(openMBVRigidBody)->setNumberOfVControlPoints(srf.ctrlPnts().cols());
      }
    }
    RigidContour::init(stage, config);
  }

  void SpatialNurbsContour::setControlPoints(const MatVx3 &cp_) {
    cp.resize(cp_.rows(),NONINIT);
    for(int i=0; i<cp.rows(); i++) {
      for(int j=0; j<3; j++)
        cp(i,j) = cp_(i,j);
      cp(i,3) = 1;
    }
  }

  void SpatialNurbsContour::setControlPoints(const vector<vector<Vec4>> &cp_) {
    cp.resize(cp_.size()*cp_[0].size(),NONINIT);
    for(size_t i=0; i<cp_.size(); i++) {
      for(size_t j=0; j<cp_[0].size(); j++) {
        for(int k=0; k<4; k++)
          cp(j*nu+i,k) = cp_[i][j](k);
      }
    }
  }

  void SpatialNurbsContour::setControlPoints(const vector<vector<Vec3>> &cp_) {
    cp.resize(cp_.size()*cp_[0].size(),NONINIT);
    for(size_t i=0; i<cp_.size(); i++) {
      for(size_t j=0; j<cp_[0].size(); j++) {
        for(int k=0; k<3; k++)
          cp(j*nu+i,k) = cp_[i][j](k);
        cp(j*nu+i,3) = 1;
      }
    }
  }

  void SpatialNurbsContour::initializeUsingXML(DOMElement * element) {
    RigidContour::initializeUsingXML(element);
    DOMElement * e;
    e=E(element)->getFirstElementChildNamed(MBSIM%"interpolation");
    if(e) {
      string interpolationStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
      if(interpolationStr=="equallySpaced") interpolation=equallySpaced;
      else if(interpolationStr=="chordLength") interpolation=chordLength;
      else if(interpolationStr=="none") interpolation=none;
      else interpolation=unknown;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"controlPoints");
    MatV pts=E(e)->getText<MatV>();
    pts.cols()==3?setControlPoints(MatVx3(pts)):setControlPoints(MatVx4(pts));
    e=E(element)->getFirstElementChildNamed(MBSIM%"numberOfEtaControlPoints");
    nu = E(e)->getText<int>();
    e=E(element)->getFirstElementChildNamed(MBSIM%"numberOfXiControlPoints");
    nv = E(e)->getText<int>();
    e=E(element)->getFirstElementChildNamed(MBSIM%"etaKnotVector");
    if(e) setEtaKnotVector(E(e)->getText<VecV>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"xiKnotVector");
    if(e) setXiKnotVector(E(e)->getText<VecV>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"etaDegree");
    if(e) setEtaDegree(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"xiDegree");
    if(e) setXiDegree(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"openEta");
    if(e) setOpenEta(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"openXi");
    if(e) setOpenXi(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVColoredBody ombv;
      ombv.initializeUsingXML(e);
      openMBVRigidBody=ombv.createOpenMBV<OpenMBV::NurbsSurface>();
    }
  }

  bool SpatialNurbsContour::isZetaOutside(const fmatvec::Vec2 &zeta) {
    if(openEta and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]))
      return true;
    if(openXi and (zeta(1) < xiNodes[0] or zeta(1) > xiNodes[xiNodes.size()-1]))
      return true;
    return false;
  }

}
