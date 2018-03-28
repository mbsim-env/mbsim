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
#include "mbsim/contours/planar_nurbs_contour.h"
#include "mbsim/utils/utils.h"

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

   MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, PlanarNurbsContour)

  void PlanarNurbsContour::updateHessianMatrix(double eta_) {
    double eta = open?eta_:mod(eta_-etaNodes[0],etaNodes[1]-etaNodes[0])+etaNodes[0];
    crv.deriveAtH(eta,2,hess);
    etaOld = eta;
  }

  Vec3 PlanarNurbsContour::evalKrPS(const Vec2 &zeta) {
    return evalHessianMatrix(zeta(0)).row(0).T()(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 PlanarNurbsContour::evalKs(const Vec2 &zeta) {
    return evalHessianMatrix(zeta(0)).row(1).T()(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 PlanarNurbsContour::evalKt(const Vec2 &zeta) {
    static Vec3 Kt("[0;0;1]");
    return Kt;
  }

  Vec3 PlanarNurbsContour::evalParDer1Ks(const Vec2 &zeta) {
    return evalHessianMatrix(zeta(0)).row(2).T()(Range<Fixed<0>,Fixed<2> >());
  }

  void PlanarNurbsContour::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit) {
      if(interpolation==unknown)
        throwError("(PlanarNurbsContour::init): interpolation unknown");
      if(interpolation==none) {
        degree = knot.size()-cp.rows()-1;
        crv.resize(cp.rows(),degree);
        crv.setCtrlPnts(cp);
        crv.setKnot(knot);
      }
      else
        crv.globalInterpH(cp,degree,NurbsCurve::Method(interpolation));
      crv.deriveAtH(etaOld,2,hess);
      etaNodes.resize(2);
      etaNodes[0] = crv.knot()(degree);
      etaNodes[1] = crv.knot()(crv.knot().size()-degree-1);
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] && openMBVRigidBody) {
        vector<vector<double> > cp_(cp.rows(),vector<double>(4));
        for(int i=0; i<cp.rows(); i++) {
          for(int j=0; j<4; j++)
            cp_[i][j] = crv.ctrlPnts()(i,j);
        }
        static_pointer_cast<OpenMBV::NurbsCurve>(openMBVRigidBody)->setControlPoints(cp_);
        static_pointer_cast<OpenMBV::NurbsCurve>(openMBVRigidBody)->setKnotVector(crv.knot());
        static_pointer_cast<OpenMBV::NurbsCurve>(openMBVRigidBody)->setNumberOfControlPoints(cp.rows());
      }
    }
    RigidContour::init(stage, config);
  }

  double PlanarNurbsContour::getCurvature(const Vec2 &zeta) {
    throw;
//    const Vec3 rs = funcCrPC->parDer(zeta(0));
//    return nrm2(crossProduct(rs,funcCrPC->parDerParDer(zeta(0))))/pow(nrm2(rs),3);
  }

  void PlanarNurbsContour::initializeUsingXML(DOMElement * element) {
    RigidContour::initializeUsingXML(element);
    DOMElement * e;
    //e=E(element)->getFirstElementChildNamed(MBSIM%"nodes");
    //etaNodes=E(e)->getText<Vec>();
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
    e=E(element)->getFirstElementChildNamed(MBSIM%"numberOfControlPoints");
    int n = E(e)->getText<int>();
    cp.resize(n);
    for(int i=0; i<n; i++) {
      for(int j=0; j<std::min(pts.cols(),4); j++)
        cp(i,j) = pts(i,j);
      if(pts.cols()<4)
        cp(i,3) = 1;
    }
    e=E(element)->getFirstElementChildNamed(MBSIM%"knotVector");
    if(e) setKnotVector(E(e)->getText<VecV>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"degreee");
    if(e) setDegree(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"open");
    if(e) setOpen(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"enableOpenMBV");
    if(e) {
      OpenMBVNurbsCurve ombv;
      ombv.initializeUsingXML(e);
      openMBVRigidBody=ombv.createOpenMBV();
    }
  }

}
