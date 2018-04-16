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
#include "mbsimFlexibleBody/contours/flexible_planar_nurbs_contour.h"
#include "mbsimFlexibleBody/node_based_body.h"
#include "mbsim/frames/floating_contour_frame.h"
#include "mbsim/utils/utils.h"
#include <openmbvcppinterface/dynamicnurbscurve.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FlexiblePlanarNurbsContour)

  void FlexiblePlanarNurbsContour::updateCurvePositions() {
    MatVx4 cp(index.size());
    for(int i=0; i<index.size(); i++) {
      const Vec3 &x = static_cast<NodeBasedBody*>(parent)->evalNodalPosition(index(i));
      for(int j=0; j<3; j++)
        cp(i,j) = x(j);
      cp(i,3) = 1;
    }
    if(not interpolation)
      crvPos.setCtrlPnts(cp);
    else {
      if(open)
        crvPos.globalInterpH(cp,degree,NurbsCurve::Method(NurbsCurve::equallySpaced));
      else
        crvPos.globalInterpClosedH(cp,degree,NurbsCurve::Method(NurbsCurve::equallySpaced));
    }
    updCrvPos = false;
  }

  void FlexiblePlanarNurbsContour::updateCurveVelocities() {
    MatVx4 cp(index.size());
    for(int i=0; i<index.size(); i++) {
      const Vec3 &x = static_cast<NodeBasedBody*>(parent)->evalNodalVelocity(index(i));
      for(int j=0; j<3; j++)
        cp(i,j) = x(j);
      cp(i,3) = 1;
    }
    if(not interpolation)
      crvVel.setCtrlPnts(cp);
    else {
      if(open)
        crvVel.globalInterpH(cp,degree,NurbsCurve::Method(NurbsCurve::equallySpaced));
      else
        crvVel.globalInterpClosedH(cp,degree,NurbsCurve::Method(NurbsCurve::equallySpaced));
    }
    updCrvVel = false;
  }

  void FlexiblePlanarNurbsContour::updateCurveJacobians() {
    MatVx4 cp(index.size());
    for(size_t k=0; k<crvJac.size(); k++) {
      for(int i=0; i<index.rows(); i++) {
        const Vec3 &x = static_cast<NodeBasedBody*>(parent)->evalNodalJacobianOfTranslation(index(i)).col(k);
        for(int j=0; j<3; j++)
          cp(i,j) = x(j);
        cp(i,3) = 1;
      }
      if(not interpolation)
        crvJac[k].setCtrlPnts(cp);
      else {
        if(open)
          crvJac[k].globalInterpH(cp,degree,NurbsCurve::Method(NurbsCurve::equallySpaced));
        else
          crvJac[k].globalInterpClosedH(cp,degree,NurbsCurve::Method(NurbsCurve::equallySpaced));
      }
    }
    updCrvJac = false;
  }

  void FlexiblePlanarNurbsContour::updateCurveGyroscopicAccelerations() {
    MatVx4 cp(index.size());
    for(int i=0; i<index.size(); i++) {
      const Vec3 &x = static_cast<NodeBasedBody*>(parent)->evalNodalGyroscopicAccelerationOfTranslation(index(i));
      for(int j=0; j<3; j++)
        cp(i,j) = x(j);
      cp(i,3) = 1;
    }
    if(not interpolation)
      crvGA.setCtrlPnts(cp);
    else {
      if(open)
        crvGA.globalInterpH(cp,degree,NurbsCurve::Method(NurbsCurve::equallySpaced));
      else
        crvGA.globalInterpClosedH(cp,degree,NurbsCurve::Method(NurbsCurve::equallySpaced));
    }
    updCrvGA = false;
  }

  double FlexiblePlanarNurbsContour::continueEta(double eta_) {
    double eta;
    if(open)
      eta = eta_;
    else if(open) {
      eta = mod(eta_-etaNodes[0],etaNodes[1]-etaNodes[0])+etaNodes[0];
    }
    return eta;
  }

  void FlexiblePlanarNurbsContour::updateHessianMatrix(double eta_) {
    if(updCrvPos) updateCurvePositions();
    double eta = continueEta(eta_);
    crvPos.deriveAtH(eta,2,hess);
    etaOld = eta_;
  }

  void FlexiblePlanarNurbsContour::updateHessianMatrix_t(double eta_) {
    if(updCrvVel) updateCurveVelocities();
    double eta = continueEta(eta_);
    crvVel.deriveAtH(eta,2,hess_t);
  }

//  Vec3 FlexiblePlanarNurbsContour::evalWn_t(const Vec2 &zeta) {
//    Vec3 Wsxt = crossProduct(evalWs(zeta),evalWt(zeta));
//    Vec3 Wsxt_t = crossProduct(evalWs_t(zeta),evalWt(zeta)) + crossProduct(evalWs(zeta),evalWt_t(zeta));
//    return Wsxt_t/nrm2(Wsxt) - Wsxt*((Wsxt.T()*Wsxt_t)/pow(nrm2(Wsxt),3));
//  }

  Vec3 FlexiblePlanarNurbsContour::evalWs_t(const Vec2 &zeta) {
    return evalHessianMatrix_t(zeta(0)).row(1).T()(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexiblePlanarNurbsContour::evalWu_t(const Vec2 &zeta) {
    Vec3 Ws = evalWs(zeta);
    Vec3 Ws_t = evalWs_t(zeta);
    return Ws_t/nrm2(Ws) - Ws*((Ws.T()*Ws_t)/pow(nrm2(Ws),3));
  }

  Vec3 FlexiblePlanarNurbsContour::evalPosition(const Vec2 &zeta) {
    return evalHessianMatrix(zeta(0)).row(0).T()(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexiblePlanarNurbsContour::evalWs(const Vec2 &zeta) {
    return evalHessianMatrix(zeta(0)).row(1).T()(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexiblePlanarNurbsContour::evalWt(const Vec2 &zeta) {
    static Vec3 Wt("[0;0;1]");
    return Wt;
  }

  Vec3 FlexiblePlanarNurbsContour::evalParDer1Ws(const Vec2 &zeta) {
    return evalHessianMatrix(zeta(0)).row(2).T()(Range<Fixed<0>,Fixed<2> >());
  }

//  Vec3 FlexiblePlanarNurbsContour::evalParDer2Ws(const Vec2 &zeta) {
//    return evalHessianMatrix(zeta)(1,1)(Range<Fixed<0>,Fixed<2> >());
//  }

//  Vec3 FlexiblePlanarNurbsContour::evalParDer1Wt(const Vec2 &zeta) {
//    return evalHessianMatrix(zeta)(1,1)(Range<Fixed<0>,Fixed<2> >());
//  }

//  Vec3 FlexiblePlanarNurbsContour::evalParDer2Wt(const Vec2 &zeta) {
//    return evalHessianMatrix(zeta)(0,2)(Range<Fixed<0>,Fixed<2> >());
//  }

  Vec3 FlexiblePlanarNurbsContour::evalParDer1Wu(const Vec2 &zeta) {
    Vec3 Ws = evalWs(zeta);
    Vec3 parDer1Ws = evalParDer1Ws(zeta);
    return parDer1Ws/nrm2(Ws) - Ws*((Ws.T()*parDer1Ws)/pow(nrm2(Ws),3));
  }

//  Vec3 FlexiblePlanarNurbsContour::evalParDer2Wu(const Vec2 &zeta) {
//    Vec3 Ws = evalWs(zeta);
//    Vec3 parDer2Ws = evalParDer2Ws(zeta);
//    return parDer2Ws/nrm2(Ws) - Ws*((Ws.T()*parDer2Ws)/pow(nrm2(Ws),3));
//  }
//
//  Vec3 FlexiblePlanarNurbsContour::evalParDer1Wv(const Vec2 &zeta) {
//    return crossProduct(evalParDer1Wn(zeta),evalWu(zeta)) + crossProduct(evalWn(zeta),evalParDer1Wu(zeta));
//  }
//
//  Vec3 FlexiblePlanarNurbsContour::evalParDer2Wv(const Vec2 &zeta) {
//    return crossProduct(evalParDer2Wn(zeta),evalWu(zeta)) + crossProduct(evalWn(zeta),evalParDer2Wu(zeta));
//  }

//  Vec3 FlexiblePlanarNurbsContour::evalParDer1Wn(const Vec2 &zeta) {
//    Vec3 Wsxt = crossProduct(evalWs(zeta),evalWt(zeta));
//    Vec3 Wsxtd = crossProduct(evalParDer1Ws(zeta),evalWt(zeta)) + crossProduct(evalWs(zeta),evalParDer1Wt(zeta));
//    return Wsxtd/nrm2(Wsxt) - Wsxt*((Wsxt.T()*Wsxtd)/pow(nrm2(Wsxt),3));
//  }

//  Vec3 FlexiblePlanarNurbsContour::evalParDer2Wn(const Vec2 &zeta) {
//    Vec3 Wsxt = crossProduct(evalWs(zeta),evalWt(zeta));
//    Vec3 Wsxtd = crossProduct(evalParDer2Ws(zeta),evalWt(zeta)) + crossProduct(evalWs(zeta),evalParDer2Wt(zeta));
//    return Wsxtd/nrm2(Wsxt) - Wsxt*((Wsxt.T()*Wsxtd)/pow(nrm2(Wsxt),3));
//  }

  void FlexiblePlanarNurbsContour::updatePositions(ContourFrame *frame) {
    throwError("(FlexiblePlanarNurbsContour::updatePositions): not implemented");
  }

  void FlexiblePlanarNurbsContour::updateVelocities(ContourFrame *frame) {
    if(updCrvVel) updateCurveVelocities();
    double eta = continueEta(frame->evalZeta()(0));
    crvVel.deriveAtH(eta,0,hessTmp);
    frame->setVelocity(hessTmp.row(0).T()(Range<Fixed<0>,Fixed<2> >()));
  }

  void FlexiblePlanarNurbsContour::updateAccelerations(ContourFrame *frame) {
    throwError("(FlexiblePlanarNurbsContour::updateAccelerations): not implemented");
  }

  void FlexiblePlanarNurbsContour::updateJacobians(ContourFrame *frame, int j) {
    if(updCrvJac) updateCurveJacobians();
    double eta = continueEta(frame->evalZeta()(0));
    frame->getJacobianOfTranslation(j,false).resize(frame->gethSize(j),NONINIT);
    for(int i=0; i<frame->gethSize(j); i++) {
      crvJac[i].deriveAtH(eta,0,hessTmp);
      frame->getJacobianOfTranslation(j,false).set(i,hessTmp.row(0).T()(Range<Fixed<0>,Fixed<2> >()));
    }
  }

  void FlexiblePlanarNurbsContour::updateGyroscopicAccelerations(ContourFrame *frame) {
    if(updCrvGA) updateCurveGyroscopicAccelerations();
    double eta = continueEta(frame->evalZeta()(0));
    crvGA.deriveAtH(eta,0,hessTmp);
    frame->setGyroscopicAccelerationOfTranslation(hessTmp.row(0).T()(Range<Fixed<0>,Fixed<2> >()));
  }

  void FlexiblePlanarNurbsContour::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit) {
      if(not interpolation) {
        crvPos.resize(index.size(),knot.size()-index.size()-1);
        crvPos.setDegree(knot.size()-index.size()-1);
        crvPos.setKnot(knot);
      }
      else {
        VecV uk(index.size(),NONINIT), U;
        if(open) {
          crvPos.resize(index.size(),degree);
          U.resize(crvPos.knot().size(),NONINIT);
          updateUVecs(0, 1, uk, degree, U);
        }
        else {
          crvPos.resize(index.size()+degree,degree);
          U.resize(crvPos.knot().size(),NONINIT);
          updateUVecsClosed(0, 1, uk, degree, U);
        }
        crvPos.setKnot(U);
      }

      etaNodes.resize(2);
      etaNodes[0] = crvPos.knot()(crvPos.degree());
      etaNodes[1] = crvPos.knot()(crvPos.knot().size()-crvPos.degree()-1);
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] and openMBVNurbsCurve) {
        openMBVNurbsCurve->setName(name);

        openMBVNurbsCurve->setKnotVector(crvPos.knot());
        openMBVNurbsCurve->setNumberOfControlPoints(crvPos.ctrlPnts().rows());

        parent->getOpenMBVGrp()->addObject(openMBVNurbsCurve);
      }
    }
    else if(stage==unknownStage) {
      crvVel.resize(index.size(),knot.size()-index.size()-1);
      crvVel.setDegree(knot.size()-index.size()-1);
      crvVel.setKnot(knot);
      crvJac.resize(gethSize());
      for(size_t i=0; i<crvJac.size(); i++) {
        crvJac[i].resize(index.size(),knot.size()-index.size()-1);
        crvJac[i].setDegree(knot.size()-index.size()-1);
        crvJac[i].setKnot(knot);
      }
      crvGA.resize(index.size(),knot.size()-index.size()-1);
      crvGA.setDegree(knot.size()-index.size()-1);
      crvGA.setKnot(knot);
    }
    FlexiblePlanarContour::init(stage, config);
  }

  ContourFrame* FlexiblePlanarNurbsContour::createContourFrame(const string &name) {
    FloatingContourFrame *frame = new FloatingContourFrame(name);
    frame->setContourOfReference(this);
    return frame;
  }

  double FlexiblePlanarNurbsContour::getCurvature(const Vec2 &zeta) {
    throwError("(FlexiblePlanarNurbsContour::getCurvature): not implemented");
  }

  void FlexiblePlanarNurbsContour::plot() {
    if(plotFeature[openMBV] and openMBVNurbsCurve) {
      if(updCrvPos) updateCurvePositions();
      vector<double> data;
      data.push_back(getTime()); //time
      //Control-Point coordinates
      for(int i=0; i<crvPos.ctrlPnts().rows(); i++) {
        for(int j=0; j<4; j++)
          data.push_back(crvPos.ctrlPnts()(i,j));
      }
      openMBVNurbsCurve->append(data);
    }
    FlexiblePlanarContour::plot();
  }

  void FlexiblePlanarNurbsContour::initializeUsingXML(DOMElement * element) {
    FlexiblePlanarContour::initializeUsingXML(element);
    DOMElement * e;
//    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"etaNodes");
//    etaNodes=E(e)->getText<Vec>();
//    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"xiNodes");
//    xiNodes=E(e)->getText<Vec>();
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"interpolation");
    if(e) setInterpolation(E(e)->getText<bool>());
//    if(e) {
//      string interpolationStr=string(X()%E(e)->getFirstTextChild()->getData()).substr(1,string(X()%E(e)->getFirstTextChild()->getData()).length()-2);
//      if(interpolationStr=="equallySpaced") interpolation=equallySpaced;
//      else if(interpolationStr=="chordLength") interpolation=chordLength;
//      else if(interpolationStr=="none") interpolation=none;
//      else interpolation=unknown;
//    }
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"indices");
    index = E(e)->getText<VecVI>();
    for(int i=0; i<index.size(); i++)
      index(i)--;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"numberOfControlPoints");
    E(e)->getText<int>();
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"knotVector");
    if(e) setKnotVector(E(e)->getText<VecV>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"degree");
    if(e) setDegree(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"open");
    if(e) setOpen(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(e) {
      openMBVNurbsCurve = OpenMBV::ObjectFactory::create<OpenMBV::DynamicNurbsCurve>();
//      OpenMBVNurbsCurve ombv;
//      ombv.initializeUsingXML(e);
//      openMBVRigidBody=ombv.createOpenMBV();
    }
  }

  bool FlexiblePlanarNurbsContour::isZetaOutside(const fmatvec::Vec2 &zeta) {
    return open and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]); 
  }

}
