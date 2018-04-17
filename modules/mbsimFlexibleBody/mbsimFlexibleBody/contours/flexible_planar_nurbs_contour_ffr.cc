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
#include "mbsimFlexibleBody/contours/flexible_planar_nurbs_contour_ffr.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_ffr.h"
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

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FlexiblePlanarNurbsContourFFR)

  double FlexiblePlanarNurbsContourFFR::continueEta(double eta_) {
    double eta;
    if(open)
      eta = eta_;
    else
      eta = mod(eta_-etaNodes[0],etaNodes[1]-etaNodes[0])+etaNodes[0];
    return eta;
  }

  void FlexiblePlanarNurbsContourFFR::updateHessianMatrix(double eta_) {
    double eta = continueEta(eta_);
    crvPos.deriveAtH(eta,2,hessPos);
    for(size_t i=0; i<crvPhi.size(); i++)
      crvPhi[i].deriveAtH(eta,2,hessPhi[i]);
    etaOld = eta_;
  }

  void FlexiblePlanarNurbsContourFFR::updateGlobalRelativePosition(double eta) {
    Vec3 KrKP = evalHessianMatrixPos(eta).row(0).T()(Range<Fixed<0>,Fixed<2> >());
    for(size_t i=0; i<crvPhi.size(); i++)
      KrKP += hessPhi[i].row(0).T()(Range<Fixed<0>,Fixed<2> >())*static_cast<FlexibleBodyFFR*>(parent)->evalqERel()(i);
    WrKP = R->evalOrientation()*KrKP;
    updPos = false;
  }

  void FlexiblePlanarNurbsContourFFR::updateGlobalRelativeVelocity(double eta) {
    if(eta!=etaOld) updateHessianMatrix(eta);
    Vec3 Kvrel;
    for(size_t i=0; i<crvPhi.size(); i++)
      Kvrel += hessPhi[i].row(0).T()(Range<Fixed<0>,Fixed<2> >())*static_cast<FlexibleBodyFFR*>(parent)->evalqdERel()(i);
    Wvrel = R->evalOrientation()*Kvrel;
    updVel = false;
  }

//  Vec3 FlexiblePlanarNurbsContourFFR::evalWn_t(const Vec2 &zeta) {
//    Vec3 Wsxt = crossProduct(evalWs(zeta),evalWt(zeta));
//    Vec3 Wsxt_t = crossProduct(evalWs_t(zeta),evalWt(zeta)) + crossProduct(evalWs(zeta),evalWt_t(zeta));
//    return Wsxt_t/nrm2(Wsxt) - Wsxt*((Wsxt.T()*Wsxt_t)/pow(nrm2(Wsxt),3));
//  }

  Vec3 FlexiblePlanarNurbsContourFFR::evalWs_t(const Vec2 &zeta) {
    if(zeta(0)!=etaOld) updateHessianMatrix(zeta(0));
    Vec3 s_t;
    for(size_t i=0; i<crvPhi.size(); i++)
      s_t += hessPhi[i].row(1).T()(Range<Fixed<0>,Fixed<2> >())*static_cast<FlexibleBodyFFR*>(parent)->evalqdERel()(i);
    return s_t;
  }

  Vec3 FlexiblePlanarNurbsContourFFR::evalWu_t(const Vec2 &zeta) {
    Vec3 Ws = evalWs(zeta);
    Vec3 Ws_t = evalWs_t(zeta);
    return Ws_t/nrm2(Ws) - Ws*((Ws.T()*Ws_t)/pow(nrm2(Ws),3));
  }

  Vec3 FlexiblePlanarNurbsContourFFR::evalPosition(const Vec2 &zeta) {
    Vec3 KrKP = evalHessianMatrixPos(zeta(0)).row(0).T()(Range<Fixed<0>,Fixed<2> >());
    for(size_t i=0; i<crvPhi.size(); i++)
      KrKP += hessPhi[i].row(0).T()(Range<Fixed<0>,Fixed<2> >())*static_cast<FlexibleBodyFFR*>(parent)->evalqERel()(i);
    return R->evalPosition() + R->evalOrientation()*KrKP;
  }

  Vec3 FlexiblePlanarNurbsContourFFR::evalWs(const Vec2 &zeta) {
    Vec3 s = evalHessianMatrixPos(zeta(0)).row(1).T()(Range<Fixed<0>,Fixed<2> >());
    for(size_t i=0; i<crvPhi.size(); i++)
      s += hessPhi[i].row(1).T()(Range<Fixed<0>,Fixed<2> >())*static_cast<FlexibleBodyFFR*>(parent)->evalqERel()(i);
    return s;
  }

  Vec3 FlexiblePlanarNurbsContourFFR::evalWt(const Vec2 &zeta) {
    static Vec3 Wt("[0;0;1]");
    return Wt;
  }

  Vec3 FlexiblePlanarNurbsContourFFR::evalParDer1Ws(const Vec2 &zeta) {
    Vec3 ds = evalHessianMatrixPos(zeta(0)).row(2).T()(Range<Fixed<0>,Fixed<2> >());
    for(size_t i=0; i<crvPhi.size(); i++)
      ds += hessPhi[i].row(2).T()(Range<Fixed<0>,Fixed<2> >())*static_cast<FlexibleBodyFFR*>(parent)->evalqERel()(i);
    return ds;
  }

  Vec3 FlexiblePlanarNurbsContourFFR::evalParDer1Wu(const Vec2 &zeta) {
    Vec3 Ws = evalWs(zeta);
    Vec3 parDer1Ws = evalParDer1Ws(zeta);
    return parDer1Ws/nrm2(Ws) - Ws*((Ws.T()*parDer1Ws)/pow(nrm2(Ws),3));
  }

//  Vec3 FlexiblePlanarNurbsContourFFR::evalParDer1Wn(const Vec2 &zeta) {
//    Vec3 Wsxt = crossProduct(evalWs(zeta),evalWt(zeta));
//    Vec3 Wsxtd = crossProduct(evalParDer1Ws(zeta),evalWt(zeta)) + crossProduct(evalWs(zeta),evalParDer1Wt(zeta));
//    return Wsxtd/nrm2(Wsxt) - Wsxt*((Wsxt.T()*Wsxtd)/pow(nrm2(Wsxt),3));
//  }

  void FlexiblePlanarNurbsContourFFR::updatePositions(ContourFrame *frame) {
    throwError("(FlexiblePlanarNurbsContourFFR::updatePositions): not implemented");
  }

  void FlexiblePlanarNurbsContourFFR::updateVelocities(ContourFrame *frame) {
    frame->setVelocity(R->evalVelocity() + crossProduct(R->evalAngularVelocity(), evalGlobalRelativePosition(frame->evalEta())) + evalGlobalRelativeVelocity(frame->evalEta()));
  }

  void FlexiblePlanarNurbsContourFFR::updateAccelerations(ContourFrame *frame) {
    throwError("(FlexiblePlanarNurbsContourFFR::updateAccelerations): not implemented");
  }

  void FlexiblePlanarNurbsContourFFR::updateJacobians(ContourFrame *frame, int j) {
    if(frame->evalEta()!=etaOld) updateHessianMatrix(frame->evalEta());
    Mat3xV Phi(crvPhi.size(),NONINIT);
    for(size_t i=0; i<crvPhi.size(); i++)
      Phi.set(i,hessPhi[i].row(0).T()(Range<Fixed<0>,Fixed<2> >()));
    Mat3xV J = R->evalJacobianOfTranslation(j) - tilde(evalGlobalRelativePosition(frame->evalEta()))*R->evalJacobianOfRotation(j);
    J.add(RangeV(0,2),RangeV(frame->gethSize(j)-crvPhi.size(),frame->gethSize(j)-1),R->getOrientation()*Phi);
    frame->setJacobianOfTranslation(J,j);
  }

  void FlexiblePlanarNurbsContourFFR::updateGyroscopicAccelerations(ContourFrame *frame) {
    frame->setGyroscopicAccelerationOfTranslation(R->evalGyroscopicAccelerationOfTranslation() + crossProduct(R->evalGyroscopicAccelerationOfRotation(),evalGlobalRelativePosition(frame->evalEta())) + crossProduct(R->evalAngularVelocity(),crossProduct(R->evalAngularVelocity(),evalGlobalRelativePosition(frame->evalEta()))) + 2.*crossProduct(R->evalAngularVelocity(),evalGlobalRelativeVelocity(frame->evalEta())));
  }

  void FlexiblePlanarNurbsContourFFR::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit) {
      R = static_cast<FlexibleBodyFFR*>(parent)->getFrameK();
      crvPos.resize(index.size(),knot.size()-index.size()-1);
      crvPos.setDegree(knot.size()-index.size()-1);
      crvPos.setKnot(knot);
      MatVx4 cp(index.size());
      for(int i=0; i<index.size(); i++) {
        const Vec3 &x = static_cast<FlexibleBodyFFR*>(parent)->getNodalRelativePosition(index(i));
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
      crvPhi.resize(static_cast<FlexibleBodyFFR*>(parent)->getNumberOfModeShapes());
      hessPhi.resize(crvPhi.size());
      for(size_t k=0; k<crvPhi.size(); k++) {
        crvPhi[k].resize(index.size(),knot.size()-index.size()-1);
        crvPhi[k].setDegree(knot.size()-index.size()-1);
        crvPhi[k].setKnot(knot);
        for(int i=0; i<index.size(); i++) {
          const Vec3 &x = static_cast<FlexibleBodyFFR*>(parent)->getNodalShapeMatrixOfTranslation(index(i)).col(k);
          for(int j=0; j<3; j++)
            cp(i,j) = x(j);
          cp(i,3) = 1;
        }
        if(not interpolation)
          crvPhi[k].setCtrlPnts(cp);
        else {
          if(open)
            crvPhi[k].globalInterpH(cp,degree,NurbsCurve::Method(NurbsCurve::equallySpaced));
          else
            crvPhi[k].globalInterpClosedH(cp,degree,NurbsCurve::Method(NurbsCurve::equallySpaced));
        }
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
    FlexibleContour::init(stage, config);
  }

  ContourFrame* FlexiblePlanarNurbsContourFFR::createContourFrame(const string &name) {
    FloatingContourFrame *frame = new FloatingContourFrame(name);
    frame->setContourOfReference(this);
    return frame;
  }

  double FlexiblePlanarNurbsContourFFR::getCurvature(const Vec2 &zeta) {
    throwError("(FlexiblePlanarNurbsContourFFR::getCurvature): not implemented");
  }

  void FlexiblePlanarNurbsContourFFR::plot() {
    if(plotFeature[openMBV] and openMBVNurbsCurve) {
      vector<double> data;
      data.push_back(getTime()); //time
      //Control-Point coordinates
      for(int i=0; i<crvPos.ctrlPnts().rows(); i++) {
        for(int j=0; j<3; j++) {
          double ctrP = crvPos.ctrlPnts()(i,j);
          for(size_t k=0; k<crvPhi.size(); k++)
            ctrP += crvPhi[k].ctrlPnts()(i,j)*static_cast<FlexibleBodyFFR*>(parent)->evalqERel()(k);
          data.push_back(ctrP);
        }
        data.push_back(1);
      }
      openMBVNurbsCurve->append(data);
    }
    FlexibleContour::plot();
  }

  void FlexiblePlanarNurbsContourFFR::initializeUsingXML(DOMElement * element) {
    FlexibleContour::initializeUsingXML(element);
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

  bool FlexiblePlanarNurbsContourFFR::isZetaOutside(const fmatvec::Vec2 &zeta) {
    return open and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]); 
  }

}
