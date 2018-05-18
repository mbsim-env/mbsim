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
#include "mbsimFlexibleBody/contours/flexible_spatial_nurbs_contour.h"
#include "mbsimFlexibleBody/node_based_body.h"
#include "mbsim/frames/floating_contour_frame.h"
#include "mbsim/utils/utils.h"
#include <openmbvcppinterface/dynamicnurbssurface.h>
#include <openmbvcppinterface/group.h>

using namespace std;
using namespace fmatvec;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimFlexibleBody {

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIMFLEX, FlexibleSpatialNurbsContour)

  void FlexibleSpatialNurbsContour::updateSurfacePositions() {
    GeneralMatrix<Vec4> cp(index.rows(),index.cols());
    for(int i=0; i<index.rows(); i++) {
      for(int j=0; j<index.cols(); j++) {
        cp(i,j).set(RangeV(0,2),static_cast<NodeBasedBody*>(parent)->evalNodalPosition(index(i,j)));
        cp(i,j)(3) = 1;
      }
    }
    if(not interpolation)
      srfPos.setCtrlPnts(cp);
    else {
      if(openEta)
        srfPos.globalInterpH(cp,etaDegree,xiDegree,NurbsSurface::Method(NurbsSurface::equallySpaced));
      else
        srfPos.globalInterpClosedUH(cp,etaDegree,xiDegree,NurbsSurface::Method(NurbsSurface::equallySpaced));
    }
    updSrfPos = false;
  }

  void FlexibleSpatialNurbsContour::updateSurfaceVelocities() {
    GeneralMatrix<Vec4> cp(index.rows(),index.cols());
    for(int i=0; i<index.rows(); i++) {
      for(int j=0; j<index.cols(); j++) {
        cp(i,j).set(RangeV(0,2),static_cast<NodeBasedBody*>(parent)->evalNodalVelocity(index(i,j)));
        cp(i,j)(3) = 1;
      }
    }
    if(not interpolation)
      srfVel.setCtrlPnts(cp);
    else {
      if(openEta)
        srfVel.globalInterpH(cp,etaDegree,xiDegree,NurbsSurface::Method(NurbsSurface::equallySpaced));
      else
        srfVel.globalInterpClosedUH(cp,etaDegree,xiDegree,NurbsSurface::Method(NurbsSurface::equallySpaced));
    }
    updSrfVel = false;
  }

  void FlexibleSpatialNurbsContour::updateSurfaceJacobians() {
    GeneralMatrix<Vec4> cp(index.rows(),index.cols());
    for(size_t k=0; k<srfJac.size(); k++) {
      for(int i=0; i<index.rows(); i++) {
        for(int j=0; j<index.cols(); j++) {
          cp(i,j).set(RangeV(0,2),static_cast<NodeBasedBody*>(parent)->evalNodalJacobianOfTranslation(index(i,j)).col(k));
          cp(i,j)(3) = 1;
        }
      }
      if(not interpolation)
        srfJac[k].setCtrlPnts(cp);
      else {
        if(openEta)
          srfJac[k].globalInterpH(cp,etaDegree,xiDegree,NurbsSurface::Method(NurbsSurface::equallySpaced));
        else
          srfJac[k].globalInterpClosedUH(cp,etaDegree,xiDegree,NurbsSurface::Method(NurbsSurface::equallySpaced));
      }
    }
    updSrfJac = false;
  }

  void FlexibleSpatialNurbsContour::updateSurfaceGyroscopicAccelerations() {
    GeneralMatrix<Vec4> cp(index.rows(),index.cols());
    for(int i=0; i<index.rows(); i++) {
      for(int j=0; j<index.cols(); j++) {
        cp(i,j).set(RangeV(0,2),static_cast<NodeBasedBody*>(parent)->evalNodalGyroscopicAccelerationOfTranslation(index(i,j)));
        cp(i,j)(3) = 1;
      }
    }
    if(not interpolation)
      srfGA.setCtrlPnts(cp);
    else {
      if(openEta)
        srfGA.globalInterpH(cp,etaDegree,xiDegree,NurbsSurface::Method(NurbsSurface::equallySpaced));
      else
        srfGA.globalInterpClosedUH(cp,etaDegree,xiDegree,NurbsSurface::Method(NurbsSurface::equallySpaced));
    }
    updSrfGA = false;
  }

  Vec2 FlexibleSpatialNurbsContour::continueZeta(const Vec2 &zeta_) {
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
    return zeta;
  }

  void FlexibleSpatialNurbsContour::updateHessianMatrix(const Vec2 &zeta_) {
    if(updSrfPos) updateSurfacePositions();
    Vec2 zeta = continueZeta(zeta_);
    srfPos.deriveAtH(zeta(0),zeta(1),2,hess);
    zetaOld = zeta_;
  }

  void FlexibleSpatialNurbsContour::updateHessianMatrix_t(const Vec2 &zeta_) {
    if(updSrfVel) updateSurfaceVelocities();
    Vec2 zeta = continueZeta(zeta_);
    srfVel.deriveAtH(zeta(0),zeta(1),2,hess_t);
  }

  Vec3 FlexibleSpatialNurbsContour::evalWn_t(const Vec2 &zeta) {
    Vec3 Wsxt = crossProduct(evalWs(zeta),evalWt(zeta));
    Vec3 Wsxt_t = crossProduct(evalWs_t(zeta),evalWt(zeta)) + crossProduct(evalWs(zeta),evalWt_t(zeta));
    return Wsxt_t/nrm2(Wsxt) - Wsxt*((Wsxt.T()*Wsxt_t)/pow(nrm2(Wsxt),3));
  }

  Vec3 FlexibleSpatialNurbsContour::evalWs_t(const Vec2 &zeta) {
    return evalHessianMatrix_t(zeta)(1,0)(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexibleSpatialNurbsContour::evalWt_t(const Vec2 &zeta) {
    return evalHessianMatrix_t(zeta)(0,1)(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexibleSpatialNurbsContour::evalWu_t(const Vec2 &zeta) {
    Vec3 Ws = evalWs(zeta);
    Vec3 Ws_t = evalWs_t(zeta);
    return Ws_t/nrm2(Ws) - Ws*((Ws.T()*Ws_t)/pow(nrm2(Ws),3));
  }

  Vec3 FlexibleSpatialNurbsContour::evalWv_t(const Vec2 &zeta) {
    return crossProduct(evalWn_t(zeta),evalWu(zeta)) + crossProduct(evalWn(zeta),evalWu_t(zeta));
  }

  Vec3 FlexibleSpatialNurbsContour::evalPosition(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(0,0)(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexibleSpatialNurbsContour::evalWs(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(1,0)(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexibleSpatialNurbsContour::evalWt(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(0,1)(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexibleSpatialNurbsContour::evalParDer1Ws(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(2,0)(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexibleSpatialNurbsContour::evalParDer2Ws(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(1,1)(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexibleSpatialNurbsContour::evalParDer1Wt(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(1,1)(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexibleSpatialNurbsContour::evalParDer2Wt(const Vec2 &zeta) {
    return evalHessianMatrix(zeta)(0,2)(Range<Fixed<0>,Fixed<2> >());
  }

  Vec3 FlexibleSpatialNurbsContour::evalParDer1Wu(const Vec2 &zeta) {
    Vec3 Ws = evalWs(zeta);
    Vec3 parDer1Ws = evalParDer1Ws(zeta);
    return parDer1Ws/nrm2(Ws) - Ws*((Ws.T()*parDer1Ws)/pow(nrm2(Ws),3));
  }

  Vec3 FlexibleSpatialNurbsContour::evalParDer2Wu(const Vec2 &zeta) {
    Vec3 Ws = evalWs(zeta);
    Vec3 parDer2Ws = evalParDer2Ws(zeta);
    return parDer2Ws/nrm2(Ws) - Ws*((Ws.T()*parDer2Ws)/pow(nrm2(Ws),3));
  }

  Vec3 FlexibleSpatialNurbsContour::evalParDer1Wv(const Vec2 &zeta) {
    return crossProduct(evalParDer1Wn(zeta),evalWu(zeta)) + crossProduct(evalWn(zeta),evalParDer1Wu(zeta));
  }

  Vec3 FlexibleSpatialNurbsContour::evalParDer2Wv(const Vec2 &zeta) {
    return crossProduct(evalParDer2Wn(zeta),evalWu(zeta)) + crossProduct(evalWn(zeta),evalParDer2Wu(zeta));
  }

  Vec3 FlexibleSpatialNurbsContour::evalParDer1Wn(const Vec2 &zeta) {
    Vec3 Wsxt = crossProduct(evalWs(zeta),evalWt(zeta));
    Vec3 Wsxtd = crossProduct(evalParDer1Ws(zeta),evalWt(zeta)) + crossProduct(evalWs(zeta),evalParDer1Wt(zeta));
    return Wsxtd/nrm2(Wsxt) - Wsxt*((Wsxt.T()*Wsxtd)/pow(nrm2(Wsxt),3));
  }

  Vec3 FlexibleSpatialNurbsContour::evalParDer2Wn(const Vec2 &zeta) {
    Vec3 Wsxt = crossProduct(evalWs(zeta),evalWt(zeta));
    Vec3 Wsxtd = crossProduct(evalParDer2Ws(zeta),evalWt(zeta)) + crossProduct(evalWs(zeta),evalParDer2Wt(zeta));
    return Wsxtd/nrm2(Wsxt) - Wsxt*((Wsxt.T()*Wsxtd)/pow(nrm2(Wsxt),3));
  }

  Vec3 FlexibleSpatialNurbsContour::evalParWvCParEta(const Vec2 &zeta) {
    return evalWs_t(zeta);
  }

  Vec3 FlexibleSpatialNurbsContour::evalParWvCParXi(const Vec2 &zeta) {
    return evalWt_t(zeta);
  }

  Vec3 FlexibleSpatialNurbsContour::evalParWuPart(const Vec2 &zeta) {
    return evalWu_t(zeta);
  }

  Vec3 FlexibleSpatialNurbsContour::evalParWvPart(const Vec2 &zeta) {
    return evalWv_t(zeta);
  }

  void FlexibleSpatialNurbsContour::updatePositions(ContourFrame *frame) {
    throwError("(FlexibleSpatialNurbsContour::updatePositions): not implemented");
  }

  void FlexibleSpatialNurbsContour::updateVelocities(ContourFrame *frame) {
    if(updSrfVel) updateSurfaceVelocities();
    Vec2 zeta = continueZeta(frame->evalZeta());
    srfVel.deriveAtH(zeta(0),zeta(1),0,hessTmp);
    frame->setVelocity(hessTmp(0,0)(Range<Fixed<0>,Fixed<2> >()));
  }

  void FlexibleSpatialNurbsContour::updateAccelerations(ContourFrame *frame) {
    throwError("(FlexibleSpatialNurbsContour::updateAccelerations): not implemented");
  }

  void FlexibleSpatialNurbsContour::updateJacobians(ContourFrame *frame, int j) {
    if(updSrfJac) updateSurfaceJacobians();
    Vec2 zeta = continueZeta(frame->evalZeta());
    frame->getJacobianOfTranslation(j,false).resize(frame->gethSize(j),NONINIT);
    for(int i=0; i<frame->gethSize(j); i++) {
      srfJac[i].deriveAtH(zeta(0),zeta(1),0,hessTmp);
      frame->getJacobianOfTranslation(j,false).set(i,hessTmp(0,0)(Range<Fixed<0>,Fixed<2> >()));
    }
  }

  void FlexibleSpatialNurbsContour::updateGyroscopicAccelerations(ContourFrame *frame) {
    if(updSrfGA) updateSurfaceGyroscopicAccelerations();
    Vec2 zeta = continueZeta(frame->evalZeta());
    srfGA.deriveAtH(zeta(0),zeta(1),0,hessTmp);
    frame->setGyroscopicAccelerationOfTranslation(hessTmp(0,0)(Range<Fixed<0>,Fixed<2> >()));
  }

  void FlexibleSpatialNurbsContour::init(InitStage stage, const InitConfigSet &config) {
    if (stage == preInit) {
      if(not interpolation) {
        srfPos.resize(index.rows(),index.cols(),uKnot.size()-index.rows()-1,vKnot.size()-index.cols()-1);
        srfPos.setDegreeU(uKnot.size()-index.rows()-1);
        srfPos.setDegreeV(vKnot.size()-index.cols()-1);
        srfPos.setKnotU(uKnot);
        srfPos.setKnotV(vKnot);
      }
      else {
        VecV uk(index.rows(),NONINIT), vk(index.cols(),NONINIT), U, V;
        if(openEta) {
          srfPos.resize(index.rows(),index.cols(),etaDegree,xiDegree);
          U.resize(srfPos.knotU().size(),NONINIT);
          V.resize(srfPos.knotV().size(),NONINIT);
          updateUVecs(0, 1, uk, etaDegree, U);
          updateUVecs(0, 1, vk, xiDegree, V);
          if(not openXi)
            throwError("(FlexibleSpatialNurbsContour::init): contour with open eta and closed xi not allowed");
        }
        else {
          srfPos.resize(index.rows()+etaDegree,index.cols(),etaDegree,xiDegree);
          U.resize(srfPos.knotU().size(),NONINIT);
          V.resize(srfPos.knotV().size(),NONINIT);
          updateUVecsClosed(0, 1, uk, etaDegree, U);
          updateUVecs(0, 1, vk, xiDegree, V);
        }
        srfPos.setKnotU(U);
        srfPos.setKnotV(V);
      }

      zetaOld.init(-1e10);
      etaNodes.resize(2);
      etaNodes[0] = srfPos.knotU()(srfPos.degreeU());
      etaNodes[1] = srfPos.knotU()(srfPos.knotU().size()-srfPos.degreeU()-1);
      xiNodes.resize(2);
      xiNodes[0] = srfPos.knotV()(srfPos.degreeV());
      xiNodes[1] = srfPos.knotV(srfPos.knotV().size()-srfPos.degreeV()-1);
    }
    else if(stage==plotting) {
      if(plotFeature[openMBV] and openMBVNurbsSurface) {
        openMBVNurbsSurface->setName(name);

        openMBVNurbsSurface->setUKnotVector(srfPos.knotU());
        openMBVNurbsSurface->setVKnotVector(srfPos.knotV());
        openMBVNurbsSurface->setNumberOfUControlPoints(srfPos.ctrlPnts().rows());
        openMBVNurbsSurface->setNumberOfVControlPoints(srfPos.ctrlPnts().cols());

        parent->getOpenMBVGrp()->addObject(openMBVNurbsSurface);
      }
    }
    else if(stage==unknownStage) {
      srfVel.resize(index.rows(),index.cols(),uKnot.size()-index.rows()-1,vKnot.size()-index.cols()-1);
      srfVel.setDegreeU(uKnot.size()-index.rows()-1);
      srfVel.setDegreeV(vKnot.size()-index.cols()-1);
      srfVel.setKnotU(uKnot);
      srfVel.setKnotV(vKnot);
      srfJac.resize(gethSize());
      for(size_t i=0; i<srfJac.size(); i++) {
        srfJac[i].resize(index.rows(),index.cols(),uKnot.size()-index.rows()-1,vKnot.size()-index.cols()-1);
        srfJac[i].setDegreeU(uKnot.size()-index.rows()-1);
        srfJac[i].setDegreeV(vKnot.size()-index.cols()-1);
        srfJac[i].setKnotU(uKnot);
        srfJac[i].setKnotV(vKnot);
      }
      srfGA.resize(index.rows(),index.cols(),uKnot.size()-index.rows()-1,vKnot.size()-index.cols()-1);
      srfGA.setDegreeU(uKnot.size()-index.rows()-1);
      srfGA.setDegreeV(vKnot.size()-index.cols()-1);
      srfGA.setKnotU(uKnot);
      srfGA.setKnotV(vKnot);
    }
    FlexibleContour::init(stage, config);
  }

  ContourFrame* FlexibleSpatialNurbsContour::createContourFrame(const string &name) {
    FloatingContourFrame *frame = new FloatingContourFrame(name);
    frame->setContourOfReference(this);
    return frame;
  }

  double FlexibleSpatialNurbsContour::getCurvature(const Vec2 &zeta) {
    throwError("(FlexibleSpatialNurbsContour::getCurvature): not implemented");
  }

  void FlexibleSpatialNurbsContour::plot() {
    if(plotFeature[openMBV] and openMBVNurbsSurface) {
      if(updSrfPos) updateSurfacePositions();
      vector<double> data;
      data.push_back(getTime()); //time
      //Control-Point coordinates
      for(int j=0; j<srfPos.ctrlPnts().cols(); j++) {
        for(int i=0; i<srfPos.ctrlPnts().rows(); i++) {
          for(int k=0; k<4; k++)
            data.push_back(srfPos.ctrlPnts()(i,j)(k));
        }
      }
      openMBVNurbsSurface->append(data);
    }
    FlexibleContour::plot();
  }

  void FlexibleSpatialNurbsContour::initializeUsingXML(DOMElement * element) {
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
    index = E(e)->getText<MatVI>();
    for(int i=0; i<index.rows(); i++)
      for(int j=0; j<index.cols(); j++)
        index(i,j)--;
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"etaKnotVector");
    if(e) setEtaKnotVector(E(e)->getText<VecV>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"xiKnotVector");
    if(e) setXiKnotVector(E(e)->getText<VecV>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"etaDegree");
    if(e) setEtaDegree(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"xiDegree");
    if(e) setXiDegree(E(e)->getText<int>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"openEta");
    if(e) setOpenEta(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"openXi");
    if(e) setOpenXi(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIMFLEX%"enableOpenMBV");
    if(e) {
      openMBVNurbsSurface = OpenMBV::ObjectFactory::create<OpenMBV::DynamicNurbsSurface>();
//      OpenMBVNurbsSurface ombv;
//      ombv.initializeUsingXML(e);
//      openMBVRigidBody=ombv.createOpenMBV();
    }
  }

  bool FlexibleSpatialNurbsContour::isZetaOutside(const fmatvec::Vec2 &zeta) {
    if(openEta and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]))
      return true;
    if(openXi and (zeta(1) < xiNodes[0] or zeta(1) > xiNodes[xiNodes.size()-1]))
      return true;
    return false;
  }

}
