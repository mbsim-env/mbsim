/* Copyright (C) 2004-2012 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include<config.h>

#include "mbsimFlexibleBody/contours/nurbs_curve_1s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_1s_33_cosserat.h"

#ifdef HAVE_NURBS
using namespace PLib;
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  NurbsCurve1s::NurbsCurve1s(const std::string &name) : MBSim::Contour1s(name), Elements(0), openStructure(false), L(0.), degU(0) {
#ifndef HAVE_NURBS
    throw MBSim::MBSimError("ERROR(NurbsCurve1s::NurbsCurve1s): External NURBS library not implemented!");
#endif
  }

  NurbsCurve1s::~NurbsCurve1s() {
#ifdef HAVE_NURBS
    if(curveTranslations) {delete curveTranslations; curveTranslations=NULL;}
    if(curveVelocities) {delete curveVelocities; curveVelocities=NULL;}
    if(uvec) {delete uvec; uvec=NULL;}
    if(uVec) {delete uVec; uVec=NULL;}
#endif
  }

  void NurbsCurve1s::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
#ifdef HAVE_NURBS
    if(ff==position || ff==position_cosy || ff==all) {
      Point3Dd Tmppt = curveTranslations->pointAt(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getPosition()(0) = Tmppt.x();
      cp.getFrameOfReference().getPosition()(1) = Tmppt.y();
      cp.getFrameOfReference().getPosition()(2) = Tmppt.z();
    }

    if(ff==normal || ff==firstTangent || ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
      Point3Dd tangent = curveTranslations->derive3D(cp.getLagrangeParameterPosition()(0), 1);
      tangent /= norm(tangent);
      Point3Dd normal = curveTranslations->derive3D(cp.getLagrangeParameterPosition()(0), 2);
      normal /= norm(normal);
      if(dot(normalRotationGrid,normal)<0) // orientate derived normal from NURBS curve to direction of closest rotation matrix of rotation grid
        normal *= -1.;
      Point3Dd binormal = crossProduct(normal, tangent);
      normal = crossProduct(tangent, binormal); // calculate normal again from cross product as second derivative is not normal to tangent
      cp.getFrameOfReference().getOrientation()(0,0) = normal.x();
      cp.getFrameOfReference().getOrientation()(1,0) = normal.y();
      cp.getFrameOfReference().getOrientation()(2,0) = normal.z();

      cp.getFrameOfReference().getOrientation()(0,1) = tangent.x();
      cp.getFrameOfReference().getOrientation()(1,1) = tangent.y();
      cp.getFrameOfReference().getOrientation()(2,1) = tangent.z();

      cp.getFrameOfReference().getOrientation()(0,2) = binormal.x();
      cp.getFrameOfReference().getOrientation()(1,2) = binormal.y();
      cp.getFrameOfReference().getOrientation()(2,2) = binormal.z();
    }

    if(ff==velocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) {
      Point3Dd Tmpv = curveVelocities->pointAt(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getVelocity()(0) = Tmpv.x();
      cp.getFrameOfReference().getVelocity()(1) = Tmpv.y();
      cp.getFrameOfReference().getVelocity()(2) = Tmpv.z();
    }

    if(ff==angularVelocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) {
      double uStaggered = cp.getLagrangeParameterPosition()(0); // interpolation of angular velocity starts from 0 --> \phi_{1/2} but contour starts from 0 --> r_0 therefore this difference of l0/2
      double l0 = L/Elements;
      if (uStaggered < l0/2.)
        uStaggered = L - l0/2. + uStaggered;
      else
        uStaggered -= l0/2.;
      Point3Dd Tmpav = curveAngularVelocities->pointAt(uStaggered);
      cp.getFrameOfReference().getAngularAcceleration()(0) = Tmpav.x();
      cp.getFrameOfReference().getAngularAcceleration()(1) = Tmpav.y();
      cp.getFrameOfReference().getAngularAcceleration()(2) = Tmpav.z();
    }
#endif
  }

  void NurbsCurve1s::updateJacobiansForFrame(ContourPointData &cp, int j /*=0*/) {
#ifdef HAVE_NURBS
    cp.getFrameOfReference().getJacobianOfTranslation().resize(qSize);
    cp.getFrameOfReference().getJacobianOfRotation().resize(qSize); // TODO open structure

    double uStaggered = cp.getLagrangeParameterPosition()(0); // interpolation of Jacobian of Rotation starts from 0 --> \phi_{1/2} but Jacobian of Translation and contour starts from 0 --> r_0 therefore this difference of l0/2
    double l0 = L/Elements;
    if (uStaggered < l0/2.)
      uStaggered = L - l0/2. + uStaggered;
    else
      uStaggered -= l0/2.;

    for(int k=0; k<qSize; k++) {
      Point3Dd TmpPtTrans = CurveJacobiansOfTranslation[k].pointAt(cp.getLagrangeParameterPosition()(0));
      Point3Dd TmpPtRot = CurveJacobiansOfRotation[k].pointAt(uStaggered);

      cp.getFrameOfReference().getJacobianOfTranslation()(0,k) = TmpPtTrans.x();
      cp.getFrameOfReference().getJacobianOfTranslation()(1,k) = TmpPtTrans.y();
      cp.getFrameOfReference().getJacobianOfTranslation()(2,k) = TmpPtTrans.z();

      cp.getFrameOfReference().getJacobianOfRotation()(0,k) = TmpPtRot.x();
      cp.getFrameOfReference().getJacobianOfRotation()(1,k) = TmpPtRot.y();
      cp.getFrameOfReference().getJacobianOfRotation()(2,k) = TmpPtRot.z();
    }
#endif
  }

#ifdef HAVE_NURBS
  void NurbsCurve1s::initContourFromBody(InitStage stage) {
    if(stage==resize) {
      degU = 3;
      Elements = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getNumberElements();
      qSize = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getNumberDOFs();
      openStructure = (static_cast<FlexibleBody1s33Cosserat*>(parent))->isOpenStructure();
      L = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getLength();

      if(openStructure) computeUVector(Elements+1);
      else computeUVector(Elements+degU);

      curveTranslations = new PlNurbsCurved;
      curveVelocities = new PlNurbsCurved;
      curveAngularVelocities = new PlNurbsCurved;
      for(int i=0; i<Elements; i++) { // TODO openstructure: jacobians of Rotation different
        jacobiansTrans.push_back(ContourPointData(i));
        jacobiansRot.push_back(ContourPointData(i, STAGGEREDNODE)); // jacobians of rotation are on staggered grid
      }
      for(int i=0; i<Elements; i++) { 
        jacobiansTrans[i].getFrameOfReference().getJacobianOfTranslation().resize();
        jacobiansRot[i].getFrameOfReference().getJacobianOfRotation().resize();
      }

      for(int k=0; k<qSize; k++) { // TODO openstructure: jacobians of Rotation different
        CurveJacobiansOfTranslation.push_back(PlNurbsCurved());
        CurveJacobiansOfRotation.push_back(PlNurbsCurved());
      }

      computeCurveTranslations();
      computeCurveVelocities();
      computeCurveAngularVelocities();
    }
    else if(stage==worldFrameContourLocation) {
      R.getOrientation() = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getFrameOfReference()->getOrientation();
      R.getPosition() = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getFrameOfReference()->getPosition();
    }
  }
#endif

#ifdef HAVE_NURBS
  void NurbsCurve1s::computeUVector(const int NbPts) {
    uvec = new PLib::Vector<double>(NbPts);
    uVec = new PLib::Vector<double>(NbPts + degU+1);

    const double stepU = L / Elements;

    (*uvec)[0] = 0;
    for(int i=1;i<uvec->size();i++) {
      (*uvec)[i] = (*uvec)[i-1] + stepU;
    }

    if(openStructure) {
      for(int i=degU+1; i<uVec->size()-(degU+1); i++) {
        (*uVec)[i] = (*uvec)[i-(degU-1)];
      }
      for(int i=0; i<degU+1; i++) {
        (*uVec)[i] = 0.;
        (*uVec)[uVec->size()-i-1]= L;
      }
    }
    else {
      (*uVec)[0] = (-degU) * stepU;
      for(int i=1; i<uVec->size();i++) {
        (*uVec)[i] = (*uVec)[i-1] + stepU;
      }
    }
  }
#endif

#ifdef HAVE_NURBS 
  void NurbsCurve1s::computeCurveTranslations() {
    if(openStructure) {
      PLib::Vector<HPoint3Dd> Nodelist(Elements+1);
      for(int i=0; i<Elements+1; i++) {
        ContourPointData cp(i);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateKinematicsForFrame(cp,position);
        Nodelist[i] = HPoint3Dd(cp.getFrameOfReference().getPosition()(0),cp.getFrameOfReference().getPosition()(1),cp.getFrameOfReference().getPosition()(2),1);
      }
      curveTranslations->globalInterpH(Nodelist, *uvec, *uVec, degU);
    }
    else {
      PLib::Vector<HPoint3Dd> Nodelist(Elements+degU);
      for(int i=0; i<Elements; i++) {
        ContourPointData cp(i);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateKinematicsForFrame(cp,position);
        Nodelist[i] = HPoint3Dd(cp.getFrameOfReference().getPosition()(0),cp.getFrameOfReference().getPosition()(1),cp.getFrameOfReference().getPosition()(2),1);
      }
      for(int i=0;i<degU;i++) {
        Nodelist[Elements+i] = Nodelist[i];
      }
      curveTranslations->globalInterpClosedH(Nodelist, *uvec, *uVec, degU);
    }
  }
#endif

#ifdef HAVE_NURBS
  void NurbsCurve1s::computeCurveVelocities() {
    if(openStructure) {
      PLib::Vector<HPoint3Dd> Nodelist(Elements+1);
      for(int i=0; i<Elements+1; i++) {
        ContourPointData cp(i);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateKinematicsForFrame(cp,velocity);
        Nodelist[i] = HPoint3Dd(cp.getFrameOfReference().getVelocity()(0),cp.getFrameOfReference().getVelocity()(1),cp.getFrameOfReference().getVelocity()(2),1);
      }
      curveVelocities->globalInterpH(Nodelist, *uvec, *uVec, degU);
    }
    else {
      PLib::Vector<HPoint3Dd> Nodelist(Elements+degU);
      for(int i=0; i<Elements; i++) {
        ContourPointData cp(i);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateKinematicsForFrame(cp,velocity);
        Nodelist[i] = HPoint3Dd(cp.getFrameOfReference().getVelocity()(0),cp.getFrameOfReference().getVelocity()(1),cp.getFrameOfReference().getVelocity()(2),1);
      }
      for(int i=0;i<degU;i++) {
        Nodelist[Elements+i] = Nodelist[i];
      }
      curveVelocities->globalInterpClosedH(Nodelist, *uvec, *uVec, degU);
    }
  }
#endif

#ifdef HAVE_NURBS
  void NurbsCurve1s::computeCurveAngularVelocities() {
    if(openStructure) {
      PLib::Vector<HPoint3Dd> Nodelist(Elements+1);
      for(int i=0; i<Elements+1; i++) {
        ContourPointData cp(i);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateKinematicsForFrame(cp,angularVelocity);
        Nodelist[i] = HPoint3Dd(cp.getFrameOfReference().getAngularVelocity()(0),cp.getFrameOfReference().getAngularVelocity()(1),cp.getFrameOfReference().getAngularVelocity()(2),1);
      }
      curveAngularVelocities->globalInterpH(Nodelist, *uvec, *uVec, degU);
    }
    else {
      PLib::Vector<HPoint3Dd> Nodelist(Elements+degU);
      for(int i=0; i<Elements; i++) {
        ContourPointData cp(i);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateKinematicsForFrame(cp,angularVelocity);
        Nodelist[i] = HPoint3Dd(cp.getFrameOfReference().getAngularVelocity()(0),cp.getFrameOfReference().getAngularVelocity()(1),cp.getFrameOfReference().getAngularVelocity()(2),1);
      }
      for(int i=0;i<degU;i++) {
        Nodelist[Elements+i] = Nodelist[i];
      }
      curveAngularVelocities->globalInterpClosedH(Nodelist, *uvec, *uVec, degU);
    }
  }
#endif

#ifdef HAVE_NURBS
  void NurbsCurve1s::computeCurveJacobians() {
    if(openStructure) { } // TODO
    else {
      PLib::Vector<HPoint3Dd> NodelistTrans(Elements+degU);
      PLib::Vector<HPoint3Dd> NodelistRot(Elements+degU);

      for(int i=0; i<Elements; i++) {
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateJacobiansForFrame(jacobiansTrans[i]);
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateJacobiansForFrame(jacobiansRot[i]); // jacobians of rotation are on staggered grid
      }
      for(int k=0; k<qSize; k++) {
        for(int i=0; i<Elements; i++) {
          NodelistTrans[i] = HPoint3Dd(jacobiansTrans[i].getFrameOfReference().getJacobianOfTranslation()(0,k),
              jacobiansTrans[i].getFrameOfReference().getJacobianOfTranslation()(1,k),
              jacobiansTrans[i].getFrameOfReference().getJacobianOfTranslation()(2,k),
              1);

          NodelistRot[i] = HPoint3Dd(jacobiansRot[i].getFrameOfReference().getJacobianOfRotation()(0,k),
              jacobiansRot[i].getFrameOfReference().getJacobianOfRotation()(1,k),
              jacobiansRot[i].getFrameOfReference().getJacobianOfRotation()(2,k),
              1);
        }
        for(int i=0;i<degU;i++) {
          NodelistTrans[Elements+i] = NodelistTrans[i];
          NodelistRot[Elements+i] = NodelistRot[i];
        }

        CurveJacobiansOfTranslation[k].globalInterpClosedH(NodelistTrans, *uvec, *uVec, degU);
        CurveJacobiansOfRotation[k].globalInterpClosedH(NodelistRot, *uvec, *uVec, degU);
      }
    }
  }
#endif

}

