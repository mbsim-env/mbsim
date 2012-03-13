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
      Point3Dd normal = curveTranslations->derive3D(cp.getLagrangeParameterPosition()(0), 2); // TODO check direction
      normal /= norm(normal);
      Point3Dd binormal = crossProduct(normal, tangent);
      normal = crossProduct(tangent, binormal); // calculate normal again from cross product as second derivative is not normal to tangent
      cp.getFrameOfReference().getOrientation().col(0)(0) = normal.x();
      cp.getFrameOfReference().getOrientation().col(0)(1) = normal.y();
      cp.getFrameOfReference().getOrientation().col(0)(2) = normal.z();

      cp.getFrameOfReference().getOrientation().col(1)(0) = tangent.x();
      cp.getFrameOfReference().getOrientation().col(1)(1) = tangent.y();
      cp.getFrameOfReference().getOrientation().col(1)(2) = tangent.z();

      cp.getFrameOfReference().getOrientation().col(2)(0) = binormal.x();
      cp.getFrameOfReference().getOrientation().col(2)(1) = binormal.y();
      cp.getFrameOfReference().getOrientation().col(2)(2) = binormal.z();
    }

    if(ff==velocity || ff==velocity_cosy || ff==velocities || ff==velocities_cosy || ff==all) {
      Point3Dd Tmpv = curveVelocities->pointAt(cp.getLagrangeParameterPosition()(0));
      cp.getFrameOfReference().getVelocity()(0) = Tmpv.x();
      cp.getFrameOfReference().getVelocity()(1) = Tmpv.y();
      cp.getFrameOfReference().getVelocity()(2) = Tmpv.z();
    }
#endif
  }

  void NurbsCurve1s::updateJacobiansForFrame(ContourPointData &cp, int j /*=0*/) {
#ifdef HAVE_NURBS
    cp.getFrameOfReference().getJacobianOfTranslation().resize(3,Elements*3);

    for(int k=0; k<Elements*3; k++) {
      Point3Dd TmpPtTrans = CurveJacobiansOfTranslation[k].pointAt(cp.getLagrangeParameterPosition()(0));

      cp.getFrameOfReference().getJacobianOfTranslation().col(k)(0) = TmpPtTrans.x();
      cp.getFrameOfReference().getJacobianOfTranslation().col(k)(1) = TmpPtTrans.y();
      cp.getFrameOfReference().getJacobianOfTranslation().col(k)(2) = TmpPtTrans.z();
    }
#endif
  }

#ifdef HAVE_NURBS
  void NurbsCurve1s::initContourFromBody(InitStage stage) {
    if(stage==resize) {
      degU = 3;
      Elements = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getNumberElements();
      openStructure = (static_cast<FlexibleBody1s33Cosserat*>(parent))->isOpenStructure();
      L = (static_cast<FlexibleBody1s33Cosserat*>(parent))->getLength();

      if(openStructure) computeUVector(Elements+1);
      else computeUVector(Elements+degU);

      curveTranslations = new PlNurbsCurved;
      curveVelocities = new PlNurbsCurved;
      for(int i=0; i<Elements; i++) {
        jacobians.push_back(ContourPointData(i));
        jacobians[jacobians.size()-1].getFrameOfReference().getJacobianOfTranslation().resize();
      }

      for(int k=0; k<Elements*3; k++) {
        CurveJacobiansOfTranslation.push_back(PlNurbsCurved());
      }

      computeCurveTranslations();
    }
    else if(stage==worldFrameContourLocation)
    {
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
  void NurbsCurve1s::computeCurveJacobians() {
    if(openStructure) { } // TODO
    else {
      PLib::Vector<HPoint3Dd> NodelistTrans(Elements+degU);

      for(int i=0; i<Elements; i++) {
        static_cast<FlexibleBody1s33Cosserat*>(parent)->updateJacobiansForFrame(jacobians[i]);
      }
      for(int k=0; k<Elements*3; k++) {
        for(int i=0; i<Elements; i++) {
          NodelistTrans[i] = HPoint3Dd(jacobians[i].getFrameOfReference().getJacobianOfTranslation()(0,k),
              jacobians[i].getFrameOfReference().getJacobianOfTranslation()(1,k),
              jacobians[i].getFrameOfReference().getJacobianOfTranslation()(2,k),
              1);
        }
        for(int i=0;i<degU;i++) {
          NodelistTrans[Elements+i] = NodelistTrans[i];
        }

        CurveJacobiansOfTranslation[k].globalInterpClosedH(NodelistTrans, *uvec, *uVec, degU);
      }
    }
  }
#endif

}

