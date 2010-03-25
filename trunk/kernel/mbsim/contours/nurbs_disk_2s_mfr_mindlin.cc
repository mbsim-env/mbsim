/* Copyright (C) 2004-2010 MBSim Development Team
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
 * Contact: thschindler@users.berlios.de
 */

#include<config.h>

#include "mbsim/contours/nurbs_disk_2s_mfr_mindlin.h"
#include "mbsim/flexible_body/flexible_body_2s_13_mfr_mindlin.h"
#include "mbsim/utils/eps.h"

#include <iostream>

#ifdef HAVE_NURBS
using namespace PLib;
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  void NurbsDisk2sMFRMindlin::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {//TODO
#ifdef HAVE_NURBS
    if(nrm2(cp.getLagrangeParameterPosition()) < epsroot()) { // center of gravity
      cp.getFrameOfReference().setOrientation(R.getOrientation() * static_cast<FlexibleBody2s13MFRMindlin*>(parent)->getA());

      cp.getFrameOfReference().setPosition(R.getPosition() + R.getOrientation() * static_cast<FlexibleBody2s13MFRMindlin*>(parent)->getq()(0,2));
      cp.getFrameOfReference().setVelocity(                  R.getOrientation() * static_cast<FlexibleBody2s13MFRMindlin*>(parent)->getu()(0,2));

      Vec diskAngularVelocity =  static_cast<FlexibleBody2s13MFRMindlin*>(parent)->getG() * static_cast<FlexibleBody2s13MFRMindlin*>(parent)->getu()(3,5);
      cp.getFrameOfReference().setAngularVelocity(R.getOrientation() *  diskAngularVelocity ); // z-rotation
    }
    else { // somewhere else
      if(ff==position || ff==position_cosy || ff==all) {
        Point3Dd Tmppt = Surface->pointAt(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0));  // U-direction is azimuthal, V-direction is radial!
        cp.getFrameOfReference().getPosition()(0) = Tmppt.x();
        cp.getFrameOfReference().getPosition()(1) = Tmppt.y();
        cp.getFrameOfReference().getPosition()(2) = Tmppt.z();
      }

      // first tangent: radial-direction, second tangent:  azimuthal-direction
      if(ff==firstTangent ||  ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all)
        cp.getFrameOfReference().getOrientation()(0,1,2,2) =  computeDirectionalDerivatives(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0),1);

      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        Point3Dd normal(Surface->normal(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0)));

        double normalLength = sqrt(normal.x()*normal.x() + normal.y()*normal.y() + normal.z()*normal.z());  // to normalize the vector

        normal *= -1; //normal should point out of the contour (as the normal is the crossproduct between the tangent in u und the tangent in v direction, the normal of nurbs++ points into the material)

        cp.getFrameOfReference().getOrientation().col(0)(0) =  normal.x() /normalLength;
        cp.getFrameOfReference().getOrientation().col(0)(1) =  normal.y() /normalLength;
        cp.getFrameOfReference().getOrientation().col(0)(2) =  normal.z() /normalLength;
      }

      if(ff==velocity || ff==velocities || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        Point3Dd Tmpv = SurfaceVelocities->pointAt(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0));
        cp.getFrameOfReference().getVelocity()(0) = Tmpv.x();
        cp.getFrameOfReference().getVelocity()(1) = Tmpv.y();
        cp.getFrameOfReference().getVelocity()(2) = Tmpv.z();
      }

      // if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
      //   throw new MBSimError("ERROR(FlexibleBody2s13MFRMindlin::updateKinematicsForFrame::angularVelocity): Not implemented!");
      // }
    }
#endif
  }

#ifdef HAVE_NURBS
  void NurbsDisk2sMFRMindlin::initContourFromBody(InitStage stage){
    if(stage==resize){
      degU = (static_cast<FlexibleBody2s13MFRMindlin*>(parent))->getAzimuthalDegree();
      degV = (static_cast<FlexibleBody2s13MFRMindlin*>(parent))->getRadialDegree();
      nr = (static_cast<FlexibleBody2s13MFRMindlin*>(parent))->getRadialNumberOfElements();
      nj = (static_cast<FlexibleBody2s13MFRMindlin*>(parent))->getAzimuthalNumberOfElements();
      Ri = (static_cast<FlexibleBody2s13MFRMindlin*>(parent))->getInnerRadius();
      Ra = (static_cast<FlexibleBody2s13MFRMindlin*>(parent))->getOuterRadius();

      computeUVector(nj+degU);
      computeVVector(nr+1);

      Surface = new PlNurbsSurfaced;
      SurfaceVelocities = new PlNurbsSurfaced;
      for(int i=0; i<nr+1; i++) {
        for(int j=0; j<nj; j++) {
          jacobians.push_back(ContourPointData(i*nj+j));
          jacobians[jacobians.size()-1].getFrameOfReference().getJacobianOfTranslation().resize();
          jacobians[jacobians.size()-1].getFrameOfReference().getJacobianOfRotation().resize();
        }
      }

      for(int k=0; k<nr*nj*3+6; k++) SurfaceJacobiansOfTranslation.push_back(PlNurbsSurfaced());

      computeSurface();
    }
    else if(stage==worldFrameContourLocation)
    {
      R.getOrientation() = (static_cast<FlexibleBody2s13MFRMindlin*>(parent))->getFrameOfReference()->getOrientation();
      R.getPosition()    = (static_cast<FlexibleBody2s13MFRMindlin*>(parent))->getFrameOfReference()->getPosition();
    }
  }
#endif

  void NurbsDisk2sMFRMindlin::updateJacobiansForFrame(ContourPointData &cp) {
#ifdef HAVE_NURBS
    cp.getFrameOfReference().getJacobianOfTranslation().resize(3,nj*nr*3+6);

    for(int k=0; k<nj*nr*3+6; k++) {
      Point3Dd TmpPt = SurfaceJacobiansOfTranslation[k].pointAt(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0));

      cp.getFrameOfReference().getJacobianOfTranslation().col(k)(0) = TmpPt.x();
      cp.getFrameOfReference().getJacobianOfTranslation().col(k)(1) = TmpPt.y();
      cp.getFrameOfReference().getJacobianOfTranslation().col(k)(2) = TmpPt.z();
    }
#endif
  }

  Vec NurbsDisk2sMFRMindlin::transformCW(const Vec& WrPoint) {
    return (static_cast<FlexibleBody2s13MFRMindlin*>(parent))->transformCW(WrPoint);
  }

#ifdef HAVE_NURBS
  void NurbsDisk2sMFRMindlin::computeSurface() {
    PLib::Matrix<Point3Dd> Nodelist(nj+degU,nr+1); // list of Cartesian node-coordinates for the nurbs interpolation (2*degU+1 is used for the 2 outline of the surface in azimuthal direction)

    // gets Points from body for interpolation
    for(int i=0; i<nr+1; i++) {
      for(int j=0; j<nj; j++) {
        ContourPointData cp(i*nj+j);
        static_cast<FlexibleBody2s13MFRMindlin*>(parent)->updateKinematicsForFrame(cp,position);
        Nodelist(j,i) = Point3Dd(cp.getFrameOfReference().getPosition()(0),cp.getFrameOfReference().getPosition()(1),cp.getFrameOfReference().getPosition()(2));
      }
      for(int j=0;j<degU;j++) { //expands the surface addicted to the degree in azimuthal direction
        Nodelist(nj+j,i) = Nodelist(j,i);
      }
    }

    Surface->globalInterpClosedU_OwnKnotVecs(Nodelist, *uVec, *vVec, *uvec, *vvec, degU, degV);
  }
#endif

#ifdef HAVE_NURBS
  void NurbsDisk2sMFRMindlin::computeSurfaceVelocities() {
    PLib::Matrix<Point3Dd> Nodelist(nj+degU,nr+1); // list of Cartesian node-velocities for the nurbs interpolation (2*degU+1 is used for the 2 outline of the surface in azimuthal direction)

    // gets velocities from body for the interpolation
    for(int i=0; i<nr+1; i++) {
      for(int j=0; j<nj; j++) {
        ContourPointData cp(i*nj+j);
        static_cast<FlexibleBody2s13MFRMindlin*>(parent)->updateKinematicsForFrame(cp,velocity);
        Nodelist(j,i) = Point3Dd(cp.getFrameOfReference().getVelocity()(0),cp.getFrameOfReference().getVelocity()(1),cp.getFrameOfReference().getVelocity()(2));
      }
      for(int j=0;j<degU;j++) { // expands the surface addicted to the degree in azimuthal direction
        Nodelist(nj+j,i) = Nodelist(j,i);
      }
    }

    SurfaceVelocities->globalInterpClosedU_OwnKnotVecs(Nodelist, *uVec, *vVec, *uvec, *vvec, degU, degV);
  }
#endif

#ifdef HAVE_NURBS
  void NurbsDisk2sMFRMindlin::computeSurfaceJacobiansOfTranslation() {
    PLib::Matrix<Point3Dd> Nodelist(nj+degU,nr+1); // list of node-data for the nurbs interpolation

    // gets Jacobians on the nodes from body for interpolation
    for(int i=0; i<nr+1; i++) {
      for(int j=0; j<nj; j++) {
        static_cast<FlexibleBody2s13MFRMindlin*>(parent)->updateJacobiansForFrame(jacobians[i*nj+j]);
      }
    }
    for(int k=0; k<nr*nj*3+6; k++) {
      for(int i=0; i<nr+1; i++) {
        for(int j=0; j<nj; j++) {
          Nodelist(j,i) = Point3Dd(jacobians[i*nj+j].getFrameOfReference().getJacobianOfTranslation()(0,k),
              jacobians[i*nj+j].getFrameOfReference().getJacobianOfTranslation()(1,k),
              jacobians[i*nj+j].getFrameOfReference().getJacobianOfTranslation()(2,k));
        }
        for(int j=0;j<degU;j++) { // expands the surface addicted to the degree in azimuthal direction
          Nodelist(nj+j,i) = Nodelist(j,i);
        }
      }

      SurfaceJacobiansOfTranslation[k].globalInterpClosedU_OwnKnotVecs(Nodelist, *uVec, *vVec, *uvec, *vvec, degU, degV);
    }
  }
#endif

}

