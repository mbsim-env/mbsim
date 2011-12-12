/* Copyright (C) 2004-2011 MBSim Development Team
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

#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_2s_13.h"
#include "mbsim/utils/eps.h"

#include <iostream>

#ifdef HAVE_NURBS
using namespace PLib;
#endif

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  NurbsDisk2s::~NurbsDisk2s() {
#ifdef HAVE_NURBS
    if(Surface) {delete Surface; Surface=0;}
    if(SurfaceVelocities) {delete SurfaceVelocities; SurfaceVelocities=0;}
    if(uvec) {delete uvec; uvec=0;}
    if(uVec) {delete uVec; uVec=0;}
    if(vvec) {delete vvec; vvec=0;}
    if(vVec) {delete vVec; vVec=0;}
#endif
  }

  void NurbsDisk2s::updateKinematicsForFrame(ContourPointData &cp, FrameFeature ff) {
#ifdef HAVE_NURBS
    if(nrm2(cp.getLagrangeParameterPosition()) < epsroot()) { // center of gravity
      cp.getFrameOfReference().setOrientation(R.getOrientation() * static_cast<FlexibleBody2s13*>(parent)->getA());

      switch(RefDofs) {
        case 2:
        cp.getFrameOfReference().setPosition(R.getPosition() + R.getOrientation() * Vec("[0;0;1]") * static_cast<FlexibleBody2s13*>(parent)->getq()(0));
        cp.getFrameOfReference().setVelocity( R.getOrientation() * Vec("[0;0;1]") * static_cast<FlexibleBody2s13*>(parent)->getu()(0));
        cp.getFrameOfReference().setAngularVelocity(R.getOrientation() * Vec("[0;0;1]") * static_cast<FlexibleBody2s13*>(parent)->getu()(1));
        break;
        case 6:
        cp.getFrameOfReference().setPosition(R.getPosition() + R.getOrientation() * static_cast<FlexibleBody2s13*>(parent)->getq()(0,2));
        cp.getFrameOfReference().setVelocity( R.getOrientation() * static_cast<FlexibleBody2s13*>(parent)->getu()(0,2));
        cp.getFrameOfReference().setAngularVelocity(R.getOrientation() * static_cast<FlexibleBody2s13*>(parent)->getA() * static_cast<FlexibleBody2s13*>(parent)->getG() * static_cast<FlexibleBody2s13*>(parent)->getu()(3,5));
        break;
        default:
        throw MBSimError("ERROR(NurbsDisk2s::updateKinematicsForFrame): Unknown number of reference dofs!");
      }
    }
    else { // somewhere else
      if(ff==position || ff==position_cosy || ff==all) {
        Point3Dd Tmppt = Surface->pointAt(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0));  // U-direction is azimuthal, V-direction is radial!
        cp.getFrameOfReference().getPosition()(0) = Tmppt.x();
        cp.getFrameOfReference().getPosition()(1) = Tmppt.y();
        cp.getFrameOfReference().getPosition()(2) = Tmppt.z();
      }

      // first tangent: radial-direction, second tangent:  azimuthal-direction
      if(ff==firstTangent || ff==secondTangent || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all)
      cp.getFrameOfReference().getOrientation()(0,1,2,2) = computeDirectionalDerivatives(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0),1);

      if(ff==normal || ff==cosy || ff==position_cosy || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        Point3Dd normal(Surface->normal(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0)));

        double normalLength = sqrt(normal.x()*normal.x() + normal.y()*normal.y() + normal.z()*normal.z());  // to normalize the vector

        normal *= -1;//normal should point out of the contour (as the normal is the crossproduct between the tangent in u und the tangent in v direction, the normal of nurbs++ points into the material)

        cp.getFrameOfReference().getOrientation().col(0)(0) = normal.x() /normalLength;
        cp.getFrameOfReference().getOrientation().col(0)(1) = normal.y() /normalLength;
        cp.getFrameOfReference().getOrientation().col(0)(2) = normal.z() /normalLength;
      }

      if(ff==velocity || ff==velocities || ff==velocity_cosy || ff==velocities_cosy || ff==all) {
        Point3Dd Tmpv = SurfaceVelocities->pointAt(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0));
        cp.getFrameOfReference().getVelocity()(0) = Tmpv.x();
        cp.getFrameOfReference().getVelocity()(1) = Tmpv.y();
        cp.getFrameOfReference().getVelocity()(2) = Tmpv.z();
      }

      // if(ff==angularVelocity || ff==velocities || ff==velocities_cosy || ff==all) {
      //   throw MBSimError("ERROR(FlexibleBody2s13::updateKinematicsForFrame::angularVelocity): Not implemented!");
      // }
    }
#endif
  }

  void NurbsDisk2s::updateJacobiansForFrame(ContourPointData &cp) {
#ifdef HAVE_NURBS
    cp.getFrameOfReference().getJacobianOfTranslation().resize(3,nj*nr*3+RefDofs);
    cp.getFrameOfReference().getJacobianOfRotation().resize(3,nj*nr*3+RefDofs);

    for(int k=0; k<nj*nr*3+RefDofs; k++) {
      Point3Dd TmpPtTrans = SurfaceJacobiansOfTranslation[k].pointAt(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0));
      Point3Dd TmpPtRot = SurfaceJacobiansOfRotation[k].pointAt(cp.getLagrangeParameterPosition()(1),cp.getLagrangeParameterPosition()(0));

      cp.getFrameOfReference().getJacobianOfTranslation().col(k)(0) = TmpPtTrans.x();
      cp.getFrameOfReference().getJacobianOfTranslation().col(k)(1) = TmpPtTrans.y();
      cp.getFrameOfReference().getJacobianOfTranslation().col(k)(2) = TmpPtTrans.z();

      cp.getFrameOfReference().getJacobianOfRotation().col(k)(0) = TmpPtRot.x();
      cp.getFrameOfReference().getJacobianOfRotation().col(k)(1) = TmpPtRot.y();
      cp.getFrameOfReference().getJacobianOfRotation().col(k)(2) = TmpPtRot.z();
    }

    /*TESTING*/
    //ofstream file("JacTrans.txt");
    //file << cp.getFrameOfReference().getJacobianOfTranslation();
    //file.close();
    //ofstream file2("JacRot.txt");
    //file2 << cp.getFrameOfReference().getJacobianOfRotation();
    //file2.close();
#endif
  }

#ifdef HAVE_NURBS
  void NurbsDisk2s::initContourFromBody(InitStage stage) {
    if(stage==resize) {
      degU = (static_cast<FlexibleBody2s13*>(parent))->getAzimuthalDegree();
      degV = (static_cast<FlexibleBody2s13*>(parent))->getRadialDegree();
      RefDofs = (static_cast<FlexibleBody2s13*>(parent))->getReferenceDegreesOfFreedom();
      nr = (static_cast<FlexibleBody2s13*>(parent))->getRadialNumberOfElements();
      nj = (static_cast<FlexibleBody2s13*>(parent))->getAzimuthalNumberOfElements();
      Ri = (static_cast<FlexibleBody2s13*>(parent))->getInnerRadius();
      Ra = (static_cast<FlexibleBody2s13*>(parent))->getOuterRadius();

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

      for(int k=0; k<nr*nj*3+RefDofs; k++) {
        SurfaceJacobiansOfTranslation.push_back(PlNurbsSurfaced());
        SurfaceJacobiansOfRotation.push_back(PlNurbsSurfaced());
      }

      computeSurface();
    }
    else if(stage==worldFrameContourLocation)
    {
      R.getOrientation() = (static_cast<FlexibleBody2s13*>(parent))->getFrameOfReference()->getOrientation();
      R.getPosition() = (static_cast<FlexibleBody2s13*>(parent))->getFrameOfReference()->getPosition();
    }
  }
#endif

  Vec NurbsDisk2s::transformCW(const Vec& WrPoint) {
    return (static_cast<FlexibleBody2s13*>(parent))->transformCW(WrPoint);
  }

#ifdef HAVE_NURBS
  Mat NurbsDisk2s::computeDirectionalDerivatives(const double &radius, const double &phi, const int &deg) {
    PLib::Matrix<Point3Dd> Derivates(deg+1,deg+1);  // matrix that contains the derivates of the surface

    Surface->deriveAt(phi, radius, deg, Derivates);// azimuthal-direction is U-direction, radial-direction is V-direction!

    //transform from Matrix of Nurbs to fmatvec Mat
    Mat ReturnMatrix(3,2);

    //no rule for the sign
    //tangent in u-direction (=azimuthal direction)
    ReturnMatrix(0,1) = Derivates(deg,0).x();
    ReturnMatrix(1,1) = Derivates(deg,0).y();
    ReturnMatrix(2,1) = Derivates(deg,0).z();
    //tangent in v-direction (=radial direction)
    ReturnMatrix(0,0) = Derivates(0,deg).x();
    ReturnMatrix(1,0) = Derivates(0,deg).y();
    ReturnMatrix(2,0) = Derivates(0,deg).z();

    //normalizes the tangent vectors
    ReturnMatrix(0,0,2,0) = 1/nrm2(ReturnMatrix(0,0,2,0)) * ReturnMatrix(0,0,2,0);
    ReturnMatrix(0,1,2,1) = 1/nrm2(ReturnMatrix(0,1,2,1)) * ReturnMatrix(0,1,2,1);

    return ReturnMatrix;
  }
#endif

#ifdef HAVE_NURBS
  Mat NurbsDisk2s::computeCurvatures(const double &radius, const double &phi) {
    return computeDirectionalDerivatives(radius, phi, 2);
  }
#endif

#ifdef HAVE_NURBS
  void NurbsDisk2s::computeUVector(const int NbPts) {
    uvec = new PLib::Vector<double>(NbPts);
    uVec = new PLib::Vector<double>(NbPts + degU+1);

    const double stepU = 2 * M_PI / nj;

    (*uvec)[0] = 0;
    for(int i=1;i<uvec->size();i++) {
      (*uvec)[i] = (*uvec)[i-1] + stepU;
    }

    (*uVec)[0] = (-degU) * stepU;
    for(int i= 1; i<uVec->size();i++) {
      (*uVec)[i] = (*uVec)[i-1] + stepU;
    }
  }
#endif

#ifdef HAVE_NURBS
  void NurbsDisk2s::computeVVector(const int NbPts) {
    vvec = new PLib::Vector<double>(NbPts);
    vVec = new PLib::Vector<double>(NbPts + degV+1);

    const double stepV = (Ra - Ri) / nr;

    (*vvec)[0] = Ri;
    for(int i=1;i<vvec->size();i++) {
      (*vvec)[i] = (*vvec)[i-1]+ stepV;
    }

    for(int i=degV+1; i<vVec->size()-(degV+1); i++) {
      (*vVec)[i] = (*vvec)[i-(degV-1)];
    }

    for(int i=0; i < degV+1; i++) { // the first and last (degV+1)-Values have to be equal
      (*vVec)[i] = Ri;
      (*vVec)[vVec->size()-i-1]= Ra;
    }
  }
#endif

#ifdef HAVE_NURBS 
  void NurbsDisk2s::computeSurface() {
    PLib::Matrix<HPoint3Dd> Nodelist(nj+degU,nr+1); // list of Cartesian node-coordinates for the nurbs interpolation (2*degU+1 is used for the 2 outline of the surface in azimuthal direction)

    // gets Points from body for interpolation
    for(int i=0; i<nr+1; i++) {
      for(int j=0; j<nj; j++) {
        ContourPointData cp(i*nj+j);
        static_cast<FlexibleBody2s13*>(parent)->updateKinematicsForFrame(cp,position);
        Nodelist(j,i) = HPoint3Dd(cp.getFrameOfReference().getPosition()(0),cp.getFrameOfReference().getPosition()(1),cp.getFrameOfReference().getPosition()(2),1);
      }
      for(int j=0;j<degU;j++) { //expands the surface addicted to the degree in azimuthal direction
        Nodelist(nj+j,i) = Nodelist(j,i);
      }
    }

    Surface->globalInterpClosedUH(Nodelist, *uVec, *vVec, *uvec, *vvec, degU, degV);

    /*TESTBLOCK...
    //testing the "real"values and the surface values at the nodes
    cout << "Surface-Berechnung:" << endl;
    double maxerr = 0;
    double minerr = 1;
    double stepU = 2*M_PI / nj;
    double stepV = ((*vvec)[vvec->size()-1] - (*vvec)[0]) / nr;
    for(int i=0; i<=nr; i++) {
      for(int j=0; j<nj; j++) {
        cout << "i = " << i << ", j = " << j << endl;
        Point3Dd PointUV = Surface->pointAt(j*stepU,(*vVec)[0]+i*stepV);
        cout << "Surface Auswertung = " << PointUV.x() << " "<< PointUV.y() << " " << PointUV.z() << endl;

        ContourPointData cp(i*nj+j);
        (static_cast<FlexibleBody2s13*>(parent))->updateKinematicsForFrame(cp,position);
        Vec InterpPoint = cp.getFrameOfReference().getPosition();
        cout << "Interpolationspunkt = " << InterpPoint(0) << " " << InterpPoint(1) << " " << InterpPoint(2) << endl << endl;

        Vec error(3);
        error(0) = PointUV.x()-InterpPoint(0);
        error(1) = PointUV.y()-InterpPoint(1);
        error(2) = PointUV.z()-InterpPoint(2);
        cout << "Fehler = " << nrm2(error) << endl;
        if(maxerr < nrm2(error)) maxerr = nrm2(error);
        if(minerr > nrm2(error)) minerr = nrm2(error);
      }
    }
    cout << "Maximaler Fehler ist: " << maxerr << endl;
    cout << "Minimaler Fehler ist: " << minerr << endl;
    cout << " ***************************************************** " << endl << endl;

    //testing derivations
    const int degDev = 10;
    Vec TestVec[degDev][2][2];//degDev is for degree of derivation, 4 is for the number of knots, 2 is for the direction of the derviation (U or V)
    int k = 0;
    int l = 0;

    cout << "TEST der Ableitungen:" << endl;

    Vec test_pt(3);
    test_pt(0) = ((*vvec)[vvec->size()-1]-(*vvec)[0])/2;
    test_pt(1) = 0;
    cout << "Punkt ist bei phi=0" << endl;
    for(int i=1;i<=degDev;i++) {
      TestVec[k][l][0] = computeDirectionalDerivatives(test_pt(0),test_pt(1),i).col(0);
      TestVec[k++][l][1] = computeDirectionalDerivatives(test_pt(0),test_pt(1),i).col(1);
    }
    l++; k =0;

    test_pt(1) = 2*M_PI;
    cout << "Punkt ist bei phi=2*PI" << endl;
    for(int i=1;i<=degDev;i++) {
      TestVec[k][l][0] = computeDirectionalDerivatives(test_pt(0),test_pt(1),i).col(0);
      TestVec[k++][l][1] = computeDirectionalDerivatives(test_pt(0),test_pt(1),i).col(1);
    }

    for(int i=0;i<degDev;i++) {
      cout << i+1 << "-te Ableitung ..." << endl;
      for(int j=0;j<2;j++) {
        cout << "j=" << j << "(j=0...V-Richtung, j=1...U-Richtung)" << endl;
        cout << "Fehler der Punkte am Rande: " << computeError(TestVec[i][0][j],TestVec[i][1][j]) << endl;
      }
    }
    */

  }
#endif

#ifdef HAVE_NURBS
  void NurbsDisk2s::computeSurfaceVelocities() {
    PLib::Matrix<HPoint3Dd> Nodelist(nj+degU,nr+1); // list of Cartesian node-velocities for the nurbs interpolation (2*degU+1 is used for the 2 outline of the surface in azimuthal direction)

    // gets velocities from body for the interpolation
    for(int i=0; i<nr+1; i++) {
      for(int j=0; j<nj; j++) {
        ContourPointData cp(i*nj+j);
        static_cast<FlexibleBody2s13*>(parent)->updateKinematicsForFrame(cp,velocity);
        Nodelist(j,i) = HPoint3Dd(cp.getFrameOfReference().getVelocity()(0),cp.getFrameOfReference().getVelocity()(1),cp.getFrameOfReference().getVelocity()(2), 1);
      }
      for(int j=0;j<degU;j++) { // expands the surface addicted to the degree in azimuthal direction
        Nodelist(nj+j,i) = Nodelist(j,i);
      }
    }

    SurfaceVelocities->globalInterpClosedUH(Nodelist, *uVec, *vVec, *uvec, *vvec, degU, degV);
  }
#endif

#ifdef HAVE_NURBS
  void NurbsDisk2s::computeSurfaceJacobians() {
    PLib::Matrix<HPoint3Dd> NodelistTrans(nj+degU,nr+1); // list of node-data for the nurbs interpolation
    PLib::Matrix<HPoint3Dd> NodelistRot(nj+degU,nr+1);// list of node-data for the nurbs interpolation

    // gets Jacobians on the nodes from body for interpolation
    for(int i=0; i<nr+1; i++) {
      for(int j=0; j<nj; j++) {
        static_cast<FlexibleBody2s13*>(parent)->updateJacobiansForFrame(jacobians[i*nj+j]);
      }
    }
    for(int k=0; k<nr*nj*3+RefDofs; k++) {
      for(int i=0; i<nr+1; i++) {
        for(int j=0; j<nj; j++) {
          NodelistTrans(j,i) = HPoint3Dd(jacobians[i*nj+j].getFrameOfReference().getJacobianOfTranslation()(0,k),
              jacobians[i*nj+j].getFrameOfReference().getJacobianOfTranslation()(1,k),
              jacobians[i*nj+j].getFrameOfReference().getJacobianOfTranslation()(2,k),
              1);

          NodelistRot(j,i) = HPoint3Dd(jacobians[i*nj+j].getFrameOfReference().getJacobianOfRotation()(0,k),
              jacobians[i*nj+j].getFrameOfReference().getJacobianOfRotation()(1,k),
              jacobians[i*nj+j].getFrameOfReference().getJacobianOfRotation()(2,k),
              1);
        }
        for(int j=0;j<degU;j++) { // expands the surface addicted to the degree in azimuthal direction
          NodelistTrans(nj+j,i) = NodelistTrans(j,i);
          NodelistRot(nj+j,i) = NodelistRot(j,i);
        }
      }

      SurfaceJacobiansOfTranslation[k].globalInterpClosedUH(NodelistTrans, *uVec, *vVec, *uvec, *vvec, degU, degV);
      SurfaceJacobiansOfRotation[k].globalInterpClosedUH(NodelistRot, *uVec, *vVec, *uvec, *vvec, degU, degV);
    }
  }
#endif

#ifdef HAVE_NURBS 
  Vec NurbsDisk2s::getControlPoints(const int u, const int v) {
    Vec TmpVec(3);

    TmpVec(0) = Surface->ctrlPnts(u,v).x();
    TmpVec(1) = Surface->ctrlPnts(u,v).y();
    TmpVec(2) = Surface->ctrlPnts(u,v).z();

    return TmpVec;
  }
#endif

#ifdef HAVE_NURBS 
  Vec NurbsDisk2s::getUVector() {
    Vec TmpUVec(Surface->knotU().rows());

    for (int i=0;i<Surface->knotU().rows();i++)
    TmpUVec(i)=Surface->knotU(i);

    return TmpUVec;
  }
#endif

#ifdef HAVE_NURBS 
  Vec NurbsDisk2s::getVVector() {
    Vec TmpVVec(Surface->knotV().rows());

    for (int i=0;i<Surface->knotV().rows();i++)
    TmpVVec(i)=Surface->knotV(i);

    return TmpVVec;
  }
#endif

  int NurbsDisk2s::testInsideBounds(const Vec &s) {
    if ((s(0) < Ri) || (s(0) > Ra))
      return 0;

    if ((s(1) < 0) || (s(1) > 2 * M_PI))
      return 0;

    return 1;
  }

  double NurbsDisk2s::computeError(const Vec &Vec1, const Vec &Vec2) {
    return nrm2(Vec1 - Vec2);
  }
}

