/* Copyright (C) 2004-2015 MBSim Development Team
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

#include <config.h>

#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"
#include "mbsimFlexibleBody/flexible_body/flexible_body_2s_13.h"
#include "mbsimFlexibleBody/frames/node_frame.h"
#include "mbsim/frames/floating_contour_frame.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/rotarymatrices.h"
#include "mbsim/mbsim_event.h"

#include <openmbvcppinterface/group.h>

using namespace PLib;

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  NurbsDisk2s::NurbsDisk2s(const string &name) : Contour2s(name), nj(0), nr(0), degU(0), degV(0), Ri(0.), Ra(0.) {
    uvec=0;
    uVec=0;
    vvec=0;
    vVec=0;
    Surface = new PlNurbsSurfaced;
    SurfaceVelocities = new PlNurbsSurfaced;
  }

  NurbsDisk2s::~NurbsDisk2s() {
    if(Surface) {delete Surface; Surface=0;}
    if(SurfaceVelocities) {delete SurfaceVelocities; SurfaceVelocities=0;}
    if(uvec) {delete uvec; uvec=0;}
    if(uVec) {delete uVec; uVec=0;}
    if(vvec) {delete vvec; vvec=0;}
    if(vVec) {delete vVec; vVec=0;}
  }

  void NurbsDisk2s::init(InitStage stage) {
    if(stage==preInit) {
      degU = (static_cast<FlexibleBody2s13*>(parent))->getAzimuthalDegree();
      degV = (static_cast<FlexibleBody2s13*>(parent))->getRadialDegree();
      RefDofs = (static_cast<FlexibleBody2s13*>(parent))->getReferenceDegreesOfFreedom();
      nr = (static_cast<FlexibleBody2s13*>(parent))->getRadialNumberOfElements();
      nj = (static_cast<FlexibleBody2s13*>(parent))->getAzimuthalNumberOfElements();
      Ri = (static_cast<FlexibleBody2s13*>(parent))->getInnerRadius();
      Ra = (static_cast<FlexibleBody2s13*>(parent))->getOuterRadius();

      computeUVector(nj+degU);
      computeVVector(nr+1);

      for(int k=0; k<nr*nj*3+RefDofs; k++) {
        SurfaceJacobiansOfTranslation.push_back(PlNurbsSurfaced());
        SurfaceJacobiansOfRotation.push_back(PlNurbsSurfaced());
      }

      computeSurface();

      Contour2s::init(stage);
    }
    else if(stage==plotting) {
      updatePlotFeatures();

      if(plotFeature[13464197197848110344ULL]==enabled and openMBVNurbsDisk) {
        openMBVNurbsDisk->setName(name);
        drawDegree = 30 / nj;
        openMBVNurbsDisk->setDiffuseColor(0.46667, 1, 1);
        openMBVNurbsDisk->setMinimalColorValue(0.);
        openMBVNurbsDisk->setMaximalColorValue(1.);
        openMBVNurbsDisk->setDrawDegree(drawDegree);
        openMBVNurbsDisk->setRadii(Ri, Ra);

        openMBVNurbsDisk->setKnotVecAzimuthal(getUVector());
        openMBVNurbsDisk->setKnotVecRadial(getVVector());

        openMBVNurbsDisk->setElementNumberRadial(nr);
        openMBVNurbsDisk->setElementNumberAzimuthal(nj);

        openMBVNurbsDisk->setInterpolationDegreeRadial(degV);
        openMBVNurbsDisk->setInterpolationDegreeAzimuthal(degU);
        parent->getOpenMBVGrp()->addObject(openMBVNurbsDisk);
      }
      Contour2s::init(stage);
    }
    else
      Contour2s::init(stage);
  }

  ContourFrame* NurbsDisk2s::createContourFrame(const string &name) {
    FloatingContourFrame *frame = new FloatingContourFrame(name);
    frame->setContourOfReference(this);
    return frame;
  }

  Vec3 NurbsDisk2s::evalPosition(const Vec2 &zeta) {
    Vec3 r(NONINIT);
    computeSurface();
    Point3Dd Tmppt = Surface->pointAt(zeta(1),zeta(0));  // U-direction is azimuthal, V-direction is radial!
    r(0) = Tmppt.x();
    r(1) = Tmppt.y();
    r(2) = Tmppt.z();
    return r;
  }

  Vec3 NurbsDisk2s::evalWs(const Vec2 &zeta) {
    computeSurface();
    return computeDirectionalDerivatives(zeta(1),zeta(0),1).col(0);
  }

  Vec3 NurbsDisk2s::evalWt(const Vec2 &zeta) {
    computeSurface();
    return computeDirectionalDerivatives(zeta(1),zeta(0),1).col(1);
  }

  Vec3 NurbsDisk2s::evalWn(const Vec2 &zeta) {
    computeSurface();
    Point3Dd normal(Surface->normal(zeta(1),zeta(0)));
    double normalLength = sqrt(normal.x()*normal.x() + normal.y()*normal.y() + normal.z()*normal.z());  // to normalize the vector
    //
    normal *= -1;//normal should point out of the contour (as the normal is the crossproduct between the tangent in u und the tangent in v direction, the normal of nurbs++ points into the material)
    //
    Vec3 n;
    n(0) = normal.x() /normalLength;
    n(1) = normal.y() /normalLength;
    n(2) = normal.z() /normalLength;
    return n;
  }

  void NurbsDisk2s::updatePositions(ContourFrame *frame) {
    computeSurface();
    Point3Dd Tmppt = Surface->pointAt(frame->getZeta()(1),frame->getZeta()(0));  // U-direction is azimuthal, V-direction is radial!
    frame->getPosition(false)(0) = Tmppt.x();
    frame->getPosition(false)(1) = Tmppt.y();
    frame->getPosition(false)(2) = Tmppt.z();
    Mat A = computeDirectionalDerivatives(frame->getZeta()(1),frame->getZeta()(0),1);
    frame->getOrientation(false).set(0, A.col(0));
    frame->getOrientation(false).set(1, A.col(1));
    Point3Dd normal(Surface->normal(frame->getZeta()(1),frame->getZeta()(0)));
    double normalLength = sqrt(normal.x()*normal.x() + normal.y()*normal.y() + normal.z()*normal.z());  // to normalize the vector
    //
    normal *= -1;//normal should point out of the contour (as the normal is the crossproduct between the tangent in u und the tangent in v direction, the normal of nurbs++ points into the material)
    //
    Vec3 n;
    n(0) = normal.x() /normalLength;
    n(1) = normal.y() /normalLength;
    n(2) = normal.z() /normalLength;
    frame->getOrientation(false).set(2, n);
    // TODO This orientation is not correct as the x-axis equals the first
    // tangent (should be the normal). However, changing this orientation makes
    // the coorindate system dithering
  }

  void NurbsDisk2s::updateVelocities(ContourFrame *frame) {
    computeSurfaceVelocities();
    Point3Dd Tmpv = SurfaceVelocities->pointAt(frame->getZeta()(1),frame->getZeta()(0));
    frame->getVelocity(false)(0) = Tmpv.x();
    frame->getVelocity(false)(1) = Tmpv.y();
    frame->getVelocity(false)(2) = Tmpv.z();
    //      THROW_MBSIMERROR("(NurbsDisk2s::updateVelocities): Not implemented!");
  }

  void NurbsDisk2s::updateAccelerations(ContourFrame *frame) {
    THROW_MBSIMERROR("(NurbsDisk2s::updateAccelerations): Not implemented!");
  }

  void NurbsDisk2s::updateJacobians(ContourFrame *frame, int j) {
    computeSurfaceJacobians();

    frame->getJacobianOfTranslation(j,false).resize(nj*nr*3+RefDofs);
    frame->getJacobianOfRotation(j,false).resize(nj*nr*3+RefDofs);

    for(int k=0; k<nj*nr*3+RefDofs; k++) {
      Point3Dd TmpPtTrans = SurfaceJacobiansOfTranslation[k].pointAt(frame->getZeta()(1),frame->getZeta()(0));
      Point3Dd TmpPtRot = SurfaceJacobiansOfRotation[k].pointAt(frame->getZeta()(1),frame->getZeta()(0));

      frame->getJacobianOfTranslation(j,false)(0,k) = TmpPtTrans.x();
      frame->getJacobianOfTranslation(j,false)(1,k) = TmpPtTrans.y();
      frame->getJacobianOfTranslation(j,false)(2,k) = TmpPtTrans.z();

      frame->getJacobianOfRotation(j,false)(0,k) = TmpPtRot.x();
      frame->getJacobianOfRotation(j,false)(1,k) = TmpPtRot.y();
      frame->getJacobianOfRotation(j,false)(2,k) = TmpPtRot.z();
    }
  }

  void NurbsDisk2s::updateGyroscopicAccelerations(ContourFrame *frame) {
    THROW_MBSIMERROR("(NurbsDisk2s::updateGyroscopicAccelerations): Not implemented!");
  }

  Vec3 NurbsDisk2s::evalPosition() {
    return static_cast<FlexibleBody2s13*>(parent)->evalPosition();
  }

  SqrMat3 NurbsDisk2s::evalOrientation() {
    return static_cast<FlexibleBody2s13*>(parent)->evalOrientation();
  }

  void NurbsDisk2s::plot() {
    if(plotFeature[13464197197848110344ULL]==enabled and openMBVNurbsDisk) {
      vector<double> data;
      data.push_back(getTime()); //time

      Vec3 r = evalPosition();
      SqrMat3 A = evalOrientation();

      //Translation of COG
      data.push_back(r(0)); //global x-coordinate
      data.push_back(r(1)); //global y-coordinate
      data.push_back(r(2)); //global z-coordinate

      //Rotation of COG
      Vec AlphaBetaGamma = AIK2Cardan(A);
      data.push_back(AlphaBetaGamma(0));
      data.push_back(AlphaBetaGamma(1));
      data.push_back(AlphaBetaGamma(2));

      //Control-Point coordinates
      for(int i = 0; i < nr + 1; i++) {
        for(int j = 0; j < nj + degU; j++) {
          data.push_back(getControlPoints(j, i)(0)); //global x-coordinate
          data.push_back(getControlPoints(j, i)(1)); //global y-coordinate
          data.push_back(getControlPoints(j, i)(2)); //global z-coordinate
        }
      }

      Vec2 zeta(NONINIT);

      //inner ring
      for(int i = 0; i < nj; i++) {
        for(int j = 0; j < drawDegree; j++) {
          zeta(0) = Ri;
          zeta(1) = 2 * M_PI * (i * drawDegree + j) / (nj * drawDegree);
          Vec3 pos = evalPosition(zeta);

          data.push_back(pos(0)); //global x-coordinate
          data.push_back(pos(1)); //global y-coordinate
          data.push_back(pos(2)); //global z-coordinate

        }
      }

      //outer Ring
      for(int i = 0; i < nj; i++) {
        for(int j = 0; j < drawDegree; j++) {
          zeta(0) = Ra;
          zeta(1) = 2 * M_PI * (i * drawDegree + j) / (nj * drawDegree);
          Vec3 pos = evalPosition(zeta);

          data.push_back(pos(0)); //global x-coordinate
          data.push_back(pos(1)); //global y-coordinate
          data.push_back(pos(2)); //global z-coordinate
        }
      }

      openMBVNurbsDisk->append(data);
    }
    Contour2s::plot();
  }

  Vec NurbsDisk2s::transformCW(const Vec& WrPoint) {
    return (static_cast<FlexibleBody2s13*>(parent))->transformCW(WrPoint);
  }

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

  Mat NurbsDisk2s::computeCurvatures(const double &radius, const double &phi) {
    return computeDirectionalDerivatives(radius, phi, 2);
  }

  void NurbsDisk2s::computeUVector(const int NbPts) {
    delete uvec;
    uvec = new PLib::Vector<double>(NbPts);
    delete uVec;
    uVec = new PLib::Vector<double>(NbPts + degU + 1);

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

  void NurbsDisk2s::computeVVector(const int NbPts) {
    delete vvec;
    vvec = new PLib::Vector<double>(NbPts);
    delete vVec;
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

  void NurbsDisk2s::computeSurface() {
    PLib::Matrix<HPoint3Dd> Nodelist(nj+degU,nr+1); // list of Cartesian node-coordinates for the nurbs interpolation (2*degU+1 is used for the 2 outline of the surface in azimuthal direction)

    // gets Points from body for interpolation
    for(int i=0; i<nr+1; i++) {
      for(int j=0; j<nj; j++) {
        NodeFrame P("P",i*nj+j);
        P.setParent(parent);
        Nodelist(j,i) = HPoint3Dd(P.evalPosition()(0),P.evalPosition()(1),P.evalPosition()(2),1);
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

  void NurbsDisk2s::computeSurfaceVelocities() {
    PLib::Matrix<HPoint3Dd> Nodelist(nj+degU,nr+1); // list of Cartesian node-velocities for the nurbs interpolation (2*degU+1 is used for the 2 outline of the surface in azimuthal direction)

    // gets velocities from body for the interpolation
    for(int i=0; i<nr+1; i++) {
      for(int j=0; j<nj; j++) {
        NodeFrame P("P",i*nj+j);
        P.setParent(parent);
        Nodelist(j,i) = HPoint3Dd(P.evalVelocity()(0),P.evalVelocity()(1),P.evalVelocity()(2), 1);
      }
      for(int j=0;j<degU;j++) { // expands the surface addicted to the degree in azimuthal direction
        Nodelist(nj+j,i) = Nodelist(j,i);
      }
    }

    SurfaceVelocities->globalInterpClosedUH(Nodelist, *uVec, *vVec, *uvec, *vvec, degU, degV);
  }

  void NurbsDisk2s::computeSurfaceJacobians() {
    PLib::Matrix<HPoint3Dd> NodelistTrans(nj+degU,nr+1); // list of node-data for the nurbs interpolation
    PLib::Matrix<HPoint3Dd> NodelistRot(nj+degU,nr+1);// list of node-data for the nurbs interpolation

    // gets Jacobians on the nodes from body for interpolation
    vector<NodeFrame> P((nr+1)*nj);
    for(int i=0; i<nr+1; i++)
      for(int j=0; j<nj; j++) {
        P[i*nj+j] = NodeFrame("P",i*nj+j);
        P[i*nj+j].setParent(parent);
        P[i*nj+j].getJacobianOfTranslation(0,false).resize();
        P[i*nj+j].getJacobianOfRotation(0,false).resize();
        P[i*nj+j].getJacobianOfTranslation(1,false).resize();
        P[i*nj+j].getJacobianOfRotation(1,false).resize();
      }
    for(int k=0; k<nr*nj*3+RefDofs; k++) {
      for(int i=0; i<nr+1; i++) {
        for(int j=0; j<nj; j++) {
          NodelistTrans(j,i) = HPoint3Dd(P[i*nj+j].evalJacobianOfTranslation()(0,k),
              P[i*nj+j].evalJacobianOfTranslation()(1,k),
              P[i*nj+j].evalJacobianOfTranslation()(2,k),
              1);

          NodelistRot(j,i) = HPoint3Dd(P[i*nj+j].evalJacobianOfRotation()(0,k),
              P[i*nj+j].evalJacobianOfRotation()(1,k),
              P[i*nj+j].evalJacobianOfRotation()(2,k),
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

  Vec NurbsDisk2s::getControlPoints(const int u, const int v) {
    Vec TmpVec(3);

    TmpVec(0) = Surface->ctrlPnts(u,v).x();
    TmpVec(1) = Surface->ctrlPnts(u,v).y();
    TmpVec(2) = Surface->ctrlPnts(u,v).z();

    return TmpVec;
  }

  Vec NurbsDisk2s::getUVector() {
    Vec TmpUVec(Surface->knotU().rows());

    for (int i=0;i<Surface->knotU().rows();i++)
    TmpUVec(i)=Surface->knotU(i);

    return TmpUVec;
  }

  Vec NurbsDisk2s::getVVector() {
    Vec TmpVVec(Surface->knotV().rows());

    for (int i=0;i<Surface->knotV().rows();i++)
    TmpVVec(i)=Surface->knotV(i);

    return TmpVVec;
  }

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
