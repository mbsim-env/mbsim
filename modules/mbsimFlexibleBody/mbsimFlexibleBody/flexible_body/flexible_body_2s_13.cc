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

#define FMATVEC_DEEP_COPY
#define FMATVEC_NO_INITIALIZATION
#define FMATVEC_NO_BOUNDS_CHECK

#include "mbsimFlexibleBody/flexible_body/flexible_body_2s_13.h"
#include "mbsimFlexibleBody/contours/nurbs_disk_2s.h"
#include "mbsim/dynamic_system.h"
#include "mbsim/utils/eps.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/nurbsdisk.h"
#endif

#include <iostream>

using namespace std;
using namespace fmatvec;
using namespace MBSim;

namespace MBSimFlexibleBody {

  Mat condenseMatrixRows(Mat A, Index I) {
    Mat B(A.rows() - (I.end() - I.start() + 1), A.cols());
    Index upperPart(0, I.start() - 1);
    Index lowerPartA(I.end() + 1, A.rows() - 1);
    Index lowerPartB(I.start(), B.rows() - 1);
    Index AllCols(0, A.cols() - 1);

    B(upperPart, AllCols) = A(upperPart, AllCols); // upper
    B(lowerPartB, AllCols) = A(lowerPartA, AllCols); // lower
    return B;
  }

  Mat condenseMatrixCols(Mat A, Index I) {
    Mat B(A.rows(), A.cols() - (I.end() - I.start() + 1));
    Index leftPart(0, I.start() - 1);
    Index rightPartA(I.end() + 1, A.cols() - 1);
    Index rightPartB(I.start(), B.cols() - 1);
    Index AllRows(0, A.rows() - 1);

    B(AllRows, leftPart) = A(AllRows, leftPart); // left
    B(AllRows, rightPartB) = A(AllRows, rightPartA); // right
    return B;
  }

  SymMat condenseMatrix(SymMat A, Index I) {
    // build size of result matrix
    SymMat B(A.size() - (I.end() - I.start() + 1));
    Index upperPart(0, I.start() - 1);
    Index lowerPartA(I.end() + 1, A.size() - 1);
    Index lowerPartB(I.start(), B.size() - 1);

    // assemble result matrix
    B(upperPart) << A(upperPart); // upper left
    B(upperPart, lowerPartB) << A(upperPart, lowerPartA); // upper right
    B(lowerPartB) << A(lowerPartA); // lower right
    return B;
  }

  double ArcTan(double x, double y) {
    double phi;
    phi = atan2(y, x);

    if(phi < 0.)
      phi += 2 * M_PI;
    //if(phi >= 2 * M_PI) TODO
    //  phi -= 2 * M_PI;
    return phi;
  }

  void MapleOutput(SymMat A, std::string MatName, std::string file)
  {
    ofstream dat(file.c_str() , ios::app);
    dat << MatName;
    dat << " := Matrix([";
    for(int i = 0; i<A.rows(); i++)
    {
      dat <<"[";
      for(int j = 0;j<A.cols(); j++)
      {
        dat << A(i,j);
        if(j<A.cols()-1)
          dat << ", ";
      }
      dat << "]";
      if(i!=A.rows()-1)
        dat << ",";
    }
    dat << "]):";
    dat << '\n';
    dat.close();
  }

  FlexibleBody2s13::FlexibleBody2s13(const string &name) : FlexibleBodyContinuum<Vec> (name), Elements(0), NodeDofs(3), RefDofs(0), E(0.), nu(0.), rho(0.), d(3,INIT,0.), Ri(0), Ra(0), dr(0), dj(0), m0(0), J0(3,INIT,0.), degV(3), degU(3), drawDegree(0), currentElement(0), nr(0), nj(0), Nodes(0), Dofs(0), LType(innerring), A(3,EYE), G(3,EYE) {
#ifdef HAVE_NURBS
    contour = new NurbsDisk2s("SurfaceContour");
    Body::addContour(contour);
#else
    contour=0;
    cout << "WARNING (FlexibleBody2s13::FlexibleBody2s13): No NURBS library installed!" << endl;
#endif

    // frame in axis
    Vec s(2, fmatvec::INIT, 0.);
    addFrame("COG", s);
  }

  void FlexibleBody2s13::updateh(double t) {
    // update positions and velocities
    qext = Jext * q;
    uext = Jext * u;

    h = -K * q;
    hObject = -K * q;
  }

  void FlexibleBody2s13::updatedhdz(double t) {
    updateh(t);
    for(int i = 0; i < dhdq.cols(); i++)
      for(int j = 0; j < dhdq.rows(); j++)
        dhdq(i, j) = -K(i, j);
  }

  void FlexibleBody2s13::updateStateDependentVariables(double t) {
    FlexibleBodyContinuum<Vec>::updateStateDependentVariables(t);

    updateAG();

#ifdef HAVE_NURBS
    contour->computeSurface();
    contour->computeSurfaceVelocities();
    contour->computeSurfaceJacobians();
#endif
  }

  void FlexibleBody2s13::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive) == enabled) {
#ifdef HAVE_OPENMBVCPPINTERFACE
#ifdef HAVE_NURBS
      if(getPlotFeature(openMBV) == enabled && openMBVBody) {
        vector<double> data;
        data.push_back(t); //time

        for(int i = 0; i < nr + 1; i++) {
          for(int j = 0; j < nj + degU; j++) {
            data.push_back(contour->getControlPoints(j, i)(0)); //global x-coordinate
            data.push_back(contour->getControlPoints(j, i)(1)); //global y-coordinate
            data.push_back(contour->getControlPoints(j, i)(2)); //global z-coordinate
          }
        }

        ContourPointData cp;
        cp.getLagrangeParameterPosition() = Vec(2, NONINIT);

        //inner ring
        for(int i = 0; i < nj; i++) {
          for(int j = 0; j < drawDegree; j++) {
            cp.getLagrangeParameterPosition()(0) = Ri;
            cp.getLagrangeParameterPosition()(1) = 2 * M_PI * (i * drawDegree + j) / (nj * drawDegree);
            contour->updateKinematicsForFrame(cp, position);
            Vec pos = cp.getFrameOfReference().getPosition();

            data.push_back(pos(0)); //global x-coordinate
            data.push_back(pos(1)); //global y-coordinate
            data.push_back(pos(2)); //global z-coordinate
          }
        }

        //outer Ring
        for(int i = 0; i < nj; i++) {
          for(int j = 0; j < drawDegree; j++) {
            cp.getLagrangeParameterPosition()(0) = Ra;
            cp.getLagrangeParameterPosition()(1) = 2 * M_PI * (i * drawDegree + j) / (nj * drawDegree);
            contour->updateKinematicsForFrame(cp, position);
            Vec pos = cp.getFrameOfReference().getPosition();

            data.push_back(pos(0)); //global x-coordinate
            data.push_back(pos(1)); //global y-coordinate
            data.push_back(pos(2)); //global z-coordinate
          }
        }

        //center of gravity
        cp.getLagrangeParameterPosition()(0) = 0.;
        cp.getLagrangeParameterPosition()(1) = 0.;
        contour->updateKinematicsForFrame(cp, position_cosy); // kinematics of the center of gravity of the disk (TODO frame feature)

        data.push_back(cp.getFrameOfReference().getPosition()(0) - (cp.getFrameOfReference().getOrientation())(0, 2) * d(0) * 0.5); //global x-coordinate
        data.push_back(cp.getFrameOfReference().getPosition()(1) - (cp.getFrameOfReference().getOrientation())(1, 2) * d(0) * 0.5); //global y-coordinate
        data.push_back(cp.getFrameOfReference().getPosition()(2) - (cp.getFrameOfReference().getOrientation())(2, 2) * d(0) * 0.5); //global z-coordinate

        for(int i = 0; i < 3; i++)
          for(int j = 0; j < 3; j++)
            data.push_back((cp.getFrameOfReference().getOrientation())(i, j));

        ((OpenMBV::NurbsDisk*) openMBVBody)->append(data);
      }
#endif
#endif
    }
    FlexibleBodyContinuum<Vec>::plot(t, dt);
  }

  void FlexibleBody2s13::setNumberElements(int nr_, int nj_) {
    nr = nr_;
    nj = nj_;
    degV = min(degV, nr); // radial adaptation of spline degree to have correct knot vector
    degU = min(degU, nj); // azimuthal adaptation of spline degree to have correct knot vector
    Elements = nr * nj;
    Nodes = (nr + 1) * nj;

    Dofs = RefDofs + Nodes * NodeDofs;

    qSize = Dofs - NodeDofs * nj; // missing one node row because of bearing
    uSize[0] = qSize;
    uSize[1] = qSize; // TODO

    qext = Vec(Dofs);
    uext = Vec(Dofs);

    q0.resize(qSize);
    u0.resize(uSize[0]);
  }

  void FlexibleBody2s13::BuildElement(const Vec &s) {
    assert(Ri <= s(0)); // is the input on the disk?
    assert(Ra >= s(0));

    currentElement = int((s(0) - Ri) / dr) * nj + int(s(1) / dj); // which element is involved?
  }

  SqrMat FlexibleBody2s13::TransformationMatrix(const double &phi) {
    SqrMat TransMat(3,EYE);

    TransMat(0,0) = cos(phi);
    TransMat(0,1) = -sin(phi);

    TransMat(1,0) = sin(phi);
    TransMat(1,1) = cos(phi);

    return TransMat.copy();
  }

  double FlexibleBody2s13::computeThickness(const double &r_) {
    return d(0) + d(1) * r_ + d(2) * r_ * r_; // quadratic parameterization
  }

}

