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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>

#include "fmatvec.h"
#include "area_polynomialfrustum.h"
#include "mbsim/contours/polynomial_frustum.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "fmatvec/linear_algebra_double.h"
#include "fmatvec/linear_algebra.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsAreaPolynomialFrustum::ContactKinematicsAreaPolynomialFrustum() :
      ContactKinematics(), iarea(-1), ifrustum(-1), area(0), frustum(0) {
  }

  ContactKinematicsAreaPolynomialFrustum::~ContactKinematicsAreaPolynomialFrustum() {
  }

  void ContactKinematicsAreaPolynomialFrustum::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Area*>(contour[0])) {
      iarea = 0;
      ifrustum = 1;
      area = static_cast<Area*>(contour[0]);
      frustum = static_cast<PolynomialFrustum*>(contour[1]);
    }
    else {
      iarea = 1;
      ifrustum = 0;
      area = static_cast<Area*>(contour[1]);
      frustum = static_cast<PolynomialFrustum*>(contour[0]);
    }

    //TODO: check for convexity of frustum
  }

  bool ContactKinematicsAreaPolynomialFrustum::cpLocationInArea(Vec & g, ContourPointData * cpData) {

    double rhs1 = 0., rhs2 = 0.;
    int status1 = 0, status2 = 0;

    //normal of the area under the frustum reference frame
    Vec3 v = frustum->getReferenceOrientation().T() * area->getReferenceOrientation().col(0);

    //right sides of the two equations
    rhs1 = sqrt(v(0) * v(0) / (v(1) * v(1) + v(2) * v(2)));
    rhs2 = -rhs1;

    const Vec & para = frustum->getPolynomialParameters(); //para = (a0,a1 ... an)

    Vec para1d(para.size(), NONINIT); //coefficient vector of 1st derivative of the polynomial,para1d=(a1,2*a2,3*a3...n*an,0)
    for (int i = 0; i < para.size() - 1; i++) {
      para1d(i) = (i + 1) * (para(i + 1));
    }
    para1d(para.size() - 1) = 0;

    ContactPolyfun *Polyfun1 = new ContactPolyfun(rhs1, para1d);
    ContactPolyfun *Polyfun2 = new ContactPolyfun(rhs2, para1d);

    //Use newton method to solve 2 equations
    NewtonMethod *solver1 = new NewtonMethod(Polyfun1);
    NewtonMethod *solver2 = new NewtonMethod(Polyfun2);

    int itmax = 200, kmax = 200; //maximum iteration, maximum damping steps, information of success
    double tol = 1e-10;

    solver1->setMaximumNumberOfIterations(itmax);
    solver1->setMaximumDampingSteps(kmax);
    solver1->setTolerance(tol);
    //x (=height-direction of frustum) -coordinate in equation one
    double x1 = solver1->solve(frustum->getHeight() / 2); //initial guess x=height/2

    solver2->setMaximumNumberOfIterations(itmax);
    solver2->setMaximumDampingSteps(kmax);
    solver2->setTolerance(tol);
    //x (=height-direction of frustum) -coordinate in equation two
    double x2 = solver2->solve(frustum->getHeight() / 2);

    //if neither equation has a root, we should use numerical method
    if ((solver1->getInfo() != 0) && (solver2->getInfo() != 0)) {
      cout << "There is no normal on the frustum contour parallel to the normal of the area!" << endl;
      return false; //TODO: Here should be some numerical method
    }

    //Check x1 and x2, and for each solution
    //Check if the solution is successful
    //1.Check if the point is on the frustum contour:y^2+z^2-f(x)^2=0
    //2.Check if the corresponding point is within the area
    //3.Find the corresponding point on the plane, check if it is in the area
    if ((solver1->getInfo() == 0) && x1 >= 0 && x1 <= frustum->getHeight())
      status1 = checkPossibleContactPoint(x1, v);

    //same thing with the other solution x2
    if ((solver2->getInfo() == 0) && x2 >= 0 && x2 <= frustum->getHeight()) {
      status2 = checkPossibleContactPoint(x2, v);
    }

    //If both solvers find a solution check, which has the smaller distance
    if (status1 && status2) {
      //TODO: Is this case possible at all?
      if (distance2Area(computeContourPoint(x1, v)) < distance2Area(computeContourPoint(x2, v)))
        status2 = 0;
      else
        status1 = 0;

    }

    if (status1) {
      cpData[ifrustum].getFrameOfReference().getPosition() = computeContourPoint(x1, v);
      return true;
    }
    else if (status2) {
      cpData[ifrustum].getFrameOfReference().getPosition() = computeContourPoint(x2, v);
      return true;
    }

    cout << "No contact point found" << endl;
    return 0;
  }

  void ContactKinematicsAreaPolynomialFrustum::updateg(Vec & g, ContourPointData * cpData, int index) {
    /*Geometry*/
    //polynomialfrustum
    double h_F = frustum->getHeight(); // height of Frustum
    Vec3 NormAxis_F = frustum->getFrame()->getOrientation().col(0); // normed axis of frustum in inertial FR

    //area
    Vec3 R_cen_A = area->getFrame()->getPosition(); //center of area
    Vec3 NormAxis_A = area->getReferenceOrientation().col(0); // normal of area in inertial FR

    /*construct the sphere enclosing the frustum*/
    //take center of the frustum as center of the sphere
    Vec3 R_cen_S = frustum->getFrame()->getPosition() + h_F / 2 * NormAxis_F;
    //search the radius of the circumsphere
    double rad_sph = frustum->getRadiSphere();

    //compute distance between the center of sphere and the plane where the area lies
    Vec3 Dis_SP = R_cen_S - R_cen_A; //distance vector
    double dis_SP = fabs(Dis_SP.T() * NormAxis_A);

    //cout<<"THE DISTANCE FROM CENTER OF SPHERE TO PLANE IS "<<dis_SP<<endl;

    /*if distance>radius, no intersection, do nothing*/

    if (dis_SP <= rad_sph) {
      // find the intersection circle between the sphere and the plane
      //radius of the circle
      double rad_inscir = sqrt(pow(rad_sph, 2) - pow(dis_SP, 2));
      //position of circle center
      Vec3 R_cen_inscir = R_cen_S - (dis_SP * NormAxis_A);

      //if the area intersects this circle on plane, then search for the contact point
      if (area->Intersect_Circle(rad_inscir, R_cen_inscir)) {

        //Check for contact point in the area
        if (cpLocationInArea(g, cpData)) {
          g(0) = distance2Area(cpData[ifrustum].getFrameOfReference().getPosition());
          cpData[iarea].getFrameOfReference().getOrientation() = area->getFrame()->getOrientation();
          cpData[iarea].getFrameOfReference().getPosition() = cpData[ifrustum].getFrameOfReference().getPosition() - area->getFrame()->getOrientation().col(0) * g(0);
          cpData[ifrustum].getFrameOfReference().getOrientation().set(0, -cpData[iarea].getFrameOfReference().getOrientation().col(0));
          cpData[ifrustum].getFrameOfReference().getOrientation().set(1, -cpData[iarea].getFrameOfReference().getOrientation().col(1));
          cpData[ifrustum].getFrameOfReference().getOrientation().set(2, cpData[iarea].getFrameOfReference().getOrientation().col(2));
          return;
        }

      }
      else
        g(0) = 1.;

    }

    g(0) = 1.;
  }

  Vec3 ContactKinematicsAreaPolynomialFrustum::computeContourPoint(const double & x, const Vec3 & n) {
    Vec3 contourPoint(NONINIT);
    double y1 = -(frustum->getValue(x)) * (frustum->getValueD1(x)) * (n(1)) / (n(0));     //y=-f(x)*f'(x)*v2/v1
    double z1 = -(frustum->getValue(x)) * (frustum->getValueD1(x)) * (n(2)) / (n(0));     //z=-f(x)*f'(x)*v3/v1

    contourPoint(0) = x;
    contourPoint(1) = y1;
    contourPoint(2) = z1;

    return frustum->getReferenceOrientation() * contourPoint;
  }

  int ContactKinematicsAreaPolynomialFrustum::checkPossibleContactPoint(const double & x, const Vec3 & n) {

    //TODO: move the following in a function (as it is the same for equation 2 again...) ?
    //double err = pow(y1, 2) + pow(z1, 2) - pow(frustum->getValue(x1), 2);

    //Check because of possible numerical instabilities --> not really necessary...
    //if (fabs(err) < 1e-6) {
    //contact point on frustum in  inertial frame

    if (area->PointInArea(computeContourPoint(x, n))) { //ArCP_A1)){
      return 1; //point is possible to be the contact point
    }
    //}

    return 0;
  }

  double ContactKinematicsAreaPolynomialFrustum::distance2Area(const Vec3 & point) {
    return area->getReferenceOrientation().col(0).T() * (point - area->getFrame()->getPosition());
  }

//  fmatvec::SqrMat3 ContactKinematicsAreaPolynomialFrustum::RotatM_2vec(fmatvec::Vec3 X, fmatvec::Vec3 V){
//    Vec3 Y,Z;
//    Z(0) = X(1)*V(2)-X(2)*V(1);
//    Z(1) = X(2)*V(0)-X(0)*V(2);
//    Z(2) = X(0)*V(1)-X(1)*V(0);
//    X = X/(nrm2(X));
//    Z = Z/(nrm2(Z));
//    Y(0) = Z(1)*X(2)-Z(2)*X(1);
//    Y(1) = Z(2)*X(0)-Z(0)*X(2);
//    Y(2) = Z(0)*X(1)-Z(1)*X(0);
//    SqrMat3 A;
//    A.col(0)=X;
//    A.col(1)=Y;
//    A.col(2)=Z;
//    A=A.T();
//    return A;
//  }

//  MBSim::CP_item_frustum ContactKinematicsAreaPolynomialFrustum::CP_toP_onfrustum3D(fmatvec::Vec3 P){
//    Vec3 V1,V2,OF,PF,Pnew;
//    CP_item_frustum cpinfo;
//    OF=frustum->getReferencePosition();//origin of frustum frame
//    V1=frustum->getReferenceOrientation().col(0);//V1 assigned to be normal axis of the frustum
//    V2=P-OF;//V2 assigned to be the vector from the frustum origin to P
//    SqrMat3 rotationA;
//    rotationA=RotatM_2vec(V1,V2);//rotation matrix rotate from current frame to the new frame
//
//
//
//    cpinfo.CP_item_frustum(P,P,0);
//    return cpinfo;
//  }

} /* namespace MBSim */
