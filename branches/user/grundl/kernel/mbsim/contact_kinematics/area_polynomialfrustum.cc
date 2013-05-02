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
  edgePolyFrustum::edgePolyFrustum(const PolynomialFrustum * frustum_) :
      frustum(frustum_), A(), dir() {
  }

  edgePolyFrustum::~edgePolyFrustum() {
  }

  void edgePolyFrustum::setAdir(const Vec3 & A_, const Vec3 & dir_) {
    A = A_;
    dir = dir_;
  }

  Vec edgePolyFrustum::operator ()(const Vec & xin, const void *) {
    Vec result(1, NONINIT);
    const double & t = xin(0);

    Vec3 contactPointArea = A + t * dir;

    const double & x = contactPointArea(0);
    const double phi = ArcTan(contactPointArea(1), contactPointArea(2));
    const double s = sin(phi);
    const double c = cos(phi);

    const double fD1 = frustum->getValueD1(x);
    const double fD1SQp1 = fD1 * fD1 + 1;
    const double sqrtfD1SQp1 = sqrt(fD1SQp1);

    result(0) = -fD1 * dir(0) + c * dir(1) + s * dir(2);
    result(0) /= sqrtfD1SQp1;

    return result;
  }

  edgePolyFrustumCriteria::edgePolyFrustumCriteria(const double & tolerance_) :
      tolerance(tolerance_), frustumHeight(-1), criteriaResults(0), ax(0.), dx(0.) {
  }

  int edgePolyFrustumCriteria::operator ()(const Vec & xin, const void *) {
    criteriaResults.push_back(nrmInf((*function)(xin)));

    if (not inBounds(xin(0)))
      return -1; //out of bounds --> assume that this thing won't converge

    if (criteriaResults.back() < tolerance)
      return 0;

    return 1;
  }

  bool edgePolyFrustumCriteria::isBetter(const Vec & x) {
    if (not inBounds(x(0)))
      return false;

    if (nrmInf((*function)(x)) < criteriaResults.back())
      return true;

    return false;
  }

  bool edgePolyFrustumCriteria::inBounds(const double & t) {
    double x = ax + t * dx;
    if (t < 0 or t > 1 or x < 0 or x > frustumHeight)
      return false;

    return true;
  }

  projectAlongNormal::projectAlongNormal(PolynomialFrustum * frustum) :
      frustum(frustum), referencePoint(), phi(0.) {
  }

  projectAlongNormal::~projectAlongNormal() {

  }

  void projectAlongNormal::setUpSystemParamters(const Vec3 & referencePoint_, const double & phi_) {
    referencePoint = referencePoint_;
    phi = phi_;
  }

  Vec projectAlongNormal::operator ()(const Vec & xin, const void *) {
    Vec result(1, NONINIT);

    const double & x = xin(0);

    const double fd = frustum->getValueD1(x);

    //to avoid numerical instabilities chose one of the two options
    double tmp = referencePoint(1) /  cos(phi);
    if(fabs(cos(phi)) < 1e-2)
      tmp = referencePoint(2) / sin(phi);

    result(0) = referencePoint(0) - (frustum->getValue(x) * fd - fd * tmp + x);

    return result;
  }

  projectAlongNormalJacobian::projectAlongNormalJacobian(PolynomialFrustum * frustum) :
      frustum(frustum), referencePoint(), phi(0.) {
  }

  projectAlongNormalJacobian::~projectAlongNormalJacobian() {
  }

  void projectAlongNormalJacobian::setUpSystemParamters(const Vec3 & referencePoint_, const double & phi_) {
    referencePoint = referencePoint_;
    phi = phi_;
  }

  SqrMat projectAlongNormalJacobian::operator ()(const Vec & xin, const void *) {
    SqrMat Jac(1, NONINIT);

    const double & x = xin(0);

    const double fd = frustum->getValueD1(x);
    const double fdd = frustum->getValueD2(x);

    //to avoid numerical instabilities chose one of the two options
    double tmp = referencePoint(1) /  cos(phi);
    if(fabs(cos(phi)) < 1e-2)
      tmp = referencePoint(2) / sin(phi);

    Jac(0,0) =  - fd*fd - frustum->getValue(x) * fdd + fdd * tmp - 1;

    return Jac;
  }


  ContactKinematicsAreaPolynomialFrustum::ContactKinematicsAreaPolynomialFrustum() :
      ContactKinematics(), iarea(-1), ifrustum(-1), area(0), frustum(0), x1(-1), x2(-1), funcProjectAlongNormal(0), newtonProjectAlongNormal(), jacobianProjectAlongNormal(0), criteriaProjectAlongNormal(), dampingProjectAlongNormal(),  funcEdge(0), newtonEdge(), jacobianEdge(), criteriaEdge(), dampingEdge(), ilast(-1), xi(1, INIT, 0.5) {
  }

  ContactKinematicsAreaPolynomialFrustum::~ContactKinematicsAreaPolynomialFrustum() {
    delete funcEdge;
    delete funcProjectAlongNormal;
    delete jacobianProjectAlongNormal;
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

    //initialize contact point heights for contact situation in the area
    x1 = frustum->getHeight() / 2;
    x2 = frustum->getHeight() / 2;

    /*Set Up system for projection of contact point onto frustum surface*/
    funcProjectAlongNormal = new projectAlongNormal(frustum);
    newtonProjectAlongNormal.setFunction(funcProjectAlongNormal);
    jacobianProjectAlongNormal = new projectAlongNormalJacobian(frustum);
    newtonProjectAlongNormal.setJacobianFunction(jacobianProjectAlongNormal);
    newtonProjectAlongNormal.setCriteriaFunction(&criteriaProjectAlongNormal);
    newtonProjectAlongNormal.setDampingFunction(&dampingProjectAlongNormal);

    /*Set Up System for edge contact search */
    //initalize edge function
    funcEdge = new edgePolyFrustum(frustum);
    xi(0) = 0.5;
    newtonEdge.setFunction(funcEdge);
    newtonEdge.setJacobianFunction(&jacobianEdge);
    criteriaEdge.setFrustumHeight(frustum->getHeight());
    newtonEdge.setCriteriaFunction(&criteriaEdge);
    //REMARK: Damping is evil here, as it always "sees" a solution out of bound and damps this solution over and over again, however it is very likely, that edge that is iterated through will have no contact point

    //TODO: check for convexity of frustum
  }

  void ContactKinematicsAreaPolynomialFrustum::setFrustumOrienationKinematics(const double & x, const double & phi, ContourPointData * cpData) {
    SqrMat3 AWF = frustum->getFrame()->getOrientation();
    cpData[ifrustum].getFrameOfReference().getOrientation().set(0, AWF * frustum->computeNormal(x, phi));
    cpData[ifrustum].getFrameOfReference().getOrientation().set(1, -AWF * frustum->computeTangentRadial(x, phi));
    cpData[ifrustum].getFrameOfReference().getOrientation().set(2, AWF * frustum->computeTangentAzimuthal(x, phi));
  }

  bool ContactKinematicsAreaPolynomialFrustum::cpLocationInArea(Vec & g, ContourPointData * cpData) {

    double rhs = 0.; //, rhs2 = 0.;
    int status = 0; // status2 = 0;

    //normal of the area under the frustum reference frame
    Vec3 v = frustum->getReferenceOrientation().T() * area->getReferenceOrientation().col(0);

    //right sides of the two equations
    rhs = sqrt(v(0) * v(0) / (v(1) * v(1) + v(2) * v(2)));
    //rhs2 = -rhs;

    const Vec & para = frustum->getPolynomialParameters(); //para = (a0,a1 ... an)

    Vec para1d(para.size(), NONINIT); //coefficient vector of 1st derivative of the polynomial,para1d=(a1,2*a2,3*a3...n*an,0)
    for (int i = 0; i < para.size() - 1; i++) {
      para1d(i) = (i + 1) * (para(i + 1));
    }
    para1d(para.size() - 1) = 0;

    ContactPolyfun *Polyfun1 = new ContactPolyfun(rhs, para1d);
    //ContactPolyfun *Polyfun2 = new ContactPolyfun(rhs2, para1d);

    //Use newton method to solve 2 equations
    NewtonMethod *solver1 = new NewtonMethod(Polyfun1);
    //NewtonMethod *solver2 = new NewtonMethod(Polyfun2);

    int itmax = 200, kmax = 200; //maximum iteration, maximum damping steps, information of success
    double tol = 1e-10;

    solver1->setMaximumNumberOfIterations(itmax);
    solver1->setMaximumDampingSteps(kmax);
    solver1->setTolerance(tol);
    //x (=height-direction of frustum) -coordinate in equation one
    x1 = solver1->solve(x1); //initial guess x=height/2

//    solver2->setMaximumNumberOfIterations(itmax);
//    solver2->setMaximumDampingSteps(kmax);
//    solver2->setTolerance(tol);
//    //x (=height-direction of frustum) -coordinate in equation two
//    x2 = solver2->solve(x2);

    //Check x1 and x2, and for each solution
    //Check if the solution is successful
    //1.Check if the point is on the frustum contour:y^2+z^2-f(x)^2=0
    //2.Check if the corresponding point is within the area
    //3.Find the corresponding point on the plane, check if it is in the area
    if ((solver1->getInfo() == 0) && x1 >= 0 && x1 <= frustum->getHeight())
      status = checkPossibleContactPoint(x1, v);

    //same thing with the other solution x2
//    if ((solver2->getInfo() == 0) && x2 >= 0 && x2 <= frustum->getHeight()) {
//      status2 = checkPossibleContactPoint(x2, v);
//    }

    //If both solvers find a solution check, which has the smaller distance
//    if (status && status2) {
//      //TODO: Is this case possible at all?
//      if (distance2Area(computeContourPoint(x1, v)) < distance2Area(computeContourPoint(x2, v)))
//        status2 = 0;
//      else
//        status = 0;
//
//    }

    if (status) {
      cpData[ifrustum].getFrameOfReference().getPosition() = computeContourPoint(x1, v);
      return true;
    }
//    else if (status2) {
//      cpData[ifrustum].getFrameOfReference().getPosition() = computeContourPoint(x2, v);
//      cout << "Solution 2 found " << endl;
//      return true;
//    }

    return 0;
  }

  bool ContactKinematicsAreaPolynomialFrustum::cornerContact(Vec & g, ContourPointData * cpData) {
    cornerPoints[0] = area->getA();
    cornerPoints[1] = area->getB();
    cornerPoints[2] = area->getC();
    cornerPoints[3] = area->getD();
    double weights[4];
    double weightsum = 0;

    for (int i = 0; i < 4; i++) {
      cornerPoints[i] = frustum->getFrame()->getOrientation().T() * (area->getFrame()->getPosition() + area->getFrame()->getOrientation() * cornerPoints[i] - frustum->getFrame()->getPosition());
      const double & x = cornerPoints[i](0);
      //radial position of point
      const double r = sqrt(pow(cornerPoints[i](1), 2) + pow(cornerPoints[i](2), 2));
      const double R = frustum->getValue(x);
      if (x >= 0 and x <= frustum->getHeight() and r <= R) {
        //Possible contact point found --> assumed to be at x-position of corner point
        weights[i] = R - r;
        weightsum += weights[i];
      }
      else
        weights[i] = 0;
    }

    if(not (weightsum > 0.0)) {
      return false;
    }

    Vec3 contactPointArea;
    for (int i = 0; i < 4; i++) {
      contactPointArea = contactPointArea + cornerPoints[i] * weights[i]/weightsum;
    }

    double phi = ArcTan(contactPointArea(1), contactPointArea(2));

    // find x-position that intersects with normal for found contact point

    Vec x(1,INIT, contactPointArea(0));

    funcProjectAlongNormal->setUpSystemParamters(contactPointArea, phi);
    jacobianProjectAlongNormal->setUpSystemParamters(contactPointArea, phi);
    x = newtonProjectAlongNormal.solve(x);

    Vec3 contactPointFrustum = frustum->computePoint(x(0), phi);

    g(0) = frustum->computeNormal(x(0), phi).T() * (contactPointArea - contactPointFrustum);

    if (g(0) < 0.) {
      //Frustum
      Vec3 rF = frustum->getFrame()->getPosition();
      SqrMat3 AWF = frustum->getFrame()->getOrientation();
      cpData[ifrustum].getFrameOfReference().getPosition() = rF + AWF * contactPointFrustum;
      setFrustumOrienationKinematics(x(0), phi, cpData);

      //Area
      cpData[iarea].getFrameOfReference().getPosition() = rF + AWF  * contactPointArea;
      cpData[iarea].getFrameOfReference().getOrientation().set(0, -cpData[ifrustum].getFrameOfReference().getOrientation().col(0));
      cpData[iarea].getFrameOfReference().getOrientation().set(1, -cpData[ifrustum].getFrameOfReference().getOrientation().col(1));
      cpData[iarea].getFrameOfReference().getOrientation().set(2, cpData[ifrustum].getFrameOfReference().getOrientation().col(2));

      return true;
    }

    return false;
  }

  bool ContactKinematicsAreaPolynomialFrustum::edgeContact(Vec & g, ContourPointData * cpData) {
    for (int i = 0; i < 4; i++) {

      //indi is start point of edge
      int indi = i;
      if (ilast != -1) { //if there was a contact the last time step on edge then use the same edge again to search for an contact
        indi += ilast;
        if (indi >= 4)
          indi -= 4;
      }

      // indj is index for the end point of the edge
      int indj = indi + 1;
      if (indj == 4)
        indj = 0;

      if (not (i == 0 and ilast != -1)) { //initialize start value for contact search if last contact can not be used
        //start values are the midpoint of the edge (0.5) and for the height and azimuthal direction the projection of this point in the coordinates of the frustum
        xi(0) = 0.5;
      }

      //set up function for solution
      //Remark: The corner-points positions are assumed to be updated in the cornerContact-routine already --> this always happens if no corner point is a valid contact point
      funcEdge->setAdir(cornerPoints[indi], cornerPoints[indj] - cornerPoints[indi]);
      criteriaEdge.setStartingXCoordinate(cornerPoints[indi](0));
      criteriaEdge.setdirectionXCoordinate(cornerPoints[indj](0) - cornerPoints[indi](0));

      xi = newtonEdge.solve(xi);

      //two intersection points are found and it is not the same
      if (newtonEdge.getInfo() == 0) {

        const double & t = xi(0);

        Vec3 contactPointArea = cornerPoints[indi] + t * (cornerPoints[indj] - cornerPoints[indi]);

        const double & x = contactPointArea(0);
        const double phi = ArcTan(contactPointArea(1), contactPointArea(2));
        Vec3 contactPointFrustum = frustum->computePoint(x, phi);

        g(0) = frustum->computeNormal(x, phi).T() * (contactPointArea - contactPointFrustum);
        if (g(0) < 0.) {

          //Frustum
          cpData[ifrustum].getFrameOfReference().getPosition() = frustum->getFrame()->getPosition() + frustum->getFrame()->getOrientation() * contactPointFrustum;
          setFrustumOrienationKinematics(x, phi, cpData);

          //Area
          cpData[iarea].getFrameOfReference().getPosition() = frustum->getFrame()->getPosition() + frustum->getFrame()->getOrientation() * contactPointArea;
          cpData[iarea].getFrameOfReference().getOrientation().set(0, -cpData[ifrustum].getFrameOfReference().getOrientation().col(0));
          cpData[iarea].getFrameOfReference().getOrientation().set(1, -cpData[ifrustum].getFrameOfReference().getOrientation().col(1));
          cpData[iarea].getFrameOfReference().getOrientation().set(2, cpData[ifrustum].getFrameOfReference().getOrientation().col(2));

          //save values for next search
          ilast = indi;
          return true;
        }
      }
    }

    return false;
  }

  void ContactKinematicsAreaPolynomialFrustum::updateg(Vec & g, ContourPointData * cpData, int index) {
    /*Geometry*/
    //area
    Vec3 R_cen_A = area->getFrame()->getPosition(); //center of area
    Vec3 NormAxis_A = area->getFrame()->getOrientation().col(0); // normal of area in inertial FR

    /*construct the sphere enclosing the frustum*/
    //take center of the frustum as center of the sphere
    Vec3 R_cen_S = frustum->getEnclosingSphereCenter();

    //search the radius of the circumsphere
    double rad_sph = frustum->getEnclosingSphereRadius();

    //compute distance between the center of sphere and the plane where the area lies
    Vec3 Dis_SP = R_cen_S - R_cen_A; //distance vector
    double dis_SP = Dis_SP.T() * NormAxis_A;

    /*if distance>radius, no intersection, do nothing*/

    if (dis_SP <= rad_sph) {
      // find the intersection circle between the sphere and the plane
      //radius of the circle
      double rad_inscir = sqrt(pow(rad_sph, 2) - pow(dis_SP, 2));
      //position of circle center (projection of the sphere center onto the area/plane)
      Vec3 R_cen_inscir = R_cen_S - (dis_SP * NormAxis_A);

      //if the area intersects this circle on plane, then search for the contact point
      if (area->Intersect_Circle(rad_inscir, R_cen_inscir)) {
        //Check for contact point in the area
        if (cpLocationInArea(g, cpData)) {
          g(0) = distance2Area(cpData[ifrustum].getFrameOfReference().getPosition());
          cpData[iarea].getFrameOfReference().getOrientation() = area->getFrame()->getOrientation();
          cpData[iarea].getFrameOfReference().getPosition() = cpData[ifrustum].getFrameOfReference().getPosition() - area->getFrame()->getOrientation().col(0) * g(0);
          cpData[ifrustum].getFrameOfReference().getOrientation().set(0, -cpData[iarea].getFrameOfReference().getOrientation().col(0));
          cpData[ifrustum].getFrameOfReference().getOrientation().set(1, cpData[iarea].getFrameOfReference().getOrientation().col(1));
          cpData[ifrustum].getFrameOfReference().getOrientation().set(2, -cpData[iarea].getFrameOfReference().getOrientation().col(2));
          ilast = -1;
          return;
        }
        else if (cornerContact(g, cpData)) {
          ilast = -1;
          return;
        }
        else if (edgeContact(g, cpData)) {
          return;
        }
      }
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
    if (area->PointInArea(computeContourPoint(x, n))) {
      return 1; //point is possible to be the contact point
    }

    return 0;
  }

  double ContactKinematicsAreaPolynomialFrustum::distance2Area(const Vec3 & point) {
    return area->getReferenceOrientation().col(0).T() * (point - area->getFrame()->getPosition());
  }

} /* namespace MBSim */
