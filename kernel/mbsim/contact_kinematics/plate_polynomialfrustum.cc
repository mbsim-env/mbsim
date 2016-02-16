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

#include "fmatvec/fmatvec.h"
#include "plate_polynomialfrustum.h"
#include "mbsim/frames/frame.h"
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

  Vec edgePolyFrustum::operator ()(const Vec &xin) {
    Vec result(1, NONINIT);
    const double & t = xin(0);

    Vec3 contactPointPlate = A + t * dir;

    const double & x = contactPointPlate(0);
    const double phi = ArcTan(contactPointPlate(1), contactPointPlate(2));
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

  int edgePolyFrustumCriteria::operator ()(const Vec &xin) {
    criteriaResults.push_back(nrmInf((*function)(xin)));

    if (not inBounds(xin(0)))
      return -1; //out of bounds --> assume that this thing won't converge

    if (criteriaResults.back() < tolerance)
      return 0;

    return 1;
  }

  bool edgePolyFrustumCriteria::isBetter(const Vec & x, const Vec & fVal) {
    if (not inBounds(x(0)))
      return false;

    Vec fValget(fVal);
    if(fVal.size() == 0)
      fValget.resize() = (*function)(x);
    if (nrmInf(fValget) < criteriaResults.back())
      return true;

    return false;
  }

  bool edgePolyFrustumCriteria::inBounds(const double & t) {
    double x = ax + t * dx;
    double signh = sign(frustumHeight);
    if (t < 0 or t > 1 or signh * x < 0 or signh * x > signh * frustumHeight)
      return false;

    return true;
  }

  ContactKinematicsPlatePolynomialFrustum::ContactKinematicsPlatePolynomialFrustum() :
      ContactKinematics(), iplate(-1), ifrustum(-1), plate(0), frustum(0), signh(1.), gridSizeY(5), gridSizeZ(5), x1(-1), x2(-1), funcProjectAlongNormal(0), newtonProjectAlongNormal(), jacobianProjectAlongNormal(0), criteriaProjectAlongNormal(), dampingProjectAlongNormal(), funcEdge(0), newtonEdge(), jacobianEdge(), criteriaEdge(), dampingEdge(), ilast(-1), xi(1, INIT, 0.5) {
  }

  ContactKinematicsPlatePolynomialFrustum::~ContactKinematicsPlatePolynomialFrustum() {
    delete funcEdge;
    delete funcProjectAlongNormal;
    delete jacobianProjectAlongNormal;
  }

  void ContactKinematicsPlatePolynomialFrustum::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Plate*>(contour[0])) {
      iplate = 0;
      ifrustum = 1;
      plate = static_cast<Plate*>(contour[0]);
      frustum = static_cast<PolynomialFrustum*>(contour[1]);
    }
    else {
      iplate = 1;
      ifrustum = 0;
      plate = static_cast<Plate*>(contour[1]);
      frustum = static_cast<PolynomialFrustum*>(contour[0]);
    }

    //initialize contact point heights for contact situation in the plate
    x1 = frustum->getHeight() / 2;
    x2 = frustum->getHeight() / 2;

    /*Set Up system for projection of contact point onto frustum surface*/
    funcProjectAlongNormal = new projectPointAlongNormal(frustum);
    newtonProjectAlongNormal.setFunction(funcProjectAlongNormal);
    jacobianProjectAlongNormal = new projectPointAlongNormalJacobian(frustum);
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

    signh = sign(frustum->getHeight());

    updateGrid();

    //TODO: check for convexity of frustum
  }

  void ContactKinematicsPlatePolynomialFrustum::setFrustumOrienationKinematics(double t, const double & x, const double & phi, std::vector<Frame*> &cFrame) {
    cFrame[ifrustum]->getOrientation(false).set(0, frustum->getWn(t, zeta));
    cFrame[ifrustum]->getOrientation(false).set(1, signh * frustum->getWu(t, zeta));
    cFrame[ifrustum]->getOrientation(false).set(2, signh * frustum->getWv(t, zeta));
  }

  bool ContactKinematicsPlatePolynomialFrustum::cpLocationInPlate(double t, double & g, std::vector<Frame*> &cFrame) {

    double rhs = 0.; //, rhs2 = 0.;
    int status = 0; // status2 = 0;

    //normal of the plate under the frustum reference frame
    Vec3 v = frustum->getFrame()->getOrientation(t).T() * plate->getFrame()->getOrientation(t).col(0);

    //right sides of the two equations
    rhs = signh * v(0) * v(0) / (v(1) * v(1) + v(2) * v(2));
    //rhs2 = -rhs;

    ContactPolyfun Polyfun1(rhs, frustum);
    //ContactPolyfun *Polyfun2 = new ContactPolyfun(rhs2, para1d);

    //Use newton method to solve 2 equations
    NewtonMethod solver1(&Polyfun1);
    //NewtonMethod *solver2 = new NewtonMethod(Polyfun2);

    int itmax = 200, kmax = 200; //maximum iteration, maximum damping steps, information of success
    double tol = 1e-10;

    solver1.setMaximumNumberOfIterations(itmax);
    solver1.setMaximumDampingSteps(kmax);
    solver1.setTolerance(tol);
    //x (=height-direction of frustum) -coordinate in equation one
    x1 = solver1.solve(x1); //initial guess x=height/2

//    solver2->setMaximumNumberOfIterations(itmax);
//    solver2->setMaximumDampingSteps(kmax);
//    solver2->setTolerance(tol);
//    //x (=height-direction of frustum) -coordinate in equation two
//    x2 = solver2->solve(x2);

    //Check x1 and x2, and for each solution
    //Check if the solution is successful
    //1.Check if the point is on the frustum contour:y^2+z^2-f(x)^2=0
    //2.Check if the corresponding point is within the plate
    //3.Find the corresponding point on the plane, check if it is in the
    //plate
    if ((solver1.getInfo() == 0) && signh * x1 >= 0 && signh * x1 <= signh * frustum->getHeight())
      status = checkPossibleContactPoint(t, x1, v);

    //same thing with the other solution x2
//    if ((solver2->getInfo() == 0) && x2 >= 0 && x2 <= frustum->getHeight()) {
//      status2 = checkPossibleContactPoint(x2, v);
//    }

    //If both solvers find a solution check, which has the smaller distance
//    if (status && status2) {
//      //TODO: Is this case possible at all?
//      if (distance2Plate(computeContourPoint(x1, v)) < distance2Plate(computeContourPoint(x2, v)))
//        status2 = 0;
//      else
//        status = 0;
//
//    }

    if (status) {
      g = distance2Plate(t, cFrame[ifrustum]->getPosition(false));

      Vec3 pF = computeContourPointFrustum(x1, v);
      cFrame[ifrustum]->setPosition(computeContourPoint(t, x1, v));
      setFrustumOrienationKinematics(t, x1, ArcTan(pF(1), pF(2)), cFrame);

      cFrame[iplate]->setPosition(cFrame[ifrustum]->getPosition(false) - plate->getFrame()->getOrientation(t).col(0) * g);
      cFrame[iplate]->getOrientation(false).set(0, -cFrame[ifrustum]->getOrientation(false).col(0));
      cFrame[iplate]->getOrientation(false).set(1, -cFrame[ifrustum]->getOrientation(false).col(1));
      cFrame[iplate]->getOrientation(false).set(2, cFrame[ifrustum]->getOrientation(false).col(2));
      return true;
    }
//    else if (status2) {
//      cFrame[ifrustum]->getPosition() = computeContourPoint(x2, v);
//      cout << "Solution 2 found " << endl;
//      return true;
//    }

    return false;
  }

  bool ContactKinematicsPlatePolynomialFrustum::gridContact(double t, double & g, std::vector<Frame*> &cFrame) {

    const double h = frustum->getHeight();
    double weightsum = 0;

    Vec3 contactPointPlate;

    for (int i = 0; i < gridSizeY; i++) {
      for (int j = 0; j < gridSizeZ; j++) {
        Vec3 currentPoint = frustum->getFrame()->getOrientation(t).T() * (plate->getFrame()->getPosition(t) + plate->getFrame()->getOrientation(t) * gridPoints[i][j] - frustum->getFrame()->getPosition(t));
        const double & x = currentPoint(0);
        //radial position of point
        const double r = sqrt(pow(currentPoint(1), 2) + pow(currentPoint(2), 2));
        const double R = frustum->getValue(x);
        if (signh * x >= 0 and signh * x <= signh * h and r <= R) {
          //Possible contact point found --> assumed to be at x-position of corner point
          double weight = R - r;
          contactPointPlate = contactPointPlate + currentPoint * weight;
          weightsum += weight;
        }
      }
    }

    if (not (weightsum > 0.0)) {
      return false;
    }

    contactPointPlate = contactPointPlate * (1. / weightsum);

    double phi = ArcTan(contactPointPlate(1), contactPointPlate(2));
    // find x-position that intersects with normal for found contact point

    Vec x(1, INIT, contactPointPlate(0));

    funcProjectAlongNormal->setUpSystemParamters(contactPointPlate, phi);
    jacobianProjectAlongNormal->setUpSystemParamters(contactPointPlate, phi);
    x = newtonProjectAlongNormal.solve(x);

    zeta(0) = x(0);
    zeta(1) = phi;

    Vec3 contactPointFrustum = frustum->getKrPS(zeta);
    g = frustum->getKn(zeta).T() * (contactPointPlate - contactPointFrustum);

    if (g < 0.) {
      //Frustum
      Vec3 rF = frustum->getFrame()->getPosition(t);
      SqrMat3 AWF = frustum->getFrame()->getOrientation(t);
      cFrame[ifrustum]->setPosition(rF + AWF * contactPointFrustum);
      setFrustumOrienationKinematics(t, x(0), phi, cFrame);

      //Plate
      cFrame[iplate]->setPosition(rF + AWF * contactPointPlate);
      cFrame[iplate]->getOrientation(false).set(0, -cFrame[ifrustum]->getOrientation(false).col(0));
      cFrame[iplate]->getOrientation(false).set(1, -cFrame[ifrustum]->getOrientation(false).col(1));
      cFrame[iplate]->getOrientation(false).set(2, cFrame[ifrustum]->getOrientation(false).col(2));

      return true;
    }

    return false;
  }

  bool ContactKinematicsPlatePolynomialFrustum::cornerContact(double t, double & g, std::vector<Frame*> &cFrame) {
    cornerPoints[0] = plate->getA();
    cornerPoints[1] = plate->getB();
    cornerPoints[2] = plate->getC();
    cornerPoints[3] = plate->getD();
    double weights[4];
    double weightsum = 0;

    double h = frustum->getHeight();

    for (int i = 0; i < 4; i++) {
      cornerPoints[i] = frustum->getFrame()->getOrientation(t).T() * (plate->getFrame()->getPosition(t) + plate->getFrame()->getOrientation(t) * cornerPoints[i] - frustum->getFrame()->getPosition(t));
      const double & x = cornerPoints[i](0);
      //radial position of point
      const double r = sqrt(pow(cornerPoints[i](1), 2) + pow(cornerPoints[i](2), 2));
      const double R = frustum->getValue(x);
      if (signh * x >= 0 and signh * x <= signh * h and r <= R) {
        //Possible contact point found --> assumed to be at x-position of corner point
        weights[i] = R - r;
        weightsum += weights[i];
      }
      else
        weights[i] = 0;
    }

    if (not (weightsum > 0.0)) {
      return false;
    }

    Vec3 contactPointPlate;
    for (int i = 0; i < 4; i++) {
      contactPointPlate = contactPointPlate + cornerPoints[i] * weights[i] / weightsum;
    }

    double phi = ArcTan(contactPointPlate(1), contactPointPlate(2));

    // find x-position that intersects with normal for found contact point

    Vec x(1, INIT, contactPointPlate(0));

    zeta(0) = x(0);
    zeta(1) = phi;

    funcProjectAlongNormal->setUpSystemParamters(contactPointPlate, phi);
    jacobianProjectAlongNormal->setUpSystemParamters(contactPointPlate, phi);
    x = newtonProjectAlongNormal.solve(x);

    Vec3 contactPointFrustum = frustum->getKrPS(zeta);

    g = frustum->getKn(zeta).T() * (contactPointPlate - contactPointFrustum);

    if (g < 0.) {
      //Frustum
      Vec3 rF = frustum->getFrame()->getPosition();
      SqrMat3 AWF = frustum->getFrame()->getOrientation();
      cFrame[ifrustum]->setPosition(rF + AWF * contactPointFrustum);
      setFrustumOrienationKinematics(t, x(0), phi, cFrame);

      //Plate
      cFrame[iplate]->setPosition(rF + AWF * contactPointPlate);
      cFrame[iplate]->getOrientation(false).set(0, -cFrame[ifrustum]->getOrientation(false).col(0));
      cFrame[iplate]->getOrientation(false).set(1, -cFrame[ifrustum]->getOrientation(false).col(1));
      cFrame[iplate]->getOrientation(false).set(2, cFrame[ifrustum]->getOrientation(false).col(2));

      return true;
    }

    return false;
  }

  bool ContactKinematicsPlatePolynomialFrustum::edgeContact(double t, double & g, std::vector<Frame*> &cFrame) {
    cornerPoints[0] = plate->getA();
    cornerPoints[1] = plate->getB();
    cornerPoints[2] = plate->getC();
    cornerPoints[3] = plate->getD();

    for (int i = 0; i < 4; i++) {
      cornerPoints[i] = frustum->getFrame()->getOrientation(t).T() * (plate->getFrame()->getPosition(t) + plate->getFrame()->getOrientation(t) * cornerPoints[i] - frustum->getFrame()->getPosition(t));
    }

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

        Vec3 contactPointPlate = cornerPoints[indi] + t * (cornerPoints[indj] - cornerPoints[indi]);

        const double & x = contactPointPlate(0);
        const double phi = ArcTan(contactPointPlate(1), contactPointPlate(2));
        zeta(0) = x;
        zeta(1) = phi;
        Vec3 contactPointFrustum = frustum->getKrPS(zeta);

        g = frustum->getKn(zeta).T() * (contactPointPlate - contactPointFrustum);
        if (g < 0.) {

          //Frustum
          cFrame[ifrustum]->setPosition(frustum->getFrame()->getPosition() + frustum->getFrame()->getOrientation() * contactPointFrustum);
          setFrustumOrienationKinematics(t, x, phi, cFrame);

          //Plate
          cFrame[iplate]->setPosition(frustum->getFrame()->getPosition() + frustum->getFrame()->getOrientation() * contactPointPlate);
          cFrame[iplate]->getOrientation(false).set(0, -cFrame[ifrustum]->getOrientation(false).col(0));
          cFrame[iplate]->getOrientation(false).set(1, -cFrame[ifrustum]->getOrientation(false).col(1));
          cFrame[iplate]->getOrientation(false).set(2, cFrame[ifrustum]->getOrientation(false).col(2));

          //save values for next search
          ilast = indi;
          return true;
        }
      }
    }

    return false;
  }

  void ContactKinematicsPlatePolynomialFrustum::updateg(double t, double & g, std::vector<Frame*> &cFrame, int index) {
    /*Geometry*/
    //plate
    Vec3 R_cen_A = plate->getFrame()->getPosition(t); //center of plate
    Vec3 NormAxis_A = plate->getFrame()->getOrientation().col(0); // normal of plate in inertial FR

    /*construct the sphere enclosing the frustum*/
    //take center of the frustum as center of the sphere
    Vec3 R_cen_S = frustum->getEnclosingSphereCenter(t);

    //search the radius of the circumsphere
    double rad_sph = frustum->getEnclosingSphereRadius();

    //compute distance between the center of sphere and the plane where the plate lies
    Vec3 Dis_SP = R_cen_S - R_cen_A; //distance vector
    double dis_SP = Dis_SP.T() * NormAxis_A;

    /*if distance>radius, no intersection, do nothing*/

    if (dis_SP <= rad_sph) {
      // find the intersection circle between the sphere and the plane
      //radius of the circle
      double rad_inscir = sqrt(pow(rad_sph, 2) - pow(dis_SP, 2));
      //position of circle center (projection of the sphere center onto the plate/plane)
      Vec3 R_cen_inscir = R_cen_S - (dis_SP * NormAxis_A);

      //if the plate intersects this circle on plane, then search for the contact point
      if (plate->Intersect_Circle(rad_inscir, R_cen_inscir)) {
        //Check for contact point in the plate
        if (cpLocationInPlate(t, g, cFrame)) {
          ilast = -1;
          return;
        }
        else if (gridContact(t, g, cFrame)) {
          ilast = -1;
          return;
        }
        else if (edgeContact(t, g, cFrame)) {
          return;
        }
      }
    }
    g = 1.;
  }

  void ContactKinematicsPlatePolynomialFrustum::setGridSizeY(int gridSizeY_) {
    if (gridSizeY_ < 2) {
      cout << "Grid Size \"" << gridSizeY_ << "\" to small in Y direction. Setting to 2" << endl;
      gridSizeY_ = 2;
    }
    gridSizeY = gridSizeY_;
  }

  void ContactKinematicsPlatePolynomialFrustum::setGridSizeZ(int gridSizeZ_) {
    if (gridSizeZ_ < 2) {
      cout << "Grid Size \"" << gridSizeZ_ << "\" to small in Y direction. Setting to 2" << endl;
      gridSizeZ_ = 2;
    }
    gridSizeZ = gridSizeZ_;
  }

  Vec3 ContactKinematicsPlatePolynomialFrustum::computeContourPointFrustum(const double & x, const Vec3 & n) {
    Vec3 contourPoint(NONINIT);
    double y1 = -(frustum->getValue(x)) * (frustum->getValueD1(x)) * (n(1)) / (n(0));     //y=-f(x)*f'(x)*v2/v1
    double z1 = -(frustum->getValue(x)) * (frustum->getValueD1(x)) * (n(2)) / (n(0));     //z=-f(x)*f'(x)*v3/v1

    contourPoint(0) = x;
    contourPoint(1) = y1;
    contourPoint(2) = z1;

    return contourPoint;
  }

  Vec3 ContactKinematicsPlatePolynomialFrustum::computeContourPoint(double t, const double & x, const Vec3 & n) {
    return frustum->getFrame()->getPosition(t) + frustum->getFrame()->getOrientation(t) * computeContourPointFrustum(x, n);
  }

  int ContactKinematicsPlatePolynomialFrustum::checkPossibleContactPoint(double t, const double & x, const Vec3 & n) {
    if (plate->PointInPlate(computeContourPoint(t, x, n))) {
      return 1; //point is possible to be the contact point
    }

    return 0;
  }

  double ContactKinematicsPlatePolynomialFrustum::distance2Plate(double t, const Vec3 & point) {
    return plate->getFrame()->getOrientation(t).col(0).T() * (point - plate->getFrame()->getPosition(t));
  }

  void ContactKinematicsPlatePolynomialFrustum::updateGrid() {
    gridPoints.clear();

    Vec3 dirY = plate->getD() - plate->getA();
    Vec3 dirZ = plate->getB() - plate->getA();

    for (int i = 0; i < gridSizeY; i++) {
      Vec3 startPoint = plate->getA() + dirZ * ((double) i / (double) (gridSizeZ - 1));
      std::vector<fmatvec::Vec3> linePoints;
      for (int j = 0; j < gridSizeZ; j++) {
        linePoints.push_back(startPoint + dirY * ((double) j / (double) (gridSizeY - 1)));
      }
      gridPoints.push_back(linePoints);
    }
  }

} /* namespace MBSim */