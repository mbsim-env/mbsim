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
#include "point_polynomialfrustum.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "fmatvec/linear_algebra_double.h"
#include "fmatvec/linear_algebra.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {
  projectPointAlongNormal::projectPointAlongNormal(PolynomialFrustum * frustum) :
      frustum(frustum), referencePoint(), phi(0.) {
  }

  projectPointAlongNormal::~projectPointAlongNormal() {

  }

  void projectPointAlongNormal::setUpSystemParamters(const Vec3 & referencePoint_, const double & phi_) {
    referencePoint = referencePoint_;
    phi = phi_;
  }

  Vec projectPointAlongNormal::operator ()(const Vec & xin, const void *) {
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

  projectPointAlongNormalJacobian::projectPointAlongNormalJacobian(PolynomialFrustum * frustum) :
      frustum(frustum), referencePoint(), phi(0.) {
  }

  projectPointAlongNormalJacobian::~projectPointAlongNormalJacobian() {
  }

  void projectPointAlongNormalJacobian::setUpSystemParamters(const Vec3 & referencePoint_, const double & phi_) {
    referencePoint = referencePoint_;
    phi = phi_;
  }

  SqrMat projectPointAlongNormalJacobian::operator ()(const Vec & xin, const void *) {
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


  ContactKinematicsPointPolynomialFrustum::ContactKinematicsPointPolynomialFrustum() :
      ContactKinematics(), ipoint(-1), ifrustum(-1), point(0), frustum(0), funcProjectAlongNormal(0), newtonProjectAlongNormal(), jacobianProjectAlongNormal(0), criteriaProjectAlongNormal(), dampingProjectAlongNormal() {
  }

  ContactKinematicsPointPolynomialFrustum::~ContactKinematicsPointPolynomialFrustum() {
    delete funcProjectAlongNormal;
    delete jacobianProjectAlongNormal;
  }

  void ContactKinematicsPointPolynomialFrustum::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Area*>(contour[0])) {
      ipoint = 0;
      ifrustum = 1;
      point = static_cast<Area*>(contour[0]);
      frustum = static_cast<PolynomialFrustum*>(contour[1]);
    }
    else {
      ipoint = 1;
      ifrustum = 0;
      point = static_cast<Area*>(contour[1]);
      frustum = static_cast<PolynomialFrustum*>(contour[0]);
    }

    /*Set Up system for projection of contact point onto frustum surface*/
    funcProjectAlongNormal = new projectPointAlongNormal(frustum);
    newtonProjectAlongNormal.setFunction(funcProjectAlongNormal);
    jacobianProjectAlongNormal = new projectPointAlongNormalJacobian(frustum);
    newtonProjectAlongNormal.setJacobianFunction(jacobianProjectAlongNormal);
    newtonProjectAlongNormal.setCriteriaFunction(&criteriaProjectAlongNormal);
    newtonProjectAlongNormal.setDampingFunction(&dampingProjectAlongNormal);
  }

  void ContactKinematicsPointPolynomialFrustum::setFrustumOrienationKinematics(const double & x, const double & phi, ContourPointData * cpData) {
    SqrMat3 AWF = frustum->getFrame()->getOrientation();
    cpData[ifrustum].getFrameOfReference().getOrientation().set(0, AWF * frustum->computeNormal(x, phi));
    cpData[ifrustum].getFrameOfReference().getOrientation().set(1, -AWF * frustum->computeTangentRadial(x, phi));
    cpData[ifrustum].getFrameOfReference().getOrientation().set(2, AWF * frustum->computeTangentAzimuthal(x, phi));
  }

  void ContactKinematicsPointPolynomialFrustum::updateg(Vec & g, ContourPointData * cpData, int index) {
    /*Geometry*/
    //point in frustum-coordinates
    Vec3 rPoint = frustum->getFrame()->getOrientation().T() * (point->getFrame()->getPosition() - frustum->getFrame()->getPosition());

    const double & h = rPoint(0);
    //radial position of point
    const double r = sqrt(pow(rPoint(1), 2) + pow(rPoint(2), 2));
    const double R = frustum->getValue(h);

    if(h >= 0 and h <= frustum->getHeight() and r <= R) {
      double phi = ArcTan(rPoint(1), rPoint(2));
      funcProjectAlongNormal->setUpSystemParamters(rPoint, phi);
      jacobianProjectAlongNormal->setUpSystemParamters(rPoint, phi);
      Vec x(1,INIT,h);
      x = newtonProjectAlongNormal.solve(x);

      Vec3 contactPointFrustum = frustum->computePoint(x(0), phi);

      g(0) = frustum->computeNormal(x(0), phi).T() * (rPoint - contactPointFrustum);

      if (g(0) < 0.) {
        //Frustum
        Vec3 rF = frustum->getFrame()->getPosition();
        SqrMat3 AWF = frustum->getFrame()->getOrientation();
        cpData[ifrustum].getFrameOfReference().getPosition() = rF + AWF * contactPointFrustum;
        setFrustumOrienationKinematics(x(0), phi, cpData);

        //Area
        cpData[ipoint].getFrameOfReference().getPosition() = rF + AWF  * rPoint;
        cpData[ipoint].getFrameOfReference().getOrientation().set(0, -cpData[ifrustum].getFrameOfReference().getOrientation().col(0));
        cpData[ipoint].getFrameOfReference().getOrientation().set(1, -cpData[ifrustum].getFrameOfReference().getOrientation().col(1));
        cpData[ipoint].getFrameOfReference().getOrientation().set(2, cpData[ifrustum].getFrameOfReference().getOrientation().col(2));

      }
    }
    else {
      g(0) = 1.;
    }
  }




} /* namespace MBSim */
