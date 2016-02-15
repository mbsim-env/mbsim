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
#include "point_polynomialfrustum.h"
#include "mbsim/frame.h"
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

  Vec projectPointAlongNormal::operator ()(const Vec & xin) {
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

  SqrMat projectPointAlongNormalJacobian::operator ()(const Vec & xin) {
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
      ContactKinematics(), ipoint(-1), ifrustum(-1), point(0), frustum(0), signh(1), funcProjectAlongNormal(0), newtonProjectAlongNormal(), jacobianProjectAlongNormal(0), criteriaProjectAlongNormal(), dampingProjectAlongNormal() {
  }

  ContactKinematicsPointPolynomialFrustum::~ContactKinematicsPointPolynomialFrustum() {
    delete funcProjectAlongNormal;
    delete jacobianProjectAlongNormal;
  }

  void ContactKinematicsPointPolynomialFrustum::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Point*>(contour[0])) {
      ipoint = 0;
      ifrustum = 1;
      point = static_cast<Point*>(contour[0]);
      frustum = static_cast<PolynomialFrustum*>(contour[1]);
    }
    else {
      ipoint = 1;
      ifrustum = 0;
      point = static_cast<Point*>(contour[1]);
      frustum = static_cast<PolynomialFrustum*>(contour[0]);
    }

    signh = sign(frustum->getHeight());

    /*Set Up system for projection of contact point onto frustum surface*/
    funcProjectAlongNormal = new projectPointAlongNormal(frustum);
    newtonProjectAlongNormal.setFunction(funcProjectAlongNormal);
    jacobianProjectAlongNormal = new projectPointAlongNormalJacobian(frustum);
    newtonProjectAlongNormal.setJacobianFunction(jacobianProjectAlongNormal);
    newtonProjectAlongNormal.setCriteriaFunction(&criteriaProjectAlongNormal);
    newtonProjectAlongNormal.setDampingFunction(&dampingProjectAlongNormal);
  }

  void ContactKinematicsPointPolynomialFrustum::updateg(double t, double & g, std::vector<Frame*> &cFrame, int index) {
    /*Geometry*/
    //point in frustum-coordinates
    Vec3 rPoint = frustum->getFrame()->getOrientation(t).T() * (point->getFrame()->getPosition(t) - frustum->getFrame()->getPosition(t));


    const double & h = rPoint(0);
    //radial position of point
    const double r = sqrt(pow(rPoint(1), 2) + pow(rPoint(2), 2));
    const double R = frustum->getValue(h);

    if(signh*h >= 0 and signh*h <= signh*frustum->getHeight() and r <= R) {
      double phi = ArcTan(rPoint(1), rPoint(2));
      funcProjectAlongNormal->setUpSystemParamters(rPoint, phi);
      jacobianProjectAlongNormal->setUpSystemParamters(rPoint, phi);
      Vec x(1,INIT,h);
      x = newtonProjectAlongNormal.solve(x);

      Vec2 zeta(NONINIT);
      zeta(0) = x(0);
      zeta(1) = phi;

      Vec3 contactPointFrustum = frustum->getKrPS(zeta);

      g = frustum->getKn(zeta).T() * (rPoint - contactPointFrustum);

      if (g < 0.) {
        //Frustum
        Vec3 rF = frustum->getFrame()->getPosition();
        SqrMat3 AWF = frustum->getFrame()->getOrientation();
        cFrame[ifrustum]->setPosition(rF + AWF * contactPointFrustum);
        cFrame[ifrustum]->getOrientation(false).set(0, frustum->getWn(t,zeta));
        cFrame[ifrustum]->getOrientation(false).set(1, signh * frustum->getWu(t,zeta));
        cFrame[ifrustum]->getOrientation(false).set(2, signh * frustum->getWv(t,zeta));

        //Point
        cFrame[ipoint]->setPosition(rF + AWF  * rPoint);
        cFrame[ipoint]->getOrientation(false).set(0, -cFrame[ifrustum]->getOrientation(false).col(0));
        cFrame[ipoint]->getOrientation(false).set(1, -cFrame[ifrustum]->getOrientation(false).col(1));
        cFrame[ipoint]->getOrientation(false).set(2, cFrame[ifrustum]->getOrientation(false).col(2));

      }
    }
    else {
      g = 1.;
    }
  }




} /* namespace MBSim */
