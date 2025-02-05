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
#include "mbsim/frames/contour_frame.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/nonlinear_algebra.h"
#include "fmatvec/linear_algebra_double.h"
#include "fmatvec/linear_algebra.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {
  projectPointAlongNormal::projectPointAlongNormal(PolynomialFrustum * frustum) :
      frustum(frustum),  phi(0.) {
  }

  projectPointAlongNormal::~projectPointAlongNormal() = default;

  void projectPointAlongNormal::setUpSystemParamters(const Vec3 & referencePoint_, const double & phi_) {
    referencePoint = referencePoint_;
    phi = phi_;
  }

  Vec projectPointAlongNormal::operator ()(const Vec & xin) {
    Vec result(1, NONINIT);

    const double & x = xin(0);

    const double fd = frustum->evalValueD1(x);

    //to avoid numerical instabilities chose one of the two options
    double tmp = referencePoint(1) /  cos(phi);
    if(fabs(cos(phi)) < 1e-2)
      tmp = referencePoint(2) / sin(phi);

    result(0) = referencePoint(0) - (frustum->evalValue(x) * fd - fd * tmp + x);

    return result;
  }

  projectPointAlongNormalJacobian::projectPointAlongNormalJacobian(PolynomialFrustum * frustum) :
      frustum(frustum),  phi(0.) {
  }

  projectPointAlongNormalJacobian::~projectPointAlongNormalJacobian() = default;

  void projectPointAlongNormalJacobian::setUpSystemParamters(const Vec3 & referencePoint_, const double & phi_) {
    referencePoint = referencePoint_;
    phi = phi_;
  }

  SqrMat projectPointAlongNormalJacobian::operator ()(const Vec & xin) {
    SqrMat Jac(1, NONINIT);

    const double & x = xin(0);

    const double fd = frustum->evalValueD1(x);
    const double fdd = frustum->evalValueD2(x);

    //to avoid numerical instabilities chose one of the two options
    double tmp = referencePoint(1) /  cos(phi);
    if(fabs(cos(phi)) < 1e-2)
      tmp = referencePoint(2) / sin(phi);

    Jac(0,0) =  - fd*fd - frustum->evalValue(x) * fdd + fdd * tmp - 1;

    return Jac;
  }


  ContactKinematicsPointPolynomialFrustum::ContactKinematicsPointPolynomialFrustum()  {
  }

  ContactKinematicsPointPolynomialFrustum::~ContactKinematicsPointPolynomialFrustum() {
    delete funcProjectAlongNormal;
    delete jacobianProjectAlongNormal;
  }

  void ContactKinematicsPointPolynomialFrustum::assignContours(const vector<Contour*> &contour) {
    ContactKinematics::assignContours(contour);
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

  void ContactKinematicsPointPolynomialFrustum::updateg(SingleContact &contact, int i) {
    /*Geometry*/
    //point in frustum-coordinates
    Vec3 rPoint = frustum->getFrame()->evalOrientation().T() * (point->getFrame()->evalPosition() - frustum->getFrame()->evalPosition());


    const double & h = rPoint(0);
    //radial position of point
    const double r = sqrt(pow(rPoint(1), 2) + pow(rPoint(2), 2));
    const double R = frustum->evalValue(h);

    double g;
    if(signh*h >= 0 and signh*h <= signh*frustum->getHeight() and r <= R) {
      double phi = ArcTan(rPoint(1), rPoint(2));
      funcProjectAlongNormal->setUpSystemParamters(rPoint, phi);
      jacobianProjectAlongNormal->setUpSystemParamters(rPoint, phi);
      Vec x(1,INIT,h);
      x = newtonProjectAlongNormal.solve(x);

      Vec2 zeta(NONINIT);
      zeta(0) = x(0);
      zeta(1) = phi;

      Vec3 contactPointFrustum = frustum->evalKrPS(zeta);

      g = frustum->evalKn(zeta).T() * (rPoint - contactPointFrustum);

      if (g < 0.) {
        //Frustum
        Vec3 rF = frustum->getFrame()->getPosition();
        SqrMat3 AWF = frustum->getFrame()->getOrientation();
        contact.getContourFrame(ifrustum)->setPosition(rF + AWF * contactPointFrustum);
        contact.getContourFrame(ifrustum)->getOrientation(false).set(0, frustum->evalWn(zeta));
        contact.getContourFrame(ifrustum)->getOrientation(false).set(1, signh * frustum->evalWu(zeta));
        contact.getContourFrame(ifrustum)->getOrientation(false).set(2, signh * frustum->evalWv(zeta));

        //Point
        contact.getContourFrame(ipoint)->setPosition(rF + AWF  * rPoint);
        contact.getContourFrame(ipoint)->getOrientation(false).set(0, -contact.getContourFrame(ifrustum)->getOrientation(false).col(0));
        contact.getContourFrame(ipoint)->getOrientation(false).set(1, -contact.getContourFrame(ifrustum)->getOrientation(false).col(1));
        contact.getContourFrame(ipoint)->getOrientation(false).set(2, contact.getContourFrame(ifrustum)->getOrientation(false).col(2));
      }
    }
    else
      g = 1.;
    contact.getGeneralizedRelativePosition(false)(0) = g;
  }




} /* namespace MBSim */
