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
#include "sphere_polynomialfrustum.h"
#include "mbsim/contours/polynomial_frustum.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace fmatvec;
using namespace std;

namespace MBSim {

  Vec PolyFurstumSphereContact::operator()(const Vec &x, const void*) {
    double res = x(0) - sphereCenter(0) + frustum->getValueD1(x(0)) * frustum->getValue(x(0)) - frustum->getValueD1(x(0)) *sqrt(sphereCenter(1) * sphereCenter(1) + sphereCenter(2) * sphereCenter(2));

    return Vec(1, INIT, res);;
  }

  SqrMat PolyFurstumSphereContactJacobian::operator()(const Vec &x, const void*) {
    double res = 1 + frustum->getValueD1(x(0)) * frustum->getValueD1(x(0)) + frustum->getValueD2(x(0)) * (frustum->getValue(x(0)) - sqrt(sphereCenter(1) * sphereCenter(1) + sphereCenter(2) * sphereCenter(2)));

    return SqrMat(1, INIT, res);;
  }

  ContactKinematicsSpherePolynomialFrustum::ContactKinematicsSpherePolynomialFrustum() :
        ContactKinematics(), isphere(-1), ifrustum(-1), sphere(0), frustum(0), newton(), func(0), jacobian(0), criteria(new GlobalResidualCriteriaFunction), damping(new StandardDampingFunction), x(1, NONINIT) {

  }

  ContactKinematicsSpherePolynomialFrustum::~ContactKinematicsSpherePolynomialFrustum() {
    delete func;
  }

  void ContactKinematicsSpherePolynomialFrustum::assignContours(const vector<Contour*> &contour) {
    if (dynamic_cast<Sphere*>(contour[0])) {
      isphere = 0;
      ifrustum = 1;
      sphere = static_cast<Sphere*>(contour[0]);
      frustum = static_cast<PolynomialFrustum*>(contour[1]);
    }
    else {
      isphere = 1;
      ifrustum = 0;
      sphere = static_cast<Sphere*>(contour[1]);
      frustum = static_cast<PolynomialFrustum*>(contour[0]);
    }

    func = new PolyFurstumSphereContact(frustum);
    jacobian = new PolyFurstumSphereContactJacobian(frustum);


    newton.setFunction(func);
    newton.setJacobianFunction(jacobian);
    newton.setCriteriaFunction(criteria);
    newton.setDampingFunction(damping);

    //Initialize first guess for solution
    x(0) = frustum->getHeight() / 2.;

  }

  void ContactKinematicsSpherePolynomialFrustum::updateg(Vec & g, ContourPointData * cpData, int index) {
    /*Geometry*/
    //sphere center in coordinates of frustum
    Vec3 rF = frustum->getFrameOfReference()->getPosition();
    Vec3 rS = sphere->getFrame()->getPosition();
    SqrMat3 AWF = frustum->getFrameOfReference()->getOrientation();
    Vec3 COG_S = AWF.T() * (rS - rF);

    /*construct the sphere enclosing the frustum*/
    //take center of the frustum as center of the sphere
    Vec3 COG_FencS = AWF.T() * (frustum->getEnclosingSphereCenter() - rF);

    //search the radius of the circumsphere
    double rad_sph = frustum->getEnclosingSphereRadius();

    //compute distance between the center of sphere and the center of the enclosing sphere of the frustum
    double dis_SP = nrm2(COG_FencS - COG_S);

    if (dis_SP <= rad_sph + sphere->getRadius()) {
      func->setCenter(COG_S);
      jacobian->setCenter(COG_S);

      x = newton.solve(x);

      if(newton.getInfo() == 0) {
        if(x(0) > 0 and x(0) < frustum->getHeight()) {
          double phi = ArcTan(COG_S(1), COG_S(2));

          //Orientation
          cpData[ifrustum].getFrameOfReference().getOrientation().set(0, AWF * frustum->computeNormal(x(0), phi));
          cpData[ifrustum].getFrameOfReference().getOrientation().set(1, - AWF * frustum->computeTangentRadial(x(0), phi));
          cpData[ifrustum].getFrameOfReference().getOrientation().set(2, AWF * frustum->computeTangentAzimuthal(x(0), phi));

          cpData[isphere].getFrameOfReference().getOrientation().set(0, - cpData[ifrustum].getFrameOfReference().getOrientation().col(0));
          cpData[isphere].getFrameOfReference().getOrientation().set(1, - cpData[ifrustum].getFrameOfReference().getOrientation().col(1));
          cpData[isphere].getFrameOfReference().getOrientation().set(2, cpData[ifrustum].getFrameOfReference().getOrientation().col(2));

          //Position
          cpData[ifrustum].getFrameOfReference().getPosition() = rF + AWF * frustum->computePoint(x(0), phi);
          cpData[isphere].getFrameOfReference().getPosition() = rS + cpData[isphere].getFrameOfReference().getOrientation().col(0) * sphere->getRadius();

          //Distance
          g(0) = cpData[ifrustum].getFrameOfReference().getOrientation().col(0).T() * (cpData[isphere].getFrameOfReference().getPosition() - cpData[ifrustum].getFrameOfReference().getPosition());

          return;
        }
      }
    }
    g(0) = 1.;
  }
}/* namespace MBSim */
