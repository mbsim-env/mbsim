/* Copyright (C) 2004-2009 MBSim Development Team
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
#include "mbsim/contact_kinematics/circle_spatialcontour.h"
#include "mbsim/frames/contour_frame.h"
#include "mbsim/contours/spatial_contour.h"
#include "mbsim/contours/circle.h"
#include "mbsim/utils/rotarymatrices.h"
#include <fmatvec/linear_algebra_fixed.h>

using namespace fmatvec;
using namespace std;

namespace MBSim {

  ContactKinematicsCircleSpatialContour::ContactKinematicsCircleSpatialContour() :
    circlePosition(NONINIT),
    circleOrientation(NONINIT),
    spatialContourPosition(NONINIT),
    spatialContourOrientation(NONINIT),
    parS(NONINIT) {}

  void ContactKinematicsCircleSpatialContour::setInitialGuess(const MatV &zeta0_) {
    if(zeta0_.rows()!=3 || zeta0_.cols()!=1)
      throw runtime_error("Initial guess value must be a vector of 3 entries [circleAngle0; xi0; eta0].");
    par.ref(curis, RangeV(0,2));
    parSol.ref(nextis, RangeV(0,2));
    par(0) = zeta0_(0,0);
    par(1) = zeta0_(1,0);
    par(2) = zeta0_(2,0);
  }

  void ContactKinematicsCircleSpatialContour::assignContours(const vector<Contour*> &contour) {
    if(dynamic_cast<Circle*>(contour[0])) {
      icircle = 0;
      ispatialContour = 1;
      circle = static_cast<Circle*>(contour[0]);
      spatialContour = static_cast<SpatialContour*>(contour[1]);
    } 
    else {
      icircle = 1;
      ispatialContour = 0;
      circle = static_cast<Circle*>(contour[1]);
      spatialContour = static_cast<SpatialContour*>(contour[0]);
    }



    auto scSymFunc = dynamic_cast<SymbolicFunction<Vector<Fixed<3>, double> (Vector<Fixed<2>, double>)>*>
                   (spatialContour->getContourFunction());
    if(!scSymFunc)
      throw runtime_error("Only SymbolicFunction's as Spatial Contour can be handled for the Circle - Spatial Contour contact for now!");

    auto zeta = scSymFunc->getIndependentVariable();
    IndependentVariable alpha;
    Vector<Fixed<3>, SymbolicExpression> cir({
      circle->getRadius() * cos(alpha),
      circle->getRadius() * sin(alpha),
      0
    });
    W_r_WC = circlePosition + circleOrientation * cir;
    W_r_WS = spatialContourPosition + spatialContourOrientation * scSymFunc->getDependentFunction();
    auto W_r_CS = W_r_WS - W_r_WC;

    // We want to minimize, if contact is open, or maximize, if contact is closed, the local distance |W_r_CS| between the curves.
    // The condition for this is that W_r_CS is orthogonal to the spatial contour tangents (t0S, t1S) and the circle tangent (tC).
    // If the contact is exact closed (without penetration) then W_r_CS = 0 and has not direction for the orthogonal condition.
    // The solution is singular at this point. To avoid this we use instead of W_r_CS a vector r = W_r_CS + nS with nS the normal
    // of the spatial contour. At a solution W_r_CS is colinear to nS (its orthogonal to t0S and t1S). Hence adding any vector
    // alpha * nS does not change the solution but avoids that the system is sigular if |W_r_CS| = 0.
    // Doing so we don't remove the singularity but just move it to W_r_CS = -nS. But is's quite unlikey that the system is solved
    // at exactly this point (where it is quite likely that the system is solved at W_r_CS = 0).
    t0S = -parDer(W_r_WS, zeta(0));
    t1S =  parDer(W_r_WS, zeta(1));
    auto nS = crossProduct(t1S, t0S);
    tC = parDer(W_r_WC, alpha);
    auto r = W_r_CS + nS;
    Vector<Fixed<3>, SymbolicExpression> rootFunctionS({
      trans(t0S) * r, // = 0 via root finding = orthogonal
      trans(t1S) * r, // = 0 via root finding = orthogonal
      trans(tC ) * r, // = 0 via root finding = orthogonal
    });

    parS = Vector<Fixed<3>, IndependentVariable>({ alpha, zeta(0), zeta(1) });
    rootFunction.setIndependentVariable(parS);
    rootFunction.setDependentFunction(rootFunctionS);
    rootFinding.setFunction(&rootFunction);

    jacobianFunction.setIndep(parS);
    jacobianFunction.setDep(SquareMatrix<Fixed<3>, SymbolicExpression>(parDer(rootFunctionS, parS)));
    rootFinding.setJacobianFunction(&jacobianFunction);

    criteria.setTolerance(1e-8);
    rootFinding.setCriteriaFunction(&criteria);

    rootFinding.setDampingFunction(&damping);
  }

  void ContactKinematicsCircleSpatialContour::updateg(SingleContact &contact, int i) {
    circlePosition ^= circle->getFrame()->evalPosition();
    circleOrientation ^= circle->getFrame()->evalOrientation();
    spatialContourPosition ^= spatialContour->getFrame()->evalPosition();
    spatialContourOrientation ^= spatialContour->getFrame()->evalOrientation();

    // solve the system using the start value par and write solution to parSol;
    // NOTE: after a sucessfull step the integrator copies parSol to par. This way the start value is update for the
    // next integrator step using the solution of the last successfull intergration step.
    parSol = rootFinding.solve(par);
    if(rootFinding.getInfo()!=0 && rootFinding.getInfo()!=-1)
      throw runtime_error("Root finding in ContactKinematicsCircleSpatialContour failed.");
    parS ^= parSol; // set solution to symbolic parS

    auto W_r_WC_ = eval(W_r_WC);
    auto W_r_WS_ = eval(W_r_WS);
    SqrMat3 T_WS_, T_WC_;
    auto t1S_ = eval(t1S);
    auto exS = crossProduct(t1S_, eval(t0S));
    exS = exS / nrm2(exS);
    auto eyS = t1S_ / nrm2(t1S_);
    auto exC = -exS;
    auto tC_ = eval(tC);
    auto ezC = tC_ / nrm2(tC_);
    auto eyC = crossProduct(ezC, exC);
    T_WS_.set(0, exS);
    T_WS_.set(1, eyS);
    T_WS_.set(2, crossProduct(exS, eyS));
    T_WC_.set(0, exC);
    T_WC_.set(1, eyC);
    T_WC_.set(2, ezC);
    contact.getContourFrame(icircle)->setPosition(W_r_WC_);
    contact.getContourFrame(ispatialContour)->setPosition(W_r_WS_);
    contact.getContourFrame(icircle)->setOrientation(T_WC_);
    contact.getContourFrame(ispatialContour)->setOrientation(T_WS_);
    contact.getGeneralizedRelativePosition(false)(0) = -trans(W_r_WS_ - W_r_WC_) * exS;
  }

}

