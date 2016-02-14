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
 * Contact: martin.o.foerg@googlemail.com
 */

#include <config.h>
#include <mbsim/functions_contact.h>
#include <mbsim/contours/contour1s.h>
#include <mbsim/contours/point.h>
#include "mbsim/contours/line.h"
#include <mbsim/contours/solid_circle.h>
#include "mbsim/contours/hollow_circle.h"
#include "mbsim/contours/circle.h"
#include "mbsim/contours/frustum2d.h"
#include "mbsim/contours/edge.h"
#include "mbsim/contours/frustum.h"
#include "mbsim/contours/plane.h"
#include "mbsim/contours/contour2s.h"
#include "mbsim/contours/planewithfrustum.h"
#include "mbsim/contours/contour_interpolation.h"
#include "mbsim/contours/contour_quad.h"
#include "mbsim/contours/cuboid.h"
#include "mbsim/contours/compound_contour.h"
#include "mbsim/frame.h"
#include "mbsim/mbsim_event.h"
#include <mbsim/utils/nonlinear_algebra.h>
#include <mbsim/numerics/nonlinear_algebra/multi_dimensional_newton_method.h>
using namespace fmatvec;

namespace MBSim {

  double FuncPairContour1sPoint::operator()(const double &alpha) {
    zeta(0) = alpha;
    Vec3 Wd = getWrD(alpha);
    Vec3 Wt = contour->getWu(t,zeta);
    return Wt.T() * Wd;
  }

  Vec3 FuncPairContour1sPoint::getWrD(const double &alpha) {
    //if(fabs(alpha-cp.getLagrangeParameterPosition()(0))>epsroot()) { TODO this is not working in all cases
    zeta(0) = alpha;
    return contour->getPosition(t,zeta) - point->getFrame()->getPosition(t);
  }

  Vec2 FuncPairContour2sPoint::operator()(const Vec2 &alpha) {  // Vec2: U and V direction
    Vec3 Wd = getWrD(alpha);
    Vec3 Wt1 = contour->getWu(t,alpha);
    Vec3 Wt2 = contour->getWv(t,alpha);
    Vec2 Wt(NONINIT);  // TODO:: check this?
    Wt(0) = Wt1.T() * Wd; // the projection of distance vector Wd into the first tangent direction: scalar value
    Wt(1) = Wt2.T() * Wd; // the projection of distance vector Wd into the second tangent direction: scalar value
    return Wt;
  }

  Vec3 FuncPairContour2sPoint::getWrD(const Vec2 &alpha) {
    return contour->getPosition(t,alpha) - point->getFrame()->getPosition(t);
  }

  void Contact1sSearch::setEqualSpacing(const int &n, const double &x0, const double &dx) {
    Vec nodesTilde(n + 1, NONINIT);
    for (int i = 0; i <= n; i++)
      nodesTilde(i) = x0 + i * dx;
    nodes = nodesTilde;
  }

  double FuncPairContour1sHollowCircle::operator()(const double &alpha) {
    zeta(0) = alpha;
    Vec3 Wd = getWrD(alpha);
    Vec3 Wt = contour->getWu(t,zeta);
    return Wt.T() * Wd;
  }

  Vec3 FuncPairContour1sHollowCircle::getWrD(const double &alpha) {
    //if(fabs(alpha-cp.getLagrangeParameterPosition()(0))>epsroot()) { TODO this is not working in all cases
    zeta(0) = alpha;
    return contour->getPosition(t,zeta) - (circle->getFrame()->getPosition(t) + circle->getRadius() * contour->getWn(t,zeta));
  }

  Vec2 FuncPairPointContourInterpolation::operator()(const Vec2 &alpha) {
    return (contour->getWu(t,alpha)).T() * (contour->getPosition(t,alpha) - point->getFrame()->getPosition(t));
  }

  Vec3 FuncPairPointContourInterpolation::getWrD(const Vec2 &alpha) {
    return contour->getPosition(t,alpha) - point->getFrame()->getPosition(t);
  }

  double FuncPairContour1sSolidCircle::operator()(const double &alpha) {
    zeta(0) = alpha;
    Vec3 Wd = getWrD(alpha);
    Vec3 Wt = contour1s->getWu(t,zeta);
    return Wt.T() * Wd;
  }

  Vec3 FuncPairContour1sSolidCircle::getWrD(const double &alpha) {
    zeta(0) = alpha;
    return contour1s->getPosition(t,zeta) - (circle->getFrame()->getPosition(t) - circle->getRadius() * contour1s->getWn(t,zeta));
  }

  double Contact1sSearch::slv() {
    Vec alphaC(nodes.size() - 1); // root
    Vec gbuf; // buffering distances for comparison in Regula-Falsi
    int nRoots = 0; // number of found roots

    if (!searchAll) {
      NewtonMethod rf(func, jac);
      alphaC(0) = rf.solve(s0);
      if (rf.getInfo() == 0 && alphaC(0) >= nodes(0) && alphaC(0) <= nodes(nodes.size() - 1)) { // converged
        nRoots = 1;
      }
      else {
        searchAll = true;
      }
    }

    if (searchAll) {
      RegulaFalsi rf(func);
      gbuf >> Vec(alphaC.size());

      for (int i = 0; i < nodes.size() - 1; i++) {
        double fa = (*func)(nodes(i));
        double fb = (*func)(nodes(i + 1));
        if (fa * fb < 0) {
          alphaC(nRoots) = rf.solve(nodes(i), nodes(i + 1));
          gbuf(nRoots) = (*func)[alphaC(nRoots)];
          nRoots++;
        }
        else if (fabs(fa) < epsroot()) {
          alphaC(nRoots) = nodes(i);
          gbuf(nRoots) = (*func)[alphaC(nRoots)];
          nRoots++;
        }
        else if (fabs(fb) < epsroot()) {
          alphaC(nRoots) = nodes(i + 1);
          gbuf(nRoots) = (*func)[alphaC(nRoots)];
          nRoots++;
        }
      }
    }

    if (nRoots > 1) { // compare roots with respect to contact distances
      double g_ = 1e10;
      double sMin = 1.0;

      for (int i = 0; i < nRoots; i++)
        if (gbuf(i) < g_) {
          sMin = alphaC(i);
          g_ = gbuf(i);
        }
      return sMin;
    }
    else { // at most one root (even if no root: solution is signalising OutOfBounds)
      return alphaC(0);  // stores the value of largrange parameter of the potential contact point.
    }
  }

  Mat Contact1sSearch::slvAll() {
    RegulaFalsi rf(func);
    Vec alphaC(nodes.size() - 1);
    Vec gbuf(alphaC.size());
    int nRoots = 0;

    for (int i = 0; i < nodes.size() - 1; i++) {
      double fa = (*func)(nodes(i));
      double fb = (*func)(nodes(i + 1));
      if (fa * fb < 0) {
        alphaC(nRoots) = rf.solve(nodes(i), nodes(i + 1));
        gbuf(nRoots) = (*func)[alphaC(nRoots)];
        nRoots++;
      }
      else if (fabs(fa) < epsroot() && fabs(fb) < epsroot()) {
        alphaC(nRoots) = 0.5 * (nodes(i) + nodes(i + 1));
        gbuf(nRoots) = (*func)[alphaC(nRoots)];
        nRoots++;
      }
      else if (fabs(fa) < epsroot()) {
        alphaC(nRoots) = nodes(i);
        gbuf(nRoots) = (*func)[alphaC(nRoots)];
        nRoots++;
      }
      else if (fabs(fb) < epsroot()) {
        alphaC(nRoots) = nodes(i + 1);
        gbuf(nRoots) = (*func)[alphaC(nRoots)];
        nRoots++;
      }
    }

    Mat results(nRoots, 2);
    results.col(0) = alphaC(Index(0, nRoots - 1));
    results.col(1) = gbuf(Index(0, nRoots - 1));

    return results;
  }

  void Contact2sSearch::setEqualSpacing(const int nU, const int nV, const double U0, const double V0, const double dU, const double dV) {
    Vec nodesTilde(nU + 1, NONINIT);
    for (int i = 0; i <= nU; i++)
      nodesTilde(i) = U0 + i * dU;
    nodesU = nodesTilde;

    nodesTilde.resize(nV + 1, NONINIT);
    for (int i = 0; i <= nV; i++)
      nodesTilde(i) = V0 + i * dV;
    nodesV = nodesTilde;
  }

  Vec2 Contact2sSearch::slv() {
    std::vector<Vec> alphaC((nodesU.size() - 1) * (nodesV.size() - 1)); // stores the largrange position of all possible candidates of the contact points searched by root finding function
    Vec gbuf; // stores the norm 2 distance of all contact point candidates
    int nRoots = 0; // number of found roots

    if (!searchAll) {
      MultiDimensionalNewtonMethod rf;
      rf.setFunction(func);
      NumericalNewtonJacobianFunction numNewJaCFun;
      rf.setJacobianFunction(&numNewJaCFun);
      std::map<Index, double> tolerances;
      tolerances.insert(std::pair<Index, double>(Index(0, 1), 1e-8));
      LocalResidualCriteriaFunction locredcritfun(tolerances);
      rf.setCriteriaFunction(&locredcritfun);

      alphaC.at(0) = rf.solve(s0);
      if (rf.getInfo() == 0 && alphaC.at(nRoots)(0) >= nodesU(0) && alphaC.at(nRoots)(0) <= nodesU(nodesU.size() - 1) && alphaC.at(nRoots)(1) >= nodesV(0) && alphaC.at(nRoots)(1) <= nodesV(nodesV.size() - 1)) { // converged
        nRoots = 1;
      }
      else {
        searchAll = true;
      }
    }

    if (searchAll) {
      MultiDimensionalNewtonMethod rf;
      rf.setFunction(func);
      NumericalNewtonJacobianFunction numNewJaCFun;
      rf.setJacobianFunction(&numNewJaCFun);
      std::map<Index, double> tolerances;
      tolerances.insert(std::pair<Index, double>(Index(0, 1), 1e-8));
      LocalResidualCriteriaFunction locredcritfun(tolerances);
      rf.setCriteriaFunction(&locredcritfun);


      gbuf >> Vec(alphaC.size());  // TODO:: ??
      Vec startingValue(2, NONINIT);

      // search in U direction
      for (int i = 0; i < nodesU.size() - 1; i++) {
        Vec temp(2, INIT, 0.);
        temp(0) = nodesU(i);
        double fa = (*func)(temp)(0);
        temp(0) = nodesU(i + 1);
        double fb = (*func)(temp)(0);
        if (fa * fb < 0) {
          startingValue(0) = nodesU(i);  // get the starting value for the U direction
          std::vector<double> startingValueV = searchVdirection(nodesU(i));
          for (size_t j = 0; j < startingValueV.size(); j++) {
            startingValue(1) = startingValueV.at(j);
            alphaC.at(nRoots) = rf.solve(startingValue);
//            if (rf.getInfo() == 0 && alphaC(nRoots)(0) >= nodesU(0) && alphaC(nRoots)(0) <= nodesU(nodesU.size() - 1) && alphaC(nRoots)(1) >= nodesV(0) && alphaC(nRoots)(1) <= nodesV(nodesV.size() - 1)) { // converged
            if (rf.getInfo() == 0) {
              gbuf(nRoots) = (*func)[alphaC.at(nRoots)];
              nRoots++;
            }
          }
        }
        else if (fabs(fa) < epsroot()) {
          startingValue(0) = nodesU(i);  // get the starting value for the U direction
          std::vector<double> startingValueV = searchVdirection(nodesU(i));
          for (size_t j = 0; j < startingValueV.size(); j++) {
            startingValue(1) = startingValueV.at(j);
            alphaC.at(nRoots) = rf.solve(startingValue);
            if (rf.getInfo() == 0) {
              gbuf(nRoots) = (*func)[alphaC.at(nRoots)];
              nRoots++;
            }
          }
        }
        else if (fabs(fb) < epsroot()) {
          startingValue(0) = nodesU(i + 1);  // get the starting value for the U direction
          std::vector<double> startingValueV = searchVdirection(nodesU(i+1));
          for (size_t j = 0; j < startingValueV.size(); j++) {
            startingValue(1) = startingValueV.at(j);
            alphaC.at(nRoots) = rf.solve(startingValue);
            if (rf.getInfo() == 0) {
              gbuf(nRoots) = (*func)[alphaC.at(nRoots)];
              nRoots++;
            }
          }
        }
      } // end of search in U direction
    } // end of search all



    if (nRoots > 1) { // compare roots with respect to contact distances
      double g_ = 1e10;
      Vec2 sMin;

      for (int i = 0; i < nRoots; i++) {
        if (gbuf(i) < g_) {
          sMin = alphaC.at(i);
          g_ = gbuf(i);
        }
      }
      return sMin;
    }
    else { // at most one root (even if no root: solution is signalising OutOfBounds)
      return alphaC.at(0);  // stores the value of largrange parameter of the potential contact point.
    }
  }

  std::vector<double> Contact2sSearch::searchVdirection(double u) {
    std::vector<double> startingValueV;
    for (int i = 0; i < nodesV.size() - 1; i++) {
      Vec temp(2, INIT, 0.);
      temp(0) = u;
      temp(1) = nodesV(i);
      double fa = (*func)(temp)(1);
      temp(1) = nodesV(i + 1);
      double fb = (*func)(temp)(1);
      if (fa * fb < 0) {
        startingValueV.push_back(nodesV(i));  // get the starting value for the U direction
      }
      else if (fabs(fa) < epsroot()) {
        startingValueV.push_back(nodesV(i));  // get the starting value for the U direction
      }
      else if (fabs(fb) < epsroot()) {
        startingValueV.push_back(nodesV(i + 1));  // get the starting value for the U direction
      }
    }
    return startingValueV;
  }

}  // end of the namespace

