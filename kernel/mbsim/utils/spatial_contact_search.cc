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
#include "mbsim/utils/spatial_contact_search.h"
#include "mbsim/functions/contact/distance_function.h"
#include "mbsim/utils/eps.h"
#include "mbsim/utils/nonlinear_algebra.h"

using namespace fmatvec;

namespace MBSim {

  void SpatialContactSearch::setEqualSpacing(const int nU, const int nV, const double U0, const double V0, const double dU, const double dV) {
    Vec nodesTilde(nU + 1, NONINIT);
    for (int i = 0; i <= nU; i++)
      nodesTilde(i) = U0 + i * dU;
    nodesU = nodesTilde;

    nodesTilde.resize(nV + 1, NONINIT);
    for (int i = 0; i <= nV; i++)
      nodesTilde(i) = V0 + i * dV;
    nodesV = nodesTilde;
  }

  Vec2 SpatialContactSearch::slv() {
    std::vector<Vec> alphaC((nodesU.size() - 1) * (nodesV.size() - 1)); // stores the largrange position of all possible candidates of the contact points searched by root finding function
    Vec gbuf; // stores the norm 2 distance of all contact point candidates
    int nRoots = 0; // number of found roots

    if (!searchAll) {
      MultiDimNewtonMethod rf(func, jac);
      alphaC[0] = rf.solve(s0);
      if (rf.getInfo() == 0)
        nRoots = 1;
      else
        searchAll = true;
    }

    if (searchAll) {
      MultiDimNewtonMethod rf(func, jac);

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
        else if (fabs(fa) < epsroot) {
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
        else if (fabs(fb) < epsroot) {
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
      return alphaC[0];  // stores the value of largrange parameter of the potential contact point.
    }
  }

  std::vector<double> SpatialContactSearch::searchVdirection(double u) {
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
      else if (fabs(fa) < epsroot) {
        startingValueV.push_back(nodesV(i));  // get the starting value for the U direction
      }
      else if (fabs(fb) < epsroot) {
        startingValueV.push_back(nodesV(i + 1));  // get the starting value for the U direction
      }
    }
    return startingValueV;
  }

}
