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
 * Contact: rzander@users.berlios.de
 */

#include <config.h>
#include <mbsim/functions_contact.h>
#include <mbsim/utils/nonlinear_algebra.h>

using namespace fmatvec;

namespace MBSim {

  void Contact1sSearch::setEqualSpacing(const int &n, const double &x0, const double &dx) {
    Vec nodesTilde(n+1,NONINIT);
    for(int i=0;i<=n;i++) nodesTilde(i) = x0 + i*dx;
    nodes = nodesTilde;
  }

  double Contact1sSearch::slv() {
    Vec alphaC(nodes.size()-1); // root
    Vec gbuf; // buffering distances for comparison in Regula-Falsi
    int nRoots = 0; // number of found roots

    if(!searchAll) {
      NewtonMethod rf(func,jac);
      alphaC(0) = rf.solve(s0);
      if(rf.getInfo() == 0 && alphaC(0) >= nodes(0) && alphaC(0) <= nodes(nodes.size()-1)) { // converged
        nRoots = 1;
      }
      else {
        searchAll = true;
      }
    }

    if(searchAll) { 
      RegulaFalsi rf(func);
      gbuf = Vec(alphaC.size());

      for(int i=0;i<nodes.size()-1; i++) {
        double fa = (*func)(nodes(i));
        double fb = (*func)(nodes(i+1));
        if(fa*fb < 0) {
          alphaC(nRoots) = rf.solve(nodes(i),nodes(i+1));
          gbuf(nRoots)   = (*func)[alphaC(nRoots)] ;
          nRoots++;
        } 
        else if(fabs(fa) < epsroot()) {
          alphaC(nRoots) = nodes(i);
          gbuf(nRoots)   = (*func)[alphaC(nRoots)] ;
          nRoots++;
        } 
        else if(fabs(fb) < epsroot()) {
          alphaC(nRoots) = nodes(i+1);
          gbuf(nRoots)   = (*func)[alphaC(nRoots)] ;
          nRoots++;
        }
      }
    }

    if(nRoots > 1) { // compare roots with respect to contact distances
      double g_=1e10;
      double sMin = 1.0;

      for(int i=0;i<nRoots;i++)
        if(gbuf(i) < g_) {
          sMin = alphaC(i);
          g_=gbuf(i);
        }
      return sMin;
    }
    else { // at most one root (even if no root: solution is signalising OutOfBounds)
      return alphaC(0);
    }
  }

  Mat Contact1sSearch::slvAll() {
    RegulaFalsi rf(func);
    Vec alphaC(nodes.size()-1);
    Vec gbuf  (alphaC.size());
    int nRoots = 0;

    for(int i=0;i<nodes.size()-1; i++) {
      double fa = (*func)(nodes(i));
      double fb = (*func)(nodes(i+1));
      if(fa*fb < 0) {
        alphaC(nRoots) = rf.solve(nodes(i),nodes(i+1));
        gbuf(nRoots)   = (*func)[alphaC(nRoots)] ;
        nRoots++;
      } 
      else if(fabs(fa) < epsroot() && fabs(fb) < epsroot()) {
        alphaC(nRoots) = 0.5 * ( nodes(i) + nodes(i+1) ) ;
        gbuf(nRoots)   = (*func)[alphaC(nRoots)] ;
        nRoots++;
      } 
      else if(fabs(fa) < epsroot()) {
        alphaC(nRoots) = nodes(i);
        gbuf(nRoots)   = (*func)[alphaC(nRoots)] ;
        nRoots++;
      } 
      else if(fabs(fb) < epsroot()) {
        alphaC(nRoots) = nodes(i+1);
        gbuf(nRoots)   = (*func)[alphaC(nRoots)] ;
        nRoots++;
      }
    }

    Mat results(nRoots,2);
    results.col(0) = alphaC(Index(0,nRoots-1));
    results.col(1) = gbuf  (Index(0,nRoots-1));

    return results;
  }
}

