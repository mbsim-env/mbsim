/* Copyright (C) 2004-2006  Roland Zander, Martin Förg
 
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
 * Contact:
 *   rzander@users.berlios.de
 *
 */

#include <config.h>
#include "functions_contact.h"
#include "nonlinear_algebra.h"

namespace MBSim {

  void Contact1sSearch::setEqualSpacing(const int &n, const double &x0, const double &dx)
  {
  	// Set equally distanced nodes
  	// INPUT	n	nnumber of search areas
  	//			x0	starting point
  	//			dx	width of search areas
  	
    Vec nodesTilde(n+1,NONINIT);
    for(int i=0;i<=n;i++) nodesTilde(i) = x0 + i*dx;
    nodes = nodesTilde;
  }

  double Contact1sSearch::slv()
  {
  	// Find point with minimal distance at contour-parameter s
  
    Vec alphaC(nodes.size()-1); // root
    Vec gbuf; // buffering distances for comparison in Regula-Falsi
    int nRoots = 0; // number of found roots
    
    if(!searchAll) {
//      if(INFO) {cout << "INFO: Using Newton-Method with Initial Value s0 = "<< s0 << endl;}
      NewtonMethod rf(func);
      alphaC(0) = rf.slv(s0);
      if(rf.getInfo() == 0 && alphaC(0) >= nodes(0) && alphaC(0) <= nodes(nodes.size()-1)) { // converged
		nRoots = 1;
      }
      else {
		searchAll = true;
      }
    }
    
    if(searchAll) { 
//      if(INFO) {cout << "INFO: Using Regula-Falsi in Areas " << trans(nodes) << endl;}
      RegulaFalsi rf(func);
      gbuf = Vec(nodes.size()-1);

      for(int i=0;i<nodes.size()-1; i++) {
		double fa = (*func)(nodes(i));
		double fb = (*func)(nodes(i+1));
		if(fa*fb < 0) {
		  alphaC(nRoots) = rf.slv(nodes(i),nodes(i+1));
		  gbuf(nRoots)   = (*func)[alphaC(nRoots)] ;
		  nRoots++;
		} 
		else if(fa == 0) {
		  alphaC(nRoots) = nodes(i);
		  gbuf(nRoots)   = (*func)[alphaC(nRoots)] ;
		  nRoots++;
		} 
		else if(fb == 0) {
		  alphaC(nRoots) = nodes(i+1);
		  gbuf(nRoots)   = (*func)[alphaC(nRoots)] ;
		  nRoots++;
		}
      }
    }

    if(nRoots > 1) { // compare roots with respect to contact distances
      double g_=1e10;
      double sMin;

      for(int i=0;i<nRoots;i++)
		if(gbuf(i) < g_) {
	  		sMin = alphaC(i);
//	  		if(WARN && g_ < 0. && gbuf(i) < 0.) {cout << "WARN: Two contact points detected!" << endl;};
	  		g_=gbuf(i);
		}
      return sMin;
    }
    else { // at most one root (even if no root: solution is signalising OutOfBounds)
      return alphaC(0);
    }
  }
  
}
