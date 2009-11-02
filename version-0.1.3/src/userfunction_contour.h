/* Copyright (C) 2004-2006  Robert Huber
 
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
#ifndef USERFUNCTION_CONTOUR_H
#define USERFUNCTION_CONTOUR_H

#include <userfunction.h>
#include <ppolynom.h>
#include <contour_pdata.h>

#include <fmatvec.h>

using namespace fmatvec;

namespace MBSim {

  //================================================================================================================================
  /** userFunction for Contours: Contourpoint and trihedral (T,N,B) *
   * Parent class */
  class  UserFunctionContour1s : public UserFunction {
    protected:
      double alphaStart, alphaEnd;
    public:
      UserFunctionContour1s(){};
      virtual Vec computeT(double alpha)   { Vec T = diff1(alpha); T/=nrm2(T); return T;}
      virtual Vec computeB(double alpha)   { Vec B = crossProduct(diff1(alpha), diff2(alpha)); B/=nrm2(B); return B;}
      virtual Vec computeN(double alpha)   { Vec N = crossProduct(computeB(alpha), computeT(alpha)); return N;}
      virtual double computeR(double alpha){ 
	Vec rs = diff1(alpha);
	double nrm2rs = nrm2(rs);
	return nrm2rs*nrm2rs*nrm2rs/nrm2(crossProduct(rs,diff2(alpha)));
      }

      virtual void init(double alpha) {};
      virtual Vec computeT(const ContourPointData &cp)   {return computeT(cp.alpha(0));};
      virtual Vec computeB(const ContourPointData &cp)   {return computeB(cp.alpha(0));};
      virtual Vec computeN(const ContourPointData &cp)   {return computeN(cp.alpha(0));};
      virtual double computeR(const ContourPointData &cp){return computeR(cp.alpha(0));}; 
      virtual void init(const ContourPointData &cp) {init(cp.alpha(0));};

      double getalphaStart() {return alphaStart;}
      double getalphaEnd() {return alphaEnd;}
      void setalphaStart(double alphaStart_) {alphaStart = alphaStart_;};
      void setalphaEnd(double alphaEnd_) {alphaEnd = alphaEnd_;};
  };  
}

#endif
