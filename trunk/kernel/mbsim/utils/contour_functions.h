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
#ifndef CONTOUR_FUNCTION_1S_H
#define CONTOUR_FUNCTION_1S_H

#include "fmatvec.h"
#include "mbsim/utils/ppolynom.h"
#include "mbsim/contour_pdata.h"

namespace MBSim {

  //================================================================================================================================
  /** userFunction for Contours: Contourpoint and trihedral (T,N,B) *
   * Parent class */
  class  ContourFunction1s {
    public:
      ContourFunction1s() {};
      virtual ~ContourFunction1s() {};
      virtual void init(const double& alpha) {};
      virtual void init(const ContourPointData &cp) { init(cp.getLagrangeParameterPosition()(0)); }
      virtual fmatvec::Vec3 operator()(const double& alpha, const void * =NULL) = 0;
      virtual fmatvec::Vec3 diff1(const double& alpha) = 0;
      virtual fmatvec::Vec3 diff2(const double& alpha) = 0;
      virtual fmatvec::Vec3 computeN(const double& alpha) { const fmatvec::Vec3 N=crossProduct(diff1(alpha),computeB(alpha)); return N/nrm2(N); }
      virtual fmatvec::Vec3 computeN(const ContourPointData &cp) { return computeN(cp.getLagrangeParameterPosition()(0)); };
      virtual fmatvec::Vec3 computeT(const double& alpha) { const fmatvec::Vec3 T=-diff1(alpha); return T/nrm2(T); }
      virtual fmatvec::Vec3 computeT(const ContourPointData &cp) { return computeT(cp.getLagrangeParameterPosition()(0)); };
      virtual fmatvec::Vec3 computeB(const double& alpha) { const fmatvec::Vec3 B = crossProduct(operator()(alpha), diff1(alpha)); return B/nrm2(B); }
      virtual fmatvec::Vec3 computeB(const ContourPointData &cp) { return computeB(cp.getLagrangeParameterPosition()(0)); };
      virtual double computeCurvature(const double& alpha) {
        const fmatvec::Vec3 rs = diff1(alpha);
        const double nrm2rs = nrm2(rs);
        return nrm2(crossProduct(rs,diff2(alpha)))/(nrm2rs*nrm2rs*nrm2rs); 
      }
      virtual double computeCurvature(const ContourPointData &cp) {return computeCurvature(cp.getLagrangeParameterPosition()(0)); }
      
      virtual double computeR(const double &alpha) {return 0; }
      virtual double computedRdAlpha(const double &alpha) {return 0; }
      virtual double computed2RdAlpha2(const double &alpha) {return 0; }
      
      double getalphaStart() { return alphaStart; }
      double getalphaEnd() { return alphaEnd; }
      void setalphaStart(double alphaStart_) { alphaStart = alphaStart_; }
      void setalphaEnd(double alphaEnd_) { alphaEnd = alphaEnd_; }

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement * element) {}
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element) {return 0;}

    protected:
      double alphaStart, alphaEnd;
  };  
}

#endif

