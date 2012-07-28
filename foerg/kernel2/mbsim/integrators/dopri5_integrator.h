/* Copyright (C) 2004-2006  Martin Förg
 
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
 *   martin.o.foerg@googlemail.com
 *
 */

#ifndef _DOPRI5_INTEGRATOR_H_
#define _DOPRI5_INTEGRATOR_H_

#include "integrator.h"

namespace MBSim {

  /** \brief ODE-Integrator DOPRI5.
  */

  class DOPRI5Integrator : public Integrator {

    private:

      static void fzdot(int* n, double* t, double* z, double* zd, double* rpar, int* ipar);
      static void plot(int* nr, double* told, double* t,double* z, int* n, double* con, int* icomp, int* nd, double* rpar, int* ipar, int* irtrn);

      static double tPlot;
      static double dtOut;
      static fmatvec::Vec zInp;
      static bool output_; 
      static std::ofstream integPlot;
      static double s0; 
      static double time;
      //static int integrationSteps;
   
      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** step size for the first step */
      double dt0;
      /** maximum number of steps */
      int maxSteps;  
      /** maximale step size */
      double dtMax;

    public:

      DOPRI5Integrator();
      ~DOPRI5Integrator() {}

      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) {aTol = aTol_;}
      void setAbsoluteTolerance(double aTol_) {aTol = fmatvec::Vec(1,fmatvec::INIT,aTol_);}
      void setRelativeTolerance(const fmatvec::Vec &rTol_) {rTol = rTol_;}
      void setRelativeTolerance(double rTol_) {rTol = fmatvec::Vec(1,fmatvec::INIT,rTol_);}
      void setInitialStepSize(double dt0_) {dt0 = dt0_;}
      void setMaxStepNumber(int maxSteps_) {maxSteps = maxSteps_;}    
      void setMaximalStepSize(double dtMax_) {dtMax = dtMax_;}
      const fmatvec::Vec& getAbsoluteTolerance() const { return aTol; }
      const fmatvec::Vec& getRelativeTolerance() const { return rTol; }
      double getInitialStepSize() const { return dt0; }
      int getMaxStepNumber() const { return maxSteps; }
      double getMaximalStepSize() const { return dtMax; }

      void integrate(DynamicSystemSolver& system);

      virtual void initializeUsingXML(TiXmlElement *element);
      virtual TiXmlElement* writeXMLFile(TiXmlNode *element);

      virtual std::string getType() const { return "DOPRI5Integrator"; }
  };

}

#endif
