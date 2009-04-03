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
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _ODEX_INTEGRATOR_H_
#define _ODEX_INTEGRATOR_H_

#include "integrator.h"

namespace MBSim {

  /** \brief ODE-Integrator ODEX
  */

  class ODEXIntegrator : public Integrator {

    private:

      static void fzdot(int* n, double* t, double* z, double* zd, double* rpar, int* ipar);
      static void plot(int* nr, double* told, double* t,double* z, int* n, double* con, int *ncon, int* icomp, int* nd, double* rpar, int* ipar, int* irtrn);

      static double tPlot;
      static double dtOut;
      static fmatvec::Vec zInp;
      static std::ofstream integPlot;
      static double s0; 
      static double time; 
      static bool output_; 

      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** Step size for the first step */
      double dt0;
      /** Maximal number of allowed steps */
      int maxSteps;

    public:

      ODEXIntegrator();
      ~ODEXIntegrator() {}

      void setaTol(const fmatvec::Vec &aTol_) {aTol.resize() = aTol_;}
      void setaTol(double aTol_) {aTol.resize() = fmatvec::Vec(1,fmatvec::INIT,aTol_);}
      void setrTol(const fmatvec::Vec &rTol_) {rTol.resize() = rTol_;}
      void setrTol(double rTol_) {rTol.resize() = fmatvec::Vec(1,fmatvec::INIT,rTol_);}
      void setdt0(double dt0_) {dt0 = dt0_;}


      void integrate(DynamicSystemSolver& system);

  };

}

#endif
