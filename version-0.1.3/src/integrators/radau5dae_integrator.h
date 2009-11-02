/* Copyright (C) 2004-2006  Martin Förg, Robert Huber

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

#ifndef _RADAU5DAE_INTEGRATOR_H_
#define _RADAU5DAE_INTEGRATOR_H_

using namespace fmatvec;

#include "integrator.h"

namespace MBSim {

  /** \brief DAE-Integrator RADAU5 (up to index 3)
   */
  class RADAU5DAEIntegrator : public Integrator {

    private:
      static void fdae(int* YSize, double* t, double* Y_, double* F_, double* rpar, int* ipar); 
      static void mdae(int* YSize, double* MasMat_, int* lMas, double* rpar, int* ipar);
      static void plot(int* nr, double* told, double* t, double* z, double* cont, int* lrc, int* n, double* rpar, int* ipar, int* irtrn);
      static void jac(int* YSize, double* t, double* Y_, double* jac_, int* ljac_, double* rpar, int* ipar);

      static double tPlot;
      static double dtOut;
      static fmatvec::Vec zInp;
      static ofstream integPlot;
      static double s0; 
      static double time; 
      static bool output_;

      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      fmatvec::Vec rTol;
      /** step size for the first step */
      double dt0;
      /** maximum number of steps */
      int maxSteps;
      /** maximal step size */
      double dtMax;
      /** index of DAE (default 2)
	possible values:  2, 21 (Index 2 formulation with Gear Gupta Leimkuhler Stabilisation (GGL)) or 3
       */
      int DAEIndex;
      /** use external jacobian A=dG/dY + CJ*dG/dY' (true) or numerical calculation by integrator (false=default) */
      bool useExternalJac;

    public:

      RADAU5DAEIntegrator();
      ~RADAU5DAEIntegrator() {}

      void setaTol(const fmatvec::Vec &aTol_) {aTol.resize() = aTol_;}
      void setaTol(double aTol_) {aTol.resize() = Vec(1,INIT,aTol_);}
      void setrTol(const fmatvec::Vec &rTol_) {rTol.resize() = rTol_;}
      void setrTol(double rTol_) {rTol.resize() = Vec(1,INIT,rTol_);}
      void setdt0(double dt0_) {dt0 = dt0_;}
      void setdtMax(double dtMax_) {dtMax = dtMax_;}
      void setMaxStepNumber(int maxSteps_) {maxSteps = maxSteps_;}
      void setDAEIndex(int index);
      void useExternalJacobian(bool flag=true) {useExternalJac = flag;}
      void setGGLStabilisation() {DAEIndex=21;};
      void integrate(MultiBodySystem& system);
  };

}

#endif
