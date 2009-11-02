/* Copyright (C) 2007 Robert Huber
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
 * Contact:
 *   rhuber@users.berlios.de
 *
 */

#ifndef _DASPK_INTEGRATOR_H_
#define _DASPK_INTEGRATOR_H_

using namespace fmatvec;

#include "integrator.h"

namespace MBSim {

  /** \brief DAE-Integrator DASPK (up to index 2)
   */
  class DASPKIntegrator : public Integrator {
    friend class DAETSIntegrator;
    private:
      static void residuum(double* t, double* Y_, double* Ydot_, double* CJ, double* res_, int* ires, double* rpar, int* ipar);
      static void jac(double* t, double* Y_, double* Ydot_, double* PD, double* CJ, double* rpar, int* ipar); 
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
      possible values:  2, 21 (Index 2 formulation with Gear Gupta Leimkuhler Stabilisation (GGL)) or 1
      */
      int DAEIndex;
      /** maximum method order (one to five)*/
      int MaxOrder;
      /** include or exclude algebraic variables from error test or scale varibles with stepsize */
      int FlagErrorTest;
      /** use external jacobian A=dG/dY + CJ*dG/dY' (true) or numerical calculation by integrator (false=default) */
      bool useExternalJac;

          
    public:

      DASPKIntegrator();
      ~DASPKIntegrator() {}

      void setaTol(const fmatvec::Vec &aTol_) {aTol.resize() = aTol_;}
      void setaTol(double aTol_) {aTol.resize() = Vec(1,INIT,aTol_);}
      void setrTol(const fmatvec::Vec &rTol_) {rTol.resize() = rTol_;}
      void setrTol(double rTol_) {rTol.resize() = Vec(1,INIT,rTol_);}
      void setdt0(double dt0_) {dt0 = dt0_;}
      void setdtMax(double dtMax_) {dtMax = dtMax_;}
      void setMaxStepNumber(int maxSteps_) {maxSteps = maxSteps_;}
      void setDAEIndex(int index);
      void setMaxOrder(int order_);
      void setFlagErrorTest(int Flag);
      void useExternalJacobian(bool flag=true) {useExternalJac = flag;}
      void setGGLStabilisation() {DAEIndex=21;};
      void integrate(MultiBodySystem& system);
  };

}

#endif


