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

#ifndef _LSODAR_INTEGRATOR_H_
#define _LSODAR_INTEGRATOR_H_

#include "integrator.h"

namespace MBSim {

  /** \brief ODE-Integrator LSODAR
    Integrator with root finding for ODEs.
    This integrator uses LSODAR from http://www.netlib.org . */
  class LSODARIntegrator : public Integrator {
    private:

      static void fzdot(int* zSize, double* t, double* z_, double* zd_);
      static void fsv(int* zSize, double* t, double* z_, int* nsv, double* sv_);

      /** maximal step size */
      double dtMax;
      /** minimal step size */
      double dtMin;
      /** Absolute Toleranz */
      fmatvec::Vec aTol;
      /** Relative Toleranz */
      double rTol;
      /** step size for the first step */
      double dt0;

      bool plotOnRoot;

      int zSize, iTol, istate, nsv, lrWork, liWork, integrationSteps;
      double t, tPlot, s0, time;
      fmatvec::Vec z, rWork;
      fmatvec::Vector<fmatvec::General, int> iWork, jsv;
      std::ofstream integPlot;
    public:

      LSODARIntegrator();
      ~LSODARIntegrator() {}

      void setMaximalStepSize(double dtMax_) {dtMax = dtMax_;}
      void setMinimalStepSize(double dtMin_) {dtMin = dtMin_;}
      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) {aTol.resize() = aTol_;}
      void setAbsoluteTolerance(double aTol_) {aTol.resize() = fmatvec::Vec(1,fmatvec::INIT,aTol_);}
      void setRelativeTolerance(double rTol_) {rTol = rTol_;}
      void setInitialStepSize(double dt0_) {dt0 = dt0_;}

      void setPlotOnRoot(bool b) {plotOnRoot = b;}

      void integrate(DynamicSystemSolver& system);
      void preIntegrate(DynamicSystemSolver& system);
      void subIntegrate(DynamicSystemSolver& system, double tStop);
      void postIntegrate(DynamicSystemSolver& system);

      virtual void initializeUsingXML(TiXmlElement *element);
  };

}

#endif
