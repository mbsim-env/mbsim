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

#ifndef _LSODER_INTEGRATOR_H_
#define _LSODER_INTEGRATOR_H_

#include "integrator.h"

namespace MBSimIntegrator {

  extern bool odePackInUse;

  /** \brief ODE-Integrator LSODER
    Integrator with root finding for ODEs.
    This integrator uses LSODE from http://www.netlib.org . */
  class LSODERIntegrator : public Integrator {

    public:

      enum Method {
        nonstiff=10,
        stiff=22
      };

    private:

      static void fzdot(int* neq, double* t, double* z_, double* zd_);
      static void fsv(int* neq, double* t, double* z_, int* nsv, double* sv_);

      /** maximal step size */
      double dtMax{0};
      /** minimal step size */
      double dtMin{0};
      /** absolute tolerance */
      fmatvec::Vec aTol;
      /** relative tolerance */
      double rTol{1e-6};
      /** step size for the first step */
      double dt0{0};
      /**  maximum number of steps allowed during one call to the solver. (default 10000) */
      int maxSteps{10000};
      /** use stiff (BDF) or nonstiff (Adams) method */
      Method method{nonstiff};

      bool plotOnRoot{true};

      /** tolerance for position constraints */
      double gMax{1e-5};
      /** tolerance for velocity constraints */
      double gdMax{1e-5};

      int neq[1+sizeof(void*)/sizeof(int)+1]; // store zSize at neq[0]; store this at neq[1..]
      int iTol, istate, nsv, lrWork, liWork, integrationSteps;
      double t, tPlot, s0, time;
      fmatvec::Vec rWork;
      fmatvec::VecInt iWork;
      std::ofstream integPlot;
    public:

      void setMaximumStepSize(double dtMax_) { dtMax = dtMax_; }
      void setMinimumStepSize(double dtMin_) { dtMin = dtMin_; }
      void setAbsoluteTolerance(const fmatvec::Vec &aTol_) { aTol = aTol_; }
      void setAbsoluteTolerance(double aTol_) { aTol = fmatvec::Vec(1,fmatvec::INIT,aTol_); }
      void setRelativeTolerance(double rTol_) { rTol = rTol_; }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      void setStepLimit(int maxSteps_) { maxSteps = maxSteps_; }

      void setPlotOnRoot(bool b) { plotOnRoot = b; }

      void setToleranceForPositionConstraints(double gMax_) { gMax = gMax_; }
      void setToleranceForVelocityConstraints(double gdMax_) { gdMax = gdMax_; }

      using Integrator::integrate;
      void integrate();
      void preIntegrate();
      void subIntegrate(double tStop);
      void postIntegrate();

      virtual void initializeUsingXML(xercesc::DOMElement *element);
  };

}

#endif
