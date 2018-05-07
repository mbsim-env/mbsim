/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _RKSUITE_INTEGRATOR_H_
#define _RKSUITE_INTEGRATOR_H_

#include "root_finding_integrator.h"

namespace MBSimIntegrator {

  /** \brief ODE-Integrator RKSuite
   *
   * This integrator uses RKSUITE (http://www.netlib.org/ode/rksuite).
   */
  class RKSuiteIntegrator : public RootFindingIntegrator {
    public:
      enum Method {
        RK23=1,
        RK45,
        RK78,
        unknownMethod
      };

      /**
       * \brief destructor
       */
      virtual ~RKSuiteIntegrator() override;

      void preIntegrate();
      void subIntegrate(double tStop);
      void postIntegrate();

      /* GETTER / SETTER */
      void setMethod(Method method_) { method = method_; }
      void setRelativeTolerance(double rTol_) { rTol = rTol_; }
      void setThreshold(const fmatvec::Vec &thres_) { thres = thres_; }
      void setThreshold(double thres_) { thres = fmatvec::Vec(1,fmatvec::INIT,thres_); }
      void setInitialStepSize(double dt0_) { dt0 = dt0_; }
      /***************************************************/

      /* INHERITED INTERFACE OF INTEGRATOR */
      using Integrator::integrate;
      void integrate() override;
      virtual void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

    private:

      static void fzdot(double* t, double* z_, double* zd_);

      int zSize;
      static RKSuiteIntegrator *selfStatic;

      /** method */
      Method method{RK45};
      /** threshold */
      fmatvec::Vec thres;
      /** relative tolerance */
      double rTol{1e-6};
      /** initial step size */
      double dt0{0};

      int lenwrk, messages{0}, integrationSteps{0}, lenint;
      double t{0}, tPlot{0}, s0{0}, time{0};
      double *work{nullptr};
      double *workint{nullptr};
      fmatvec::Vec z, zd, zMax, zWant, zdWant;

      std::ofstream integPlot;
  };

}

#endif
