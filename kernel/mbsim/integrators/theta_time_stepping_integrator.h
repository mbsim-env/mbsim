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

#ifndef _THETA_TIME_STEPPING_INTEGRATOR_H_
#define _THETA_TIME_STEPPING_INTEGRATOR_H_

#include "integrator.h"

namespace MBSimIntegrator {

  /** 
   * brief theta-time-stepping integrator of first order
   * \author Roland Zander
   * \author Thorsten Schindler
   * \date 2009-07-18 new kernel_dev (Thorsten Schindler)
   * \date 2009-07-19 Delassus matrix / split of update (Thorsten Schindler)
   * \date 2009-07-27 some fixes (Thorsten Schindler)
   * \date 2010-04-23 integrate splitted (Thorsten Schindler)
   */
  class ThetaTimeSteppingIntegrator : public Integrator { 
    public:
      /**
       * \brief destructor
       */
      ~ThetaTimeSteppingIntegrator() override = default;

      void preIntegrate() override;
      void subIntegrate(double tStop) override;
      void postIntegrate() override;

      /* INHERITED INTERFACE OF INTEGRATOR */
      using Integrator::integrate;
      void integrate() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setStepSize(double dt_) { dt = dt_; }
      void setTheta(double theta_ ) { theta  = theta_; }
      void setToleranceForPositionConstraints(double gMax_) { gMax = gMax_; }
      /***************************************************/

    private:
      /**
       * \brief step size
       */
      double dt{1e-3};

      /**
       * \brief convex combination parameter between explicit (0) and implicit (1) Euler scheme
       */
      double theta{0.5};

      /**
       * \brief time and plot time
       */
      double t{0}, tPlot{0};

      /**
       * \brief iteration counter for constraints, plots, integration, maximum constraints, cummulation constraint
       */
      int iter{0}, step{0}, integrationSteps{0}, maxIter{0}, sumIter{0};

      /**
       * \brief computing time counter
       */
      double s0{0}, time{0};

      /**
       * \brief plot step difference
       */
      int stepPlot{0};

      /** tolerance for position constraints */
      double gMax{1};
      /**
       * \brief file stream for integration information
       */
      std::ofstream integPlot;
  };

}

#endif /* _THETA_TIME_STEPPING_INTEGRATOR_H_ */
