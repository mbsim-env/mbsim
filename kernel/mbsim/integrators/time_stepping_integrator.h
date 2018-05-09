/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _TIME_STEPPING_INTEGRATOR_H_ 
#define _TIME_STEPPING_INTEGRATOR_H_

#include "integrator.h"

namespace MBSimIntegrator {

  /** 
   * brief half-explicit time-stepping integrator of first order
   * \author Martin Foerg
   * \date 2009-07-13 some comments (Thorsten Schindler)
   */
  class TimeSteppingIntegrator : public Integrator { 
    public:
      /**
       * \brief destructor
       */
      ~TimeSteppingIntegrator() override = default;

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
      void setToleranceForPositionConstraints(double gMax_) { gMax = gMax_; }
      /***************************************************/
    
    private:
      void resize();

      /**
       * \brief step size
       */
      double dt{1e-3};

      /**
       * \brief time and plot time
       */
      double tPlot{0.};

      /**
       * \brief iteration counter for constraints, plots, integration, maximum constraints, cummulation constraint
       */
      int step{0}, integrationSteps{0}, maxIter{0}, sumIter{0};

      /**
       * \brief computing time counter
       */
      double s0{0.}, time{0.};

      /**
       * \brief plot step difference
       */
      int stepPlot{0};

      /** tolerance for position constraints */
      double gMax{-1};
  };

}

#endif /* _TIME_STEPPING_INTEGRATOR_H_ */
