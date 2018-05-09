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

#ifndef _EXPLICIT_EULER_INTEGRATOR_H_
#define _EXPLICIT_EULER_INTEGRATOR_H_

#include "integrator.h"

namespace MBSimIntegrator {

  /** \brief Explicit Euler integrator. */
  class ExplicitEulerIntegrator : public Integrator {
    public:
      /**
       * \brief destructor
       */
      ~ExplicitEulerIntegrator() override = default;

      void preIntegrate() override;
      void subIntegrate(double tStop) override;
      void postIntegrate() override;

      /* INHERITED INTERFACE OF INTEGRATOR */
      using Integrator::integrate;
      void integrate() override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setStepSize(double dt_) {dt = dt_;}
      /***************************************************/

    private:
      /**
       * \brief step size
       */
      double dt{1e-3};

      double tPlot;
      int iter, step, integrationSteps;
      double s0, time;
      int stepPlot;
  };

}

#endif
