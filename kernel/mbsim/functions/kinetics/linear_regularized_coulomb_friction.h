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
 * Contact: markus.ms.schneider@gmail.com
 */

#ifndef _LINEAR_REGULARIZED_COULOMB_FRICTION_H_
#define _LINEAR_REGULARIZED_COULOMB_FRICTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  /*! 
   * \brief function describing a linear regularized relationship between the input relative velocity and the output for Coulomb friction
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \date 2010-01-09 beauty correction (Thorsten Schindler)
   * \todo put in function_library TODO
   */
  class LinearRegularizedCoulombFriction : public Function<fmatvec::Vec(fmatvec::Vec,double)> {
    public:
      /**
       * \brief constructor
       */
      LinearRegularizedCoulombFriction()  {}

      /**
       * \brief constructor
       * \param friction coefficient
       * \param border with respect to the relative velocity for the linear regularized increase of the friction force
       */
      LinearRegularizedCoulombFriction(double mu_, double gdLim_=0.01) : mu(mu_), gdLim(gdLim_) {}

      /* INHERITED INTERFACE OF FUNCTION2 */
      fmatvec::Vec operator()(const fmatvec::Vec &gd, const double& laN) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setFrictionCoefficient(double mu_) { mu=mu_; }
      void setMarginalVelocity(double gdLim_) { gdLim=gdLim_; }
      /***************************************************/

    protected:
      /**
       * \brief friction coefficient, border with respect to the relative velocity for the linear regularized increase of the friction force
       */
      double mu{0}, gdLim{0.01};
  };

}

#endif
