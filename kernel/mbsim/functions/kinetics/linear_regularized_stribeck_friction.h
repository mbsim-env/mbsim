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

#ifndef _LINEAR_REGULARIZED_STRIBECK_FRICTION_H_
#define _LINEAR_REGULARIZED_STRIBECK_FRICTION_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  /**
   * \brief function describing a linear regularized relationship between the input relative velocity and the output for Stribeck friction
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   * \date 2010-01-09 beauty correction (Thorsten Schindler)
   * \todo delete function pointer
   */
  class LinearRegularizedStribeckFriction : public Function<fmatvec::Vec(fmatvec::Vec,double)> {
    public:
      /**
       * \brief constructor
       */
      LinearRegularizedStribeckFriction() : fmu(NULL), gdLim(0.01) {}

      /**
       * \brief constructor
       * \param function for friction coefficient depending on norm of relative velocity
       * \param border with respect to the relative velocity for the linear regularized increase of the friction force
       */
      LinearRegularizedStribeckFriction(Function<double(double)> *fmu_, double gdLim_=0.01) : fmu(fmu_), gdLim(gdLim_) {
        fmu->setParent(this);
      }

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual fmatvec::Vec operator()(const fmatvec::Vec &gd, const double& laN);
      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual void init(Element::InitStage stage, const InitConfigSet &config) {
        Function<fmatvec::Vec(fmatvec::Vec,double)>::init(stage, config);
        fmu->init(stage, config);
      }
      /***************************************************/

      /* GETTER / SETTER */
      void setFrictionFunction(Function<double(double)> *fmu_) {
        fmu=fmu_;
        fmu->setParent(this);
        fmu->setName("Friction");
      }
      void setMarginalVelocity(double gdLim_) { gdLim=gdLim_; }
      /***************************************************/

    protected:
      /**
       * \brief friction coefficient function
       */
      Function<double(double)> *fmu;
      
      /**
       * \brief border with respect to the relative velocity for the linear regularized increase of the friction force
       */
      double gdLim;
  };

}

#endif
