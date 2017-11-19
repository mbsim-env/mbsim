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

#ifndef _NONLINEAR_SPRING_DAMPER_FORCE_H_
#define _NONLINEAR_SPRING_DAMPER_FORCE_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  /**
   * \brief function describing a nonlinear relationship between the input deflection / relative velocity and the output for a spring
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  class NonlinearSpringDamperForce : public Function<double(double,double)> {
    public:
      /** 
       * \brief constructor
       */
      NonlinearSpringDamperForce() = default;

      /** 
       * \brief destructor
       */
      ~NonlinearSpringDamperForce() override {
        delete sF;
        delete sdF;
      }

      /** 
       * \brief constructor
       * \param distance depending force function
       * \param relative velocity depending force function
       */
      NonlinearSpringDamperForce(Function<double(double)> *sF_, Function<double(double)> *sdF_) : sF(sF_), sdF(sdF_) {
        sF->setParent(this);
        sdF->setParent(this);
      }

      /* INHERITED INTERFACE OF FUNCTION2 */
      double operator()(const double& s, const double& sd) override { return (*sF)(s) + (*sdF)(sd); }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(Element::InitStage stage, const InitConfigSet &config) override {
        Function<double(double,double)>::init(stage, config);
        sF->init(stage, config);
        sdF->init(stage, config);
      }
      /***************************************************/

      /* GETTER / SETTER */
      void setForceDeflectionFunction(Function<double(double)> * sF_) {
        sF=sF_;
        sF->setParent(this);
        sF->setName("ForceDeflection");
      }

      void setForceVelocityFunction(Function<double(double)> * sdF_) {
        sdF=sdF_;
        sdF->setParent(this);
        sdF->setName("ForceVelocity");
      }
      /***************************************************/

    protected:
      /**
       * \brief deflection depending force function
       */
      Function<double(double)> * sF;

      /**
       * \brief relative velocity depending force function
       */
      Function<double(double)> * sdF;
  };

}

#endif
