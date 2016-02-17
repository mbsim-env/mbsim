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
   * \brief function describing a nonlinear relationship between the input relative distance / velocity and the output for a spring
   * \author Martin Foerg
   * \date 2009-08-31 some comments (Thorsten Schindler)
   */
  class NonlinearSpringDamperForce : public Function<double(double,double)> {
    public:
      /** 
       * \brief constructor
       */
      NonlinearSpringDamperForce() {}

      /** 
       * \brief destructor
       */
      ~NonlinearSpringDamperForce() {
        delete gForceFun;
        delete gdForceFun;
      }

      /** 
       * \brief constructor
       * \param distance depending force function
       * \param relative velocity depending force function
       */
      NonlinearSpringDamperForce(Function<double(double)> * gForceFun_, Function<double(double)> * gdForceFun_) : gForceFun(gForceFun_), gdForceFun(gdForceFun_) {
        gForceFun->setParent(this);
        gdForceFun->setParent(this);
      }

      /* INHERITED INTERFACE OF FUNCTION2 */
      virtual double operator()(const double& g, const double& gd) { return (*gForceFun)(g) + (*gdForceFun)(gd); }
      void initializeUsingXML(xercesc::DOMElement *element);
      void init(Element::InitStage stage) {
        Function<double(double,double)>::init(stage);
        gForceFun->init(stage);
        gdForceFun->init(stage);
      }
      /***************************************************/

      /* GETTER / SETTER */
      void setDistanceFunction(Function<double(double)> * gForceFun_) {
        gForceFun=gForceFun_;
        gForceFun->setParent(this);
        gForceFun->setName("Distance");
      }

      void setVelocityFunction(Function<double(double)> * gdForceFun_) {
        gdForceFun=gdForceFun_;
        gdForceFun->setParent(this);
        gdForceFun->setName("Velocity");
      }
      /***************************************************/

    protected:
      /**
       * \brief distance depending force function
       */
      Function<double(double)> * gForceFun;

      /**
       * \brief relative velocity depending force function
       */
      Function<double(double)> * gdForceFun;
  };

}

#endif
