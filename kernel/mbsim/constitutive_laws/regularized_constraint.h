/* Copyright (C) 2004-2018 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _REGULARIZED_CONSTRAINT_H_
#define _REGULARIZED_CONSTRAINT_H_

#include <mbsim/constitutive_laws/generalized_force_law.h>

namespace MBSim {

  /**
   * \brief basic regularized force law on acceleration level for constraint description
   * \author Martin Foerg
   */
  class RegularizedConstraint : public GeneralizedForceLaw {
    public:
      /**
       * \brief constructor
       */
      RegularizedConstraint(Function<double(double,double)> *forceFunc_=nullptr) : forceFunc(forceFunc_) {
        if(forceFunc)
          forceFunc->setParent(this);
      }

      /**
       * \brief destructor
       */
      ~RegularizedConstraint() override { delete forceFunc; };

      void init(Element::InitStage stage, const InitConfigSet &config) override;

      /* INHERITED INTERFACE */
      bool isSetValued() const override { return false; }
      /***************************************************/

      void initializeUsingXML(xercesc::DOMElement *element) override;

      /**
       * \brief
       *
       * \param g          distance of the contact points
       * \param gd         relative velocity in normal direction of contact points
       */
      double operator()(double g, double gd) override { return (*forceFunc)(g,gd); }

      /** \brief Set the force function for use in regularisized constitutive laws
       * The first input parameter to the force function is g.
       * The second input parameter to the force function is gd.
       * The return value is the force.
       */
      void setForceFunction(Function<double(double,double)> *forceFunc_) { 
        forceFunc=forceFunc_; 
        forceFunc->setParent(this);

      }

      Function<double(double,double)>* getForceFunction() { return forceFunc; }

    protected:
      /*!
       * \brief force function for a regularized contact law
       */
      Function<double(double,double)> *forceFunc;
  };

}

#endif
