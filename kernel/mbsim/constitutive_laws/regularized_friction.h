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

#ifndef _REGULARIZED_FRICTION_H_
#define _REGULARIZED_FRICTION_H_

#include <mbsim/constitutive_laws/friction_force_law.h>

namespace MBSim {

  /**
   * \brief basic regularized friction force law on acceleration level for constraint description
   * \author Martin Foerg
   */
  class RegularizedFriction : public FrictionForceLaw {
    public:
      RegularizedFriction(Function<fmatvec::Vec(fmatvec::Vec,double)> *frictionForceFunc_=nullptr) : frictionForceFunc(frictionForceFunc_) {
        if(frictionForceFunc)
          frictionForceFunc->setParent(this);
      }
      ~RegularizedFriction() override { delete frictionForceFunc; }
      void init(Element::InitStage stage, const InitConfigSet &config) override;
      bool isSetValued() const override { return false; }
      void initializeUsingXML(xercesc::DOMElement *element) override;

      fmatvec::Vec operator()(const fmatvec::Vec &gd, double laN) override { return (*frictionForceFunc)(gd,laN); }
      fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd) override { return frictionForceFunc->parDer2(gd,1); }

      /** \brief Set the friction force function for use in regularisized constitutive friction laws
       * The first input parameter to the friction force function is gd.
       * The second input parameter to the friction force function is laN.
       * The return value is the force vector.
       */
      void setFrictionForceFunction(Function<fmatvec::Vec(fmatvec::Vec,double)> *frictionForceFunc_) { 
        frictionForceFunc=frictionForceFunc_; 
        frictionForceFunc->setParent(this);
      }

    protected:
      Function<fmatvec::Vec(fmatvec::Vec,double)> *frictionForceFunc{nullptr};
  };

}

#endif
