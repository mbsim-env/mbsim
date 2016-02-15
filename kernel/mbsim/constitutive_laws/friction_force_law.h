/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _FRICTION_FORCE_LAW_H_
#define _FRICTION_FORCE_LAW_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  /**
   * \brief basic friction force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   */
  class FrictionForceLaw : public Element {
    public:
      /**
       * \brief constructor
       */
      FrictionForceLaw(Function<fmatvec::Vec(fmatvec::Vec,double)> *frictionForceFunc_=NULL) : Element(uniqueDummyName(this)), frictionForceFunc(frictionForceFunc_) {
        if(frictionForceFunc)
          frictionForceFunc->setParent(this);
      }

      /**
       * \brief destructor
       */
      virtual ~FrictionForceLaw() { if(frictionForceFunc) delete frictionForceFunc; frictionForceFunc = NULL; };

      void init(Element::InitStage stage) {
        Element::init(stage);
        if(frictionForceFunc)
          frictionForceFunc->init(stage);
      }

      /* INTERFACE FOR DERIVED CLASSES */
      virtual fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r) { return fmatvec::Vec(2); }
      virtual fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r) { return fmatvec::Mat(2,2); }
      virtual fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, double laN) { return fmatvec::Vec(2); }
      virtual bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double tolla, double tolgd) { return true; }
      virtual fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd) {
        THROW_MBSIMERROR("(FrictionForceLaw::dlaTdlaN): Not implemented.");
        return 0;
      }
      virtual int getFrictionDirections() = 0;
      virtual bool isSticking(const fmatvec::Vec& s, double sTol) = 0;
      virtual double getFrictionCoefficient(double gd) { return 0; }
      virtual bool isSetValued() const = 0;
      virtual void initializeUsingXML(xercesc::DOMElement *element) {}
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent);

      /**
       * \return std::string representation
       */
      virtual std::string getType() const { return "FrictionForceLaw"; }
      /***************************************************/
      
      fmatvec::Vec operator()(const fmatvec::Vec &gd, double laN) { assert(frictionForceFunc); return (*frictionForceFunc)(gd,laN); }

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
      Function<fmatvec::Vec(fmatvec::Vec,double)> *frictionForceFunc;
  };

}

#endif
