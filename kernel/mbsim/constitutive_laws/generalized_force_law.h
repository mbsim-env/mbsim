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

#ifndef _GENERALIZED_FORCE_LAW_H_
#define _GENERALIZED_FORCE_LAW_H_

#include "mbsim/functions/function.h"

namespace MBSim {

  /**
   * \brief basic force law on acceleration level for constraint description
   * \author Martin Foerg
   * \date 2009-07-29 some comments (Thorsten Schindler)
   */
  class GeneralizedForceLaw : public Element {
    public:
      /**
       * \brief constructor
       */
      GeneralizedForceLaw(Function<double(double,double)> *forceFunc_=nullptr) : Element(uniqueDummyName(this)), forceFunc(forceFunc_) { 
        if(forceFunc)
          forceFunc->setParent(this);
        plotFeature[plotRecursive]=false;
      }

      /**
       * \brief destructor
       */
      ~GeneralizedForceLaw() override { if(forceFunc) delete forceFunc; forceFunc = nullptr; };

      void init(Element::InitStage stage, const InitConfigSet &config) override {
        Element::init(stage, config);
        if(forceFunc)
          forceFunc->init(stage, config);
      }

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief decides, if force law is active
       * \param gap distance
       * \param tolerance
       * \return flag, if force law is active
       */
      virtual bool isClosed(double g, double gTol) { return true; }
      virtual bool remainsClosed(double s, double sTol) { return true; }

      /**
       * \brief prox function evaluation
       * \param kinetic variable
       * \param kinematic variable
       * \param relaxation factor
       * \param minimal threshold for kinetic variable
       * \return result of prox function evaluation
       */
      virtual double project(double la, double gdn, double r, double laMin=0) { return 0; }
      virtual fmatvec::Vec diff(double la, double gdn, double r, double laMin=0) { return fmatvec::Vec(2, fmatvec::INIT, 0); }
      virtual double solve(double G, double gdn) { return 0; }

      /**
       * \param contact force parameter
       * \param contact relative velocity
       * \param tolerance for contact force parameters
       * \param tolerance for relative velocity
       * \return flag if the force law is valid given the parameters
       */
      virtual bool isFulfilled(double la,  double gdn, double tolla, double tolgd, double laMin=0) { return true; }

      /**
       * \return flag if the force law is setvalued
       */
      virtual bool isSetValued() const = 0;

      /**
       * \brief initialize the force law using XML
       * \param XML element
       */
      void initializeUsingXML(xercesc::DOMElement *element) override {}
      /***************************************************/
      
      /**
       * \brief
       *
       * \param g          distance of the contact points
       * \param gd         relative velocity in normal direction of contact points
       * \param additional ??
       */
      double operator()(double g, double gd) { assert(forceFunc); return (*forceFunc)(g,gd); }

      /** \brief Set the force function for use in regularisized constitutive laws
       * The first input parameter to the force function is g.
       * The second input parameter to the force function is gd.
       * The return value is the force.
       */
      void setForceFunction(Function<double(double,double)> *forceFunc_) { 
        forceFunc=forceFunc_; 
        forceFunc->setParent(this);
      }

    protected:
      /*!
       * \brief force function for a regularized contact law
       */
      Function<double(double,double)> *forceFunc;
  };

}

#endif
