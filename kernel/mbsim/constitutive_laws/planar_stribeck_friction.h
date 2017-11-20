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

#ifndef _PLANAR_STRIBECK_FRICTION_H_
#define _PLANAR_STRIBECK_FRICTION_H_

#include <mbsim/constitutive_laws/friction_force_law.h>

namespace MBSim {

  /**
   * \brief planar Stribeck friction force law on acceleration level for constraint description
   * \author Thorsten Schindler
   * \date 2009-09-02 inital commit (Thorsten Schindler)
   * \todo high oscillations in normal relative velocity TODO
   */
  class PlanarStribeckFriction : public FrictionForceLaw {
    public:
      /**
       * \brief constructor
       */
      PlanarStribeckFriction(Function<double(double)> *fmu_=nullptr) : fmu(fmu_) {
        if(fmu) fmu->setParent(this);
      }

      /**
       * \brief destructor
       */
      ~PlanarStribeckFriction() override { delete fmu; fmu=nullptr; }

      void init(Element::InitStage stage, const InitConfigSet &config) override {
        FrictionForceLaw::init(stage, config);
        fmu->init(stage, config);
      }

      /* INHERITED INTERFACE */
      fmatvec::Vec project(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r) override;
      fmatvec::Mat diff(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double r) override;
      fmatvec::Vec solve(const fmatvec::SqrMat& G, const fmatvec::Vec& gdn, double laN) override;
      bool isFulfilled(const fmatvec::Vec& la, const fmatvec::Vec& gdn, double laN, double laTol, double gdTol) override;
      fmatvec::Vec dlaTdlaN(const fmatvec::Vec& gd) override;
      int getFrictionDirections() override { return 1; }
      bool isSticking(const fmatvec::Vec& s, double sTol) override { return fabs(s(0)) <= sTol; }
      double getFrictionCoefficient(double gd) override { return (*fmu)(gd); }
      bool isSetValued() const override { return true; }
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      void setFrictionFunction(Function<double(double)> *fmu_) {
        fmu = fmu_;
        if(fmu) fmu->setParent(this);
      }

    protected:
      /**
       * friction coefficient function
       */
      Function<double(double)> *fmu;
  };

}

#endif
