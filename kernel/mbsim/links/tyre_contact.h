/* Copyright (C) 2004-2022 MBSim Development Team
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

#ifndef _TYRE_CONTACT_H
#define _TYRE_CONTACT_H

#include "mbsim/links/contour_link.h"

namespace MBSim {

  class TyreModel;

  class TyreContact : public ContourLink {
    public:
      TyreContact(const std::string &name="");
      ~TyreContact();

      void setTyreModel(TyreModel *model_);

      void init(InitStage stage, const InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void plot() override;

      void calcSize() override;
      void calcxSize() override { xSize = 1; }

      const fmatvec::Vec3& evalForwardVelocity() { if(updfvel) updateForwardVelocity(); return RvC; }
      fmatvec::Vec3& getForwardVelocity(bool check=true) {  assert((not check) or (not updfvel)); return RvC; }
      double evalsRelax() { if(updla) updateGeneralizedForces(); return sRelax; }
      double& getsRelax(bool check=true) {  assert((not check) or (not updla)); return sRelax; }
      void updatePositions(Frame *frame) override;
      void updateGeneralizedPositions() override;
      void updateGeneralizedVelocities() override;
      void updateForceDirections() override;
      void updateForwardVelocity();
      void updateGeneralizedForces() override;
      void updateh(int i=0) override;

      void updatexd() override;

      bool isActive() const override { return false; }
      bool gActiveChanged() override { return false; }
      bool isSingleValued() const override { return true; }

      void resetUpToDate() override;

    protected:
      TyreModel *model{nullptr};
      fmatvec::Vec3 RvC;
      double sRelax;

      bool updfvel{true};
  };

}

#endif
