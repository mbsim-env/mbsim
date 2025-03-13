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
  class RigidContour;

  class TyreContact : public ContourLink {
    public:
      TyreContact(const std::string &name="");
      ~TyreContact();

      void setTyreModel(TyreModel *model_);
      TyreModel* getTyreModel() { return model; }

      void init(InitStage stage, const InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void plot() override;

      void calcSize() override;
      void calcxSize() override;
      void calcisSize() override;

      const fmatvec::Vec3& evalForwardVelocity() { if(updfvel) updateForwardVelocity(); return RvC; }
      fmatvec::Vec3& getForwardVelocity(bool check=true) {  assert((not check) or (not updfvel)); return RvC; }
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

      void setInitialGuess(const fmatvec::MatV &zeta0_) { zeta0 <<= zeta0_; }
      void setTolerance(double tol_) { tol = tol_; }

      void resetUpToDate() override;

      RigidContour* getRigidContour(int i);

    protected:
      TyreModel *model{nullptr};
      fmatvec::Vec3 RvC;
      fmatvec::MatV zeta0;
      double tol{1e-10};
      bool updfvel{true};
  };

}

#endif
