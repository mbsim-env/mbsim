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

      void calcSize() override;
      void calcxSize() override { xSize = 1; }

      const fmatvec::Vec3& evalForwardVelocity() { if(updfvel) updateForwardVelocity(); return RvC; }
      fmatvec::Vec3& getForwardVelocity(bool check=true) {  assert((not check) or (not updfvel)); return RvC; }
      double evalKyalr() { if(updla) updateGeneralizedForces(); return Kyalr; }
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
      double Kyalr;

      double c_z_RW{141000.000000};
      double d_z_RW{800.000000};
      double pKy1{15.791000};
      double pKy2{1.693500};
      double pKy3{1.460400};
      double pKy4{0.669000};
      double pKy5{0.187080};
      double Fz0_RW{1600.000000};
      double pDx1{1.355000};
      double pDx2{-0.060300};
      double pEx1{0.026300};
      double pEx2{0.270560};
      double pEx3{-0.076900};
      double pEx4{1.126800};
      double pKx1{25.940000};
      double pKx2{-4.233000};
      double pKx3{0.336900};
      double Cx{1.606400};
      double rBx1{13.476000};
      double rBx2{11.354000};
      double Cxal{1.123100};
      double pDy1{1.300000};
      double pDy2{0.000000};
      double pDy3{0.000000};
      double pEy1{-2.222700};
      double pEy2{-1.669000};
      double pEy4{-4.288000};
      double Cy{0.900000};
      double pKy6{0.455120};
      double pKy7{0.013293};
      double Cga{0.613970};
      double Ega{-19.990000};
      double rBy1{7.785600};
      double rBy2{8.169700};
      double rBy3{-0.059140};
      double Cyka{1.053300};
      double qHz3{-0.028448};
      double qHz4{-0.009862};
      double qBz1{10.041000};
      double qBz2{-0.000000};
      double qBz5{-0.767840};
      double qBz6{0.734220};
      double qBz9{16.390000};
      double qBz10{-0.355490};
      double qDz1{0.263310};
      double qDz2{0.030987};
      double qDz3{-0.620130};
      double qDz4{0.985240};
      double qDz8{0.504530};
      double qDz9{0.363120};
      double qDz10{-0.191680};
      double qDz11{-0.407090};
      double qEz1{-0.199240};
      double qEz2{-0.017638};
      double qEz5{3.651100};
      double Ct{1.315300};

      bool updfvel{true};
  };

}

#endif
