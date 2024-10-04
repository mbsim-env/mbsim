/* Copyright (C) 2004-2024 MBSim Development Team
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

#ifndef _LINEAR_TYRE_MODEL_H_
#define _LINEAR_TYRE_MODEL_H_

#include <mbsim/constitutive_laws/tyre_model.h>

namespace MBSim {

  class LinearTyreModel : public TyreModel {
    public:
      LinearTyreModel() = default;
      ~LinearTyreModel() override = default;

      void setcz(double cz_) { cz = cz_; }
      void setdz(double dz_) { dz = dz_; }
      void setcka(double cka_) { cka = cka_; }
      void setcal(double cal_) { cal = cal_; }
      void setcga(double cga_) { cga = cga_; }
      void setcMzga(double cMzga_) { cMzga = cMzga_; }
      void sett(double t_) { t = t_; }
      void setScaleFactorForLongitudinalForce(double sfFx_) { sfFx = sfFx_; }
      void setScaleFactorForLateralForce(double sfFy_) { sfFy = sfFy_; }
      void setScaleFactorForAligningMoment(double sfMz_) { sfMz = sfMz_; }

      void initPlot(std::vector<std::string> &plotColumns) override;
      void plot(std::vector<double> &plotVector) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      fmatvec::VecV getContourParameters() const override;
      double evalFreeRadius() override;

      void updateGeneralizedForces() override;

      fmatvec::VecV getData() const override;

      bool motorcycleKinematics() const override { return true; }

    private:
      double cz{0};
      double dz{0};
      double cka{0};
      double cal{0};
      double cga{0};
      double cMzga{0};
      double t{0};
      double sfFx{1};
      double sfFy{1};
      double sfMz{1};

      double vcx, vcy, vsx, ka, ga, al, rhoz, Re, Rs;
 };

}

#endif
