/* Copyright (C) 2004-2023 MBSim Development Team
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

#ifndef _MAGIC_FORMULA_62_H_
#define _MAGIC_FORMULA_62_H_

#include <mbsim/constitutive_laws/tyre_model.h>

namespace MBSim {

  class MagicFormula62 : public TyreModel {
    public:
      MagicFormula62() = default;
      ~MagicFormula62() override = default;

      void setInputDataFile(const std::string& inputDataFile_) { inputDataFile = inputDataFile_; }
      void setVerticalStiffness(double cz_) { cz = cz_; }
      void setVerticalDamping(double dz_) { dz = dz_; }
      void setNominalLoad(double Fz0_) { Fz0 = Fz0_; }
      void setInflationPressure(double p_) { p = p_; }
      void setNominalPressure(double p0_) { p0 = p0_; }
      void setepsx(double epsx_) { epsx = epsx_; }
      void setepsk(double epsk_) { epsk = epsk_; }
      void setRelaxationLength(double sRelax_) { sRelax = sRelax_; }
      void setScaleFactorForLongitudinalForce(double sfFLo_) { sfFLo = sfFLo_; }
      void setScaleFactorForLateralForce(double sfFLa_) { sfFLa = sfFLa_; }
      void setScaleFactorForAligningMoment(double sfM_) { sfM = sfM_; }
      void setScaleFactorForLongitudinalFricitionCoefficient(double sfmux_) { sfmux = sfmux_; }
      void setScaleFactorForLateralFricitionCoefficient(double sfmuy_) { sfmuy = sfmuy_; }
      void setScaleFactorForLongitudinalSlipStiffness(double sfkx_) { sfkx = sfkx_; }
      void setScaleFactorForCorneringStiffness(double sfky_) { sfky = sfky_; }
      void setScaleFactorForCamberStiffness(double sfkg_) { sfkg = sfkg_; }
      void setScaleFactorForResidualTorque(double sfkm_) { sfkm = sfkm_; }

      void init(InitStage stage, const InitConfigSet &config) override;
      void initPlot(std::vector<std::string> &plotColumns) override;
      void plot(std::vector<double> &plotVector) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void updateGeneralizedForces() override;

      int getDataSize() const override { return 8; }
      fmatvec::VecV getData() const override;

    private:
      void importData();
      std::string inputDataFile;
      double PCX1, PDX1, PDX2, PDX3, PEX1, PEX2, PEX3, PEX4, PKX1, PKX2, PKX3, PHX1, PHX2, PVX1, PVX2, PPX1, PPX2, PPX3, PPX4, RBX1, RBX2, RBX3, RCX1, REX1, REX2, RHX1, QSX1, QSX2, QSX3, QSX4, QSX5, QSX6, QSX7, QSX8, QSX9, QSX10, QSX11, QSX12, QSX13, QSX14, PPMX1, PCY1, PDY1, PDY2, PDY3, PEY1, PEY2, PEY3, PEY4, PEY5, PKY1, PKY2, PKY3, PKY4, PKY5, PKY6, PKY7, PHY1, PHY2, PVY1, PVY2, PVY3, PVY4, PPY1, PPY2, PPY3, PPY4, PPY5, RBY1, RBY2, RBY3, RBY4, RCY1, REY1, REY2, RHY1, RHY2, RVY1, RVY2, RVY3, RVY4, RVY5, RVY6, QSY1, QSY2, QSY3, QSY4, QSY5, QSY6, QSY7, QSY8, QBZ1, QBZ2, QBZ3, QBZ4, QBZ5, QBZ9, QBZ10, QCZ1, QDZ1, QDZ2, QDZ3, QDZ4, QDZ6, QDZ7, QDZ8, QDZ9, QDZ10, QDZ11, QEZ1, QEZ2, QEZ3, QEZ4, QEZ5, QHZ1, QHZ2, QHZ3, QHZ4, PPZ1, PPZ2, SSZ1, SSZ2, SSZ3, SSZ4, PDXP1, PDXP2, PDXP3, PKYP1, PDYP1, PDYP2, PDYP3, PDYP4, PHYP1, PHYP2, PHYP3, PHYP4, PECP1, PECP2, QDTP1, QCRP1, QCRP2, QBRP1, QDRP1;

      double rUnloaded{0};
      double rRim{0};
      double p{-1};
      double p0{-1};
      double Fz0{-1};
      double cz{-1};
      double dz{-1};
      double sRelax{1};
      double epsx{0.1};
      double epsk{0.1};
      double sfFLo{1};
      double sfFLa{1};
      double sfM{1};
      double sfmux{-1};
      double sfmuy{-1};
      double sfkx{-1};
      double sfky{-1};
      double sfkg{-1};
      double sfkm{-1};

      double RvSx, vRoll, vSpin, slip, phi, Kyal, slipAnglePT1, rScrub;
 };

}

#endif
