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

  class ContourFrame;

  class MagicFormula62 : public TyreModel {
    public:
      enum TyreSide {
	file=0,
        left,
        right,
        unknown
      };

      MagicFormula62() : slipPoint(2) { }
      ~MagicFormula62() override;

      void setInputDataFile(const std::string& inputDataFile_) { inputDataFile = inputDataFile_; }
      void setTyreSide(TyreSide tyreSide_) { tyreSide = tyreSide_; }
      void setVerticalStiffness(double cz_) { cz = cz_; }
      void setVerticalDamping(double dz_) { dz = dz_; }
      void setNominalLoad(double Fz0_) { Fz0 = Fz0_; }
      void setInflationPressure(double p_) { p = p_; }
      void setRelaxationLengthForLongitudinalSlip(double six_) { six = six_; }
      void setRelaxationLengthForSideslip(double siy_) { siy = siy_; }
      void setScaleFactorForLongitudinalForce(double LFX_) { LFX = LFX_; }
      void setScaleFactorForLateralForce(double LFY_) { LFY = LFY_; }
      void setScaleFactorForOverturningMoment(double LMX_) { LMX = LMX_; }
      void setScaleFactorForRollingResistanceMoment(double LMY_) { LMY = LMY_; }
      void setScaleFactorForAligningMoment(double LMZ_) { LMZ = LMZ_; }
      void setScaleFactorForMomentArmOfLongitudinalForce(double LS_) { LS = LS_; }
      void setScaleFactorForLongitudinalFricitionCoefficient(double LMUX_) { LMUX = LMUX_; }
      void setScaleFactorForLateralFricitionCoefficient(double LMUY_) { LMUY = LMUY_; }
      void setScaleFactorForLongitudinalSlipStiffness(double LKX_) { LKX = LKX_; }
      void setScaleFactorForCorneringStiffness(double LKY_) { LKY = LKY_; }
      void setScaleFactorForCamberStiffness(double LKYC_) { LKYC = LKYC_; }
      void setScaleFactorForResidualTorque(double LKZC_) { LKZC = LKZC_; }
      void setMotorcycleKinematics(bool mck_) { mck = mck_; }
      void setContactPointTransformation(bool contactPointTransformation_) { contactPointTransformation = contactPointTransformation_; }
      void setTurnSlip(bool ts_) { ts = ts_; }
      void setReferenceTreadWidth(double rtw_) { rtw = rtw_; }

      void init(InitStage stage, const InitConfigSet &config) override;
      void initPlot(std::vector<std::string> &plotColumns) override;
      void plot(std::vector<double> &plotVector) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      int getxSize() const override;
      int getDMSize() const override { return 3; }

      fmatvec::VecV getContourParameters() const override;
      double evalFreeRadius() override;

      void updateGeneralizedForces() override;
      void updatexd() override;

      fmatvec::VecV getData() const override;

      bool motorcycleKinematics() const override { return mck; }

      void resetUpToDate() override;

    private:
      void importData();
      std::string inputDataFile;
      TyreSide tyreSide{file};
      int FITTYP;
      std::string TYRESIDE;
      bool mirroring{false};
      double v0, R0{0}, w{0}, rRim{0}, p{-1}, p0, Fz0{-1}, cz{-1}, dz{-1}, MC_CONTOUR_A, MC_CONTOUR_B, BREFF, DREFF, FREFF, Q_RE0, Q_V1, Q_V2, Q_FZ2, Q_FCX, Q_FCY, Q_CAM, PFZ1, Q_FCY2, Q_CAM1, Q_CAM2, Q_CAM3, Q_FYS1, Q_FYS2, Q_FYS3, rhobtm, czbtm, PCX1, PDX1, PDX2, PDX3, PEX1, PEX2, PEX3, PEX4, PKX1, PKX2, PKX3, PHX1, PHX2, PVX1, PVX2, PPX1, PPX2, PPX3, PPX4, RBX1, RBX2, RBX3, RCX1, REX1, REX2, RHX1, QSX1, QSX2, QSX3, QSX4, QSX5, QSX6, QSX7, QSX8, QSX9, QSX10, QSX11, QSX12, QSX13, QSX14, PPMX1, PCY1, PDY1, PDY2, PDY3, PEY1, PEY2, PEY3, PEY4, PEY5, PKY1, PKY2, PKY3, PKY4, PKY5, PKY6, PKY7, PHY1, PHY2, PVY1, PVY2, PVY3, PVY4, PPY1, PPY2, PPY3, PPY4, PPY5, RBY1, RBY2, RBY3, RBY4, RCY1, REY1, REY2, RHY1, RHY2, RVY1, RVY2, RVY3, RVY4, RVY5, RVY6, QSY1, QSY2, QSY3, QSY4, QSY5, QSY6, QSY7, QSY8, QBZ1, QBZ2, QBZ3, QBZ4, QBZ5, QBZ9, QBZ10, QCZ1, QDZ1, QDZ2, QDZ3, QDZ4, QDZ6, QDZ7, QDZ8, QDZ9, QDZ10, QDZ11, QEZ1, QEZ2, QEZ3, QEZ4, QEZ5, QHZ1, QHZ2, QHZ3, QHZ4, PPZ1, PPZ2, SSZ1, SSZ2, SSZ3, SSZ4, PDXP1, PDXP2, PDXP3, PKYP1, PDYP1, PDYP2, PDYP3, PDYP4, PHYP1, PHYP2, PHYP3, PHYP4, PECP1, PECP2, QDTP1, QCRP1, QCRP2, QBRP1, QDRP1, PRESMIN, PRESMAX, FZMIN, FZMAX, KPUMIN, KPUMAX, ALPMIN, ALPMAX, CAMMIN, CAMMAX, LFZ0, LCX, LMUX{-1}, LEX, LKX{-1}, LHX, LVX, LCY, LMUY{-1}, LEY, LKY{-1}, LKYC{-1}, LKZC{-1}, LHY, LVY, LTR, LRES, LXAL, LYKA, LVYKA, LS{-1}, LMX{-1}, LVMX, LMY{-1}, LMP, Cx0, Cy0, PCFX1, PCFX2, PCFX3, PCFY1, PCFY2, PCFY3, PCMZ1, Q_RB1, Q_RB2;
      double zeta0{1}, zeta1{1}, zeta2{1}, zeta3{1}, zeta4{1}, zeta5{1}, zeta6{1}, zeta7{1}, zeta8{1};
      double six{-1}, siy{-1};
      double rtw{-1};
      double LFX{1}, LFY{1}, LMZ{1};
      double LFM{1};
      int side{1};
      bool constsix{false}, constsiy{false};

      double vx, vsx, vcx, vcy, vc, ka, ga, Kyal, alF, rhoz, dpi, Re, Rs, phit{0}, phiF{0}, epsga{0};

      bool mck{false}, ts{false}, contactPointTransformation{true};

      std::vector<ContourFrame*> slipPoint;
 };

}

#endif
