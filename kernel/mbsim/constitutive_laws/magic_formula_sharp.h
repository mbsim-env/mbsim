/* Copyright (C) 2004-2022 MBSim Development Team
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

#ifndef _MAGIC_FORMULA_SHARP_H_
#define _MAGIC_FORMULA_SHARP_H_

#include <mbsim/constitutive_laws/tyre_model.h>

namespace MBSim {

  class MagicFormulaSharp : public TyreModel {
    public:
      MagicFormulaSharp() = default;
      ~MagicFormulaSharp() override = default;

      void setcz(double cz_) { cz = cz_; }
      void setdz(double dz_) { dz = dz_; }
      void setFz0(double Fz0_) { Fz0 = Fz0_; }
      void setpKy1(double pKy1_) { pKy1 = pKy1_; }
      void setpKy2(double pKy2_) { pKy2 = pKy2_; }
      void setpKy3(double pKy3_) { pKy3 = pKy3_; }
      void setpKy4(double pKy4_) { pKy4 = pKy4_; }
      void setpKy5(double pKy5_) { pKy5 = pKy5_; }
      void setpKy6(double pKy6_) { pKy6 = pKy6_; }
      void setpKy7(double pKy7_) { pKy7 = pKy7_; }
      void setpDx1(double pDx1_) { pDx1 = pDx1_; }
      void setpDx2(double pDx2_) { pDx2 = pDx2_; }
      void setpEx1(double pEx1_) { pEx1 = pEx1_; }
      void setpEx2(double pEx2_) { pEx2 = pEx2_; }
      void setpEx3(double pEx3_) { pEx3 = pEx3_; }
      void setpEx4(double pEx4_) { pEx4 = pEx4_; }
      void setpKx1(double pKx1_) { pKx1 = pKx1_; }
      void setpKx2(double pKx2_) { pKx2 = pKx2_; }
      void setpKx3(double pKx3_) { pKx3 = pKx3_; }
      void setCx(double Cx_) { Cx = Cx_; }
      void setCy(double Cy_) { Cy = Cy_; }
      void setrBx1(double rBx1_) { rBx1 = rBx1_; }
      void setrBx2(double rBx2_) { rBx2 = rBx2_; }
      void setCxal(double Cxal_) { Cxal = Cxal_; }
      void setpDy1(double pDy1_) { pDy1 = pDy1_; }
      void setpDy2(double pDy2_) { pDy2 = pDy2_; }
      void setpDy3(double pDy3_) { pDy3 = pDy3_; }
      void setpEy1(double pEy1_) { pEy1 = pEy1_; }
      void setpEy2(double pEy2_) { pEy2 = pEy2_; }
      void setpEy4(double pEy4_) { pEy4 = pEy4_; }
      void setCga(double Cga_) { Cga = Cga_; }
      void setEga(double Ega_) { Ega = Ega_; }
      void setrBy1(double rBy1_) { rBy1 = rBy1_; }
      void setrBy2(double rBy2_) { rBy2 = rBy2_; }
      void setrBy3(double rBy3_) { rBy3 = rBy3_; }
      void setCyka(double Cyka_) { Cyka = Cyka_; }
      void setqHz3(double qHz3_) { qHz3 = qHz3_; }
      void setqHz4(double qHz4_) { qHz4 = qHz4_; }
      void setqBz1(double qBz1_) { qBz1 = qBz1_; }
      void setqBz2(double qBz2_) { qBz2 = qBz2_; }
      void setqBz5(double qBz5_) { qBz5 = qBz5_; }
      void setqBz6(double qBz6_) { qBz6 = qBz6_; }
      void setqBz9(double qBz9_) { qBz9 = qBz9_; }
      void setqBz10(double qBz10_) { qBz10 = qBz10_; }
      void setqDz1(double qDz1_) { qDz1 = qDz1_; }
      void setqDz2(double qDz2_) { qDz2 = qDz2_; }
      void setqDz3(double qDz3_) { qDz3 = qDz3_; }
      void setqDz4(double qDz4_) { qDz4 = qDz4_; }
      void setqDz8(double qDz8_) { qDz8 = qDz8_; }
      void setqDz9(double qDz9_) { qDz9 = qDz9_; }
      void setqDz10(double qDz10_) { qDz10 = qDz10_; }
      void setqDz11(double qDz11_) { qDz11 = qDz11_; }
      void setqEz1(double qEz1_) { qEz1 = qEz1_; }
      void setqEz2(double qEz2_) { qEz2 = qEz2_; }
      void setqEz5(double qEz5_) { qEz5 = qEz5_; }
      void setCt(double Ct_) { Ct = Ct_; }
      void setc1Rel(double c1Rel_) { c1Rel = c1Rel_; }
      void setc2Rel(double c2Rel_) { c2Rel = c2Rel_; }
      void setc3Rel(double c3Rel_) { c3Rel = c3Rel_; }
      void setScaleFactorForCamberStiffness(double sfKyga_) { sfKyga = sfKyga_; }
      void setScaleFactorForLongitudinalForce(double sfFx_) { sfFx = sfFx_; }
      void setScaleFactorForLateralForce(double sfFy_) { sfFy = sfFy_; }
      void setScaleFactorForAligningMoment(double sfMz_) { sfMz = sfMz_; }

      void initPlot(std::vector<std::string> &plotColumns) override;
      void plot(std::vector<double> &plotVector) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void updateGeneralizedForces() override;

      int getDataSize() const override { return 8; }
      fmatvec::VecV getData() const override;

    private:
      double cz{0};
      double dz{0};
      double Fz0{0};
      double pKy1{0};
      double pKy2{0};
      double pKy3{0};
      double pKy4{0};
      double pKy5{0};
      double pKy6{0};
      double pKy7{0};
      double pDx1{0};
      double pDx2{0};
      double pEx1{0};
      double pEx2{0};
      double pEx3{0};
      double pEx4{0};
      double pKx1{0};
      double pKx2{0};
      double pKx3{0};
      double Cx{0};
      double Cy{0};
      double rBx1{0};
      double rBx2{0};
      double Cxal{0};
      double pDy1{0};
      double pDy2{0};
      double pDy3{0};
      double pEy1{0};
      double pEy2{0};
      double pEy4{0};
      double Cga{0};
      double Ega{0};
      double rBy1{0};
      double rBy2{0};
      double rBy3{0};
      double Cyka{0};
      double qHz3{0};
      double qHz4{0};
      double qBz1{0};
      double qBz2{0};
      double qBz5{0};
      double qBz6{0};
      double qBz9{0};
      double qBz10{0};
      double qDz1{0};
      double qDz2{0};
      double qDz3{0};
      double qDz4{0};
      double qDz8{0};
      double qDz9{0};
      double qDz10{0};
      double qDz11{0};
      double qEz1{0};
      double qEz2{0};
      double qEz5{0};
      double Ct{0};
      double c1Rel{0};
      double c2Rel{0};
      double c3Rel{0};
      double sfKyga{1};
      double sfFx{1};
      double sfFy{1};
      double sfMz{1};

      double vsx, vx, ka, ga, Kyal, si, be, Rs;
 };

}

#endif
