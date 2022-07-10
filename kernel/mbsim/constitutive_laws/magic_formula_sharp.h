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

      void initializeUsingXML(xercesc::DOMElement *element) override;

      void updateGeneralizedForces() override;

    private:
      double cz{141000.000000};
      double dz{800.000000};
      double Fz0{1600.000000};
      double pKy1{15.791000};
      double pKy2{1.693500};
      double pKy3{1.460400};
      double pKy4{0.669000};
      double pKy5{0.187080};
      double pKy6{0.455120};
      double pKy7{0.013293};
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
      double Cy{0.900000};
      double rBx1{13.476000};
      double rBx2{11.354000};
      double Cxal{1.123100};
      double pDy1{1.300000};
      double pDy2{0.000000};
      double pDy3{0.000000};
      double pEy1{-2.222700};
      double pEy2{-1.669000};
      double pEy4{-4.288000};
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
 };

}

#endif
