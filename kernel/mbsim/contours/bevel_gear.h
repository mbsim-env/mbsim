/* Copyright (C) 2004-2019 MBSim Development Team
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

#ifndef BEVELGEAR_H_
#define BEVELGEAR_H_

#include "mbsim/contours/rigid_contour.h"
#include "mbsim/functions/function.h"
#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>
#include <openmbvcppinterface/bevelgear.h>

namespace MBSim {

  /**
   * \brief bevel gear contour
   * \author Martin Förg
   */
  class BevelGear : public RigidContour {
    public:

      BevelGear(const std::string& name="", Frame *R=nullptr) : RigidContour(name,R) { }

      /*!
       * \brief destructor
       */
      ~BevelGear() override = default;

      /* INHERITED INTERFACE OF ELEMENT */
      void init(InitStage stage, const InitConfigSet &config) override;
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Ks(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Ks(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Kt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Kt(const fmatvec::Vec2 &zeta) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setNumberOfTeeth(int N_) { N = N_; }
      int getNumberOfTeeth() { return N; }
      void setWidth(double w_) { w = w_; }
      double getWidth() { return w; }
      void setHelixAngle(double be_) { be = be_; }
      double getHelixAngle() { return be; }
      void setPitchAngle(double ga_) { ga = ga_; }
      double getPitchAngle() { return ga; }
      void setModule(double m_) { m = m_; }
      double getModule() { return m; }
      void setPressureAngle(double al_) { al = al_; }
      double getPressureAngle() { return al; }
      void setBacklash(double b_) { b = b_; }
      double getBacklash() { return b; }
      void setFlank(int flank) { signi = flank; }
      void setTooth(int tooth) { k = tooth; }
      double getPhiMax(double h, double s, int signi);
      double getPhiMaxHigh(int i) { return phiMaxHigh[i]; }
      double getPhiMaxLow(int i) { return phiMaxLow[i]; }
      double getPhiMinHigh(int i) { return phiMinHigh[i]; }
      double getPhiMinLow(int i) { return phiMinLow[i]; }
      double getSPhiMaxHigh(int i) { return sPhiMaxHigh[i]; }
      double getSPhiMinHigh(int i) { return sPhiMinHigh[i]; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        OpenMBVColoredBody ombv(diffuseColor,transparency,pointSize,lineWidth);
        openMBVRigidBody=ombv.createOpenMBV<OpenMBV::BevelGear>(); 
      }
      
    void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      class Residuum : public Function<double(double)> {
        private:
          double h, s, r0, r1, al, be, ga;
          int signi;
        public:
          Residuum(double h_, double s_, double r0_, double r1_, double al_, double be_, double ga_, int signi_) : h(h_), s(s_), r0(r0_), r1(r1_), al(al_), be(be_), ga(ga_), signi(signi_) { }
          double operator()(const double &phi) override;
      };

      int N{15};
      double w{5e-2};
      double be{0};
      double ga{0.785398163397448};
      double m{16e-3};
      double al{0.349065850398866};
      double b{0};

      int signi{0};
      int k{0};
      double delh;
      double r0;
      double r1;
      double d;

      double phiMaxHigh[2], phiMaxLow[2], phiMinHigh[2], phiMinLow[2], sPhiMaxHigh[2], sPhiMinHigh[2];
  };

}

#endif
