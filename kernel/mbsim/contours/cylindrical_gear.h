/* Copyright (C) 2004-2018 MBSim Development Team
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

#ifndef CYLINDRICAL_GEAR_H_
#define CYLINDRICAL_GEAR_H_

#include "mbsim/contours/rigid_contour.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#include <openmbvcppinterface/cylindricalgear.h>

namespace MBSim {

  /**
   * \brief cylindrical gear contour
   * \author Martin Förg
   */
  class CylindricalGear : public RigidContour {
    public:

      CylindricalGear(const std::string& name="", Frame *R=nullptr) : RigidContour(name,R) { }

      /*!
       * \brief destructor
       */
      ~CylindricalGear() override = default;

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
      void setModule(double m_) { m = m_; }
      double getModule() { return m; }
      void setPressureAngle(double al_) { al = al_; }
      double getPressureAngle() { return al; }
      void setBacklash(double b_) { b = b_; }
      double getBacklash() { return b; }
      void setExternalToothed(bool ext_=true) { ext = ext_; }
      double getOutsideRadius() const { return R; }
      void setOutsideRadius(double R_) { R = R_; }
      bool getExternalToothed() const { return ext; }
      void setFlank(int flank) { signi = flank; }
      void setTooth(int tooth) { k = tooth; }
      double getEtaMax(double r, double s);
      double getPhiMaxHigh(int i) { return phiHigh[i]; }
      double getPhiMaxLow(int i) { return phiLow[i]; }
      double getPhiMinHigh(int i) { return -phiHigh[not i]; }
      double getPhiMinLow(int i) { return -phiLow[not i]; }
      double getSPhiMaxHigh(int i) { return sPhiHigh; }
      double getSPhiMinHigh(int i) { return -sPhiHigh; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        OpenMBVColoredBody ombv(diffuseColor,transparency,pointSize,lineWidth);
        openMBVRigidBody=ombv.createOpenMBV<OpenMBV::CylindricalGear>();
      }
      
    void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      int N{15};
      double w{5e-2};
      double be{0};
      double m{16e-3};
      double al{0.349065850398866};
      double b{0};
      bool ext{true};
      double R{0};

      int signi{0};
      int k{0};
      double delh;
      double r0;

      double phiHigh[2], phiLow[2], sPhiHigh;
  };

}

#endif
