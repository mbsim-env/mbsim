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

#ifndef PLANAR_GEAR_H_
#define PLANAR_GEAR_H_

#include "mbsim/contours/rigid_contour.h"
#include "mbsim/functions/function.h"
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#include <openmbvcppinterface/planargear.h>

namespace MBSim {

  /**
   * \brief planar gear contour
   * \author Martin Förg
   */
  class PlanarGear : public RigidContour {
    public:

      PlanarGear(const std::string& name="", Frame *R=nullptr) : RigidContour(name,R) { }

      /*!
       * \brief destructor
       */
      ~PlanarGear() override = default;

      /* INHERITED INTERFACE OF ELEMENT */
      void init(InitStage stage, const InitConfigSet &config) override;
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setNumberOfTeeth(int N_) { N = N_; }
      int getNumberOfTeeth() { return N; }
      double getHeight() { return h; }
      void setHeight(double h_) { h = h_; }
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
      void setFlank(int flank) { signi = flank; }
      void setTooth(int tooth) { k = tooth; }
      double getPhiMax(double h, double s);
      double getPhiMaxHigh(int i) { return phiMaxHigh; }
      double getPhiMaxLow(int i) { return phiMaxLow; }
      double getPhiMinHigh(int i) { return phiMinHigh; }
      double getPhiMinLow(int i) { return phiMinLow; }
      double getSPhiMaxHigh(int i) { return sPhiMaxHigh; }
      double getSPhiMinHigh(int i) { return sPhiMinHigh; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        OpenMBVColoredBody ombv(diffuseColor,transparency,pointSize,lineWidth);
        openMBVRigidBody=ombv.createOpenMBV<OpenMBV::PlanarGear>(); 
      }
      
    void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      class Residuum : public Function<double(double)> {
        private:
          double h, s, r0, al, be;
        public:
          Residuum(double h_, double s_, double r0_, double al_, double be_) : h(h_), s(s_), r0(r0_), al(al_), be(be_) { }
          double operator()(const double &phi) override;
      };

      int N{15};
      double h{5e-2};
      double w{5e-2};
      double be{0};
      double m{16e-3};
      double al{0.349065850398866};
      double b{0};

      int signi{0};
      int k{0};
      double delh;
      double r0;

      double phiMaxHigh, phiMaxLow, phiMinHigh, phiMinLow, sPhiMaxHigh, sPhiMinHigh;
  };

}

#endif
