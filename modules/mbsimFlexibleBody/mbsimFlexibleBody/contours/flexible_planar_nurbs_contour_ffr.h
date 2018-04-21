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

#ifndef _FLEXIBLE_PLANAR_NURBS_CONTOUR_FFR_H_
#define _FLEXIBLE_PLANAR_NURBS_CONTOUR_FFR_H_

#include "mbsimFlexibleBody/contours/flexible_contour.h"

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>
#include <mbsim/numerics/nurbs/nurbs_curve.h>

namespace OpenMBV {
  class DynamicNurbsCurve;
}

namespace MBSim {
  class ContourFrame;
}

namespace MBSimFlexibleBody {

  /*!  
   * \brief flexible planar nurbs contour with local interpolation 
   * \author Martin Foerg
   */
  class FlexiblePlanarNurbsContourFFR : public FlexibleContour {
    public:
      /**
       * \brief constructor 
       * \param name of contour
       */
      FlexiblePlanarNurbsContourFFR(const std::string &name="") : FlexibleContour(name) { }

      /**
       * \brief destructor
       */
      ~FlexiblePlanarNurbsContourFFR() override = default;  

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      MBSim::ContourFrame* createContourFrame(const std::string &name="P") override;
      double getCurvature(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWs_t(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWu_t(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalPosition(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Ws(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer1Wu(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParWvCParEta(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParWuPart(const fmatvec::Vec2 &zeta) override;

      fmatvec::Vec3 evalKs_t(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalKu_t(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer1Ks(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer1Ku(const fmatvec::Vec2 &zeta);

      void updatePositions(MBSim::ContourFrame *frame) override;
      void updateVelocities(MBSim::ContourFrame *frame) override;
      void updateAccelerations(MBSim::ContourFrame *frame) override;
      void updateJacobians(MBSim::ContourFrame *frame, int j=0) override;
      void updateGyroscopicAccelerations(MBSim::ContourFrame *frame) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setInterpolation(bool interpolation_) { interpolation = interpolation_; }
      void setIndices(const fmatvec::VecVI &index_) { index = index_; }
      void setKnotVector(const fmatvec::VecV &knot_) { knot = knot_; }
      void setDegree(int degree_) { degree = degree_; }
      /***************************************************/

      void plot() override;

//      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
//        OpenMBVNurbsCurve ombv(diffuseColor,transparency);
//        openMBVRigidBody=ombv.createOpenMBV();
//      }
      
      void initializeUsingXML(xercesc::DOMElement *element) override;

//      void setNodes(const std::vector<double> &nodes_) { etaNodes = nodes_; }

      bool isZetaOutside(const fmatvec::Vec2 &zeta) override;

      void setOpen(bool open_) { open = open_; }

      void resetUpToDate() override { updPos = true; updVel = true; }

    protected:
      double continueEta(double eta_);
      void updateHessianMatrix(double eta);
      void updateGlobalRelativePosition(double eta);
      void updateGlobalRelativeVelocity(double eta);
      const fmatvec::MatVx4& evalHessianMatrixPos(double eta) { if(eta!=etaOld) updateHessianMatrix(eta); return hessPos; }
      const std::vector<fmatvec::MatVx4>& evalHessianMatrixPhi(double eta) { if(eta!=etaOld) updateHessianMatrix(eta); return hessPhi; }
      const fmatvec::Vec3& evalGlobalRelativePosition(double eta) { if(updPos) updateGlobalRelativePosition(eta); return WrKP; }
      const fmatvec::Vec3& evalGlobalRelativeVelocity(double eta) { if(updVel) updateGlobalRelativeVelocity(eta); return Wvrel; }

      MBSim::Frame *R;
      bool interpolation{false};
      fmatvec::VecVI index;
      fmatvec::VecV knot;
      int degree{3};
      bool open{false};
      MBSim::NurbsCurve crvPos;
      std::vector<MBSim::NurbsCurve> crvPhi;
      double etaOld{-1e10};
      fmatvec::MatVx4 hessPos;
      std::vector<fmatvec::MatVx4> hessPhi;
      fmatvec::Vec3 WrKP, Wvrel;
      bool updPos{true};
      bool updVel{true};

      std::shared_ptr<OpenMBV::DynamicNurbsCurve> openMBVNurbsCurve;
  };

}

#endif
