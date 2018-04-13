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
 * Contact: thorsten.schindler@mytum.de
 */

#ifndef _FLEXIBLE_PLANAR_NURBS_CONTOUR_H_
#define _FLEXIBLE_PLANAR_NURBS_CONTOUR_H_

#include "mbsim/contours/contour.h"
#include "mbsimFlexibleBody/utils/contact_utils.h"

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
   * \brief flexible nurbs curve
   * \author Martin Foerg
   */
  class FlexiblePlanarNurbsContour : public MBSim::Contour {
    public:
      /**
       * \brief constructor 
       * \param name of contour
       */
      FlexiblePlanarNurbsContour(const std::string &name="") : MBSim::Contour(name) { }

      /**
       * \brief destructor
       */
      ~FlexiblePlanarNurbsContour() override = default;  

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      MBSim::ContourFrame* createContourFrame(const std::string &name="P") override;
      double getCurvature(const fmatvec::Vec2 &zeta);
//      fmatvec::Vec3 evalWn_t(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWs_t(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWu_t(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalPosition(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Ws(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer1Wu(const fmatvec::Vec2 &zeta) override;

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

      MBSim::ContactKinematics * findContactPairingWith(const std::type_info &type0, const std::type_info &type1) override { return findContactPairingFlexible(type0, type1); }

//      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
//        OpenMBVNurbsCurve ombv(diffuseColor,transparency);
//        openMBVRigidBody=ombv.createOpenMBV();
//      }
      
      void initializeUsingXML(xercesc::DOMElement *element) override;

//      void setNodes(const std::vector<double> &nodes_) { etaNodes = nodes_; }

      bool isZetaOutside(const fmatvec::Vec2 &zeta) override;

      void setOpen(bool open_) { open = open_; }

      void resetUpToDate() override { updCrvPos = true; updCrvVel = true; updCrvJac = true; updCrvGA = true; }

      void updateCurvePositions();
      void updateCurveVelocities();
      void updateCurveJacobians();
      void updateCurveGyroscopicAccelerations();

    protected:
      double continueEta(double eta_);
      void updateHessianMatrix(double eta);
      void updateHessianMatrix_t(double eta);
      const fmatvec::MatVx4& evalHessianMatrix(double eta){ if(updCrvPos or eta!=etaOld) updateHessianMatrix(eta); return hess; }
      const fmatvec::MatVx4& evalHessianMatrix_t(double eta){ updateHessianMatrix_t(eta); return hess_t; }

      bool interpolation{false};
      fmatvec::VecVI index;
      fmatvec::VecV knot;
      int degree{3};
      bool open{false};
      MBSim::NurbsCurve crvPos, crvVel, crvGA;
      std::vector<MBSim::NurbsCurve> crvJac;
      double etaOld{-1e10};
      fmatvec::MatVx4 hess, hess_t, hessTmp;
      bool updCrvPos{true};
      bool updCrvVel{true};
      bool updCrvJac{true};
      bool updCrvGA{true};

      std::shared_ptr<OpenMBV::DynamicNurbsCurve> openMBVNurbsCurve;
  };

}

#endif
