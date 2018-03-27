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

#ifndef _SPATIAL_NURBS_CONTOUR_H_
#define _SPATIAL_NURBS_CONTOUR_H_

#include "mbsim/contours/rigid_contour.h"

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>
#include <mbsim/numerics/nurbs/nurbs_surface.h>

namespace MBSim {

  /** 
   * \brief nurbs surface
   * \author Martin Foerg
   */
  class SpatialNurbsContour : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      SpatialNurbsContour(const std::string &name="", Frame *R=nullptr) : RigidContour(name,R) { }

      /**
       * \brief destructor
       */
      ~SpatialNurbsContour() override = default;

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      void init(InitStage stage, const InitConfigSet &config) override;
      double getCurvature(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Ks(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Ks(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Kt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Kt(const fmatvec::Vec2 &zeta) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setControlPoints(const fmatvec::GeneralMatrix<fmatvec::Vec4> &cp_) { cp = cp_; }
      void setEtaKnotVector(const fmatvec::Vec &uKnot_) { uKnot = uKnot_; }
      void setXiKnotVector(const fmatvec::Vec &vKnot_) { vKnot = vKnot_; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVNurbsSurface ombv(diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV();
      }
      
      void initializeUsingXML(xercesc::DOMElement *element) override;

//      void setNodes(const std::vector<double> &nodes_) { etaNodes = nodes_; }

      bool isZetaOutside(const fmatvec::Vec2 &zeta) override { return open and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1] or zeta(1) < xiNodes[0] or zeta(1) > xiNodes[xiNodes.size()-1]); }

      void setOpen(bool open_=true) { open = open_; }

    protected:
      void updateHessianMatrix(const fmatvec::Vec2 &zeta);
      const fmatvec::GeneralMatrix<fmatvec::Vec4>& evalHessianMatrix(const fmatvec::Vec2 &zeta){ if(zeta!=zetaOld) updateHessianMatrix(zeta); return hess; }

      fmatvec::GeneralMatrix<fmatvec::Vec4> cp;
      fmatvec::Vec uKnot, vKnot;
      bool open{false};
      NurbsSurface srf;
      fmatvec::Vec2 zetaOld;
      fmatvec::GeneralMatrix<fmatvec::Vec4> hess;
  };

}

#endif
