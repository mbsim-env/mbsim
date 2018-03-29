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

#ifndef _PLANAR_NURBS_CONTOUR_H_
#define _PLANAR_NURBS_CONTOUR_H_

#include "mbsim/contours/rigid_contour.h"

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>
#include <mbsim/numerics/nurbs/nurbs_curve.h>

namespace MBSim {

  /** 
   * \brief nurbs curve
   * \author Martin Foerg
   */
  class PlanarNurbsContour : public RigidContour {
    public:
      enum Interpolation {
        equallySpaced = 0,
        chordLength,
        none,
        unknown,
      };

      /**
       * \brief constructor
       * \param name of contour
       */
      PlanarNurbsContour(const std::string &name="", Frame *R=nullptr) : RigidContour(name,R) { }

      /**
       * \brief destructor
       */
      ~PlanarNurbsContour() override = default;

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      void init(InitStage stage, const InitConfigSet &config) override;
      double getCurvature(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Ks(const fmatvec::Vec2 &zeta) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setInterpolation(Interpolation interpolation_) { interpolation = interpolation_; }
      void setControlPoints(const fmatvec::MatVx4 &cp_) { cp = cp_; }
      void setKnotVector(const fmatvec::VecV &knot_) { knot = knot_; }
      void setDegree(int degree_) { degree = degree_; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (nodes,(const std::vector<double>&),std::vector<double>())(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVNurbsCurve ombv(diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }
      
      void initializeUsingXML(xercesc::DOMElement *element) override;

//      void setNodes(const std::vector<double> &nodes_) { etaNodes = nodes_; }

      bool isZetaOutside(const fmatvec::Vec2 &zeta) override { return open and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]); }

      void setOpen(bool open_=true) { open = open_; }

    protected:
      void updateHessianMatrix(double eta);
      const fmatvec::MatVx4& evalHessianMatrix(double eta){ if(eta!=etaOld) updateHessianMatrix(eta); return hess; }

      Interpolation interpolation{none};
      fmatvec::MatVx4 cp;
      fmatvec::VecV knot;
      int degree{3};
      bool open{false};
      NurbsCurve crv;
      double etaOld;
      fmatvec::MatVx4 hess;
  };

}

#endif
