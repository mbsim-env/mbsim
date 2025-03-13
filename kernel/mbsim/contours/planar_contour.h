/* Copyright (C) 2004-2009 MBSim Development Team
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

#ifndef _PLANAR_CONTOUR_H_
#define _PLANAR_CONTOUR_H_

#include "mbsim/contours/rigid_contour.h"
#include "mbsim/frames/frame.h"
#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  template <class Sig> class Function;

  /** 
   * \brief analytical description of contours with one contour parameter
   * \author Robert Huber
   * \date 2009-04-20 some comments (Thorsten Schindler)
   * \date 2009-06-04 new file (Thorsten Schindler)
   */
  class PlanarContour : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      PlanarContour(const std::string &name="", Frame *R=nullptr) : RigidContour(name,R) { }

      /**
       * \brief destructor
       */
      ~PlanarContour() override;

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      void init(InitStage stage, const InitConfigSet &config) override;
      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalKn(const fmatvec::Vec2 &zeta) override { return crossProduct(evalKu(zeta),ez); }
      fmatvec::Vec3 evalParDer1Ks(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Kt(const fmatvec::Vec2 &zeta) override { return zero3; }
      fmatvec::Vec3 evalParDer1Kn(const fmatvec::Vec2 &zeta) override;

      fmatvec::Vec3 evalWn(const fmatvec::Vec2 &zeta) override { return crossProduct(evalWu(zeta),R->evalOrientation().col(2)); }

      fmatvec::Vec2 evalCurvatures(const fmatvec::Vec2 &zeta) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setContourFunction(Function<fmatvec::Vec3(double)> *func);
      Function<fmatvec::Vec3(double)>* getContourFunction() { return funcCrPC; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (nodes,(const std::vector<double>&),std::vector<double>())(filled,(bool),0)(diffuseColor,(const fmatvec::Vec3&),fmatvec::Vec3(std::vector<double>{-1,1,1}))(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        ombv = std::shared_ptr<OpenMBVPlanarContour>(new OpenMBVPlanarContour(nodes,filled,diffuseColor,transparency,pointSize,lineWidth));
        openMBVRigidBody=ombv->createOpenMBV();
      }
      
      void initializeUsingXML(xercesc::DOMElement *element) override;

      void setNodes(const std::vector<double> &nodes_) { etaNodes = nodes_; }

      bool isZetaOutside(const fmatvec::Vec2 &zeta) override { return open and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]); }

      void setOpen(bool open_=true) { open = open_; }

    protected:
      Function<fmatvec::Vec3(double)> * funcCrPC{nullptr};
      bool open{false};
      std::shared_ptr<OpenMBVPlanarContour> ombv;
  };

}

#endif
