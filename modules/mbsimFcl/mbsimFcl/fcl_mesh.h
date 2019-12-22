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

#ifndef _MBSIMFCL_FCL_MESH_H_
#define _MBSIMFCL_FCL_MESH_H_

#include "mbsimFcl/fcl_contour.h"
#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>
#include "mbsim/utils/index.h"
#include <openmbvcppinterface/indexedfaceset.h>

namespace MBSimFcl {

  /**
   * \brief FclMesh
   */
  class FclMesh : public FclContour {
    public:
      /**
       * \brief possible collision structures
       */
      enum CollisionStructure {
        AABB=0,
        KDOP16,
        KDOP18,
        KDOP24,
        kIOS,
        OBB,
        OBBRSS,
        RSS,
        unknown
      };

      /**
       * \brief constructor
       * \param name of box
       * \param length of box
       * \param R frame of reference
       */
      FclMesh(const std::string &name="", MBSim::Frame *R=nullptr) : FclContour(name,R) { }

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void initializeUsingXML(xercesc::DOMElement *element) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setVertices(const fmatvec::MatVx3 &vertex_) { vertex <<= vertex_; }
      void setTriangles(const fmatvec::Matrix<fmatvec::General, fmatvec::Var, fmatvec::Fixed<3>, MBSim::Index> &triangle_) { triangle <<= triangle_; }
      void setCollisionStructure(CollisionStructure collisionStructure_) { collisionStructure = collisionStructure_; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, MBSim::tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0)(pointSize,(double),0)(lineWidth,(double),0))) {
        MBSim::OpenMBVColoredBody ombv(diffuseColor,transparency,pointSize,lineWidth);
        openMBVRigidBody=ombv.createOpenMBV<OpenMBV::IndexedFaceSet>();
      }

    private:
      /**
       * \brief vertices
       */
      fmatvec::MatVx3 vertex;

      /**
       * \brief triangles
       */
      fmatvec::Matrix<fmatvec::General, fmatvec::Var, fmatvec::Fixed<3>, MBSim::Index> triangle;

      /**
       * \brief collision structure
       */
      CollisionStructure collisionStructure{AABB};
  };
}

#endif
