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

#ifndef _FCL_MESH_H_
#define _FCL_MESH_H_

#include "mbsim/contours/fcl_contour.h"
#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>
#include "mbsim/utils/index.h"

namespace MBSim {

  /**
   * \brief FCLMesh
   */
  class FCLMesh : public FCLContour {
    public:
      /**
       * \brief possible collision structures
       */
      enum CollisionStructure {
        AABB=0,
        KDOP,
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
      FCLMesh(const std::string &name="", Frame *R=nullptr) : FCLContour(name,R) { }

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/
      void initializeUsingXML(xercesc::DOMElement *element) override;
      void init(InitStage stage, const InitConfigSet &config) override;
      /***************************************************/

      /* GETTER / SETTER */
      void setVertices(const fmatvec::MatVx3 &vertex_) { vertex = vertex_; }
      void setTriangles(const fmatvec::Matrix<fmatvec::General, fmatvec::Var, fmatvec::Fixed<3>, Index> &triangle_) { triangle = triangle_; }
      void setCollisionStructure(CollisionStructure collisionStructure_) { collisionStructure = collisionStructure_; }
      void setN(bool N_) { N = N_; }
      void setComputeLocalAABB(bool computeLocalAABB_) { computeLocalAABB = computeLocalAABB_; }
      /***************************************************/

//      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
//        OpenMBVIndexedFaceSet ombv(fmatvec::Vec3(),diffuseColor,transparency);
//        openMBVRigidBody=ombv.createOpenMBV(); 
//      }

    private:
      /**
       * \brief vertices
       */
      fmatvec::MatVx3 vertex;

      /**
       * \brief triangles
       */
      fmatvec::Matrix<fmatvec::General, fmatvec::Var, fmatvec::Fixed<3>, Index> triangle;

      /**
       * \brief collision structure
       */
      CollisionStructure collisionStructure{AABB};

      /**
       * \brief parameter for KDOP collision structure
       */
      int N{16};

      /**
       * \brief compute local AABB
       */
      bool computeLocalAABB{true};
  };
}

#endif
