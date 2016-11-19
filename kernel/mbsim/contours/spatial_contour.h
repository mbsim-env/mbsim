/* Copyright (C) 2004-2016 MBSim Development Team
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

#ifndef _SPATIAL_CONTOUR_H_
#define _SPATIAL_CONTOUR_H_

#include "mbsim/contours/rigid_contour.h"

#include "mbsim/utils/boost_parameters.h"
#include <mbsim/utils/openmbv_utils.h>

namespace MBSim {

  class ContactKinematics;
  template <class Sig> class Function;

  /** 
   * \brief analytical description of contours with one contour parameter
   * \author Robert Huber
   * \date 2009-04-20 some comments (Thorsten Schindler)
   * \date 2009-06-04 new file (Thorsten Schindler)
   */
  class SpatialContour : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      SpatialContour(const std::string &name="", Frame *R=NULL) : RigidContour(name,R), funcCrPC(NULL), open(false) { }

      /**
       * \brief destructor
       */
      virtual ~SpatialContour();

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "SpatialContour"; }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      void init(InitStage stage);
      double getCurvature(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer1Ks(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer2Ks(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer1Kt(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer2Kt(const fmatvec::Vec2 &zeta);
      /***************************************************/

      /* GETTER / SETTER */
      void setContourFunction(Function<fmatvec::Vec3(fmatvec::Vec2)> *f);
      Function<fmatvec::Vec3(fmatvec::Vec2)>* getContourFunction() { return funcCrPC; }
      /***************************************************/

      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (etaNodes,(const std::vector<double>&),std::vector<double>())(xiNodes,(const std::vector<double>&),std::vector<double>())(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) {
        OpenMBVIndexedFaceSet ombv(diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV();
        ombvEtaNodes = etaNodes;
        ombvXiNodes = xiNodes;
      }
      
      virtual void initializeUsingXML(xercesc::DOMElement *element);

      void setNodes(const std::vector<double> &nodes_) { etaNodes = nodes_; }

      virtual bool isZetaOutside(const fmatvec::Vec2 &zeta) { return open and (zeta(0) < etaNodes[0] or zeta(0) > etaNodes[etaNodes.size()-1]); }

      void setOpen(bool open_=true) { open = open_; }

    protected:
      Function<fmatvec::Vec3(fmatvec::Vec2)> * funcCrPC;
      bool open;

      std::vector<double> ombvEtaNodes;
      std::vector<double> ombvXiNodes;
  };

}

#endif

