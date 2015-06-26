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

#ifndef SPHERE_H_
#define SPHERE_H_

#include "fmatvec/fmatvec.h"
#include "mbsim/contour.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include <mbsim/utils/openmbv_utils.h>
#endif

namespace MBSim {

  /**
   * \brief sphere 
   * \author Martin Foerg
   * \date 2009-04-20 some comments (Thorsten Schindler) 
   * \date 2009-05-28 new interface (Bastian Esefeld)
   */
  class Sphere : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Sphere(const std::string &name="", Frame *R=0) : RigidContour(name,R), r(0.) {}

      /**
       * \brief constructor
       * \param name of sphere
       * \param radius of sphere
       */
      Sphere(const std::string &name, double r_, Frame *R=0) : RigidContour(name,R), r(r_) {}
      
      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Sphere"; }
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      fmatvec::Vec2 getLagrangeParameter(const fmatvec::Vec3 &WrPoint);
      /**********************************/

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const { return r; }
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBV, tag, (optional (diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVSphere ombv(1,diffuseColor,transparency);
        openMBVRigidBody=ombv.createOpenMBV(); 
      }
#endif

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

    protected:
      /** 
       * \brief radius
       */
      double r;
  };

}

#endif /* SPHERE_H_ */

