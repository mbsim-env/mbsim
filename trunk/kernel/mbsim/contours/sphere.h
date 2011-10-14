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

#include "fmatvec.h"
#include "mbsim/contour.h"

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
      Sphere(const std::string &name) : RigidContour(name), r(0.) {}

      /**
       * \brief constructor
       * \param name of sphere
       * \param radius of sphere
       */
      Sphere(const std::string &name, double r_) : RigidContour(name), r(r_) {}
      
      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Sphere"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setRadius(double r_) { r = r_; }
      double getRadius() const { return r; }
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBV(bool enable=true);
#endif

      virtual void initializeUsingXML(TiXmlElement *element);

    protected:
      /** 
       * \brief radius
       */
      double r;
  };

}

#endif /* SPHERE_H_ */

