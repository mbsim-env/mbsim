/* Copyright (C) 2004-2010 MBSim Development Team
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

#ifndef _FACE_H_
#define _FACE_H_

#include "mbsim/contour.h"

namespace MBSim {


  /** 
   * \brief plane with borders
   * \author Martin Foerg
   *
   * normal equals first column in orientation matrix
   */
  class Face : public RigidContour {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Face(const std::string &name);

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBV(bool enable=true, double size=1., int number=10);
#endif

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);

      void setBounds(const fmatvec::Mat &bound_) {bound = bound_;}
      const fmatvec::Mat& getBounds() const { return bound; }

    private:
      fmatvec::Mat bound;
  };
}

#endif /* _FACE_H_ */
