/* Copyright (C) 2004-2011 MBSim Development Team
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

#ifndef _LINE_SEGMENT_H_
#define _LINE_SEGMENT_H_

#include "mbsim/contour.h"

namespace MBSim {

  class ContourPointData;

  /**
   * \brief line segment with two bounds
   * \author Martin Foerg
   * \date 2010-05-20 initial commit (Martin Foerg)
   * \date 2011-01-27 some comments (Thomas Cebulla)
   */ 
  class LineSegment : public MBSim::RigidContour {	
    public:
      /**
       * \brief constructor
       * \param name of line segment
       */
      LineSegment(const std::string& name="");

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "LineSegment"; }
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual double computeCurvature(ContourPointData &cp) { return 0; }
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBV(bool enable=true, double size=1., int number=10);
#endif

      virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
      virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);

      /* GETTER / SETTER */
      void setLength(double length_) {length = length_; }
      double getLength() const {return length; }
      void setThickness(double thickness_) {thickness = thickness_;}
      double getThickness() const {return thickness;}
      /***************************************************/

    private:
      /**
       * \brief length of line segment
       */
      double length;

      /**
       * \brief thickness of line segment
       */
      double thickness;
  };      
}

#endif /* _LINE_SEGMENT_H_ */

