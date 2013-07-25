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
 * Contact: thschindler@users.berlios.de
 */

#ifndef CIRCLE_H_
#define CIRCLE_H_

#include "fmatvec.h"
#include "mbsim/contour.h"
#include "mbsim/utils/eps.h"

namespace MBSim {

  /**
   * \brief circular contour with contact possibility from outside and inside and binormal in direction of the third column of the contour reference frame
   * \author Thorsten Schindler
   * \date 2009-07-13 initial commit (Thorsten Schindler)
   * \date 2009-12-21 adaptations concerning CircleHollow and CircleSolid
   */
  class Circle : public RigidContour {
    public:
      /*!
       * \brief constructor
       * \param name of circle
       * \default contact from inside
       */
      Circle(const std::string& name);

      /*! 
       * \brief constructor
       * \param name of circle
       * \param contact from outside?
       */
      Circle(const std::string& name, bool outCont_);

      /*! 
       * \brief constructor
       * \param name of circle
       * \param radius
       * \param contact from outside?
       */
      Circle(const std::string& name, double r_, bool outCont_);

      /*!
       * \brief destructor
       */
      virtual ~Circle();

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Circle"; }
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual double computeCurvature(ContourPointData &cp) { return curvature; }
      virtual fmatvec::Vec2 computeLagrangeParameter(const fmatvec::Vec3& WrPoint);
      /***************************************************/

      /* GETTER / SETTER */
      void setRadius(double r_);    	
      void setOutCont(bool outCont_);
      double getRadius() const;
      bool getOutCont() const;
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBV(bool enable=true);
#endif

    virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element);
    virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *element);

    protected:
      /** 
       * \brief radius
       */
      double r;

      /**
       * \brief curvature of circle
       */
      double curvature;

    private:
      /** 
       * \brief contact on outer surface?
       */
      bool outCont;	
  };

  inline void Circle::setRadius(double r_) { r = r_; outCont ? curvature = 1./r_ : curvature = -1./r_; }    	
  inline void Circle::setOutCont(bool outCont_) { outCont = outCont_; if(fabs(r)>epsroot()) outCont ? curvature = 1./r : curvature = -1./r; }
  inline double Circle::getRadius() const { return r; }
  inline bool Circle::getOutCont() const { return outCont; }

}

#endif /* CIRCLE_H_ */

