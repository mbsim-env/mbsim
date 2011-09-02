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
 * Contact: mfoerg@users.berlios.de
 */

#ifndef _FRUSTUM_H_
#define _FRUSTUM_H_

#include "mbsim/contour.h"

namespace MBSim {

  /**
   * \brief frustum with axis in direction of second column of contour reference frame
   * \author Martin Foerg
   * \author Thorsten Schindler
   * \date 2009-04-20 some comments (Thorsten Schindler)
   */
  class Frustum : public RigidContour {
    public:
      /**
       * \brief constructor with contact from inside
       * \param name of contour
       */
      Frustum(const std::string &name) : RigidContour(name), r(2), h(0.), outCont(false) {}

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Frustum"; }
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF CONTOUR */
      virtual fmatvec::Vec computeLagrangeParameter(const fmatvec::Vec &WrPoint);
      /***************************************************/

      /**
       * \brief constructor
       * \param name of the contour
       * \param contact from outside?
       */
      Frustum(const std::string &name, bool outCont_) : RigidContour(name), r(2), h(0.), outCont(outCont_) {}

      /* GETTER / SETTER */
      void setRadii(const fmatvec::Vec &r_);
      const fmatvec::Vec& getRadii() const;
      void setHeight(double h_);
      double getHeight() const;
      void setOutCont(bool outCont_);
      bool getOutCont() const;
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
      void enableOpenMBV(bool enable=true);
#endif
      
      virtual void initializeUsingXML(TiXmlElement *element);

      private:
      /** 
       * \brief upper r(1) and lower radius r(0) in direction of the axis
       */
      fmatvec::Vec r;

      /** 
       * \brief height
       */
      double h;

      /** 
       * \brief contact on outer or inner surface?
       */
      bool outCont;
  };

  inline void Frustum::setRadii(const fmatvec::Vec &r_) { r = r_; }
  inline const fmatvec::Vec& Frustum::getRadii() const { return r; }
  inline void Frustum::setHeight(double h_) { h = h_; }
  inline double Frustum::getHeight() const { return h; }
  inline void Frustum::setOutCont(bool outCont_) { outCont = outCont_; }
  inline bool Frustum::getOutCont() const { return outCont; } 
}

#endif /* _FRUSTUM_H_ */
