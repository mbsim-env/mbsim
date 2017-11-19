/* Copyright (C) 2004-2016 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _CONTOUR_LINK_H_
#define _CONTOUR_LINK_H_

#include "mbsim/links/mechanical_link.h"

namespace MBSim {

  class Contour;
  class ContourFrame;

  /** 
   * \brief contour link
   * \author Martin Foerg
   */
  class ContourLink : public MechanicalLink {
    public:
      /**
       * \brief constructor
       * \param name of link machanics
       */
      ContourLink(const std::string &name);

      /**
       * \brief destructor
       */
      ~ContourLink() override;

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      void init(InitStage stage, const InitConfigSet &config) override;
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      void updateWRef(const fmatvec::Mat& ref, int i=0) override;
      void updateVRef(const fmatvec::Mat& ref, int i=0) override;
      void updatehRef(const fmatvec::Vec &hRef, int i=0) override;
      virtual void updatedhdqRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& ref, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& ref, int i=0);
      void updaterRef(const fmatvec::Vec &ref, int i=0) override;
      /***************************************************/

      void connect(Contour *contour0, Contour* contour1);

      Contour* getContour(int i) { return contour[i]; }
      ContourFrame* getContourFrame(int i) { return cFrame[i]; }

      void resetUpToDate() override;

      void updatePositions() override { }
      void updateVelocities() override { }
      void updateForce() override;
      void updateMoment() override;
      void updateForceDirections() override;
      const fmatvec::Vec3& evalGlobalRelativePosition() { if(updPos) updatePositions(); return WrP0P1; }
      const fmatvec::Vec3& evalGlobalRelativeVelocity() { if(updVel) updateVelocities(); return WvP0P1; }
      const fmatvec::Vec3& evalGlobalRelativeAngularVelocity() { if(updVel) updateVelocities(); return WomP0P1; }
      const fmatvec::Mat3xV& evalGlobalForceDirection(int i=0) { if(updDF) updateForceDirections(); return DF; }
      const fmatvec::Mat3xV& evalGlobalMomentDirection(int i=0) { if(updDF) updateForceDirections(); return DM; }


      void initializeUsingXML(xercesc::DOMElement *element) override;

    protected:
      /**
       * \brief difference vector of position, velocity and angular velocity
       */
      fmatvec::Vec3 WrP0P1, WvP0P1, WomP0P1;

      fmatvec::Mat3xV DF, DM;

      std::vector<Contour*> contour;

      std::vector<ContourFrame*> cFrame;

      bool updPos, updVel, updDF;

    private:
      std::string saved_ref1, saved_ref2;
  };

}

#endif
