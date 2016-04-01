/* Copyright (C) 2004-2014 MBSim Development Team
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

#include "mbsim/links/link.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
#endif

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class Group;
  class Arrow;
}
#endif

namespace MBSim {

  class Contour;
  class ContourFrame;

  /** 
   * \brief contour link
   * \author Martin Foerg
   */
  class ContourLink : public Link {
    public:
      /**
       * \brief constructor
       * \param name of link machanics
       */
      ContourLink(const std::string &name);

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updateh(double t, int i=0);
      virtual void updateW(double t, int i=0);
      virtual void updatedhdz(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void init(InitStage stage);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Link"; }
      virtual void plot(double t, double dt = 1);
      virtual void closePlot();
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual void updateWRef(const fmatvec::Mat& ref, int i=0);
      virtual void updateVRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatehRef(const fmatvec::Vec &hRef, int i=0);
      virtual void updatedhdqRef(const fmatvec::Mat& ref, int i=0);
      virtual void updatedhduRef(const fmatvec::SqrMat& ref, int i=0);
      virtual void updatedhdtRef(const fmatvec::Vec& ref, int i=0);
      virtual void updaterRef(const fmatvec::Vec &ref, int i=0);
      /***************************************************/

      void connect(Contour *contour0, Contour* contour1) {
        contour[0] = contour0;
        contour[1] = contour1;
      }

      Contour* getContour(int i) { return contour[i]; }
      ContourFrame* getContourFrame(int i) { return cFrame[i]; }

      void resetUpToDate();
      virtual void updatePositions(double t) { }
      virtual void updateVelocities(double t) { }
      virtual void updateForceDirections(double t);
      void updateForce(double t);
      void updateMoment(double t);
      const fmatvec::Vec3& evalGlobalRelativePosition() { if(updPos) updatePositions(0.); return WrP0P1; }
      const fmatvec::Vec3& evalGlobalRelativeVelocity() { if(updVel) updateVelocities(0.); return WvP0P1; }
      const fmatvec::Vec3& evalGlobalRelativeAngularVelocity() { if(updVel) updateVelocities(0.); return WomP0P1; }
      const fmatvec::Mat3xV& evalGlobalForceDirection() { if(updFD) updateForceDirections(0.); return DF; }
      const fmatvec::Mat3xV& evalGlobalMomentDirection() { if(updFD) updateForceDirections(0.); return DM; }
      const fmatvec::Vec3& evalForce() { if(updF) updateForce(0.); return F; }
      const fmatvec::Vec3& evalMoment() { if(updM) updateMoment(0.); return M; }

#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVForce(const boost::shared_ptr<OpenMBV::Arrow> &arrow) { openMBVArrowF = arrow; }
      void setOpenMBVMoment(const boost::shared_ptr<OpenMBV::Arrow> &arrow) { openMBVArrowM = arrow; }
#endif

    protected:
      /**
       * \brief difference vector of position, velocity and angular velocity
       */
      fmatvec::Vec3 WrP0P1, WvP0P1, WomP0P1;

      fmatvec::Mat3xV DF, DM;

      fmatvec::Vec3 F, M;

      fmatvec::Mat3xV RF, RM;

      /**
       * \brief indices of forces and torques
       */
      fmatvec::Index iF, iM;

      std::vector<Contour*> contour;

      std::vector<ContourFrame*> cFrame;

#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::Group> openMBVForceGrp;
      boost::shared_ptr<OpenMBV::Arrow> openMBVArrowF;
      boost::shared_ptr<OpenMBV::Arrow> openMBVArrowM;
#endif

      bool updPos, updVel, updFD, updF, updM, updR;
  };
}

#endif
