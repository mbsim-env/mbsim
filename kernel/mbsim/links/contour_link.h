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

#include "mbsim/link.h"

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

      /**
       * \brief destructor
       */
      virtual ~ContourLink();

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
      Frame* getContourFrame(int i) { return cFrame[i]; }

      void resetUpToDate();
      virtual void updatePositions(double t) { }
      virtual void updateVelocities(double t) { }
      virtual void updateForceDirections(double t);
      void updateForce(double t);
      void updateMoment(double t);
      const fmatvec::Vec3& getGlobalRelativePosition(double t) { if(updPos) updatePositions(t); return WrP0P1; }
      const fmatvec::Vec3& getGlobalRelativeVelocity(double t) { if(updVel) updateVelocities(t); return WvP0P1; }
      const fmatvec::Vec3& getGlobalRelativeAngularVelocity(double t) { if(updVel) updateVelocities(t); return WomP0P1; }
      const fmatvec::Mat3xV& getGlobalForceDirection(double t) { if(updFD) updateForceDirections(t); return DF; }
      const fmatvec::Mat3xV& getGlobalMomentDirection(double t) { if(updFD) updateForceDirections(t); return DM; }
      const fmatvec::Vec3& getForce(double t) { if(updF) updateForce(t); return F; }
      const fmatvec::Vec3& getMoment(double t) { if(updM) updateMoment(t); return M; }

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

      Contour* contour[2];

//      ContourPointData cpData[2];

      std::vector<Frame*> cFrame;

#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::Group> openMBVForceGrp;
      boost::shared_ptr<OpenMBV::Arrow> openMBVArrowF;
      boost::shared_ptr<OpenMBV::Arrow> openMBVArrowM;
#endif

      bool updPos, updVel, updFD, updF, updM, updR;
  };
}

#endif

