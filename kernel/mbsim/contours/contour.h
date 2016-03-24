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

#ifndef _CONTOUR_H_
#define _CONTOUR_H_

#include "mbsim/element.h"

namespace MBSim {

  class ContourFrame;
  class ContactKinematics;

  /**
   * \brief basic class for contour definition for rigid (which do not know about their shape) and flexible (they know how they look like) bodies
   * \author Martin Foerg
   * \date 2009-03-23 some comments (Thorsten Schindler)
   * \date 2009-04-20 RigidContour added (Thorsten Schindler)
   * \date 2009-06-04 not rigid things are in separate files
   * \date 2009-07-16 split from concret contours into new folder contours
   *
   * kinematics is stored in coordinate system class and is individually evaluated in specific contact kinematics
   */
  class Contour : public Element {
    public:
      /**
       * \brief constructor
       * \param name of contour
       */
      Contour(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Contour() { }

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "Contour"; }
      /***************************************************/

      /**
       * \brief find contact kinematics
       * \author Markus Schneider
       * \date 2010-11-05 initial commit (Markus Schneider)
       */
      virtual ContactKinematics * findContactPairingWith(std::string type0, std::string type1) = 0;

      virtual ContourFrame* createContourFrame(const std::string &name="P") { return NULL; }

      const std::vector<double>& getEtaNodes() const { return etaNodes; }
      const std::vector<double>& getXiNodes() const { return xiNodes; }

      void setEtaNodes(const std::vector<double> &etaNodes_) { etaNodes = etaNodes_; }
      void setXiNodes(const std::vector<double> &xiNodes_) { xiNodes = xiNodes_; }

      virtual int gethSize(int i=0) const { return hSize[i]; }
      virtual int gethInd(int i=0) const { return hInd[i]; }
      virtual void sethSize(int size, int i=0) { hSize[i] = size; }
      virtual void sethInd(int ind, int i=0) { hInd[i] = ind; }

      virtual fmatvec::Vec3 getKrPS(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getKs(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getKt(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getParDer1Ks(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getParDer2Ks(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getParDer1Kt(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 getParDer2Kt(const fmatvec::Vec2 &zeta);

      /**
       * \return position in world frame
       * \param contour position
       */
      virtual fmatvec::Vec3 getPosition(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return first tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getWu(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return second tangent in world frame
       * \param Lagrangian position
       */
      virtual fmatvec::Vec3 getWv(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return normal in world frame
       * \param contour position
       */
      virtual fmatvec::Vec3 getWn(double t, const fmatvec::Vec2 &zeta);

      virtual fmatvec::Vec3 getParDer1Kn(const fmatvec::Vec2 &zeta);

      virtual fmatvec::Vec3 getParDer2Kn(const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of first tangent
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getParDer1Ku(const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of first tangent
       * \param t time
       * \param cp contour position
       */

      virtual fmatvec::Vec3 getParDer2Ku(const fmatvec::Vec2 &zeta);
      /**
       * \return derivative of second tangent
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getParDer1Kv(const fmatvec::Vec2 &zeta);

      /**
       * \return derivative second first tangent
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getParDer2Kv(const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of normal in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getParDer1Wn(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of normal in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getParDer2Wn(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of first tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getParDer1Wu(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of first tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getParDer2Wu(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of second tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getParDer1Wv(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of second tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getParDer2Wv(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of first tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getWrPS(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return first tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getWs(double t, const fmatvec::Vec2 &zeta);

      /**
       * \return second tangent in world frame
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 getWt(double t, const fmatvec::Vec2 &zeta);

      virtual fmatvec::Mat3x2 getWN(double t, const fmatvec::Vec2 &zeta);

      virtual fmatvec::Mat3x2 getWR(double t, const fmatvec::Vec2 &zeta);

      virtual fmatvec::Mat3x2 getWU(double t, const fmatvec::Vec2 &zeta);

      virtual fmatvec::Mat3x2 getWV(double t, const fmatvec::Vec2 &zeta);

      virtual fmatvec::Vec2 getZeta(double t, const fmatvec::Vec3 &WrPS);

      virtual void updatePositions(double t, ContourFrame *frame);
      virtual void updateVelocities(double t, ContourFrame *frame);
      virtual void updateAccelerations(double t, ContourFrame *frame);
      virtual void updateJacobians(double t, ContourFrame *frame, int j=0);
      virtual void updateGyroscopicAccelerations(double t, ContourFrame *frame);

      void setThickness(double thickness_) { thickness = thickness_; }
      double getThickness() const { return thickness; }

      virtual bool isZetaOutside(const fmatvec::Vec2 &zeta) { return false; }

    protected:
      /**
       * \brief size and index of right hand side for frame JACOBIAN settings
       */
      int hSize[2], hInd[2];

      std::vector<double> etaNodes;
      std::vector<double> xiNodes;

      /**
       * \brief thickness of contour
       */
      double thickness;
  };

}

#endif /* _CONTOUR_H_ */
