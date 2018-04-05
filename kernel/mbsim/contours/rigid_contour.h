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

#ifndef _RIGID_CONTOUR_H_
#define _RIGID_CONTOUR_H_

#include "mbsim/contours/contour.h"

namespace OpenMBV {
  class RigidBody;
}

namespace MBSim {

  /**
   * \brief basic class for rigid contours
   * \author Thorsten Schindler
   * \date 2009-04-20 initial commit (Thorsten Schindler)
   * \date 2009-07-15 initPlot (Thorsten Schindler)
   */
  class RigidContour : public Contour {
    public:
      /**
       * \brief constructor
       * \param name of point
       */
      RigidContour(const std::string &name, Frame *R_=nullptr) : Contour(name), R(R_) {}

      ~RigidContour() override = default;

      ContourFrame* createContourFrame(const std::string &name="P") override;

      Frame* getFrame() { return R; }
      Frame* getFrameOfReference() { return R; }
      void setFrameOfReference(Frame *frame) { R = frame; }
      void setFrameOfReference(const std::string &frame) { saved_frameOfReference = frame; }

      /* INHERITED INTERFACE OF ELEMENT */
      void plot() override;
      void init(InitStage stage, const InitConfigSet &config) override;
      /***************************************************/

      virtual fmatvec::Vec3 evalKrPS(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalKs(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalKt(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalKu(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalKv(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalKn(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalParDer1Ks(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalParDer2Ks(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalParDer1Kt(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalParDer2Kt(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalParDer1Kn(const fmatvec::Vec2 &zeta);
      virtual fmatvec::Vec3 evalParDer2Kn(const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of first tangent
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 evalParDer1Ku(const fmatvec::Vec2 &zeta);

      /**
       * \return derivative of first tangent
       * \param t time
       * \param cp contour position
       */

      virtual fmatvec::Vec3 evalParDer2Ku(const fmatvec::Vec2 &zeta);
      /**
       * \return derivative of second tangent
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 evalParDer1Kv(const fmatvec::Vec2 &zeta);

      /**
       * \return derivative second first tangent
       * \param t time
       * \param cp contour position
       */
      virtual fmatvec::Vec3 evalParDer2Kv(const fmatvec::Vec2 &zeta);

      fmatvec::Vec3 evalPosition(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Wn(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Wn(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Wu(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Wu(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer1Wv(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalParDer2Wv(const fmatvec::Vec2 &zeta) override;
      virtual fmatvec::Vec3 evalWrPS(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta) override;
      fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta) override;

      /**
       * \brief contact search for RigidContours
       * \author Markus Schneider
       * \date 2010-11-05 initial commit (Markus Schneider)
       */
      ContactKinematics * findContactPairingWith(const std::type_info &type0, const std::type_info &type1) override;

      void initializeUsingXML(xercesc::DOMElement *element) override;

      std::shared_ptr<OpenMBV::RigidBody>& getOpenMBVRigidBody() { return openMBVRigidBody; }

      void setOpenMBVRigidBody(const std::shared_ptr<OpenMBV::RigidBody> &ombvBody) { openMBVRigidBody = ombvBody; }

    protected:
      /**
       * \brief coordinate system of contour
       */
      Frame *R;

      std::shared_ptr<OpenMBV::RigidBody> openMBVRigidBody;

    private:
      std::string saved_frameOfReference;
  };

}

#endif /* _CONTOUR_H_ */

