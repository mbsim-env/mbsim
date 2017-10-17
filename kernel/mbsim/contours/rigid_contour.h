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
      RigidContour(const std::string &name, Frame *R_=NULL) : Contour(name), R(R_) {}

      virtual ~RigidContour() { }

      ContourFrame* createContourFrame(const std::string &name="P");

      Frame* getFrame() { return R; }
      Frame* getFrameOfReference() { return R; }
      void setFrameOfReference(Frame *frame) { R = frame; }
      void setFrameOfReference(const std::string &frame) { saved_frameOfReference = frame; }

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "RigidContour"; }
      virtual void plot();
      virtual void init(InitStage stage, const InitConfigSet &config);
      /***************************************************/

      fmatvec::Vec3 evalPosition(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer1Wn(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer2Wn(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer1Wu(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer2Wu(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer1Wv(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalParDer2Wv(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWrPS(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWs(const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 evalWt(const fmatvec::Vec2 &zeta);

      /**
       * \brief contact search for RigidContours
       * \author Markus Schneider
       * \date 2010-11-05 initial commit (Markus Schneider)
       */
      ContactKinematics * findContactPairingWith(std::string type0, std::string type1);

      virtual void initializeUsingXML(xercesc::DOMElement *element);

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

