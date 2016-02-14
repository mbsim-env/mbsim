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

#include "mbsim/contour.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class RigidBody;
}
#endif

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

      Frame* createContourFrame(const std::string &name="P");

      Frame* getFrame() { return R; }
      Frame* getFrameOfReference() { return R; }
      void setFrameOfReference(Frame *frame) { R = frame; }
      void setFrameOfReference(const std::string &frame) { saved_frameOfReference = frame; }

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "RigidContour"; }
      virtual void plot(double t, double dt = 1);
      virtual void init(InitStage stage);
      /***************************************************/

      fmatvec::Vec3 getPosition(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getParDer1Wn(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getParDer2Wn(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getParDer1Wu(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getParDer2Wu(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getParDer1Wv(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getParDer2Wv(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getWrPS(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getWs(double t, const fmatvec::Vec2 &zeta);
      fmatvec::Vec3 getWt(double t, const fmatvec::Vec2 &zeta);

      /**
       * \brief contact search for RigidContours
       * \author Markus Schneider
       * \date 2010-11-05 initial commit (Markus Schneider)
       */
      ContactKinematics * findContactPairingWith(std::string type0, std::string type1);

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::RigidBody>& getOpenMBVRigidBody() { return openMBVRigidBody; }

      void setOpenMBVRigidBody(const boost::shared_ptr<OpenMBV::RigidBody> &ombvBody) { openMBVRigidBody = ombvBody; }
#endif

    protected:
      /**
       * \brief coordinate system of contour
       */
      Frame *R;

#ifdef HAVE_OPENMBVCPPINTERFACE
      boost::shared_ptr<OpenMBV::RigidBody> openMBVRigidBody;
#endif

    private:
      std::string saved_frameOfReference;
  };

}

#endif /* _CONTOUR_H_ */

