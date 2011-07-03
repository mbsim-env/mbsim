/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: rzander@users.berlios.de
 */

#ifndef _ROTATIONALSPRINGDAMPER_H_
#define _ROTATIONALSPRINGDAMPER_H_

#include "mbsim/link_mechanics.h"
#include "mbsim/utils/function.h"

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class CoilSpring;
}
#endif

namespace MBSim {

  class RigidBody;
  /** \brief A spring damper force law between .
   * This class connects two frames and applies a torque in it, which depends in the
   * relative rotation and velocity  between the two frames. ONLY between relative rotational bodies!!!
   */
  class RelativeRotationalSpringDamper : public LinkMechanics {
    protected:
      Function2<double,double,double> *func;
      Frame *refFrame;
      RigidBody *body;
      fmatvec::Vec torqueDir, WtorqueDir;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::CoilSpring *coilspringOpenMBV;
#endif
    public:
      RelativeRotationalSpringDamper(const std::string &name);
      void updateh(double, int i=0);
      void updateg(double);
      void updategd(double);

      /** \brief Connect the RelativeRotationalSpringDamper to frame1 and frame2 */
      void connect(Frame *frame1, Frame* frame2);

      bool isActive() const { return true; }
      bool gActiveChanged() { return false; }
      std::string getType() const { return "RotationalSpringDamper"; }
      void init(InitStage stage);

      /** \brief Set function for the torque calculation.
       * The first input parameter to that function is the relative rotation g between frame2 and frame1.
       * The second input parameter to that function is the relative rotational velocity gd between frame2 and frame1.
       * The return value of that function is used as the torque of the RelativeRotationalSpringDamper.
       */
      void setForceFunction(Function2<double,double,double> *func_) { func=func_; }

      /** \brief Set a projection direction for the resulting torque
       * If this function is not set, or frame is NULL, than torque calculated by setForceFunction
       * is applied on the two connected frames in the direction of the two connected frames.
       * If this function is set, than this torque is first projected in direction dir and then applied on
       * the two connected frames in the projected direction; (!) this might induce violation of the global equality of torques (!).
       * The direction vector dir is given in coordinates of frame refFrame.
       */
      void setRelativeBody(RigidBody* body_);

      void plot(double t, double dt=1);
      void initializeUsingXML(TiXmlElement *element);

#ifdef HAVE_OPENMBVCPPINTERFACE
      /** \brief Visualise the RelativeRotationalSpringDamper using a OpenMBV::CoilSpring */
      void setOpenMBVSpring(OpenMBV::CoilSpring *spring_) {coilspringOpenMBV=spring_;}

      /** \brief Visualize a force arrow acting on each of both connected frames */
      void setOpenMBVForceArrow(OpenMBV::Arrow *arrow) {
        std::vector<bool> which; which.resize(2, true);
        LinkMechanics::setOpenMBVForceArrow(arrow, which);
      }
#endif
    private:
      std::string saved_frameOfReference, saved_ref1, saved_ref2;
      fmatvec::Vec saved_direction;
  };

}

#endif /* _ROTATIONALSPRINGDAMPER_H_ */

