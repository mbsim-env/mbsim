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

#ifndef _JOINT_H_
#define _JOINT_H_

#include "mbsim/link_mechanics.h"
#include "mbsim/frame.h"

namespace MBSim {

  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;
  class FrictionForceLaw;
  class FrictionImpactLaw;

  /** 
   * \brief class for connections: constraints on frames
   * \author Martin Foerg
   * \date 2009-04-06 LinkMechanics added (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \todo visualisation / hSize Frame C / One Force Direction TODO
   */
  class Joint: public LinkMechanics {
    public: 
      /**
       * \brief constructor
       * \param name
       */
      Joint(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Joint();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updatewb(double t);
      virtual void updateW(double t);
      virtual void updateh(double t);
      virtual void updateg(double t);
      virtual void updategd(double t);
      virtual void updateJacobians(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void updatexd(double t);
      virtual void updatedx(double t, double dt);
      virtual void calcxSize();
      virtual void init();
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual void calclaSize();
      virtual void calclaSizeForActiveg();
      virtual void calcgSize();
      virtual void calcgSizeActive();
      virtual void calcgdSize();
      virtual void calcgdSizeActive();
      virtual void calcrFactorSize();
      virtual bool isSetValued() const;
      virtual bool isActive() const { return true; }
      virtual bool gActiveChanged() { return false; }
      virtual void solveImpactsFixpointSingle();
      virtual void solveConstraintsFixpointSingle();
      virtual void solveImpactsGaussSeidel();
      virtual void solveConstraintsGaussSeidel();
      virtual void solveImpactsRootFinding();
      virtual void solveConstraintsRootFinding();
      virtual void jacobianConstraints();
      virtual void jacobianImpacts();
      virtual void updaterFactors();
      virtual void checkImpactsForTermination();
      virtual void checkConstraintsForTermination();
      virtual void checkActiveg() {}
      virtual void checkActivegd() {}
      virtual void resizeJacobians(int j); 
      /***************************************************/

      /* INTERFACE FOR DERIVED CLASSES */
      /**
       * \brief \param first frame to connect
       * \brief second frame to connect
       */
      virtual void connect(Frame *frame1, Frame* frame2);
      /***************************************************/

      /* GETTER / SETTER */
      void setForceLaw(GeneralizedForceLaw * rc) { ffl = rc; }
      void setMomentLaw(GeneralizedForceLaw * rc) { fml = rc; }
      void setImpactForceLaw(GeneralizedImpactLaw * rc) { fifl = rc; }
      void setImpactMomentLaw(GeneralizedImpactLaw * rc) { fiml = rc; }
      /***************************************************/

      void plot(double t, double dt = 1);

      /**
       * \param local force direction
       */
      void setForceDirection(const fmatvec::Mat& fd);

      /**
       * \param local moment direction
       */
      void setMomentDirection(const fmatvec::Mat& md);

      virtual void initializeUsingXML(TiXmlElement *element);

#ifdef HAVE_OPENMBVCPPINTERFACE
      /** \brief Visualize a force arrow acting on frame2 */
      void setOpenMBVForceArrow(OpenMBV::Arrow *arrow) {
        std::vector<bool> which; which.resize(2, false);
        which[1]=true;
        LinkMechanics::setOpenMBVForceArrow(arrow, which);
      }

      /** \brief Visualize a moment arrow acting on frame2 */
      void setOpenMBVMomentArrow(OpenMBV::Arrow *arrow) {
        std::vector<bool> which; which.resize(2, false);
        which[1]=true;
        LinkMechanics::setOpenMBVMomentArrow(arrow, which);
      }
#endif

    protected:
      /**
       * \brief indices of forces and torques
       */
      fmatvec::Index IT, IR;

      /**
       * \brief local force and moment direction
       */
      fmatvec::Mat forceDir, momentDir;

      /**
       * \brief global force and moment direction
       */
      fmatvec::Mat Wf, Wm;

      /**
       * \brief translational JACOBIAN (not empty for e.g. prismatic joints)
       */
      fmatvec::Mat JT;

      /**
       * \brief difference vector of position, velocity and angular velocity
       */
      fmatvec::Vec WrP0P1, WvP0P1, WomP0P1;

      /**
       * constitutive laws on acceleration and velocity level for forces and torques
       */
      GeneralizedForceLaw *ffl, *fml;
      GeneralizedImpactLaw *fifl, *fiml;

      /**
       * \brief relative velocity and acceleration after an impact for event driven scheme summarizing all possible contacts
       */
      fmatvec::Vec gdn, gdd;

      /**
       * \brief TODO
       */
      Frame C;
  };

}

#endif

