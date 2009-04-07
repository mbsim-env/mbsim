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

#ifndef _CONTACT_H_
#define _CONTACT_H_

#include <mbsim/link_mechanics.h>

namespace MBSim {

  class ContactKinematics;
  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;
  class FrictionForceLaw;
  class FrictionImpactLaw;
  class ContourPointData;

  /*! \brief class for contacts
   * \author Martin Foerg
   * \date 2009-04-02 some comments (Thorsten Schindler)
   *
   * basic class for contacts between contours, mainly implementing geometrical informations of contact-pairings
   */
  class Contact: public LinkMechanics {
    public:
      /*!
       * \brief constructor
       * \param name of contact
       */      
      Contact(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Contact();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updater(double t);
      virtual void updatewb(double t);
      virtual void updateW(double t);
      virtual void updateV(double t);
      virtual void updateh(double t);
      virtual void updateg(double t);
      virtual void updategd(double t);
      virtual void updateStopVector(double t);
      virtual void updateJacobians(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual void updateWRef(const fmatvec::Mat &ref, int j=0);
      virtual void updateVRef(const fmatvec::Mat &ref, int j=0);
      virtual void updatehRef(const fmatvec::Vec &ref, int j=0);
      virtual void updatewbRef(const fmatvec::Vec &ref);
      virtual void updatelaRef(const fmatvec::Vec& ref);
      virtual void updategRef(const fmatvec::Vec& ref);
      virtual void updategdRef(const fmatvec::Vec& ref);
      virtual void updaterFactorRef(const fmatvec::Vec &ref);
      virtual void updatesvRef(const fmatvec::Vec &ref);
      virtual void updatejsvRef(const fmatvec::Vector<int> &ref);
      virtual void calcxSize();
      virtual void calclaSize();
      virtual void calclaSizeForActiveg();
      virtual void calcgSize();
      virtual void calcgSizeActive();
      virtual void calcgdSize(); // TODO not consistent
      virtual void calcgdSizeActive();
      virtual void calcrFactorSize();
      virtual void calcsvSize();
      virtual void init();
      virtual void preinit();
      virtual bool isSetValued() const;
      virtual bool isActive() const;
      virtual bool gActiveChanged();
      virtual void solveImpactsFixpointSingle();
      virtual void solveConstraintsFixpointSingle();
      virtual void solveImpactsGaussSeidel();
      virtual void solveConstraintsGaussSeidel();
      virtual void solveImpactsRootFinding();
      virtual void solveConstraintsRootFinding();
      virtual void jacobianConstraints();
      virtual void jacobianImpacts();
      virtual void updaterFactors();
      virtual void checkConstraintsForTermination();
      virtual void checkImpactsForTermination();
      using LinkMechanics::connect;
      virtual void checkActiveg();
      virtual void checkActivegd();
      virtual void checkActivegdn();
      virtual void checkActivegdd(); 
      virtual void checkAllgd();
      virtual void updateCondition();
      virtual void resizeJacobians(int j); 
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "Contact"; }
      /***************************************************/

      /* GETTER / SETTER */
      void setContactForceLaw(GeneralizedForceLaw *fcl_) { fcl = fcl_; }
      void setContactImpactLaw(GeneralizedImpactLaw *fnil_) { fnil = fnil_; }
      void setFrictionForceLaw(FrictionForceLaw *fdf_) { fdf = fdf_; }
      void setFrictionImpactLaw(FrictionImpactLaw *ftil_) { ftil = ftil_; }
      void setContactKinematics(ContactKinematics* ck) { contactKinematics = ck; }
      /***************************************************/

      /**
       * \return number of considered friction directions
       */
      virtual int getFrictionDirections(); 

      /*! connect two contours
       * \param first contour
       * \param second contour
       */
      void connect(Contour *contour1, Contour* contour2);

    protected:
      /**
       * \brief used contact kinematics
       */
      ContactKinematics *contactKinematics;

      /**
       * \brief force laws in normal and tangential direction on acceleration and velocity level
       */
      GeneralizedForceLaw *fcl;
	  /** force law defining relation between tangential velocities and tangential forces
	   */
      FrictionForceLaw *fdf;
	  /** force law defining relation between penetration velocity and resulting normal impulses
	   */
      GeneralizedImpactLaw *fnil;
	  /** force law defining relation between tangential velocities and forces impulses
	   */
      FrictionImpactLaw *ftil;

      /**
       * \brief vector of frames for definition of relative contact situation
       */
      std::vector<ContourPointData*> cpData;

      /** 
       * \brief boolean vector symbolising activity of contacts on position level with possibility to save previous time step
       */
      std::vector<unsigned int> gActive, gActive0;
      
      /** 
       * \brief boolean vector symbolising activity of contacts on velocity level
       */
      std::vector<unsigned int*> gdActive; 

      /** 
       * \brief index for tangential directions in projection matrices
       */
      fmatvec::Index iT;

      /**
       * \brief relative velocity and acceleration after an impact for event driven scheme summarizing all possible contacts
       */
      fmatvec::Vec gdn, gdd;

      /**
       * \brief vectors of relative distance, velocity, velocity after impact in event driven scheme, acceleration in event driven scheme, force parameters, acceleration description with respect to contour parameters, stop vector, relaxation factors for possible contact points
       */
      std::vector<fmatvec::Vec> gk, gdk, gdnk, gddk, lak, wbk, svk, rFactork;

      /**
       * \brief boolean evaluation of stop vector for possible contact points
       */
      std::vector<fmatvec::Vector<int> > jsvk;
      
      /**
       * \brief single-valued forces for possible contact points
       */
      std::vector<fmatvec::Mat*> fF;

      /**
       * \brief set-valued forces for possible contact points
       */
      std::vector<fmatvec::Vec*> WF;

      /**
       * \brief condensed and full force direction matrix for possible contact points
       */
      std::vector<fmatvec::Mat*> Vk, Wk;

      /**
       * \brief size and index of force parameters, relative distances, relative velocities, stop vector and relaxation factors for possible contact points
       */
      std::vector<int> laSizek, laIndk, gSizek, gIndk, gdSizek, gdIndk, svSizek, svIndk, rFactorSizek, rFactorIndk;
  };

}

#endif

