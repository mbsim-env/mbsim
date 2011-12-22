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

#ifndef _CONTACT_H_
#define _CONTACT_H_

#include <mbsim/link_mechanics.h>
#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class Frame;
  class Arrow;
}
#endif

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
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-08-03 contacts can now visualize their ContactPointFrames (Markus Schneider)
   * \date 2010-07-06 added LinkStatus and LinearImpactEstimation for timestepper ssc (Robert Huber)
   *
   * basic class for contacts between contours, mainly implementing geometrical informations of contact-pairings
   * 
   * Remarks:
   * - constitutive laws on acceleration and velocity level have to be set pairwise
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
      virtual void updatewb(double t, int i=0);
      virtual void updateW(double t, int i=0);
      virtual void updateV(double t, int i=0);
      virtual void updateh(double t, int i=0);
      virtual void updateg(double t);
      virtual void updategd(double t);
      virtual void updateStopVector(double t);
      virtual void updateJacobians(double t, int j=0);
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual void updateWRef(const fmatvec::Mat &ref, int j=0);
      virtual void updateVRef(const fmatvec::Mat &ref, int j=0);
      virtual void updatehRef(const fmatvec::Vec &hRef, int j=0);
      virtual void updatewbRef(const fmatvec::Vec &ref);
      virtual void updatelaRef(const fmatvec::Vec& ref);
      virtual void updategRef(const fmatvec::Vec& ref);
      virtual void updategdRef(const fmatvec::Vec& ref);
      virtual void updaterFactorRef(const fmatvec::Vec &ref);
      virtual void updatesvRef(const fmatvec::Vec &ref);
      virtual void updatejsvRef(const fmatvec::Vector<int> &ref);
      virtual void calcxSize();
      virtual void calclaSize(int j);
      virtual void calcgSize(int j);
      virtual void calcgdSize(int j);
      virtual void calcrFactorSize(int j);
      virtual void calcsvSize();
      virtual void calcLinkStatusSize();
      virtual void init(InitStage stage);
      virtual bool isSetValued() const;
      virtual void updateLinkStatus(double dt);
      virtual bool isActive() const;
      virtual bool gActiveChanged();
      virtual void solveImpactsFixpointSingle(double dt);
      virtual void solveConstraintsFixpointSingle();
      virtual void solveImpactsGaussSeidel(double dt);
      virtual void solveConstraintsGaussSeidel();
      virtual void solveImpactsRootFinding(double dt);
      virtual void solveConstraintsRootFinding();
      virtual void jacobianConstraints();
      virtual void jacobianImpacts();
      virtual void updaterFactors();
      virtual void checkConstraintsForTermination();
      virtual void checkImpactsForTermination(double dt);
      using LinkMechanics::connect;
      virtual void checkActive(int j);
      virtual void updateCondition(int i);
      virtual void LinearImpactEstimation(fmatvec::Vec &gInActive_,fmatvec::Vec &gdInActive_,int *IndInActive_,fmatvec::Vec &gAct_,int *IndActive_);
      virtual void SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_);
 
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const { return "Contact"; }
      virtual void plot(double t, double dt = 1);
      virtual void closePlot();
      /***************************************************/

#ifdef HAVE_OPENMBVCPPINTERFACE
      /** 
       * \brief Draw two OpenMBV::Frame's of size 'size' at the contact points if 'enable'==true, otherwise the object is available but disabled.
       * If the contact is closed, then the two contact points are the same on each contour.
       * If the contact is not closed, then the two contact point lie on the contours with minimal distance in between.
       * The x-axis of this frames are orientated to the other frame origin (normal vector).
       */
      void enableOpenMBVContactPoints(double size=1.,bool enable=true) { openMBVContactFrameSize=size; openMBVContactFrameEnabled=enable; }

      /** 
       * \brief Sets the OpenMBV::Arrow to be used for drawing the normal force vector.
       * This vector is the force which is applied on the second contour.
       * The reactio (not drawn) is applied on the first contour.
       */
      void setOpenMBVNormalForceArrow(OpenMBV::Arrow *arrow) { contactArrow=arrow; }

      /** 
       * \brief Sets the OpenMBV::Arrow to be used for drawing the friction force vector.
       * This vector is the friction which is applied on the second contour.
       * The reactio (not drawn) is applied on the frist contour.
       * If using a set-valued friction law, then the arrow is drawn in green if the contact
       * is in slip and in red, if the contact is in stick.
       */
      void setOpenMBVFrictionArrow(OpenMBV::Arrow *arrow) { frictionArrow=arrow; }
#endif

      /* GETTER / SETTER */
      void setContactForceLaw(GeneralizedForceLaw *fcl_) { fcl = fcl_; }
      GeneralizedForceLaw * getContactForceLaw() const {return fcl; }
      void setContactImpactLaw(GeneralizedImpactLaw *fnil_) { fnil = fnil_; }
      void setFrictionForceLaw(FrictionForceLaw *fdf_) { fdf = fdf_; }
      void setFrictionImpactLaw(FrictionImpactLaw *ftil_) { ftil = ftil_; }
      void setContactKinematics(ContactKinematics* ck) { contactKinematics = ck; }
      ContactKinematics* getContactKinematics() const { return contactKinematics; } 
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

      void computeCurvatures(fmatvec::Vec & r) const;

      virtual void initializeUsingXML(TiXmlElement *element);

      void calccorrSize(int j);
      void updatecorr(int j);
      void updatecorrRef(const fmatvec::Vec& ref);

      void checkRoot();

    protected:
      /**
       * \brief used contact kinematics
       */
      ContactKinematics *contactKinematics;

      /**
       * \brief force laws in normal and tangential direction on acceleration and velocity level
       */
      GeneralizedForceLaw *fcl;

      /**
       * \brief force law defining relation between tangential velocities and tangential forces
       */
      FrictionForceLaw *fdf;
      
      /**
       * \brief force law defining relation between penetration velocity and resulting normal impulses
      */
      GeneralizedImpactLaw *fnil;
      
      /** 
       * \brief force law defining relation between tangential velocities and forces impulses
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
      std::vector<unsigned int*> gddActive; 

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
      std::vector<fmatvec::Mat*> Vk[2], Wk[2];

      /**
       * \brief size and index of force parameters, relative distances, relative velocities, stop vector and relaxation factors for possible contact points
       */
      std::vector<int> laSizek, laIndk, gSizek, gIndk, gdSizek, gdIndk, svSizek, svIndk, rFactorSizek, rFactorIndk;

#ifdef HAVE_OPENMBVCPPINTERFACE
      /**
       * \brief contact group to draw
       */
      OpenMBV::Group * openMBVContactGrp;

      /**
       * \brief container of ContactFrames to draw
       */
      std::vector<std::vector<OpenMBV::Frame*> > openMBVContactFrame;

      /**
       * \brief container of normal and friction forces to draw
       */
      std::vector<OpenMBV::Arrow *> openMBVNormalForceArrow, openMBVFrictionArrow;

      /**
       * \brief size of ContactFrames to draw
       */
      double openMBVContactFrameSize;

      /**
       * \brief enable flag of ContactFrames to draw
       */
      bool openMBVContactFrameEnabled;

      /**
       * \brief pointer to memory of normal and friction forces to draw
       */
      OpenMBV::Arrow *contactArrow, *frictionArrow;
#endif

      std::vector<fmatvec::Vec> corrk;
      std::vector<int> corrSizek, corrIndk;

      std::vector<int> rootID;

      std::vector<fmatvec::Vec> gddkBuf;

    private:
      std::string saved_ref1, saved_ref2;
  };

}

#endif /* _CONTACT_H_ */

