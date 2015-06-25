/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _SINGLE_CONTACT_H_
#define _SINGLE_CONTACT_H_

#include <mbsim/mechanical_link.h>
#include <map>

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "mbsim/utils/boost_parameters.h"
#include "mbsim/utils/openmbv_utils.h"
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
   * \date 2012-05-08 added LinkStatusReg for AutoTimeSteppingSSCIntegrator (Jan Clauberg)
   * \date 2014-09-16 contact forces are calculated on acceleration level (Thorsten Schindler)
   *
   * basic class for contacts between contours, mainly implementing geometrical informations of contact-pairings
   * 
   * Remarks:
   * - constitutive laws on acceleration and velocity level have to be set pairwise
   */
  class SingleContact: public MechanicalLink {
    public:
      /*!
       * \brief constructor
       * \param name of contact
       */      
      SingleContact(const std::string &name="");

      /**
       * \brief destructor
       */
      virtual ~SingleContact();

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updatewb(double t);
      virtual void updateW(double t, int i=0);
      virtual void updateV(double t, int i=0);
      virtual void updateh(double t, int i=0);
      virtual void updateGeneralizedSetValuedForces(double t);
      virtual void updateGeneralizedSingleValuedForces(double t);
      virtual void updateGeneralizedPositions(double t);
      virtual void updateGeneralizedVelocities(double t);
      virtual void updateForceDirections(double t);
      virtual void updatePositions(double t);
      virtual void updateVelocities(double t);
      virtual void updateg(double t);
      virtual void updategd(double t);
      virtual void updateStopVector(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      virtual void updateWRef(const fmatvec::Mat &ref, int j=0);
      virtual void updateVRef(const fmatvec::Mat &ref, int j=0);
      virtual void updatehRef(const fmatvec::Vec &hRef, int j=0);
      virtual void updatelaRef(const fmatvec::Vec& ref);
      virtual void updateLaRef(const fmatvec::Vec& ref);
      virtual void updategdRef(const fmatvec::Vec& ref);
      virtual void calcxSize();
      virtual void calclaSize(int j);
      virtual void calcgSize(int j);
      virtual void calcgdSize(int j);
      virtual void calcrFactorSize(int j);
      virtual void calcsvSize();
      virtual void calcLinkStatusSize();
      virtual void calcLinkStatusRegSize();
      virtual void init(InitStage stage);
      virtual bool isSetValued() const;
      virtual bool isSingleValued() const;
      virtual void updateLinkStatus(double dt);
      virtual void updateLinkStatusReg(double dt);
      virtual bool isActive() const;
      virtual bool gActiveChanged();
      virtual bool detectImpact();
      virtual void solveImpactsFixpointSingle(double t, double dt);
      virtual void solveConstraintsFixpointSingle(double t);
      virtual void solveImpactsGaussSeidel(double t, double dt);
      virtual void solveConstraintsGaussSeidel(double t);
      virtual void solveImpactsRootFinding(double t, double dt);
      virtual void solveConstraintsRootFinding(double t);
      virtual void jacobianConstraints(double t);
      virtual void jacobianImpacts(double t);
      virtual void updaterFactors(double t);
      virtual void checkConstraintsForTermination(double t);
      virtual void checkImpactsForTermination(double t, double dt);
      using MechanicalLink::connect;
      virtual void checkActive(double t, int j);
//      virtual void LinearImpactEstimation(fmatvec::Vec &gInActive_,fmatvec::Vec &gdInActive_,int *IndInActive_,fmatvec::Vec &gAct_,int *IndActive_);
//      virtual void SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_);
 
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
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVContactPoints, tag, (optional (size,(double),1)(offset,(double),1)(transparency,(double),0))) { 
        OpenMBVFrame ombv(size,offset,"[-1;1;1]",transparency);
        setOpenMBVContactPoints(ombv.createOpenMBV());
      }
      void setOpenMBVContactPoints(const boost::shared_ptr<OpenMBV::Frame> &frame) { 
        openMBVContactFrame[0]=frame;
        openMBVContactFrame[1]=OpenMBV::ObjectFactory::create(openMBVContactFrame[0]);
      }

      /** 
       * \brief Sets the OpenMBV::Arrow to be used for drawing the normal force vector.
       * This vector is the force which is applied on the second contour.
       * The reactio (not drawn) is applied on the first contour.
       */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVNormalForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        contactArrow=ombv.createOpenMBV(); 
      }
      void setOpenMBVNormalForce(const boost::shared_ptr<OpenMBV::Arrow> &arrow) { contactArrow=arrow; }

      /** 
       * \brief Sets the OpenMBV::Arrow to be used for drawing the friction force vector.
       * This vector is the friction which is applied on the second contour.
       * The reactio (not drawn) is applied on the frist contour.
       * If using a set-valued friction law, then the arrow is drawn in green if the contact
       * is in slip and in red, if the contact is in stick.
       */
      BOOST_PARAMETER_MEMBER_FUNCTION( (void), enableOpenMBVTangentialForce, tag, (optional (scaleLength,(double),1)(scaleSize,(double),1)(referencePoint,(OpenMBV::Arrow::ReferencePoint),OpenMBV::Arrow::toPoint)(diffuseColor,(const fmatvec::Vec3&),"[-1;1;1]")(transparency,(double),0))) { 
        OpenMBVArrow ombv(diffuseColor,transparency,OpenMBV::Arrow::toHead,referencePoint,scaleLength,scaleSize);
        frictionArrow=ombv.createOpenMBV(); 
      }
      void setOpenMBVTangentialForce(const boost::shared_ptr<OpenMBV::Arrow> &arrow) { frictionArrow=arrow; }
#endif

      /* GETTER / SETTER */
      void setNormalForceLaw(GeneralizedForceLaw *fcl_);
      GeneralizedForceLaw * getNormalForceLaw() const {return fcl; }
      void setNormalImpactLaw(GeneralizedImpactLaw *fnil_);
      void setTangentialForceLaw(FrictionForceLaw *fdf_);
      void setTangentialImpactLaw(FrictionImpactLaw *ftil_);
      void setContactKinematics(ContactKinematics* ck) { contactKinematics = ck; }
      ContactKinematics* getContactKinematics() const { return contactKinematics; }
      ContourPointData* & getcpData() { return cpData; }
      ContourPointData* getcpData() const { return cpData; }
      fmatvec::Vec & getlaN() { return laN; }
      fmatvec::Vec getlaN() const { return laN; }
      fmatvec::Vec & getlaT() { return laT; }
      fmatvec::Vec getlaT() const { return laT; }
      fmatvec::Vec & getLaN() { return LaN; }
      fmatvec::Vec getLaN() const { return LaN; }
      fmatvec::Vec & getLaT() { return LaT; }
      fmatvec::Vec getLaT() const { return LaT; }
      fmatvec::Vec & getgdN() { return gdN; }
      fmatvec::Vec getgdN() const { return gdN; }
      fmatvec::Vec & getgdT() { return gdT; }
      fmatvec::Vec getgdT() const { return gdT; }
      /***************************************************/

      /**
       * \return number of considered friction directions
       */
      virtual int getFrictionDirections();
      virtual int getFrictionDirections() const;

      /*! connect two contours
       * \param first contour
       * \param second contour
       * \param specify the contact kinematics
       *
       * REMARK: The contact frame of the first contour is used to plot the contacts data in
       */
      void connect(Contour *contour1, Contour* contour2, ContactKinematics* contactKinematics = 0);

      /*!
       * \brief apply forces to the h-vector
       * \param t time of the integration
       * \param j position in h-vector (0 or 1)
       */
      void applyh(double t, int j);

      void getCurvatures(fmatvec::Vec & r) const;

      virtual void initializeUsingXML(xercesc::DOMElement *element);

      void calccorrSize(int j);
      void updatecorr(int j);

      void checkRoot(double t);

      void resetUpToDate();

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
      ContourPointData* cpData;

      /*!
       * \brief force in normal direction
       *
       * \todo: only double needed here
       */
      fmatvec::Vec laN, LaN;

      /*!
       * \brief force in tangential direction
       *
       * \todo: use fixed size?
       */
      fmatvec::Vec laT, LaT;

      /*!
       * \brief relative velocity in normal direction
       *
       */
      fmatvec::Vec gdN;

      /*!
       * \brief relative velocity in tangential direction
       *
       * \todo: use fixed size?
       */
      fmatvec::Vec gdT;

      /** 
       * \brief boolean vector symbolising activity of contacts on position level with possibility to save previous time step
       */
      unsigned int gActive, gActive0;

      /** 
       * \brief boolean vector symbolising activity of contacts on velocity level
       *
       * length of array is two with index 0 for normal direction and index one for tangential direction
       */
      unsigned int* gdActive;

      /** 
       * \brief boolean vector symbolising activity of contacts on acceleration level
       *
       * length of array is two with index 0 for normal direction and index one for tangential direction
       */
      unsigned int* gddActive;

      /** 
       * \brief index for tangential directions in projection matrices
       */
      fmatvec::Index iT;

      /**
       * \brief new gap velocity after an impact for event driven scheme
       */
      fmatvec::Vec gdnN, gdnT;
      
      /**
       * \brief gap acceleration for event driven scheme on acceleration level
       */
      fmatvec::Vec gddN, gddT;

      /*!
       * \brief gap acceleration buffer
       */
      fmatvec::Vec gddNBuf, gddTBuf;

#ifdef HAVE_OPENMBVCPPINTERFACE
      /**
       * \brief contact group to draw
       */
      boost::shared_ptr<OpenMBV::Group> openMBVContactGrp;

      /**
       * \brief container of ContactFrames to draw
       */
      std::vector<boost::shared_ptr<OpenMBV::Frame> > openMBVContactFrame;

      /**
       * \brief pointer to memory of normal and friction forces to draw
       */
      boost::shared_ptr<OpenMBV::Arrow> contactArrow, frictionArrow;
#endif

      /**
       * \brief type of detected root
       *
       * 1 = close-open transition / stick-slip transition
       * 2 = slip-stick transition
       * 3 = open-close transition, i.e., impact
       */
      int rootID;

    private:
      std::string saved_ref1, saved_ref2;
  };

}

#endif /* _SINGLE_CONTACT_H_ */

