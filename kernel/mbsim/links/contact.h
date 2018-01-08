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

#ifndef _CONTACT_H_
#define _CONTACT_H_

#include <mbsim/links/single_contact.h>
#include <map>

namespace MBSim {

  class ContactKinematics;
  class GeneralizedForceLaw;
  class GeneralizedImpactLaw;
  class FrictionForceLaw;
  class FrictionImpactLaw;

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
  class Contact : public Link {
    public:
      /*!
       * \brief constructor
       * \param name of contact
       */
      Contact(const std::string &name = "");

      /**
       * \brief destructor
       */
      ~Contact() override;

      /* INHERITED INTERFACE OF LINKINTERFACE */
      void updatewb() override;
      void updateW(int i = 0) override;
      void updateV(int i = 0) override;
      void updateh(int i = 0) override;
      void updateg() override;
      void updategd() override;
      void updateStopVector() override;
      void updateJacobians(int j = 0) override;
      /***************************************************/

      /* INHERITED INTERFACE OF LINK */
      void updateWRef(const fmatvec::Mat &ref, int j = 0) override;
      void updateVRef(const fmatvec::Mat &ref, int j = 0) override;
      void updatehRef(const fmatvec::Vec &hRef, int j = 0) override;
      void updaterRef(const fmatvec::Vec &hRef, int j = 0) override;
      void updatewbRef(const fmatvec::Vec &ref) override;
      void updatelaRef(const fmatvec::Vec& ref) override;
      void updateLaRef(const fmatvec::Vec& ref) override;
      void updategRef(const fmatvec::Vec& ref) override;
      void updategdRef(const fmatvec::Vec& ref) override;
      void updateresRef(const fmatvec::Vec& ref) override;
      void updaterFactorRef(const fmatvec::Vec &ref) override;
      void updatesvRef(const fmatvec::Vec &ref) override;
      void updatejsvRef(const fmatvec::VecInt &ref) override;
      void updateLinkStatusRef(const fmatvec::VecInt &LinkStatusParent) override;
      void updateLinkStatusRegRef(const fmatvec::VecInt &LinkStatusRegParent) override;
      void calclaSize(int j) override;
      void calcgSize(int j) override;
      void calcgdSize(int j) override;
      void calcrFactorSize(int j) override;
      void calcsvSize() override;
      void calcLinkStatusSize() override;
      void calcLinkStatusRegSize() override;
      void init(InitStage stage, const InitConfigSet &config) override;
      bool isSetValued() const override;
      bool isSingleValued() const override;
      void updateLinkStatus() override;
      void updateLinkStatusReg() override;
      bool isActive() const override;
      bool gActiveChanged() override;
      bool detectImpact() override;
      void solveImpactsFixpointSingle() override;
      void solveConstraintsFixpointSingle() override;
      void solveImpactsGaussSeidel() override;
      void solveConstraintsGaussSeidel() override;
      void solveImpactsRootFinding() override;
      void solveConstraintsRootFinding() override;
      void jacobianConstraints() override;
      void jacobianImpacts() override;
      void updaterFactors() override;
      void checkConstraintsForTermination() override;
      void checkImpactsForTermination() override;
      void checkActive(int j) override;
      void setGeneralizedForceTolerance(double tol) override;
      void setGeneralizedImpulseTolerance(double tol) override;
      void setGeneralizedRelativePositionTolerance(double tol) override;
      void setGeneralizedRelativeVelocityTolerance(double tol) override;
      void setGeneralizedRelativeAccelerationTolerance(double tol) override;
      void setrMax(double rMax_) override;
      void setLinkStatusInd(int LinkStatusInd_) override;
      void setLinkStatusRegInd(int LinkStatusRegInd_) override;
      void setsvInd(int svInd_) override;
      void setlaInd(int laInd_) override;
      void setgInd(int gInd_) override;
      void setgdInd(int gdInd_) override;
      void setrFactorInd(int rFactorInd_) override;
      void LinearImpactEstimation(double t, fmatvec::Vec &gInActive_, fmatvec::Vec &gdInActive_, int *IndInActive_, fmatvec::Vec &gAct_, int *IndActive_) override;
      void SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) override;
      void updatecorrRef(const fmatvec::Vec& ref) override;
      void updatecorr(int j) override;
      void calccorrSize(int j) override;
      void setcorrInd(int corrInd_) override;
      void checkRoot() override;
      virtual void updateGeneralizedNormalForce();
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      void plot() override;
      void setDynamicSystemSolver(DynamicSystemSolver *sys) override;
      /***************************************************/

      void resetUpToDate() override;

      /* GETTER / SETTER */

      void setNormalForceLaw(GeneralizedForceLaw *fcl_);
      GeneralizedForceLaw * getNormalForceLaw() const { return fcl; }
      void setNormalImpactLaw(GeneralizedImpactLaw *fnil_);
      void setTangentialForceLaw(FrictionForceLaw *fdf_); 
      void setTangentialImpactLaw(FrictionImpactLaw *ftil_);
      void setContactKinematics(ContactKinematics* ck, size_t index = 0) {
        assert(index < contactKinematics.size());
        contactKinematics[index] = ck;
      }
      ContactKinematics* getContactKinematics(size_t index = 0) const {
        assert(index < contactKinematics.size());
        return contactKinematics[index];
      }

      ContactKinematics* findContactKinematics(const std::string& cKName) const {
        int pos = find(ckNames.begin(), ckNames.end(), cKName) - ckNames.begin();
        if (pos < static_cast<int>(ckNames.size())) {
          return contactKinematics[pos];
        }
        throw MBSimError("Name of contact Kinematics is not valid");
        return nullptr;
      }

      const std::vector<std::vector<SingleContact> > & getSubcontacts() const {
        return contacts;
      }

      virtual void setPlotFeatureContactKinematics(std::string cKName, std::size_t pf, bool value);
      /***************************************************/

      /**
       * \return number of considered friction directions
       */
      virtual int getFrictionDirections();

      /*! connect two contours
       * \param contour0          first contour
       * \param contour1          second contour
       * \param contactKinematics The contact kinematics that should be used to compute the contact point.
       * \param name              Name of the contact in the output
       *
       */
      void connect(Contour *contour1, Contour* contour2, ContactKinematics* contactKinematics = nullptr, const std::string & name = "");

      Contour* getContour(int i, int j=0) { return contour[i][j]; }

      SingleContact& getSingleContact(int i, int j=0) { return contacts[i][j]; }

      void initializeUsingXML(xercesc::DOMElement *element) override;

      void setSearchAllContactPoints(bool searchAllCP_) { searchAllCP = searchAllCP_; }
      void setInitialGuess(const fmatvec::VecV &zeta0_) { zeta0 = zeta0_; }

    protected:
      /**
       * \brief list of the single sub-contact(-points)
       */
      std::vector<std::vector<SingleContact> > contacts;

      /**
       * \brief list of the single contact kinematics
       */
      std::vector<ContactKinematics*> contactKinematics;

      std::vector<std::vector<Contour*> > contour;

      /*!
       * \brief names for the contact kinematics
       *
       * \todo: what really is annoying is the fact, that due to the concept of the compound contour the sub contacts can not be build when the contours are connected. Thus it is not possible before the contact kinematics are assigned (what happens in the preInit-stage) that plot featers (and everything else, like names for the plot and so on) can not be set before. Thus this properties have to be saved in a special list in the multiple contact or the things are set later on...
       */
      std::vector<std::string> ckNames;

      /*!
       * \brief plotFeatures of sub-contacts
       *
       * \todo: see remark of ckNames
       */
      std::map<std::pair<std::string, std::size_t>, bool> plotFeatureMap;

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

      bool searchAllCP;

      fmatvec::VecV zeta0;

    private:
      struct saved_references {
          std::string name1;
          std::string name2;
          std::string contourPairingName;
      };
      std::vector<saved_references> saved_ref;
  };

}

#endif /* _CONTACT_H_ */
