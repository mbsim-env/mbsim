/* Copyright (C) 2004-2011  Kilian Grundl

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
 * Contact:
 *   TODO contact and Copyright time
 *
 */

#ifndef MAXWELL_CONTACT_H_
#define MAXWELL_CONTACT_H_

#include <mbsim/link_mechanics.h>
#include <mbsim/utils/function.h>
#include <mbsim/utils/nonlinear_algebra.h>

#include <fmatvec.h>
#include <map>

#ifdef HAVE_OPENMBVCPPINTERFACE
namespace OpenMBV {
  class Frame;
  class Arrow;
}
#endif

namespace MBSim {

  class ContactKinematics;
  class GeneralizedForceLaw;
  class FrictionForceLaw;
  class GeneralizedImpactLaw;
  class FrictionImpactLaw;
  class ContourPointData;

  /*! \brief class for Maxwell contact force law
   * \author Kilian Grundl
   * \date 2011-09-02 first check in
   *
   * Remarks:
   * - constitutive laws on acceleration and velocity level have to be set pairwise
   *
   * \todo create a new class like ContourPairing or something like that and use a vector<ContourPairing> instead of the single vectors in this class --> should make things much more readable and easier
   *        probably it leads to more similarity between contact and maxwellContact
   */

  class MaxwellContact : public MBSim::LinkMechanics {
    public:

      /**
       * \brief constructor
       * \param name of contact
       */
      MaxwellContact(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~MaxwellContact();

      //      /* INHERITED INTERFACE OF LINKINTERFACE */
      //      virtual void updatewb(double t);
      //      virtual void updateW(double t);
      //      virtual void updateV(double t);
      /*
       * solve the LCP (compute the elastic distances and the contact forces)
       */
      virtual void updateh(double t, int k=0);
      /*
       * \brief compute the rigid-body distance between the contact pairings
       */
      virtual void updateg(double t);
      virtual void updategd(double t);
      //      virtual void updateStopVector(double t);
      virtual void updateJacobians(double t, int j=0);
      //      /***************************************************/
      //
      //      /* INHERITED INTERFACE OF LINK */
      //      virtual void updateWRef(const fmatvec::Mat &ref, int j=0);
      //      virtual void updateVRef(const fmatvec::Mat &ref, int j=0);
      virtual void updatehRef(const fmatvec::Vec &hRef, int k=0);
      //      virtual void updatewbRef(const fmatvec::Vec &ref);
      //      virtual void updatelaRef(const fmatvec::Vec& ref);
      //      virtual void updategRef(const fmatvec::Vec& ref);
//      virtual void updategdRef(const fmatvec::Vec& ref);
      //      virtual void updaterFactorRef(const fmatvec::Vec &ref);
      //      virtual void updatesvRef(const fmatvec::Vec &ref);
      //      virtual void updatejsvRef(const fmatvec::Vector<int> &ref);
      //      virtual void calcxSize();
      //      virtual void calclaSize();
      //      virtual void calclaSizeForActiveg();
      //      virtual void calcgSize();
      //      virtual void calcgSizeActive();
      //      virtual void calcgdSize(); // TODO not consistent
      //      virtual void calcgdSizeActive();
      //      virtual void calcrFactorSize();
      //      virtual void calcsvSize();
      //      virtual void calcLinkStatusSize();
      virtual void init(InitStage stage);
      virtual bool isSetValued() const {
        return false; //The MaxwellContact isn't a set valued contact, but also not regularized (single valued)
      }
      //      virtual void updateLinkStatus(double dt);
      virtual bool isActive() const {
        return true; //the Maxwell contact is always active as the the distances are computed dependent on the forces
      }
      virtual bool gActiveChanged() {
        return false; //gActive (probably) changes every timestep //TODO what does this function?
      }
      //      virtual void solveImpactsFixpointSingle(double dt);
      //      virtual void solveConstraintsFixpointSingle();
      //      virtual void solveImpactsGaussSeidel(double dt);
      //      virtual void solveConstraintsGaussSeidel();
      //      virtual void solveImpactsRootFinding(double dt);
      //      virtual void solveConstraintsRootFinding();
      //      virtual void jacobianConstraints();
      //      virtual void jacobianImpacts();
      //      virtual void updaterFactors();
      //      virtual void checkConstraintsForTermination();
      //      virtual void checkImpactsForTermination(double dt);
      virtual void checkActiveg();
      //      virtual void checkActivegd();
      //      virtual void checkActivegdn();
      //      virtual void checkActivegdd();
      //      virtual void checkAllgd();
      //      virtual void updateCondition();
      //      virtual void LinearImpactEstimation(fmatvec::Vec &gInActive_,
      //          fmatvec::Vec &gdInActive_, int *IndInActive_, fmatvec::Vec &gAct_,
      //          int *IndActive_);
      //      virtual void SizeLinearImpactEstimation(int *sizeInActive_,
      //          int *sizeActive_);
      //
      //      /***************************************************/
      //
      /* INHERITED INTERFACE OF ELEMENT */
      virtual std::string getType() const {
        return "MaxwellContact";
      }
      virtual void plot(double t, double dt = 1);
      virtual void closePlot();
      /***************************************************/
      //
#ifdef HAVE_OPENMBVCPPINTERFACE
      /**
       * TODO Es muss jetzt deutlich mehr Frames geben
       * \brief Draw many OpenMBV::Frame's of size 'size' at the contact points if 'enable'==true, otherwise the object is available but disabled.
       * If the contact is closed, then the two contact points are the same on each contour.
       * If the contact is not closed, then the two contact point lie on the contours with minimal distance in between.
       * The x-axis of this frames are orientated to the other frame origin (normal vector).
       */
      void enableOpenMBVContactPoints(double size = 1., bool enable = true) {
        openMBVContactFrameSize = size;
        openMBVContactFrameEnabled = enable;
      }

      /**
       * \brief Sets the OpenMBV::Arrow to be used for drawing the normal force vector.
       * This vector is the force which is applied on the second contour.
       * The reactio (not drawn) is applied on the first contour.
       */
      void enableOpenMBVNormalForceArrow(OpenMBV::Arrow *normalForceArrow_) {
        normalForceArrow = normalForceArrow_;
      }

      /**
       * \brief Sets the OpenMBV::Arrow to be used for drawing the friction force vector.
       * This vector is the friction which is applied on the second contour.
       * The reactio (not drawn) is applied on the frist contour.
       * If using a set-valued friction law, then the arrow is drawn in green if the contact
       * is in slip and in red, if the contact is in stick.
       */
      void enableOpenMBVFrictionForceArrow(OpenMBV::Arrow *frictionForceArrow_) {
        frictionForceArrow = frictionForceArrow_;
      }
#endif

      /* GETTER / SETTER */
      void setFrictionForceLaw(FrictionForceLaw *fdf_) {
        fdf = fdf_;
      }
      void setContactKinematics(ContactKinematics* ck, int contactNumber) {
        contactKinematics[contactNumber] = ck;
      }
      ContactKinematics* getContactKinematics(int contactNumber) const {
        return contactKinematics[contactNumber];
      }
      /**
       * \brief output information to console?
       */
      void setINFO(const bool &INFO_) {
        INFO = INFO_;
      }
      virtual void setPlotContactPoint(const int & contactNumber, bool enable);
      /***************************************************/
      //
      //
      /**
       * \brief get number of friction directions
       *
       * \todo use parameter for each individual contact pairing (problem of resizing of g, la etc.)
       */
      int getFrictionDirections();

      /*!
       * \brief add a contour-pairing to force law
       * \param first contour
       * \param second contour
       * \param contactKinematics for both contours
       *
       * \General Remark: The parameters (LagrangeParameterPositions) of the function (type: Function2) for the influence numbers have to be in the same order like the contours here.
       *                  So the first input parameter for the function will be the LagrangeParameterPosition of contour1 and the second input parameter for the function will be the LagrangeParameterPosition for contour2
       */
      void add(Contour *contour1, Contour *contour2, bool plotContact = true, ContactKinematics* contactKinematics_ = 0);

      /**
       * \brief add a function that represents the coupling between two contours
       * \param name of first contour
       * \param name of second contour
       * \param Function to describe coupling between contours
       *
       * \General Remark: The parameters (LagrangeParameterPositions) of the function have to be in the same order as it was given the add(...)-method
       */
      void addContourCoupling(Contour *contour1, Contour *contour2, InfluenceFunction *fct);

      //
      //      void computeCurvatures(fmatvec::Vec & r) const;
      //
      //      virtual void initializeUsingXML(TiXmlElement *element);

    protected:
      /**
       * \brief saves all possible contacts in a vector
       */
      virtual void updatePossibleContactPoints();

      /**
       * \brief updates the influence matrix C
       */
      virtual void updateC(const double t);

      /**
       * \brief computes the coupling factor for the influence matrix on one contact point (two contours)
       * \param number of contact point
       */
      virtual double computeFactorC(const int &currentContactNumber);

      /**
       * \brief computes the coupling factor for the influence matrix between two contact points (four contours)
       * \param number of contact point
       * \param number of coupling contact point
       */
      virtual double computeFactorC(const int &currentContactNumber, const int &couplingContactNumber);

      /*
       * \brief computes the "material constant" to have a good guess for the lambda-vector
       */
      virtual void computeMaterialConstant(const double & t);

      /**
       * \brief vector that saves each contact kinematics  for each contour-pairing
       */
      std::vector<ContactKinematics*> contactKinematics;

      /**
       * \brief force law defining relation between tangential velocities and tangential forces
       *
       * \todo one friction-force-law for each contact-point?
       */
      FrictionForceLaw* fdf;

      /**
       * \brief vector of an array of contourPointData (length of 2) for definition of relative contact situation
       *
       * Every contour-pairing within the maxwell contact has its own cpData. For the both contours there is a array of the length of 2 that saves the kinematic data of the both contours.
       * So cpData[0] has saves ContourPointData (cpData[0][0] for the first contour of the contact-pairing and cpData[0][1] for the second contour of the contact-paring)
       */
      std::vector<ContourPointData*> cpData;

      /**
       * \brief boolean vector symbolising activity of contacts on position level with possibility to save previous time step
       */
      std::vector<bool> gActive, gActive0;

      /**
       * \brief boolean vector symbolising activity of contacts on velocity level
       */
//      std::vector<unsigned int*> gdActive;
      /**
       * \brief index for tangential directions in projection matrices
       */
      //      fmatvec::Index iT;
      /**
       * \brief relative velocity and acceleration after an impact for event driven scheme summarizing all possible contacts
       */
//      fmatvec::Vec gdn, gdd;
      /**
       * \brief vector of relative distance
       *
       * remark: every entry i (gk[i]) references to its position in the g-vector of the link-class
       */
      std::vector<fmatvec::Vec> gk;

      /**
       * \brief vector of relative velocity for each contour-pairing in event driven scheme
       *
       * remark: every entry i (gdk[i]) references to its position in the gd-vector of the link-class
       */
      std::vector<fmatvec::Vec> gdk;

      /**
       * \brief vector of relative velocity for each contour-pairing after impact in event driven scheme
       *
       * remark: every entry i (gdnk[i]) references to its position in the gdn-vector of the link-class
       */
//      std::vector<fmatvec::Vec> gdnk;
      /**
       * \brief vector of relative acceleration for each contour-pairing in event driven scheme
       *
       * remark: every entry i (gddk[i]) references to its position in the gdd-vector of the link-class
       */
//      std::vector<fmatvec::Vec> gddk;
      /**
       * \brief vectors of relative force parameters for each contour-pairing
       *
       * remark: every entry i (lak[i]) references to its position in the la-vector of the link-class
       */
      std::vector<fmatvec::Vec> lak;

      /**
       * \brief vector for relative acceleration description with respect to contour parameters
       *
       * remark: every entry i (wbk[i]) references to its position in the wb-vector of the link-class
       */
//      std::vector<fmatvec::Vec> wbk;
      /**
       * \brief stop vectors for each contour-pairing
       *
       * remark: every entry i (svk[i]) references to its position in the sv-vector of the link-class
       */
//      std::vector<fmatvec::Vec> svk;
      /**
       * \brief vector of relaxation factors for possible contact points
       */
//      std::vector<fmatvec::Vec> rFactork; //todo_grundl: make rFactors for each contact (?) --> stabilize the Newton-Method --> read in thesis
      /**
       * \brief boolean evaluation of stop vector for possible contact points
       */
//      std::vector<fmatvec::Vector<int> > jsvk;
      /**
       * \brief single-valued forces for possible contact points
       */
//      std::vector<fmatvec::Mat*> fF;
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

      /**
       * \brief saves the indices of all active contacts
       */
      std::vector<int> possibleContactPoints;

      /*
       * \brief vector that holds the indices for those contacts, that should be plottet
       */
      std::vector<int> plotContactPoints;

      /**
       * \brief Influence matrix between contact points
       */
      fmatvec::SymMat C;

      /**
       * \brief saves the influence functions for a pair of contours. The key is the pair of contour names
       */
      std::map<std::pair<Contour*, Contour*>, InfluenceFunction*> influenceFunctions;

      /**
       * \brief parameter for guessing starting values of contact force (average eigenvalue of influence-matrix)
       */
      double matConst;

      /**
       * \brief parameter to save if matConst has been computed already
       */
      bool matConstSetted;

      /**
       * \brief print INFO output?
       *
       * \todo wouldn't a logger for MBSim be nice
       */
      bool INFO;

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
       * \brief container of normal forces to draw
       */
      std::vector<OpenMBV::Arrow *> openMBVNormalForceArrow;

      /**
       * \brief container of friction forces to draw
       */
      std::vector<OpenMBV::Arrow *> openMBVFrictionForceArrow;

      /**
       * \brief size of ContactFrames to draw
       */
      double openMBVContactFrameSize;

      /**
       * \brief enable flag of ContactFrames to draw
       */
      bool openMBVContactFrameEnabled;

      /**
       * \brief enable openMBV arrows for every contact in normal direction
       */
      OpenMBV::Arrow *normalForceArrow;

      /**
       * \brief enable openMBV arrows for every contact in friction direction
       */
      OpenMBV::Arrow *frictionForceArrow;

#endif

    private:
      std::string saved_ref1, saved_ref2;
  };

  class MaxwellFunction : public MBSim::Function1<fmatvec::Vec, fmatvec::Vec> {
    public:
      /**
       * \brief constructor
       * \param rigidBodyGap_  gap between the bodies if they were totally rigid
       * \param C_             Influence-Matrix
       * \param r_             r-factor for the project-function
       * \param INFO_          print information to console?
       */
      MaxwellFunction(const fmatvec::Vec &rigidBodyGap_, const fmatvec::SymMat &C_, const double &r_ = 10, bool INFO_ = false);

      /**
       * \brief destructor
       */
      virtual ~MaxwellFunction();

      /* INHERITED INTERFACE */
      /**
       * \param start parameter: first entries: gap, last entries: contact forces (lambda)
       */
      fmatvec::Vec operator()(const fmatvec::Vec &gapLambda, const void * = NULL);
      /***************************************************/

      /* GETTER / SETTER*/
      void setSolverType(const SolverType &solverType_) {
        solverType = solverType_;
      }
      /******************/

    private:

      /**
       * \brief Number of possible contact points (= dimension of the MFL)
       */
      int NumberOfContacts;

      /**
       * \brief vector of all rigid body gaps
       */
      fmatvec::Vec rigidBodyGap;

      /**
       * \brief Influence matrix for the contacts
       */
      fmatvec::SymMat C;

      /**
       * \brief parameter for the prox-function (r>0)
       */
      double r;

      /**
       *  \brief which solver is used (leads to different return values of function)
       */
      SolverType solverType;

      /**
       * \brief parameter to print information
       */
      bool INFO;

  };

  class MaxwellJacobian : public MBSim::Function1<fmatvec::SqrMat, fmatvec::Vec> {
    public:
      /**
       * \brief constructor
       */
      MaxwellJacobian();

      /**
       * \brief destructor
       */
      virtual ~MaxwellJacobian();

      /* INHERITED INTERFACE */
      fmatvec::SqrMat operator()(const fmatvec::Vec& distance, const void * = NULL);
      /*******************************************/
  };
}
#endif /* MAXWELL_CONTACT_H_ */
