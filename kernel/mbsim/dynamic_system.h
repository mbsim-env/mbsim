/* Copyright (C) 2004-2014 MBSim Development Team
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

#ifndef _DYNAMIC_SYSTEM_H_
#define _DYNAMIC_SYSTEM_H_

#include "mbsim/element.h"
#include "mbsim/mbsim_event.h"

namespace H5 {
  class Group;
}

namespace MBSim {
  class Frame;
  class FixedRelativeFrame;
  class Contour;
  class RigidContour;
  class Object;
  class Link;
  class Constraint;
  class ModellingInterface;
  class Contact;
  class Observer;

  /**
   * \brief dynamic system as topmost hierarchical level
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   * \date 2009-04-06 ExtraDynamicInterface included (Thorsten Schindler)
   * \date 2009-06-14 OpenMP (Thorsten Schindler)
   * \date 2009-07-08 relative dynamic system location (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-27 implicit integration improvement (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   * \date 2009-12-14 revised inverse kinetics (Martin Foerg)
   * \date 2010-07-06 modifications for timestepper ssc, e.g LinkStatus and buildListOfSetValuedLinks (Robert Huber)
   * \date 2012-05-08 OpenMP completely removed; will be inserted again soon (Jan Clauberg)
   * \date 2012-05-08 modifications for AutoTimeSteppingSSCIntegrator (Jan Clauberg)
   * \date 2013-06-23 removed extra dynamics (Martin Foerg)
   * \date 2014-09-16 contact forces are calculated on acceleration level (Thorsten Schindler)
   */
  class DynamicSystem : public Element {
    public:
      /** 
       * \brief constructor
       */
      DynamicSystem(const std::string &name);

      /**
       * \brief destructor
       */
      ~DynamicSystem() override;

      /* INTERFACE FOR DERIVED CLASSES */
      virtual void updateT();
      virtual void updateh(int k=0);
      virtual void updateM();
      virtual void updateLLM();
      virtual void updatedq();
      virtual void updatedu();
      virtual void updatedx();
      virtual void updateqd();
      virtual void updateud();
      virtual void updatexd();
      virtual void updatezd();
      virtual void updatewb();
      virtual void updateW(int j=0);
      virtual void updateV(int j=0);
      virtual void updater(int j=0);
      virtual void updateJrla(int j=0);
      virtual void updateg();
      virtual void updategd();
      virtual void updateStopVector();
      virtual void updateStopVectorParameters();
      virtual void updateLinkStatus();
      virtual void updateLinkStatusReg();
      virtual void updateWInverseKinetics();
      virtual void updatebInverseKinetics();
      virtual void sethSize(int hSize_, int j=0);
      virtual int gethSize(int i=0) const { return hSize[i]; }
      virtual int getqSize() const { return qSize; }
      virtual int getuSize(int i=0) const { return uSize[i]; }
      virtual void calcqSize();
      virtual void calcuSize(int j=0);
      virtual int getuInd(int i=0) { return uInd[i]; }
      virtual void setqInd(int qInd_);
      virtual void setuInd(int uInd_, int j=0);
      virtual void sethInd(int hInd_, int j=0);
      virtual void setxInd(int xInd_);
      virtual const fmatvec::Vec& getq() const { return q; };
      virtual fmatvec::Vec& getq() { return q; };
      virtual const fmatvec::Vec& getu() const { return u; };
      virtual fmatvec::Vec& getu() { return u; };
      void setq(const fmatvec::Vec& q_) { q = q_; }
      void setu(const fmatvec::Vec& u_) { u = u_; }
      void setx(const fmatvec::Vec& x_) { x = x_; }
      void setjsv(const fmatvec::VecInt& jsv_) { jsv = jsv_; }
      void setInternalState(const fmatvec::Vec& internalState) { curis = internalState; nextis = internalState; }
      virtual H5::GroupBase *getPlotGroup() { return plotGroup; }
      std::shared_ptr<OpenMBV::Group> getOpenMBVGrp() override { return openMBVGrp; }
      std::shared_ptr<OpenMBV::Group> getFramesOpenMBVGrp() override { return framesOpenMBVGrp; }
      std::shared_ptr<OpenMBV::Group> getContoursOpenMBVGrp() override { return contoursOpenMBVGrp; }
      std::shared_ptr<OpenMBV::Group> getGroupsOpenMBVGrp() override { return groupsOpenMBVGrp; }
      std::shared_ptr<OpenMBV::Group> getObjectsOpenMBVGrp() override { return objectsOpenMBVGrp; }
      std::shared_ptr<OpenMBV::Group> getLinksOpenMBVGrp() override { return linksOpenMBVGrp; }
      std::shared_ptr<OpenMBV::Group> getConstraintsOpenMBVGrp() override { return constraintsOpenMBVGrp; }
      std::shared_ptr<OpenMBV::Group> getObserversOpenMBVGrp() override { return observersOpenMBVGrp; }

      virtual void calcxSize();
      const fmatvec::Vec& getx() const { return x; };
      fmatvec::Vec& getx() { return x; };
      int getxSize() const { return xSize; }
      void updatexRef(fmatvec::Vec &xParent);
      void updatexdRef(fmatvec::Vec &xdParent);
      void updatedxRef(fmatvec::Vec &dxParent);
      void init(InitStage stage, const InitConfigSet &config) override;
      virtual void initz();
      virtual void writez(H5::GroupBase *parent);
      virtual void readz0(H5::GroupBase *parent);
      /*****************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      /** DEPRECATED */
      void setDynamicSystemSolver(DynamicSystemSolver* sys) override;
      void plot() override;
      void plotAtSpecialEvent() override;
      /*****************************************************/

      /**
       * \brief solve contact equations with single step fixed point scheme
       * \return iterations of solver
       */
      virtual int solveConstraintsFixpointSingle();

      /**
       * \brief solve impact equations with single step fixed point scheme on velocity level 
       * \param time step-size
       * \return iterations of solver
       */
      virtual int solveImpactsFixpointSingle();

      /**
       * \brief solve contact equations with Gauss-Seidel scheme
       * \return iterations of solver
       */
      virtual int solveConstraintsGaussSeidel();

      /**
       * \brief solve impact equations with Gauss-Seidel scheme on velocity level 
       * \param time step-size
       * \return iterations of solver
       */
      virtual int solveImpactsGaussSeidel();

      /**
       * \brief solve contact equations with Newton scheme
       * \return iterations of solver
       */
      virtual int solveConstraintsRootFinding();

      /**
       * \brief solve impact equations with Newton scheme on velocity level 
       * \param time step-size
       * \return iterations of solver
       */
      virtual int solveImpactsRootFinding();

      /**
       * \brief compute JACOBIAN of contact equations
       */
      virtual int jacobianConstraints();

      /**
       * \brief compute JACOBIAN of contact equations on velocity level
       */
      virtual int jacobianImpacts();

      /**
       * \brief validate force laws concerning given tolerances
       */
      virtual void checkConstraintsForTermination();

      /**
       * \brief validate force laws concerning given tolerances on velocity level
       */
      virtual void checkImpactsForTermination();

      /**
       * \brief update relaxation factors for contact equations
       */
      virtual void updaterFactors();

      /**
       * \param name of the frame
       * \param check for existence of frame
       * \return frame
       */
      virtual Frame* getFrame(const std::string &name, bool check=true) const;

      /**
       * \param name of the contour
       * \param check for existence of contour
       * \return contour
       */
      virtual Contour* getContour(const std::string &name, bool check=true) const;
      /*****************************************************/

      /* GETTER / SETTER */
      void setFrameOfReference(Frame *frame) { R = frame; };
      const Frame* getFrameOfReference() const { return R; };

      const fmatvec::Mat& getT(bool check=true) const;
      const fmatvec::Vec& geth(int i=0, bool check=true) const;
      const fmatvec::SymMat& getM(bool check=true) const;
      const fmatvec::SymMat& getLLM(bool check=true) const;
      const fmatvec::Vec& getdq(bool check=true) const;
      const fmatvec::Vec& getdu(bool check=true) const;
      const fmatvec::Vec& getdx(bool check=true) const;
      const fmatvec::Vec& getqd(bool check=true) const;
      const fmatvec::Vec& getud(bool check=true) const;
      const fmatvec::Vec& getxd(bool check=true) const;
      const fmatvec::Mat& getW(int i=0, bool check=true) const;
      const fmatvec::Mat& getV(int i=0, bool check=true) const;
      const fmatvec::Vec& getla(bool check=true) const;
      const fmatvec::Vec& getLa(bool check=true) const;
      const fmatvec::Vec& getg(bool check=true) const;
      const fmatvec::Vec& getgd(bool check=true) const;
      fmatvec::Vec& getla(bool check=true);
      fmatvec::Vec& getLa(bool check=true);
      void setla(const fmatvec::Vec &la_) { la = la_; }
      void setLa(const fmatvec::Vec &La_) { La = La_; }
      fmatvec::VecInt& getjsv() { return jsv; }
      const fmatvec::VecInt& getjsv() const { return jsv; }
      fmatvec::Mat& getW(int i=0, bool check=true);
      fmatvec::SymMat& getLLM(bool check=true);
      fmatvec::Vec& getdq(bool check=true);
      fmatvec::Vec& getdu(bool check=true);
      fmatvec::Vec& getdx(bool check=true);

      fmatvec::VecInt& getLinkStatus() { return LinkStatus; }
      fmatvec::VecInt& getLinkStatusReg() { return LinkStatusReg; }
      const fmatvec::VecInt& getLinkStatus() const { return LinkStatus; }
      const fmatvec::VecInt& getLinkStatusReg() const { return LinkStatusReg; }

      const fmatvec::Mat& evalT();
      const fmatvec::Vec& evalh(int i=0);
      const fmatvec::SymMat& evalM();
      const fmatvec::SymMat& evalLLM();
      const fmatvec::Mat& evalW(int i=0);
      const fmatvec::Mat& evalV(int i=0);
      const fmatvec::Vec& evalwb();
      const fmatvec::Vec& evalr(int i=0);
      const fmatvec::Mat& evalJrla(int i=0);
      const fmatvec::Vec& evalrdt();
      const fmatvec::Vec& evalg();
      const fmatvec::Vec& evalgd();
      const fmatvec::Mat& evalWInverseKinetics();
      const fmatvec::Mat& evalbInverseKinetics();

      void setqd(const fmatvec::Vec& qd_) { qd = qd_; }
      void setud(const fmatvec::Vec& ud_) { ud = ud_; }
      void setxd(const fmatvec::Vec& xd_) { xd = xd_; }

      int getxInd() { return xInd; }
      int getlaInd() const { return laInd; } 

      int gethInd(int i=0) { return hInd[i]; }
      void setlaInd(int ind) { laInd = ind; }
      void setisInd(int ind) { isInd = ind; }
      void setgInd(int ind) { gInd = ind; }
      void setgdInd(int ind) { gdInd = ind; }
      void setrFactorInd(int ind) { rFactorInd = ind; }
      virtual void setsvInd(int svInd_);
      void setLinkStatusInd(int LinkStatusInd_) {LinkStatusInd = LinkStatusInd_;};      
      void setLinkStatusRegInd(int LinkStatusRegInd_) {LinkStatusRegInd = LinkStatusRegInd_;};

      void setqSize(int qSize_) { qSize = qSize_; }
      void setuSize(int uSize_, int i=0) { uSize[i] = uSize_; }
      void setxSize(int xSize_) { xSize = xSize_; }

      int getisSize() const { return isSize; } 
      int getlaSize() const { return laSize; } 
      int getgSize() const { return gSize; } 
      int getgdSize() const { return gdSize; } 
      int getrFactorSize() const { return rFactorSize; } 
      int getsvSize() const { return svSize; }
      int getLinkStatusSize() const { return LinkStatusSize; }
      int getLinkStatusRegSize() const { return LinkStatusRegSize; }
      /*****************************************************/

      const std::vector<Object*>& getObjects() const { return object; }
      const std::vector<Link*>& getLinks() const { return link; }
      const std::vector<DynamicSystem*>& getDynamicSystems() const { return dynamicsystem; }
      const std::vector<Frame*>& getFrames() const { return frame; }
      const std::vector<Contour*>& getContours() const { return contour; }
      const std::vector<Link*>& getSetValuedLinks() const { return linkSetValued; }

      /**
       * \brief references to positions of dynamic system parent
       * \param vector to be referenced
       */
      void updateqRef(fmatvec::Vec &qParent);

      /**
       * \brief references to differentiated positions of dynamic system parent
       * \param vector to be referenced
       */
      void updateqdRef(fmatvec::Vec &qdParent);

      void updatedqRef(fmatvec::Vec &dqParent);

      /**
       * \brief references to velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updateuRef(fmatvec::Vec &uParent);

      /**
       * \brief references to velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updateuallRef(fmatvec::Vec &uParent);

      /**
       * \brief references to differentiated velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updateudRef(fmatvec::Vec &udParent);

      void updateduRef(fmatvec::Vec &duParent);

      /**
       * \brief references to velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updateudallRef(fmatvec::Vec &udParent);

      /**
       * \brief references to smooth right hand side of dynamic system parent
       * \param complete vector to be referenced
       * \param vector concerning objects to be referenced
       * \param vector concerning links to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updatehRef(fmatvec::Vec &hParent, int j=0);

      /**
       * \brief references to nonsmooth right hand side of dynamic system parent
       * \param vector to be referenced
       */
      void updaterRef(fmatvec::Vec &rParent, int j=0);

      void updateJrlaRef(fmatvec::Mat &rParent, int j=0);

      /**
       * \brief references to nonsmooth right hand side of dynamic system parent
       * \param vector to be referenced
       */
      void updaterdtRef(fmatvec::Vec &rdtParent);

      /**
       * \brief references to linear transformation matrix between differentiated positions and velocities of dynamic system parent
       * \param matrix to be referenced
       */
      void updateTRef(fmatvec::Mat &TParent);

      /**
       * \brief references to mass matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateMRef(fmatvec::SymMat &MParent);

      /**
       * \brief references to Cholesky decomposition of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateLLMRef(fmatvec::SymMat &LLMParent);

      /**
       * \brief references to internal state of dynamic system parent
       */
      virtual void updateInternalStateRef(fmatvec::Vec &curisParent, fmatvec::Vec &nextisParent);

      /**
       * \brief references to relative distances of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updategRef(fmatvec::Vec &gParent);

      /**
       * \brief references to relative velocities of dynamic system parent
       * \param vector to be referenced
       */
      virtual void updategdRef(fmatvec::Vec &gdParent);

      /**
       * \brief references to contact forces of dynamic system parent
       * \param vector to be referenced
       */
      void updatelaRef(fmatvec::Vec &laParent);

      /**
       * \brief references to contact impulses of dynamic system parent
       * \param vector to be referenced
       */
      void updateLaRef(fmatvec::Vec &LaParent);

      void updatelaInverseKineticsRef(fmatvec::Vec &laParent);
      void updatebInverseKineticsRef(fmatvec::Mat &bParent);

      /**
       * \brief references to TODO of dynamic system parent
       * \param vector to be referenced
       */      
      virtual void updatewbRef(fmatvec::Vec &wbParent);

      /**
       * \brief references to contact force direction matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updateWRef(fmatvec::Mat &WParent, int j=0);

      /**
       * \brief references to contact force direction matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateWInverseKineticsRef(fmatvec::Mat &WParent);

      /**
       * \brief references to condensed contact force direction matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      virtual void updateVRef(fmatvec::Mat &VParent, int j=0);

      /**
       * \brief references to stopvector (rootfunction for event driven integrator) of dynamic system parent
       * \param vector to be referenced
       */
      void updatesvRef(fmatvec::Vec& svParent);

      /**
       * \brief references to boolean evaluation of stopvector concerning roots of dynamic system parent
       * \param vector to be referenced
       */
      void updatejsvRef(fmatvec::VecInt &jsvParent);

      /**
       * \brief references to status vector of set valued links with piecewise link equations (which piece is valid)
       * \param vector to be referenced 
       */
      void updateLinkStatusRef(fmatvec::VecInt &LinkStatusParent);

      /**
       * \brief references to status vector of single valued links
       * \param vector to be referenced 
       */
      void updateLinkStatusRegRef(fmatvec::VecInt &LinkStatusRegParent);
            
      /**
       * \brief references to residuum of contact equations of dynamic system parent
       * \param vector to be referenced
       */
      void updateresRef(fmatvec::Vec &resParent);

      /**
       * \brief references to relaxation factors for contact equations of dynamic system parent
       * \param vector to be referenced
       */
      void updaterFactorRef(fmatvec::Vec &rFactorParent);

      void clearElementLists();

      /**
       * \brief build flat list of dynamic systems
       * \param list of dynamic systems
       */
      void buildListOfDynamicSystems(std::vector<DynamicSystem*> &sys);

      /**
       * \brief build flat list of objects
       * \param list of objects
       */
      void buildListOfObjects(std::vector<Object*> &obj);

      /**
       * \brief build flat list of links
       * \param list of links
       */
      void buildListOfLinks(std::vector<Link*> &lnk);

      /**
       * \brief build flat list of all constraints
       * \param list of constraints
       */
      void buildListOfConstraints(std::vector<Constraint*> &crt);

      /**
       * \brief build flat list of frames
       * \param list of frames
       */
      void buildListOfFrames(std::vector<Frame*> &frm);

      /**
       * \brief build flat list of contours
       * \param list of contours
       */
      void buildListOfContours(std::vector<Contour*> &cnt);

      /**
       * \brief build flat list of models
       * \param list of models
       */
      void buildListOfModels(std::vector<ModellingInterface*> &modelList);

      /**
       * \brief build flat list of inverse kinetics links
       * \param list of inverse kinetics links
       */
      void buildListOfInverseKineticsLinks(std::vector<Link*> &iklnk);

      /**
       * \brief build flat list of observers
       * \param list of observers
       */
      void buildListOfObservers(std::vector<Observer*> &obsrv);

      /**
       * \brief analyse constraints of dynamic systems for usage in inverse kinetics
       */
      void setUpInverseKinetics();

      /**
       * \brief distribute links to set- and single valued container
       */
      void setUpLinks();

      /**
       * \return flag, if vector of active relative distances has changed (set-valued laws)
       */
      bool gActiveChanged();
      
      /**
       * \return flag, if vector of active relative distances has changed (single-valued laws)
       */
      bool gActiveChangedReg();
      
      /**
       * \return flag, if an impact occured in the system
       */
      bool detectImpact();

      /**
       * \brief calculates size of stop vector
       */
      void calcsvSize();

      /**
       * \brief calculates size of contact force parameters
       */
      void calclaSize(int j);

      /**
       * \brief calculates size of set-valued link status vector
       */
      void calcLinkStatusSize();

      /**
       * \brief calculates size of single-valued link status vector
       */
      void calcLinkStatusRegSize();

      /**
       * \brief calculates size of contact force parameters
       */
      void calclaInverseKineticsSize();

      /**
       * \brief calculates size of contact force parameters
       */
      void calcbInverseKineticsSize();

      void calcisSize();

      /**
       * This function is called just before the internal state vector gets updated.
       * This means that the internal state is still at its old value when this function is called but get updated immediately afterwards.
       * You can use this function to do actions which should only be done at valid integrator states (but not at integrates states which may be rejected later on).
       */
      virtual void aboutToUpdateInternalState();

      virtual void postprocessing();

      /**
       * \brief calculates size of relative distances
       */
      void calcgSize(int j);

      /**
       * \brief calculates size of gap velocities
       * \param flag to decide which contacts are included in the calculation
       * 0 = all contacts
       * 1 = closed contacts
       * 2 = contacts which stay closed
       * 3 = sticking contacts
       *
       * see SingleContact for the implementation
       */
      void calcgdSize(int j);

      /**
       * \brief calculates size of relaxation factors for contact equations
       */
      void calcrFactorSize(int j);

      /** 
       * \brief rearrange vector of active setvalued links
       */
      void setUpObjectsWithNonConstantMassMatrix();

      /**
       * \brief rearrange vector of active setvalued links
       */
      void setUpActiveLinks();

      /**
       * \brief check if set-valued contacts are active and set corresponding attributes
       * \param flag to decide which criteria are used to define 'activity'
       * 1 = position level activity (gActive = normal gap not larger than tolerance)
       * 2 = velocity level activity (if gActive, gdActive[normal] = normal gap velocity not larger than tolerance ; if gdActive[normal], gdActive[tangential] = tangential gap velocity not larger than tolerance, i.e., sticking)
       * 3 = velocity level activity (if gActive, gdActive[normal] = new normal gap velocity AFTER impact not larger than tolerance ; if gdActive[normal], gdActive[tangential] = new tangential gap velocity AFTER impact not larger than tolerance, i.e., sticking)
       * 4 = acceleration level activity (if gActive and gdActive[normal], gddActive[normal] = normal gap acceleration not larger than tolerance ; if gddActive[normal] and gdActive[tangential], gddActive[tangential] = tangential gap acceleration not larger than tolerance, i.e., stay sticking)
       * 5 = activity clean-up: if there is no activity on acceleration or velocity level, also more basic levels are set to non-active
       * 6 = closing sets all activities
       * 7 = slip-stick transition sets tangential velocity and acceleration activity
       * 8 = opening and stick-slip transition set corresponding acceleration activity to non-active
       *
       * see SingleContact for the implementation
       */
      void checkActive(int j);

      /**
       * \param tolerance for relative position
       */
      virtual void setGeneralizedRelativePositionTolerance(double tol);

      /**
       * \param tolerance for relative velocity
       */
      virtual void setGeneralizedRelativeVelocityTolerance(double tol);

      /**
       * \param tolerance for relative acceleration
       */
      virtual void setGeneralizedRelativeAccelerationTolerance(double tol);

      /**
       * \param tolerance for contact force
       */
      virtual void setGeneralizedForceTolerance(double tol);

      /**
       * \param tolerance for impact
       */
      virtual void setGeneralizedImpulseTolerance(double tol);

      /** \param correction value for relative position
       */
      virtual void setGeneralizedRelativePositionCorrectionValue(double corr);

      /**
       * \param correction value for relative velocity
       */
      virtual void setGeneralizedRelativeVelocityCorrectionValue(double corr);

      /**
       * \param maximum relaxation factor for contact equations
       */
      void setrMax(double rMax);

      /**
       * \param frame
       * \return index of frame TODO renaming
       */
      int frameIndex(const Frame *frame_) const;

      void addFrame(FixedRelativeFrame *frame);

      void addContour(RigidContour *contour);

      /**
       * \param dynamic system to add
       */
      void addGroup(DynamicSystem *sys);

      /**
       * \param name of the dynamic system
       * \param check for existence of dynamic system
       * \return dynamic system
       */
      DynamicSystem* getGroup(const std::string &name,bool check=true) const;

      /**
       * \param object to add
       */
      void addObject(Object *obj);

      /**
       * \param name of the object
       * \param check for existence of object
       * \return object
       */
      Object* getObject(const std::string &name,bool check=true) const;

      /**
       * \param link to add
       */
      void addLink(Link *lnk);

      /**
       * \param constraint to add
       */
      void addConstraint(Constraint *crt);

      /**
       * \param add link for inverse kinetics
       */
      void addInverseKineticsLink(Link *lnk);

      Observer* getObserver(const std::string &name,bool check=true) const;
      void addObserver(Observer *ele);

      /**
       * \param name of the link
       * \param check for existence of link
       * \return link
       */
      Link* getLink(const std::string &name,bool check=true) const;

      /**
       * \param name of the constraint
       * \param check for existence of constraint
       * \return constraint
       */
      Constraint* getConstraint(const std::string &name,bool check=true) const;

      /**
       * \param modell to add
       */
      void addModel(ModellingInterface *model_);

      /**
       * \param name of the model
       * \param check for existence of model
       * \return modelling interface
       */
      ModellingInterface* getModel(const std::string &name, bool check=true) const;

      /** Return frame "I" */
      Frame *getFrameI() { return I; }

      Element *getChildByContainerAndName(const std::string &container, const std::string &name) const override;

      virtual void updatecorr(int j);
      void updatecorrRef(fmatvec::Vec &ref);
      void calccorrSize(int j);

      void checkRoot();

      void resetUpToDate() override;

      void updateStateTable();

      H5::GroupBase *getFramesPlotGroup() override { return framesPlotGroup; }
      H5::GroupBase *getContoursPlotGroup() override { return contoursPlotGroup; }
      H5::GroupBase *getGroupsPlotGroup() override { return groupsPlotGroup; }
      H5::GroupBase *getObjectsPlotGroup() override { return objectsPlotGroup; }
      H5::GroupBase *getLinksPlotGroup() override { return linksPlotGroup; }
      H5::GroupBase *getConstraintsPlotGroup() override { return constraintsPlotGroup; }
      H5::GroupBase *getObserversPlotGroup() override { return observersPlotGroup; }

    private:
      friend class DynamicSystemSolver;
      void addFrame(Frame *frame_);
      void addContour(Contour *contour_);

    protected:
      /**
       * \brief parent frame
       */
      Frame *R;

      /** 
       * \brief container for possible ingredients
       */
      std::vector<Object*> object;
      std::vector<Object*> objectWithNonConstantMassMatrix;
      std::vector<Link*> link;
      std::vector<Link*> linkSingleValued;
      std::vector<Link*> linkSetValued;
      std::vector<Link*> linkSetValuedActive;
      std::vector<Link*> linkWithStopVector;
      std::vector<ModellingInterface*> model;
      std::vector<DynamicSystem*> dynamicsystem;
      std::vector<Link*> inverseKineticsLink;
      std::vector<Observer*> observer;
      std::vector<Constraint*> constraint;

      /** 
       * \brief linear relation matrix of position and velocity parameters
       */
      fmatvec::Mat T;

      /**
       * \brief mass matrix
       */
      fmatvec::SymMat M;

      /** 
       * \brief Cholesky decomposition of mass matrix
       */
      fmatvec::SymMat LLM;

      /**
       * \brief positions, differentiated positions, initial positions
       */
      fmatvec::Vec q, qd, dq;

      /**
       * \brief velocities, differentiated velocities, initial velocities
       */
      fmatvec::Vec u, ud, du;

      /**
       * \brief order one parameters, differentiated order one parameters, initial order one parameters
       */
      fmatvec::Vec x, xd, dx;

      /**
       * \brief smooth, smooth with respect to objects, smooth with respect to links and nonsmooth
       */
      fmatvec::Vec h[2], r[2], rdt;

      /**
       * \brief Jacobian dr/dla
       */
      fmatvec::Mat Jrla[2];

      /**
       * \brief 
       */
      fmatvec::Mat W[2], V[2];

      /**
       * \brief contact force parameters
       */
      fmatvec::Vec la, La;

      fmatvec::Vec curis, nextis;

      /** 
       * \brief relative distances and velocities
       */
      fmatvec::Vec g, gd;

      /**
       * \brief TODO
       */
      fmatvec::Vec wb;

      /**
       * \brief residuum of nonlinear contact equations for Newton scheme
       */
      fmatvec::Vec res;

      /**
       * \brief rfactors for relaxation nonlinear contact equations
       */
      fmatvec::Vec rFactor;

      /**
       * \brief stop vector (root functions for event driven integration
       */
      fmatvec::Vec sv;

      /**
       * \brief boolean evaluation of stop vector concerning roots
       */
      fmatvec::VecInt jsv;

      /**
       * \brief status of set-valued links 
       */
      fmatvec::VecInt LinkStatus;

      /**
       * \brief status of single-valued links 
       */
      fmatvec::VecInt LinkStatusReg;

      /** 
       * \brief size and local start index of positions relative to parent
       */
      int qSize, qInd;

      /** 
       * \brief size and local start index of velocities relative to parent
       */
      int uSize[2], uInd[2];

      /** 
       * \brief size and local start index of order one parameters relative to parent
       */
      int xSize, xInd;

      /** 
       * \brief size and local start index of order smooth right hand side relative to parent
       */
      int hSize[2], hInd[2];

      int isSize, isInd;

      /** 
       * \brief size and local start index of relative distances relative to parent
       */
      int gSize, gInd;

      /** 
       * \brief size and local start index of relative velocities relative to parent
       */
      int gdSize, gdInd;

      /** 
       * \brief size and local start index of contact force parameters relative to parent
       */
      int laSize, laInd;

      /** 
       * \brief size and local start index of rfactors relative to parent
       */
      int rFactorSize, rFactorInd;

      /** 
       * \brief size and local start index of stop vector relative to parent
       */
      int svSize, svInd;

      /**
       * \brief size and local start index of set-valued link status vector relative to parent
       */
      int LinkStatusSize, LinkStatusInd;
      
      /**
       * \brief size and local start index of single-valued link status vector relative to parent
       */
      int LinkStatusRegSize, LinkStatusRegInd;
      
      /**
       * \brief vector of frames and contours
       */
      std::vector<Frame*> frame;
      std::vector<Contour*> contour;

      std::shared_ptr<OpenMBV::Group> openMBVGrp;
      std::shared_ptr<OpenMBV::Group> framesOpenMBVGrp;
      std::shared_ptr<OpenMBV::Group> contoursOpenMBVGrp;
      std::shared_ptr<OpenMBV::Group> groupsOpenMBVGrp;
      std::shared_ptr<OpenMBV::Group> objectsOpenMBVGrp;
      std::shared_ptr<OpenMBV::Group> linksOpenMBVGrp;
      std::shared_ptr<OpenMBV::Group> constraintsOpenMBVGrp;
      std::shared_ptr<OpenMBV::Group> observersOpenMBVGrp;
      std::shared_ptr<H5::File> hdf5File;

      /** A pointer to frame "I" */
      Frame *I;

      /** 
       * \brief size of contact force parameters of special links relative to parent
       */
      int laInverseKineticsSize, bInverseKineticsSize;

      fmatvec::Mat WInverseKinetics, bInverseKinetics;
      fmatvec::Vec laInverseKinetics;

      int corrSize, corrInd;
      fmatvec::Vec corr;

      std::string saved_frameOfReference;

      H5::GroupBase *framesPlotGroup;
      H5::GroupBase *contoursPlotGroup;
      H5::GroupBase *groupsPlotGroup;
      H5::GroupBase *objectsPlotGroup;
      H5::GroupBase *linksPlotGroup;
      H5::GroupBase *constraintsPlotGroup;
      H5::GroupBase *observersPlotGroup;
  };
}

#endif

