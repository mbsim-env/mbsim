/* Copyright (C) 2004-2014 MBSim Development Team
 * 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 * 
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details. 
 *
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef _LINK_H_
#define _LINK_H_

#include "mbsim/element.h"
#include "mbsim/mbsim_event.h"

namespace H5 {
  class Group;
}

namespace MBSim {

  /** 
   * \brief general link to one or more objects
   * \author Martin Foerg
   * \date 2009-03-26 some comments (Thorsten Schindler)
   * \date 2009-04-06 ExtraDynamicInterface included / MechanicalLink added (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-27 enhanced structure for implicit integration (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   * \date 2009-12-14 revised inverse kinetics (Martin Foerg)
   * \date 2010-07-06 added LinkStatus and LinearImpactEstimation for timestepper ssc (Robert Huber)
   * \date 2012-05-08 added LinkStatusReg for AutoTimeSteppingSSCIntegrator (Jan Clauberg)
   * \date 2014-09-16 contact forces are calculated on acceleration level (Thorsten Schindler) 
   */
  //class Link : public Element, public LinkInterface, public ExtraDynamicInterface {
  class Link : public Element {
    public:
      /**
       * \brief constructor
       * \param name of link
       */
      Link(const std::string &name);

      /**
       * \brief destructor
       */
      virtual ~Link() { }

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updateg() { }
      virtual void updategd() { }
      virtual void updatewb() { }
      virtual void updateW(double t, int i=0) { }
      virtual void updateV(double t, int i=0) { }
      virtual void updateh(double t, int i=0) { }
      virtual void updateStopVector() { }
      virtual void updateLinkStatus() { }
      virtual void updateLinkStatusReg() { }
      virtual void updateJacobians(int j=0) { }
      virtual void updateb() { }
      /***************************************************/

      /* INHERITED INTERFACE OF EXTRADYNAMICINTERFACE */
      virtual void updatedx(double t, double dt) { }
      virtual void updatexd() { }
      virtual void calcxSize() { xSize = 0; }
      virtual const fmatvec::Vec& getx() const { return x; }
      virtual fmatvec::Vec& getx() { return x; }
      virtual void setxInd(int xInd_) { xInd = xInd_; };
      virtual int getxSize() const { return xSize; }
      virtual void updatexRef(const fmatvec::Vec& ref);
      virtual void updatexdRef(const fmatvec::Vec& ref);
      virtual void updatebRef(const fmatvec::Mat &hRef);
      virtual void init(InitStage stage);
      virtual void initz();
      virtual void writez(H5::GroupBase *group);
      virtual void readz0(H5::GroupBase *group);
      /***************************************************/

      virtual void setbInd(int bInd_) { bInd = bInd_; };

      /* INHERITED INTERFACE OF ELEMENT */
      std::string getType() const { return "Link"; }
      virtual void plot();
      virtual void closePlot();
      /***************************************************/

      /* INTERFACE TO BE DEFINED IN DERIVED CLASS */
      /**
       * \brief references to contact force direction matrix of dynamic system parent
       */
      virtual void updateWRef(const fmatvec::Mat& ref, int i=0) = 0;

      /**
       * \brief references to condensed contact force direction matrix of dynamic system parent
       */
      virtual void updateVRef(const fmatvec::Mat& ref, int i=0) = 0;

      /**
       * \brief references to complete and link smooth force vector of dynamic system parent
       */
      virtual void updatehRef(const fmatvec::Vec &hRef, int i=0) = 0;

      /**
       * \brief references to nonsmooth force vector of dynamic system parent
       */
      virtual void updaterRef(const fmatvec::Vec &ref, int i=0) = 0;

      /**
       * \brief references to TODO of dynamic system parent
       */
      virtual void updatewbRef(const fmatvec::Vec &ref);

      /**
       * \brief references to contact force parameter of dynamic system parent
       */
      virtual void updatelaRef(const fmatvec::Vec& ref);

      /**
       * \brief references to contact force parameter of dynamic system parent
       */
      virtual void updateLaRef(const fmatvec::Vec& ref);

      /**
       * \brief delete reference to contact force parameter of dynamic system parent
       */
      virtual void deletelaRef();

      /**
       * \brief references to contact relative distances of dynamic system parent
       */
      virtual void updategRef(const fmatvec::Vec& ref);

      /**
       * \brief references to contact relative velocities of dynamic system parent
       */
      virtual void updategdRef(const fmatvec::Vec& ref);

      /**
       * \brief references to residuum of nonlinear contact equations of dynamic system parent
       */
      virtual void updateresRef(const fmatvec::Vec& ref);

      /**
       * \brief references to rfactors of dynamic system parent
       */
      virtual void updaterFactorRef(const fmatvec::Vec& ref);

      /**
       * \brief references to stopvector of dynamic system parent (root function for event driven integration)
       */
      virtual void updatesvRef(const fmatvec::Vec &sv);

      /**
       * \brief references to stopvector evaluation of dynamic system parent (root detection with corresponding bool array by event driven integrator)
       */
      virtual void updatejsvRef(const fmatvec::VecInt &jsvParent);

      /**
       * \brief reference to vector of link status (for set valued links with piecewise link equations)
       */
       virtual void updateLinkStatusRef(const fmatvec::VecInt &LinkStatusParent);

      /**
       * \brief reference to vector of link status (for single-valued links)
       */
       virtual void updateLinkStatusRegRef(const fmatvec::VecInt &LinkStatusRegParent);

      /**
       * \brief calculates size of contact force parameters
       */
      virtual void calclaSize(int j) { laSize = 0; }

      /**
       * \brief calculates size of relative distances
       */
      virtual void calcgSize(int j) { gSize = 0; }

      /**
       * \brief calculates size of gap velocities
       * \param flag to decide which contacts are included in the calculation
       *
       * see SingleContact for the implementation and DynamicSystem for explanation
       */
      virtual void calcgdSize(int j) { gdSize = 0; }

      /**
       * \brief calculates size of rfactors
       */
      virtual void calcrFactorSize(int j) { rFactorSize = 0; }

      /**
       * \brief calculates size of rfactors
       */
      virtual void calcbSize() { bSize = 0; }

      /**
       * \brief calculates size of stopvector (root function for event driven integration)
       */
      virtual void calcsvSize() { svSize = 0; }
      
      /**
       * \brief calculates size of vector LinkStatus
       */
      virtual void calcLinkStatusSize() { LinkStatusSize =0;}

      /**
       * \brief calculates size of vector LinkStatusReg
       */
      virtual void calcLinkStatusRegSize() { LinkStatusRegSize =0;}

      /**
       * \brief asks the link if it contains force laws that contribute to the lagrange multiplier and is therefore set valued
       *
       * \return set valued force laws used within the link?
       */
      virtual bool isSetValued() const { return false; }

      /*!
       * \brief asks the link if it contains single valued force laws that contribute to the right-hand side vector h
       *
       * \return single valued force laws used within link?
       */
      virtual bool isSingleValued() const { return false; }

      /**
       * \return updateh is needed if set valued force law is not active?
       *
       * \todo: is this used anyhow?
       */
      virtual bool hasSmoothPart() const { return false; }

      /**
       * \return are link equations active?
       */
      virtual bool isActive() const = 0;

      /**
       * \return has the relative distance vector changed?
       */
      virtual bool gActiveChanged() = 0;
      
      /**
       * \return has an impact occured?
       */
      virtual bool detectImpact() { return false; }

      /**
       * solve impact equations of motion with single step fixed point scheme on velocity level
       * \param time step-size
       */
      virtual void solveImpactsFixpointSingle(double t, double dt) { THROW_MBSIMERROR("(Link::solveImpactsFixpointSingle): Not implemented."); }

      /**
       * solve contact equations of motion with single step fixed point scheme
       */
      virtual void solveConstraintsFixpointSingle() { THROW_MBSIMERROR("(Link::solveConstraintsFixpointSingle): Not implemented."); }

      /**
       * solve impact equations of motion with Gauss-Seidel scheme on velocity level
       * \param time step-size
       */
      virtual void solveImpactsGaussSeidel(double t, double dt) { THROW_MBSIMERROR("(Link::solveImpactsGaussSeidel): Not implemented."); }

      /**
       * solve contact equations of motion with Gauss-Seidel scheme
       */
      virtual void solveConstraintsGaussSeidel() { THROW_MBSIMERROR("(Link::solveConstraintsGaussSeidel): Not implemented."); }

      /**
       * solve impact equations of motion with Newton scheme on velocity level
       * \param time step-size
       */
      virtual void solveImpactsRootFinding(double t, double dt) { THROW_MBSIMERROR("(Link::solveImpactsRootFinding): Not implemented."); }

      /**
       * solve contact equations of motion with Newton scheme
       */
      virtual void solveConstraintsRootFinding() { THROW_MBSIMERROR("(Link::solveConstraintsRootFinding): Not implemented."); }

      /**
       * \brief computes JACOBIAN and mass action matrix of nonlinear contact equations
       */
      virtual void jacobianConstraints() { THROW_MBSIMERROR("(Link::jacobianConstraints): Not implemented."); }

      /**
       * \brief computes JACOBIAN and mass action matrix of nonlinear contact equations on velocity level
       */
      virtual void jacobianImpacts(double t, double dt) { THROW_MBSIMERROR("(Link::jacobianImpacts): Not implemented."); }

      /**
       * \brief update relaxation factors for contact equations
       */
      virtual void updaterFactors() { THROW_MBSIMERROR("(Link::updaterFactors): Not implemented."); }

      /**
       * \brief verify underlying force laws on velocity level concerning given tolerances
       */
      virtual void checkImpactsForTermination(double t, double dt) { THROW_MBSIMERROR("(Link::checkImpactsForTermination): Not implemented."); }
      
      /**
       * \brief verify underlying force laws concerning given tolerances
       */
      virtual void checkConstraintsForTermination() { THROW_MBSIMERROR("(Link::checkConstraintsForTermination): Not implemented."); }

      /**
       * \brief check if set-valued contacts are active and set corresponding attributes
       * \param flag to decide which criteria are used to define 'activity'
       *
       * see SingleContact for the implementation and DynamicSystem for explanation
       */
      virtual void checkActive(double t, int j) { }

      /**
       * \brief compute potential energy
       */
      virtual double computePotentialEnergy() { return 0; }

      virtual void setlaTol(double tol) { laTol = tol; }
      virtual void setLaTol(double tol) { LaTol = tol; }
      virtual void setgTol(double tol) { gTol = tol; }
      virtual void setgdTol(double tol) { gdTol = tol; }
      virtual void setgddTol(double tol) { gddTol = tol; }
      virtual void setrMax(double rMax_) { rMax = rMax_; }
      virtual void setLinkStatusInd(int LinkStatusInd_) { LinkStatusInd = LinkStatusInd_; };
      virtual void setLinkStatusRegInd(int LinkStatusRegInd_) { LinkStatusRegInd = LinkStatusRegInd_; };
      virtual void setlaInd(int laInd_) { laInd = laInd_;Ila=fmatvec::Index(laInd,laInd+laSize-1); }
      virtual void setgInd(int gInd_) { gInd = gInd_; Ig=fmatvec::Index(gInd,gInd+gSize-1); }
      virtual void setgdInd(int gdInd_) { gdInd = gdInd_; }
      virtual void setrFactorInd(int rFactorInd_) { rFactorInd = rFactorInd_; }
      /**
       * \brief get gap distance and calculate gap velocity of unilateral links to estimate impacts within the next step
       * \param gInActive gap distance of inactive links (return)
       * \param gdInActive gap velocities of inactive links (return)
       * \param IndInActive index for gInActive/gdInActive; incremented with size after storage (return and input)
       * \param gAct gap distance of active links (return)
       * \param IndActive index for gActive; incremented with size after storage (return and input)
      */
      virtual void LinearImpactEstimation(double t, fmatvec::Vec &gInActive_,fmatvec::Vec &gdInActive_,int *IndInActive_,fmatvec::Vec &gAct_,int *IndActive_) { }
      
      /**
       * \brief calculates the number of active and inactive unilateral constraints and increments sizeActive/sizeInActive
       */
      virtual void SizeLinearImpactEstimation(int *sizeInActive_, int *sizeActive_) { }
      virtual void updatecorr(int j) { corr.init(0); }
      virtual void updatecorrRef(const fmatvec::Vec &ref);
      virtual void calccorrSize(int j) { corrSize = 0; }
      virtual void setcorrInd(int corrInd_) { corrInd = corrInd_; }
      virtual void checkRoot() { }
      /***************************************************/

      /* GETTER / SETTER */
      const std::vector<fmatvec::Mat>& getW(int i=0) const { return W[i]; }
      const std::vector<fmatvec::Mat>& getV(int i=0) const { return V[i]; }
      const std::vector<fmatvec::Vec>& geth(int i=0) const { return h[i]; }

      void setx(const fmatvec::Vec &x_) { x = x_; }
      const fmatvec::Vec& getxd() const { return xd; }
      
      virtual void setsvInd(int svInd_) { svInd = svInd_; };
      int getsvSize() const { return svSize; }

      int getLinkStatusSize() const { return LinkStatusSize; }

      int getLinkStatusRegSize() const { return LinkStatusRegSize; }

      const fmatvec::Vec& getla() const { return la; }
      const fmatvec::Vec& getLa() const { return La; }
      fmatvec::Vec& getla() { return la; }
      fmatvec::Vec& getLa() { return La; }

      int getlaInd() const { return laInd; } 
      int getlaSize() const { return laSize; } 
      int getbSize() const { return bSize; }

//      const fmatvec::Vec& getg() const { return g; }
//      const fmatvec::Vec& getgd() const { return gd; }
//      const fmatvec::Vec& getwb() const { return wb; }
//      fmatvec::Vec& getg() { return g; }
//      fmatvec::Vec& getgd() { return gd; }
//      fmatvec::Vec& getwb() { return wb; }
      const fmatvec::Vec& evalg();
      const fmatvec::Vec& evalgd();
      const fmatvec::Vec& evalwb();

      int getgdInd() const { return gdInd; } 
      int getgSize() const { return gSize; } 
      int getgdSize() const { return gdSize; } 

//      const fmatvec::Index& getgIndex() const { return Ig; }
//      const fmatvec::Index& getlaIndex() const { return Ila; }
      
      int getrFactorSize() const { return rFactorSize; } 
      
      const fmatvec::VecInt& getrFactorUnsure() const { return rFactorUnsure; }

      void resetUpToDate() { updrrel = true; updvrel = true; updla = true; }

      virtual void updateGeneralizedPositions() { }
      virtual void updateGeneralizedVelocities() { }
      virtual void updateGeneralizedForces() { }

      const fmatvec::VecV& evalGeneralizedRelativePosition() { if(updrrel) updateGeneralizedPositions(); return rrel; }
      const fmatvec::VecV& evalGeneralizedRelativeVelocity() { if(updvrel) updateGeneralizedVelocities(); return vrel; }
      const fmatvec::VecV& evalGeneralizedForce() { if(updla) updateGeneralizedForces(); return lambda; }

      fmatvec::VecV& getGeneralizedForce(bool check=true) {  assert((not check) or (not updla)); return lambda; }

      /**
       * \brief saves contact forces for use as starting value in next time step
       */
      void savela(double dt=1.0);

      /**
       * \brief saves contact impulses for use as starting value in next time step
       */
      void saveLa(double dt=1.0);

      /**
       * \brief load contact forces for use as starting value
       */
      void initla(double dt=1.0);

      /**
       * \brief load contact impulses for use as starting value
       */
      void initLa(double dt=1.0);

      /**
       * \brief decrease rfactor if mass action matrix is not diagonal dominant (cf. Foerg: Dissertation, page 80 et seq.) 
       */
      void decreaserFactors();

      
      int getcorrSize() const { return corrSize; } 

    protected:
      /** 
       * \brief order one parameters
       */
      fmatvec::Vec x;

      /** 
       * \brief differentiated order one parameters 
       */
      fmatvec::Vec xd;

      /**
       * \brief order one initial value
       */
      fmatvec::Vec x0;

      /**
       * \brief size  and local index of order one parameters
       */
      int xSize, xInd;

      /**
       * \brief stop vector for event driven integration (root function)
       */
      fmatvec::Vec sv;

      /**
       * \brief evaluation of roots of stop vector with a boolean vector
       */
      fmatvec::VecInt jsv;

      /**
       * \brief size and local index of stop vector
       */
      int svSize, svInd;
             
      /**
       * for set valued links with piecewise link equation (e.g. unilateral contacts or coulomb friction)
       * \brief status of link (default 0) describing which piece of the equation is valid (e.g. stick or slip)
       */
      fmatvec::VecInt LinkStatus;

      /**
       * \brief size and local index of link status vector (set-valued)
       */
       int LinkStatusSize, LinkStatusInd;
    
       /**
       * for single valued links
       * \brief status of link
       */
      fmatvec::VecInt LinkStatusReg;

      /**
       * \brief size and local index of single-valued link status vector
       */
       int LinkStatusRegSize, LinkStatusRegInd; 

      /**
       * \brief relative distance, relative velocity, contact force parameters
       */
      fmatvec::Vec g, gd, la, La;
      
      /*!
       * \brief contact forces of smooth contact laws
       */
      fmatvec::Vec laS;

      /**
       * \brief size and local index of relative distances
       */
      int gSize, gInd;

      /**
       * \brief size and local index of relative velocities
       */
      int gdSize, gdInd;

      /**
       * \brief size and local index of contact force parameters
       */
      int laSize, laInd;

      /**
       * \brief size and local index of contact force parameters
       */
      int bSize, bInd;

      /**
       * \brief local index of relative distances and contact force parameters
       */
      fmatvec::Index Ig, Ila;
      
      /**
       * \brief tolerance for relative velocity, relative acceleration, force and impact  
       */
      double gTol, gdTol, gddTol, laTol, LaTol;
      
      /**
       * \brief attribute to save contact force parameter of previous time step
       */
      fmatvec::Vec la0, La0;

      /**
       * \brief vector of rfactors for relaxation of contact equations
       */
      fmatvec::Vec rFactor;

      /**
       * \brief boolean vector defining if rfactor belongs to not diagonal dominant mass action matrix (cf. Foerg Dissertation, page 80 et seq.)
       */
      fmatvec::VecInt rFactorUnsure;

      /**
       * \brief size and local index of rfactors
       */
      int rFactorSize, rFactorInd;
      
      /**
       * \brief maximum r-factor
       */
      double rMax;

      /**
       * residuum of nonlinear contact equations
       */
      fmatvec::Vec res;

      /** 
       * \brief force direction matrix for nonsmooth right hand side
       */
      std::vector<fmatvec::Mat> W[2];

      /**
       * \brief reduced force direction matrix for nonsmooth right hand side
       */
      std::vector<fmatvec::Mat> V[2];
      
      /**
       * \brief smooth complete and link right hand side
       */
      std::vector<fmatvec::Vec> h[2];
      
      /**
       * \brief smooth Jacobians for implicit integration
       */
      std::vector<fmatvec::Mat> dhdq;
      std::vector<fmatvec::Mat> dhdu;
      std::vector<fmatvec::Vec> dhdt;
      
      /**
       * \brief nonsmooth right hand side
       */
      std::vector<fmatvec::Vec> r[2];
      
      /**
       * \brief TODO
       */
      fmatvec::Vec wb;

      /**
       * \brief TODO
       */
      fmatvec::Mat b;

      int corrSize, corrInd;
      fmatvec::Vec corr;

      fmatvec::VecV rrel, vrel, lambda;

      bool updrrel, updvrel, updla;
  };
}

#endif /* _LINK_H_ */
