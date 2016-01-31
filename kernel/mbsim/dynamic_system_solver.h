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
 *          rzander@users.berlios.de
 */

#ifndef _DYNAMIC_SYSTEM_SOLVER_H_
#define _DYNAMIC_SYSTEM_SOLVER_H_

#include "mbsim/group.h"
#include "fmatvec/sparse_matrix.h"

namespace MBSim {

  class Graph;

  /**
   * \brief solver interface for modelling and simulation of dynamic systems
   * \author Martin Foerg
   * \date 2009-03-31 some comments (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-27 implicit integration (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   * \date 2009-08-07 preintegration (Thorsten Schindler)
   * \date 2009-08-21 reorganize hierarchy (Thorsten Schindler)
   * \date 2009-12-14 revised inverse kinetics (Martin Foerg)
   * \date 2010-07-06 modifications for timestepper ssc - e.g LinkStatus (Robert Huber)
   * \date 2012-05-08 modifications for AutoTimeSteppingSSCIntegrator (Jan Clauberg)
   * \date 2012-05-08 dhdq and dhdu with lower and upper bound (Jan Clauberg)
   * \date 2014-09-16 contact forces are calculated on acceleration level (Thorsten Schindler)
   *
   * \todo projectGeneralizedPositions seems to be buggy with at least TimeSteppingIntegrator (see SliderCrank)
   * \todo RootFinding seems to be buggy (see EdgeMill)
   */
  class DynamicSystemSolver : public Group {
    public:

      /**
       * \brief solver for contact equations
       */
      enum Solver { FixedPointTotal, FixedPointSingle, GaussSeidel, LinearEquations, RootFinding };

      /**
       * \brief relaxation strategies in solution of contact equations
       */
      enum Strategy { global, local };

      /**
       * \brief linear algebra for Newton scheme in solution of contact equations
       */
      enum LinAlg { LUDecomposition, LevenbergMarquardt, PseudoInverse };

      /**
       * \brief constructor
       * \param name of dynamic system
       */
      DynamicSystemSolver(const std::string &name="");

      /**
       * \brief destructor
       */
      virtual ~DynamicSystemSolver();

      /** \brief Initialize the system.
       *
       * This function calls init(InitStage stage) with all stages
       * defined in the enumeration InitStage, in the order in which
       * they are defined in this enumeration.
       *
       * The init(InitStage stage) functions of all classes MUST call
       * the init(InitStage stage) functions of all objects
       * for which this class holds as a container. This call is
       * always done at the end of the function independent of the
       * stage.
       *
       * If the init(InitStage stage) function implements a given
       * stage, the init(InitStage stage) function of the base class
       * should be call or not as expected by the programer for this
       * stage. For all other (not implemented) stages the function
       * of the base call must always be called.
       *
       * Example for a proper init(InitStage stage) function:
       * \code
       * void MyClass::init(InitStage stage) {
       *   if(stage==preInit) {
       *     // do something
       *     MyBaseClass::init(stage); // base must be called
       *     // do something
       *   }
       *   else if(stage==plot) {
       *     // do something
       *     // base need not be called
       *   }
       *   else
       *     MyBaseClass::init(stage);
       *
       *   for(int i=0; i<container.size(); i++)
       *     container[i]->init(stage);
       * }
       * \endcode
       */
      void initialize();

      /*
       * If true (the default) then
       * the simulation output files (h5 files) are deleted/truncated/regenerated.
       * If false, then these files left are untouched from a previous run.
       * This is useful to regenerate the
       * e.g. OpenMBV XML files without doing a new integration.
       */
      void setTruncateSimulationFiles(bool trunc) { truncateSimulationFiles=trunc; }

      /* INHERITED INTERFACE OF GROUP */
      void init(InitStage stage);
      using Group::plot;
      /***************************************************/

      /* INHERITED INTERFACE OF DYNAMICSYSTEM */
      virtual int solveConstraintsFixpointSingle(double t);
      virtual int solveImpactsFixpointSingle(double t, double dt = 0);
      virtual int solveConstraintsGaussSeidel(double t);
      virtual int solveImpactsGaussSeidel(double t, double dt = 0);
      virtual int solveConstraintsRootFinding(double t);
      virtual int solveImpactsRootFinding(double t, double dt = 0);
      virtual void checkConstraintsForTermination(double t);
      virtual void checkImpactsForTermination(double t, double dt = 0);
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateT(double t);
      virtual void updateh(double t, int i=0);
      virtual void updateM(double t, int i=0);
      virtual void updateLLM(double t, int i=0);
      /***************************************************/

      /* INHERITED INTERFACE OF LINKINTERFACE */

      /*!
       * for links holding non-smooth contributions \f$\dd\vLambda\f$, the respective forces are projected
       * into the minimal coordinate representation of the associated Body%s using the
       * JACOBIAN matrices \f$\vJ\f$.
       * \f[ \vr = \vJ\dd\vLambda \f]
       * The JACOBIAN is provided by the connected Frame%s (which might be
       * user-defined for Joint%s or internally defined for Contact%s).
       * \brief update smooth link force law
       * \param simulation time
       */
      virtual void updater(double t, int j=0);
      virtual void updaterdt(double t, int j=0);
      virtual void updatewb(double t);
      virtual void updateg(double t);
      virtual void updategd(double t);
      virtual void updateW(double t, int j=0);
      virtual void updateV(double t, int j=0);
      virtual void updateb(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      /** DEPRECATED */
      virtual std::string getType() const { return "DynamicSystemSolver"; }
      virtual void plot(const fmatvec::Vec& z, double t, double dt=1); // TODO completely rearrange
      virtual void plot2(const fmatvec::Vec& z, double t, double dt=1); // TODO completely rearrange

      virtual void closePlot();
      /***************************************************/

      /* INHERITED INTERFACE FOR DERIVED CLASS */
      /**
       * \brief solves prox-functions for contact forces using sparsity structure
       * \return iterations of solver
       */
      virtual int solveConstraints(double t);

      /**
       * \brief solves prox-functions for impacts on velocity level using sparsity structure
       * \param time step-size
       * \return iterations of solver
       */
      virtual int solveImpacts(double t, double dt = 0);
      /***************************************************/

      /* GETTER / SETTER */

      void setConstraintSolver(Solver solver_) { contactSolver = solver_; }
      void setImpactSolver(Solver solver_) { impactSolver = solver_; }
      const Solver& getConstraintSolver() { return contactSolver; }
      const Solver& getImpactSolver() { return impactSolver; }
      void setTermination(bool term_) { term = term_; }
      void setStrategy(Strategy strategy_) { strategy = strategy_; }
      void setMaxIter(int iter) { maxIter = iter; }
      void setHighIter(int iter) { highIter = iter; }
      void setNumJacProj(bool numJac_) { numJac = numJac_; }
      void setMaxDampingSteps(int maxDSteps) { maxDampingSteps = maxDSteps; }
      void setLevenbergMarquardtParam(double lmParm_) { lmParm = lmParm_; }
      void setLinAlg(LinAlg linAlg_) { linAlg = linAlg_; }

      void setUseOldla(bool flag) { useOldla = flag; }
      void setDecreaseLevels(const fmatvec::VecInt &decreaseLevels_) { decreaseLevels = decreaseLevels_; }
      void setCheckTermLevels(const fmatvec::VecInt &checkTermLevels_) { checkTermLevels = checkTermLevels_; }
      void setCheckGSize(bool checkGSize_) { checkGSize = checkGSize_; }
      void setLimitGSize(int limitGSize_) { limitGSize = limitGSize_; checkGSize = false; }

      const fmatvec::SqrMat& getG(bool check=true) const { assert((not check) or (not updG)); return G; }
      const fmatvec::SparseMat& getGs(bool check=true) const { assert((not check) or (not updG)); return Gs; }
      const fmatvec::Vec& getb(bool check=true) const { assert((not check) or (not updb)); return b; }
      const fmatvec::SqrMat& getJprox() const { return Jprox; }
      fmatvec::SqrMat& getG(bool check=true) { assert((not check) or (not updG)); return G; }
      fmatvec::SparseMat& getGs(bool check=true) { assert((not check) or (not updG)); return Gs; }
      fmatvec::Vec& getb(bool check=true) { assert((not check) or (not updb)); return b; }
      fmatvec::SqrMat& getJprox() { return Jprox; }

      const fmatvec::SqrMat& getG(double t);
      const fmatvec::SparseMat& getGs(double t);
      const fmatvec::Vec& getb(double t);

      const fmatvec::Mat& getWParent(int i=0) const { return WParent[i]; }
      const fmatvec::Mat& getVParent(int i=0) const { return VParent[i]; }
      const fmatvec::Vec& getlaParent() const { return laParent; }
      const fmatvec::Vec& getLaParent() const { return LaParent; }
      const fmatvec::Vec& getgdParent() const { return gdParent; }
      const fmatvec::Vec& getresParent() const { return resParent; }
      const fmatvec::Vec& getrFactorParent() const { return rFactorParent; }

      DynamicSystemSolver* getDynamicSystemSolver() { return this; }
      bool getIntegratorExitRequest() { return integratorExitRequest; }
      int getMaxIter()  {return maxIter;}
      /***************************************************/

      /**
       * \brief compute initial condition for links for event driven integrator
       */
      void computeInitialCondition();

      /**
       * \param state
       * \param time
       * \param time step
       * \return velocity difference for current time
       */
      fmatvec::Vec deltau(const fmatvec::Vec &zParent, double t, double dt);

      /**
       * \return position difference for current time
       * \param state
       * \param time
       * \param time step
       */
      fmatvec::Vec deltaq(const fmatvec::Vec &zParent, double t, double dt);

      /**
       * \brief return x-state difference for current time
       * \param parent state
       * \param time
       * \param time step
       */
      fmatvec::Vec deltax(const fmatvec::Vec &zParent, double t, double dt);

      /**
       * \brief initialises state variables
       */
      virtual void initz(fmatvec::Vec& z0);

      /**
       * \function pointer for election of prox-solver for contact equations
       * \return iterations of solver
       */
      int (DynamicSystemSolver::*solveConstraints_)(double t);

      /**
       * \brief function pointer for election of prox-solver for impact equations on velocity level
       * \param time step
       * \return iterations of solver
       */
      int (DynamicSystemSolver::*solveImpacts_)(double t, double dt);

      /**
       * \brief solution of contact equations with Cholesky decomposition
       * \return iterations of solver
       * \todo put in dynamic system? TODO
       */
      int solveConstraintsLinearEquations(double t);

      /**
       * \brief solution of contact equations with Cholesky decomposition on velocity level
       * \param time step-size, if 0 non-impulsive contributions vanish
       * \return iterations of solver
       * \todo put in dynamic system? TODO
       */
      int solveImpactsLinearEquations(double t, double dt = 0);

      /**
       * \brief updates mass action matrix
       * \param time
       */
      virtual void updateG(double t, int i=0);

      /**
       * \brief decrease relaxation factors if mass action matrix is not diagonal dominant
       */
      void decreaserFactors();

      /**
       * \brief update of dynamic system for time-stepping integrator
       * \param state
       * \param time
       * \param options  = 0 (default) nothing
       *                 = 1 force updates as if active constraints has changed
       */
      void update(const fmatvec::Vec &z, double t, int options=0);

      /**
       * \brief update for event driven integrator for event
       * \param state (return)
       * \param boolean evaluation of stop vector
       * \param time
       */
      virtual void shift(fmatvec::Vec& z, const fmatvec::VecInt& jsv, double t);

      /**
       * \brief update for event driven integrator during smooth phase
       * \param state
       * \param differentiated state (return)
       * \param time
       */
      virtual void zdot(const fmatvec::Vec& z, fmatvec::Vec& zd, double t);

      /**
       * \brief standard invocation of smooth update for event driven integration
       * \param state
       * \param time
       */
      virtual fmatvec::Vec zdot(const fmatvec::Vec &zParent, double t);

      /**
       * \brief evaluation of stop vector
       * \param state
       * \param TODO
       * \param time
       */
      virtual void getsv(const fmatvec::Vec& z, fmatvec::Vec& svExt, double t);

      /** brief collect status of all set-valued links
       * \param result vector
       * \param time
       */
      void getLinkStatus(fmatvec::VecInt &LinkStatusExt, double t);

      /** brief collect status of all single-valued links
       * \param result vector
       * \param time
       */

      void getLinkStatusReg(fmatvec::VecInt &LinkStatusRegExt, double t);
      /**
       * \brief drift projection for positions
       * \param time
       */
      void projectGeneralizedPositions(double t, int mode, bool fullUpdate=false);

      /**
       * \brief drift projection for positions
       * \param time
       */
      void projectGeneralizedVelocities(double t, int mode);

      /**
       * \brief save contact forces for use as starting value in next time step
       * \todo put in dynamic system TODO
       */
      void savela(double dt=1.0);

      /**
       * \brief load contact forces for use as starting value
       * \todo put in dynamic system TODO
       */
      void initla(double dt=1.0);

      /**
       * \brief save contact impulses for use as starting value in next time step
       * \todo put in dynamic system TODO
       */
      void saveLa(double dt=1.0);

      /**
       * \brief load contact impulses for use as starting value
       * \todo put in dynamic system TODO
       */
      void initLa(double dt=1.0);

      /**
       * \brief compute kinetic energy of entire dynamic system
       */
      double computeKineticEnergy() { return 0.5*u.T()*M[0]*u; }

      /**
       * \brief compute potential energy of entire dynamic system
       * \tofo change? TODO
       */
      double computePotentialEnergy();

      /**
       * \param element to add
       * \todo necessary? TODO
       */
      void addElement(Element *element_);

      /**
       * \param name of the element
       * \return the pointer to an element
       * \todo not activated TODO
       */
      Element* getElement(const std::string &name);

      /**
       * \return information for solver including strategy and linear algebra
       */
      std::string getSolverInfo();

      /**
       * \param specify whether time integration should be stopped in case of no convergence of constraint-problem
       * \param specify whether contact informations should be dropped to file
       */
      void setStopIfNoConvergence(bool flag, bool dropInfo = false) { stopIfNoConvergence = flag; dropContactInfo=dropInfo; }

      /**
       * writes a file with relevant matrices for debugging
       */
      void dropContactMatrices(double t);

      /**
       * \brief handler for user interrupt signal
       */
      static void sigInterruptHandler(int);

      /**
       * \brief handler for abort signals
       */
      static void sigAbortHandler(int);

      static void sigSegfaultHandler(int);

      // TODO just for testing
      void setPartialEventDrivenSolver(bool peds_) { peds = peds_; }

      /**
       * \brief writes state to a file
       * \param name of the file
       * \param h5, else ascii
       */
      void writez(std::string fileName, bool formatH5=true);

      /**
       * \brief reads state from a file
       * \param name of the file
       */
      void readz0(std::string fileName);

      virtual void initializeUsingXML(xercesc::DOMElement *element);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element);

      static DynamicSystemSolver* readXMLFile(const std::string &filename);
      void writeXMLFile(const std::string &name);
      void writeXMLFile() { writeXMLFile(getName()); }


      /**
       * \brief set tolerance for projection of generalized position
       * \param tolerance
       */
      void setProjectionTolerance(double tol) { tolProj = tol; }

     /**
       * \param tolerance for relative velocity
       */
      void setgTol(double tol) {gTol = tol; Group::setgTol(tol);}

      /**
       * \param tolerance for relative velocity
       */
      void setgdTol(double tol) {gdTol = tol; Group::setgdTol(tol);}

      /**
       * \param tolerance for relative acceleration
       */
      void setgddTol(double tol) {gddTol = tol; Group::setgddTol(tol);}

      /**
       * \param tolerance for contact force
       */
      void setlaTol(double tol) {laTol = tol; Group::setlaTol(tol);}

      /**
       * \param tolerance for impact
       */
      void setLaTol(double tol) {LaTol = tol; Group::setLaTol(tol);}

      /**
       * \brief references to external state
       * \param external state
       */
      void updatezRef(const fmatvec::Vec &ext);

      /**
       * \brief set the number of plot-routine-calls after which all hdf5-files will be flushed
       * \param flag
       */
      void setFlushEvery(unsigned int every) {flushEvery = every;}

      void setAlwaysConsiderContact(bool alwaysConsiderContact_) {alwaysConsiderContact = alwaysConsiderContact_;}

      void setInverseKinetics(bool inverseKinetics_) {inverseKinetics = inverseKinetics_;}

      void setInitialProjection(bool initialProjection_) {initialProjection = initialProjection_;}

      void setUseConstraintSolverForPlot(bool useConstraintSolverForPlot_) {useConstraintSolverForPlot = useConstraintSolverForPlot_;}

      fmatvec::Mat dhdq(double t, int lb=0, int ub=0);
      fmatvec::Mat dhdu(double t, int lb=0, int ub=0);
      fmatvec::Mat dhdx(double t);
      fmatvec::Vec dhdt(double t);

      void setRootID(int ID) {rootID = ID;}
      int getRootID() const {return rootID;}

      void resetUpToDate();

      bool updateT() { return updT; }
      bool updateM(int j) { return updM[j]; }
      bool updateLLM(int j) { return updLLM[j]; }
      bool updateh(int j) { return updh[j]; }
      bool updater(int j) { return updr[j]; }
      bool updaterdt(int j) { return updrdt[j]; }
      bool updateW(int j) { return updW[j]; }
      bool updateV(int j) { return updV[j]; }
      bool updatewb() { return updwb; }
      bool updateg() { return updg; }
      bool updategd() { return updgd; }

      void resize_(double t);

      /**
       * \brief references to relative distances of dynamic system parent
       * \param vector to be referenced
       */
      void updategRef(const fmatvec::Vec &ref) { Group::updategRef(ref); updg = true; }

      /**
       * \brief references to relative velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updategdRef(const fmatvec::Vec &ref) { Group::updategdRef(ref); updgd = true; }

      /**
       * \brief references to TODO of dynamic system parent
       * \param vector to be referenced
       */      
      void updatewbRef(const fmatvec::Vec &ref) { Group::updatewbRef(ref); updwb = true; }

      /**
       * \brief references to contact force direction matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateWRef(const fmatvec::Mat &ref, int i=0) { Group::updateWRef(ref,i); updW[i] = true; }

      /**
       * \brief references to condensed contact force direction matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateVRef(const fmatvec::Mat &ref, int i=0) { Group::updateVRef(ref,i); updV[i] = true; }

    protected:
      /**
       * \brief mass matrix
       */
      fmatvec::SymMat MParent[2];

      /**
       * \brief Cholesky decomposition of mass matrix
       */
      fmatvec::SymMat LLMParent[2];

      /**
       * \brief matrix of linear relation between differentiated positions and velocities
       */
      fmatvec::Mat TParent;

      /**
       * \brief contact force directions
       */
      fmatvec::Mat WParent[2];

      /**
       * \brief condensed contact force directions
       */
      fmatvec::Mat VParent[2];

      /**
       * \brief TODO
       */
      fmatvec::Vec wbParent;

      /**
       * \brief contact force parameters
       */
      fmatvec::Vec laParent, LaParent;

      /**
       * \brief relaxation parameters for contact equations
       */
      fmatvec::Vec rFactorParent;

      /**
       * \brief TODO
       */
      fmatvec::Vec sParent;

      /**
       * \brief residuum of contact equations
       */
      fmatvec::Vec resParent;

      /**
       * \brief relative distances
       */
      fmatvec::Vec gParent;

      /**
       * \brief relative velocities
       */
      fmatvec::Vec gdParent;

      /**
       * \brief state
       */
      fmatvec::Vec zParent;

      /**
       * \brief differentiated state
       */
      fmatvec::Vec zdParent;

      fmatvec::Vec udParent1;

      /**
       * \brief smooth, smooth with respect to objects, smooth with respect to links right hand side
       */
      fmatvec::Vec hParent[2];

      /**
       * \brief nonsmooth right hand side
       */
      fmatvec::Vec rParent[2], rdtParent[2];

      /**
       * \brief right hand side of order one parameters
       */
      fmatvec::Vec fParent;

      /**
       * \brief stopvector (rootfunctions for event driven integration
       */
      fmatvec::Vec svParent;

      /**
       * \brief boolean evaluation of stopvector
       */
      fmatvec::VecInt jsvParent;

      /**
       * \brief status vector of set valued links with piecewise link equation (which piece is valid)
       */
      fmatvec::VecInt LinkStatusParent;

      /**
       * \brief status vector of single valued links
       */

      fmatvec::VecInt LinkStatusRegParent;

      /**
       * \brief sparse mass action matrix
       */
      fmatvec::SparseMat Gs;

      /**
       * \brief JACOBIAN of contact equations for Newton scheme
       */
      fmatvec::SqrMat Jprox;

      /**
       * \brief mass action matrix
       */
      fmatvec::SqrMat G;

      /**
       * \brief TODO
       */
      fmatvec::Vec b;

      /**
       * \brief boolean to check for termination of contact equations solution
       */
      bool term;

      /**
       * \brief maximum number of contact iterations, high number of contact iterations for warnings, maximum number of damping steps for Newton scheme
       */
      int maxIter, highIter, maxDampingSteps;

      /**
       * \brief Levenberg-Marquard parameter
       */
      double lmParm;

      /**
       * \brief solver for contact equations and impact equations
       */
      Solver contactSolver, impactSolver;

      /**
       * \brief relaxarion strategy for solution of fixed-point scheme
       */
      Strategy strategy;

      /**
       * \brief linear system solver used for Newton scheme in contact equations
       */
      LinAlg linAlg;

      /**
       * \brief flag if the contact equations should be stopped if there is no convergence
       */
      bool stopIfNoConvergence;

      /**
       * \brief flag if contact matrices for debugging should be dropped in no-convergence case
       */
      bool dropContactInfo;

      /**
       * \brief flag if contac force parameter of last time step should be used
       */
      bool useOldla;

      /**
       * \brief flag if Jacobian for Newton scheme should be calculated numerically
       */
      bool numJac;

      /**
       * \brief decreasing relaxation factors is done in levels containing the number of contact iterations as condition
       */
      fmatvec::VecInt decreaseLevels;

      /**
       * \brief TODO
       */
      fmatvec::VecInt checkTermLevels;

      /**
       * \brief boolean if force action matrix should be resized in each step
       */
      bool checkGSize;

      /**
       * \brief TODO
       */
      int limitGSize;

      /**
       * \brief level for warning output (0-2)
       */
      int warnLevel;

      /**
       * \brief TODO, flag for occuring impact and sticking in event driven solver
       */
      bool peds;

      /**
       * \brief TODO, additional stop in event driven solver for drift correction
       */
      unsigned int driftCount;

      /**
       * \brief flushes all hdf5-files every x-times the plot-routine is called
       * TODO
       */
      unsigned int flushEvery;

      /**
       * \brief counts plot-calls until files to be flushed
       * TODO
       */
      unsigned int flushCount;

      /**
       * \brief Tolerance for projection of generalized position.
       */
      double tolProj;

      /**
       * \brief references to differentiated external state
       * \param differentiated external state
       */
      void updatezdRef(const fmatvec::Vec &ext);

      /**
       * \brief update relaxation factors for contact equations
       * \todo global not available because of unsymmetric mass action matrix TODO
       */
      virtual void updaterFactors(double t);

      /**
       * \brief compute inverse kinetics constraint forces
       * \param current time
       */
      void computeConstraintForces(double t);

      /**
       * \brief
       */
      fmatvec::Vec laInverseKineticsParent;
      fmatvec::Mat bInverseKineticsParent;

      /**
       * \brief
       */
      fmatvec::Mat WInverseKineticsParent[2];

      bool alwaysConsiderContact;
      bool inverseKinetics;
      bool initialProjection;
      bool useConstraintSolverForPlot;

      fmatvec::Vec corrParent;

      int rootID;

      double gTol, gdTol, gddTol, laTol, LaTol;

      bool updT, updh[2], updr[2], updrdt[2], updM[2], updLLM[2], updW[2], updV[2], updwb, updg, updgd, updG, updb;

    private:
      /**
       * \brief set plot feature default values
       */
      void constructor();

      /**
       * \brief boolean signal evaluation for end integration set by program
       */
      bool integratorExitRequest;

      /**
       * \brief boolean signal evaluation for end integration set by user
       */
      static bool exitRequest;

      /**
       * \brief is a state read from a file
       */
      bool READZ0;

      /**
       * \brief adds list of objects to tree
       * \param tree where to add objects
       * \param current node in the tree
       * \param matrix of dependencies
       * \param current entry of matrix of dependencies
       * \param list of objects
       */
      void addToGraph(Graph* graph, fmatvec::SqrMat &A, int i, std::vector<Element*> &objList);

      bool truncateSimulationFiles;

      // Holds the dynamic systems before the "reorganize hierarchy" takes place.
      // This is required since all elements of all other containers from DynamicSystem are readded to DynamicSystemSolver,
      // except this container (which is a "pure" container = no calculation is done in DynamicSystem)
      // However, we must hold this container until the dtor of DynamicSystemSolver is called to avoid the deallocation of other
      // elements hold by DynamicSystem elements (especially (currently only) hdf5File)
      std::vector<DynamicSystem*> dynamicsystemPreReorganize;
  };

}

#endif /* _DYNAMIC_SYSTEM_SOLVER_H_ */

