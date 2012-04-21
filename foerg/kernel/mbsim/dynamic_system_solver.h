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
 *          rzander@users.berlios.de
 */

#ifndef _DYNAMIC_SYSTEM_SOLVER_H_
#define _DYNAMIC_SYSTEM_SOLVER_H_

#include "mbsim/group.h"
#include "sparse_matrix.h"

namespace MBSim {

  class Frame;
  class Node;
  class Graph;
  class ExtraDynamicInterface;
  class Integrator;

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
   * \brief solver interface for modelling and simulation of dynamic systeme
   * \author Martin Foerg
   * \date 2009-03-31 some comments (Thorsten Schindler)
   * \date 2009-07-16 splitted link / object right hand side (Thorsten Schindler)
   * \date 2009-07-27 implicit integration (Thorsten Schindler)
   * \date 2009-07-28 splitted interfaces (Thorsten Schindler)
   * \date 2009-08-07 preintegration (Thorsten Schindler)
   * \date 2009-08-21 reorganize hierarchy (Thorsten Schindler)
   * \date 2009-12-14 revised inverse kinetics (Martin Foerg)
   * \date 2010-07-06 modifications for timestepper ssc - e.g LinkStatus (Robert Huber)
   * \todo projectGeneralizedPositions seems to be buggy with at least TimeSteppingIntegrator (see SliderCrank)
   * \todo RootFinding seems to be buggy (see EdgeMill)
   */
  class DynamicSystemSolver : public Group {
    public:
      /** 
       * \brief constructor
       */
      DynamicSystemSolver();

      /**
       * \brief constructor
       * \param name of dynamic system
       */
      DynamicSystemSolver(const std::string &projectName);

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
       * This is usefull to regenerate the
       * e.g. OpenMBV XML files whithout doing a new integration.
       */
      void setTruncateSimulationFiles(bool trunc) { truncateSimulationFiles=trunc; }

      /* INHERITED INTERFACE OF GROUP */
      void init(InitStage stage);
      using Group::plot;
      /***************************************************/

      /* INHERITED INTERFACE OF DYNAMICSYSTEM */
      virtual int solveConstraintsFixpointSingle(); 
      virtual int solveImpactsFixpointSingle(double dt = 0); 
      virtual int solveConstraintsGaussSeidel();
      virtual int solveImpactsGaussSeidel(double dt = 0);
      virtual int solveConstraintsRootFinding(); 
      virtual int solveImpactsRootFinding(double dt = 0); 
      virtual void checkConstraintsForTermination(); 
      virtual void checkImpactsForTermination(double dt = 0); 
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateh(double t, int i=0);
      virtual void updateh0Fromh1(double t);
      virtual void updateW0FromW1(double t);
      virtual void updateV0FromV1(double t);
      virtual void updateM(double t, int i=0);
      virtual void updateStateDependentVariables(double t); // this function is called once every time step by every integrator
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
      virtual void updatewb(double t, int j=0);
      virtual void updateW(double t, int j=0);
      virtual void updateV(double t, int j=0);
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
       * \brief solves prox-functions on acceleration level using sparsity structure but not decoupled
       * \return
       */
      virtual int solveConstraints(); 

      /**
       * \brief solves prox-functions on velocity level using sparsity structure but not decoupled
       */
      virtual int solveImpacts(double dt = 0); 
      /***************************************************/

      /* GETTER / SETTER */

      void setConstraintSolver(Solver solver_) { contactSolver = solver_; }                         
      void setImpactSolver(Solver solver_) { impactSolver = solver_; }                         
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
      void setDecreaseLevels(const fmatvec::Vector<fmatvec::General, int> &decreaseLevels_) { decreaseLevels = decreaseLevels_; }
      void setCheckTermLevels(const fmatvec::Vector<fmatvec::General, int> &checkTermLevels_) { checkTermLevels = checkTermLevels_; }
      void setCheckGSize(bool checkGSize_) { checkGSize = checkGSize_; }
      void setLimitGSize(int limitGSize_) { limitGSize = limitGSize_; checkGSize = false; }


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
      void initz(fmatvec::Vec& z0);

      /**
       * \return successful flag for function pointer for election of prox-solver on acceleration level
       */
      int (DynamicSystemSolver::*solveConstraints_)();

      /**
       * \return successful flag for function pointer for election of prox-solver on velocity level
       * \param time step
       */
      int (DynamicSystemSolver::*solveImpacts_)(double dt);

      /**
       * \return successful solution of contact equations with Cholesky decomposition on acceleration level
       * \todo put in dynamic system? TODO
       */
      int solveConstraintsLinearEquations(); 

      /**
       * \return successful solution of contact equations with Cholesky decomposition on velocity level
       * \todo put in dynamic system? TODO
       */
      int solveImpactsLinearEquations(double dt = 0); 

      /**
       * \brief updates mass action matrix
       * \param time
       */
      void updateG(double t, int i=0);

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
      virtual void shift(fmatvec::Vec& z, const fmatvec::Vector<fmatvec::General, int>& jsv, double t);

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

      /** brief collect status of all links
       * \param result vector
       * \param time
       */
      void getLinkStatus(fmatvec::Vector<fmatvec::General, int> &LinkStatusExt, double t);

      /**
       * \brief drift projection for positions
       * \param time
       */
      void projectGeneralizedPositions(double t, int mode);

      /**
       * \brief drift projection for positions
       * \param time
       */
      void projectGeneralizedVelocities(double t, int mode);

      /**
       * \brief save contact force parameter for use as starting value in next time step
       * \todo put in dynamic system TODO 
       */
      void savela(double dt=1.0);

      /**
       * \brief load contact force parameter for use as starting value
       * \todo put in dynamic system TODO 
       */
      void initla(double dt=1.0); 

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
      void dropContactMatrices();

      /**
       * \brief handler for user interrupt signal
       */
      static void sigInterruptHandler(int);

      /**
       * \brief handler for abort signals
       */
      static void sigAbortHandler(int);

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

      virtual void initializeUsingXML(TiXmlElement *element);

      /**
       * \brief decide, whether the model-hierarchy should be reorganized.
       * \param true for reorganisation, false otherwise
       */
      void setReorganizeHierarchy(bool flag) { reorganizeHierarchy = flag; }

      /**
       * \brief set tolerance for projection of generalized position
       * \param tolerance
       */
      void setProjectionTolerance(double tol) { tolProj = tol; }

      /**
       * \param decide, whether information should be printed on standard output.
       */
      void setInformationOutput(bool INFO_) { INFO = INFO_; }

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

       fmatvec::Mat dhdq(double t);
       fmatvec::Mat dhdu(double t);
       fmatvec::Mat dhdx(double t);
       fmatvec::Vec dhdt(double t);

       void setRootID(int ID) {rootID = ID;}
       int getRootID() const {return rootID;}

       fmatvec::SymMat& getM(int i=0) { return M[i]; };
       const fmatvec::SymMat& getM(int i=0) const { return M[i]; };
       fmatvec::SymMat& getLLM(int i=0) { return LLM[i]; };
       const fmatvec::SymMat& getLLM(int i=0) const { return LLM[i]; };
       fmatvec::Vec& geth(int j=0) { return h[j]; };
       const fmatvec::Vec& geth(int j=0) const { return h[j]; };
       fmatvec::Vec& getr(int j=0) { return r[j]; };
       const fmatvec::Vec& getr(int j=0) const { return r[j]; };
       fmatvec::Vec& getf() { return f; };
       const fmatvec::Vec& getf() const { return f; };
       fmatvec::Mat& getW(int i=0) { return W[i]; }
       const fmatvec::Mat& getW(int i=0) const { return W[i]; }
       fmatvec::Mat& getV(int i=0) { return V[i]; }
       const fmatvec::Mat& getV(int i=0) const { return V[i]; }
       fmatvec::Mat& getT() { return T; };
       const fmatvec::Mat& getT() const { return T; };
       fmatvec::Vec& getq() { return q; };
       const fmatvec::Vec& getq() const { return q; };
       fmatvec::Vec& getu() { return u; };
       const fmatvec::Vec& getu() const { return u; };
       fmatvec::Vec& getx() { return x; };
       const fmatvec::Vec& getx() const { return x; };
       fmatvec::Vec& getqd() { return qd; };
       const fmatvec::Vec& getqd() const { return qd; };
       fmatvec::Vec& getud(int i=0) { return ud[i]; };
       const fmatvec::Vec& getud(int i=0) const { return ud[i]; };
       fmatvec::Vec& getxd() { return xd; };
       const fmatvec::Vec& getxd() const { return xd; };
       fmatvec::Vec& getx0() { return x0; };
       const fmatvec::Vec& getx0() const { return x0; };
       fmatvec::Vec& getla() { return la; }
       const fmatvec::Vec& getla() const { return la; }
       fmatvec::Vec& getg() { return g; }
       const fmatvec::Vec& getg() const { return g; }
       fmatvec::Vec& getgd() { return gd; }
       const fmatvec::Vec& getgd() const { return gd; }
       fmatvec::Vec& getrFactor() { return rFactor; }
       const fmatvec::Vec& getrFactor() const { return rFactor; }
       fmatvec::Vec& getsv() { return sv; }
       const fmatvec::Vec& getsv() const { return sv; }
       fmatvec::Vector<fmatvec::General, int>& getjsv() { return jsv; }
       const fmatvec::Vector<fmatvec::General, int>& getjsv() const { return jsv; }
       fmatvec::Vec& getres() { return res; }
       const fmatvec::Vec& getres() const { return res; }
       fmatvec::Vector<fmatvec::General, int>& getLinkStatus() { return LinkStatus; }
       const fmatvec::Vector<fmatvec::General, int>& getLinkStatus() const { return LinkStatus; }
      fmatvec::Matrix<fmatvec::Sparse, double>& getGs() { return Gs; }
      const fmatvec::Matrix<fmatvec::Sparse, double>& getGs() const { return Gs; }
      fmatvec::SqrMat& getG() { return G; }
      const fmatvec::SqrMat& getG() const { return G; }
      fmatvec::Vec& getb() { return b; }
      const fmatvec::Vec& getb() const { return b; }
      fmatvec::Mat& getB() { return b; }
      const fmatvec::Mat& getB() const { return b; }
      fmatvec::SqrMat& getJprox() { return Jprox; }
      const fmatvec::SqrMat& getJprox() const { return Jprox; }
      fmatvec::Vec& getcorr() { return corr; }
      const fmatvec::Vec& getcorr() const { return corr; }
      fmatvec::Vec& getwb() { return wb; }
      const fmatvec::Vec& getwb() const { return wb; }

       //void setx(const fmatvec::Vec& x_) { x = x_; }
       //void setx0(const fmatvec::Vec &x0_) { x0 = x0_; }
       //void setx0(double x0_) { x0 = fmatvec::Vec(1,fmatvec::INIT,x0_); }

    protected:

      /**
       * \brief mass matrix
       */
      fmatvec::SymMat M[2];

      /** 
       * \brief Cholesky decomposition of mass matrix
       */
      fmatvec::SymMat LLM[2];

      /**
       * \brief smooth, smooth with respect to objects, smooth with respect to links, nonsmooth and order one right hand side
       */
      fmatvec::Vec h[2], r[2], f;

      /**
       * \brief 
       */
      fmatvec::Mat W[2], V[2];

      /** 
       * \brief linear relation matrix of position and velocity parameters
       */
      fmatvec::Mat T;

      /**
       * \brief positions, differentiated positions, initial positions
       */
      fmatvec::Vec q, qd, q0;

      /**
       * \brief velocities, differentiated velocities, initial velocities
       */
      fmatvec::Vec u, ud[2], u0;

      /**
       * \brief order one parameters, differentiated order one parameters, initial order one parameters
       */
      fmatvec::Vec x, xd, x0;

      /**
       * \brief contact force parameters
       */
      fmatvec::Vec la;

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
      fmatvec::Vector<fmatvec::General, int> jsv;

      /**
       * \brief status of set valued links 
       */
      fmatvec::Vector<fmatvec::General, int> LinkStatus;

      /**
       * \brief differentiated state
       */
      fmatvec::Vec zParent, zdParent;

      fmatvec::Vec udParent1;

      /**
       * \brief sparse mass action matrix
       */
      fmatvec::Matrix<fmatvec::Sparse, double> Gs;

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

      fmatvec::Mat B;

      fmatvec::Vec corr;

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
       * \brief solver for contact equations on acceleration and velocity level
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
      fmatvec::Vector<fmatvec::General, int> decreaseLevels;

      /**
       * \brief TODO
       */
      fmatvec::Vector<fmatvec::General, int> checkTermLevels;

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
       * \brief Flag for reorganisation of hierarchy. 
       * This flag will be removed in the future.
       */
      bool reorganizeHierarchy;

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
      void updaterFactors();

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

      int rootID;

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
       * \brief information on standard output
       */
      bool INFO;

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
      void addToGraph(Graph* graph, fmatvec::SqrMat &A, int i, std::vector<Object*> &objList);

      bool truncateSimulationFiles;

  };

}

#endif /* _DYNAMIC_SYSTEM_SOLVER_H_ */

