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
 *          rzander@users.berlios.de
 */

#ifndef _DYNAMIC_SYSTEM_SOLVER_H_
#define _DYNAMIC_SYSTEM_SOLVER_H_

#include "mbsim/group.h"
#include "sparse_matrix.h"
#include "H5Cpp.h"
#include <string>
#include "mbsimtinyxml/tinyxml-src/tinyxml.h"

namespace MBSim {

  class Frame;
  class Tree;
  class Node;
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
   * \todo split init process TODO
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

      /* INHERITED INTERFACE OF SUBSYSTEM */
      virtual int solveConstraintsFixpointSingle(); 
      virtual int solveImpactsFixpointSingle(double dt = 0); 
      virtual int solveConstraintsGaussSeidel();
      virtual int solveImpactsGaussSeidel(double dt = 0);
      virtual int solveConstraintsRootFinding(); 
      virtual int solveImpactsRootFinding(double dt = 0); 
      virtual void checkConstraintsForTermination(); 
      virtual void checkImpactsForTermination(); 
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateh(double t);
      virtual void updatedhdz(double t);
      virtual void updateM(double t);
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
      virtual void updater(double t);
      virtual void updatewb(double t);
      virtual void updateW(double t);
      virtual void updateV(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      /** DEPRECATED */
      virtual std::string getType() const { return "DynamicSystemSolver"; }
      virtual void plot(const fmatvec::Vec& z, double t, double dt=1); // TODO completely rearrange
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
      void setImpact(bool impact_) { impact = impact_; }
      void setSticking(bool sticking_) { sticking = sticking_; }

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
      void setDecreaseLevels(const fmatvec::Vector<int> &decreaseLevels_) { decreaseLevels = decreaseLevels_; }
      void setCheckTermLevels(const fmatvec::Vector<int> &checkTermLevels_) { checkTermLevels = checkTermLevels_; }
      void setCheckGSize(bool checkGSize_) { checkGSize = checkGSize_; }
      void setLimitGSize(int limitGSize_) { limitGSize = limitGSize_; checkGSize = false; }

      const fmatvec::Matrix<fmatvec::Sparse, double>& getGs() const { return Gs; }
      fmatvec::Matrix<fmatvec::Sparse, double>& getGs() { return Gs; }
      const fmatvec::SqrMat& getG() const { return G; }
      fmatvec::SqrMat& getG() { return G; }
      const fmatvec::Vec& getb() const { return b; }
      fmatvec::Vec& getb() { return b; }
      const fmatvec::SqrMat& getJprox() const { return Jprox; }
      fmatvec::SqrMat& getJprox() { return Jprox; }

      const fmatvec::Mat& getWParent() const { return WParent; }
      const fmatvec::Mat& getVParent() const { return VParent; }
      const fmatvec::Vec& getlaParent() const { return laParent; }
      const fmatvec::Vec& getgdParent() const { return gdParent; }
      const fmatvec::Vec& getresParent() const { return resParent; }
      const fmatvec::Vec& getrFactorParent() const { return rFactorParent; }

      DynamicSystemSolver* getDynamicSystemSolver() { return this; }
      bool getIntegratorExitRequest() { return integratorExitRequest; }
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
       * \brief TODO
       * \param TODO
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
       * TODO dynamic system?
       */
      int solveConstraintsLinearEquations(); 

      /**
       * \return successful solution of contact equations with Cholesky decomposition on velocity level
       * TODO dynamic system?
       */
      int solveImpactsLinearEquations(double dt = 0); 

      /**
       * \brief updates mass action matrix
       * \param time
       */
      void updateG(double t);

      /**
       * \brief decrease relaxation factors if mass action matrix is not diagonal dominant
       */
      void decreaserFactors();

      /**
       * \brief update of dynamic system for time-stepping integrator
       * \param state
       * \param time
       */
      void update(const fmatvec::Vec &z, double t);

      /**
       * \brief update for event driven integrator for event
       * \param state (return)
       * \param boolean evaluation of stop vector
       * \param time
       */
      virtual void shift(fmatvec::Vec& z, const fmatvec::Vector<int>& jsv, double t);

      /**
       * \brief update for event driven integrator during smooth phase
       * \param state
       * \param differentiated state (return)
       * \param time
       */
      void zdot(const fmatvec::Vec& z, fmatvec::Vec& zd, double t);

      /**
       * \brief evaluation of stop vector
       * \param state
       * \param TODO
       * \param time
       */
      void getsv(const fmatvec::Vec& z, fmatvec::Vec& svExt, double t);

      /**
       * \brief update for event driven integrator during smooth phase
       * \param state
       * \param time
       */
      fmatvec::Vec zdot(const fmatvec::Vec& z, double t) { return (this->*zdot_)(z,t); }

      /**
       * \brief drift projection for positions
       * \param time
       */
      void projectGeneralizedPositions(double t);

      /**
       * \brief drift projection for positions
       * \param time
       */
      void projectGeneralizedVelocities(double t);

      /**
       * save contact force parameter for use as starting value in next time step
       */
      void savela(); // TODO put in dynamic system 

      /**
       * load contact force parameter for use as starting value
       */
      void initla(); // TODO put in dynamic system

      /** 
       * \brief compute kinetic energy of entire dynamic system
       */
      double computeKineticEnergy() { return 0.5*trans(u)*M*u; }

      /** 
       * \brief compute potential energy of entire dynamic system TODO change
       */
      double computePotentialEnergy();

      /**
       * \param element to add
       * TODO necessary
       */
      void addElement(Element *element_);

      /**
       * \param name of the element
       * \return the pointer to an element
       * TODO not activated
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
       * \brief initialise data interface base
       */
      void initDataInterfaceBase();

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
       */
      void writez(std::string fileName);
      
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
       * \param decide, whether information should be printed on standard output.
       */
      void setInformationOutput(bool INFO_) { INFO = INFO_; }
      
      /**
       * \brief references to external state
       * \param external state
       */
      void updatezRef(const fmatvec::Vec &ext);


    protected:
      /**
       * \brief mass matrix
       */
      fmatvec::SymMat MParent;
      
      /**
       * \brief Cholesky decomposition of mass matrix
       */
      fmatvec::SymMat LLMParent;

      /**
       * \brief matrix of linear relation between differentiated positions and velocities
       */
      fmatvec::Mat TParent;

      /**
       * \brief contact force directions
       */
      fmatvec::Mat WParent;

      /**
       * \brief condensed contact force directions
       */
      fmatvec::Mat VParent;

      /**
       * \brief TODO
       */
      fmatvec::Vec wbParent;

      /**
       * \brief contact force parameters
       */
      fmatvec::Vec laParent;

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
       * \brief differentiated state
       */
      fmatvec::Vec zdParent;

      /**
       * \brief smooth, smooth with respect to objects, smooth with respect to links right hand side
       */
      fmatvec::Vec hParent, hObjectParent, hLinkParent;

      /**
       * \brief matrices for implicit integration
       */
      fmatvec::Mat    dhdqObjectParent, dhdqLinkParent;
      fmatvec::SqrMat dhduObjectParent, dhduLinkParent;
	  fmatvec::Vec    dhdtObjectParent, dhdtLinkParent;

      /**
       * \brief nonsmooth right hand side
       */
      fmatvec::Vec rParent;

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
      fmatvec::Vector<int> jsvParent;
      
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
      fmatvec::Vector<int> decreaseLevels;

      /**
       * \brief TODO
       */
      fmatvec::Vector<int> checkTermLevels;

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
      bool peds, impact, sticking;

      /**
       * \brief additional stop in event driven solver for drift correction
       * TODO
       */
      int k;

      /**
       * \brief Flag for reorganisation of hierarchy. 
       * This flag will be removed in the future.
       */
      bool reorganizeHierarchy;
      
      /**
       * \brief references to differentiated external state
       * \param differentiated external state
       */
      void updatezdRef(const fmatvec::Vec &ext);
      
      /**
       * \brief update relaxation factors for contact equations
       * TODO global not available because of unsymmetric mass action matrix
       */
      void updaterFactors();

      /**
       * \brief TODO
       */
      void computeConstraintForces(double t);

      /**
       * \brief function pointer for election of smooth update for event driven integrator
       */
      fmatvec::Vec (DynamicSystemSolver::*zdot_)(const fmatvec::Vec &zParent, double t);

      /**
       * \brief standard invocation of smooth update for event driven integration without inverse kinetics
       * \param state
       * \param time
       */
      fmatvec::Vec zdotStandard(const fmatvec::Vec &zParent, double t);

      /**
       * \brief invocation of smooth update for event driven integration with inverse kinetics
       * \param state
       * \param time
       */
      fmatvec::Vec zdotResolveConstraints(const fmatvec::Vec &zParent, double t);

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
      void addToTree(Tree* tree, Node* node, fmatvec::SqrMat &A, int i, std::vector<Object*> &objList);

      bool truncateSimulationFiles;
  };

}

#endif /* _DYNAMIC_SYSTEM_SOLVER_H_ */

