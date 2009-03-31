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

#ifndef _MULTI_BODY_SYSTEM_H_
#define _MULTI_BODY_SYSTEM_H_

#include "mbsim/group.h"
#include "sparse_matrix.h"
#include "H5Cpp.h"
#include <string>

using namespace std;

namespace MBSim {

  class Frame;
  class HydFluid;
  class ExtraDynamicInterface;
  class DataInterfaceBase;
  class Integrator;

  /** 
   * \brief solver for contact equations
   */
  enum Solver { FixedPointTotal, FixedPointSingle, GaussSeidel, LinearEquations, RootFinding };

  /** 
   * \brief relaxation strategies in solution of contact equations
   */
  enum Strategy {global, local};

  /**
   * \brief linear algebra for Newton scheme in solution of contact equations
   */
  enum LinAlg {LUDecomposition,LevenbergMarquardt,PseudoInverse};

  /**
   * \brief solver interface for modelling and simulation of multibody systeme
   * \author Martin Foerg
   * \date 2009-03-31 some comments (Thorsten Schindler)
   */
  class MultiBodySystem : public Group {
    public:
      /** 
       * \brief constructor
       */
      MultiBodySystem();

      /**
       * \brief constructor
       * \param name of multibody system
       */
      MultiBodySystem(const string &projectName);

      /**
       * \brief destructor
       */
      virtual ~MultiBodySystem();

      /* INHERITED INTERFACE OF GROUP */
      void init();
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
      virtual void initPlot();
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      virtual void updateh(double t);
      virtual void updateM(double t);
      virtual void updateKinematics(double t); // this function is called once every time step by every integrator
      /***************************************************/

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updater(double t);
      virtual void updatewb(double t);
      virtual void updateW(double t);
      virtual void updateV(double t);
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      virtual string getFullName() const { return name; }
      virtual void load(const string &path, ifstream& inputfile); // TODO replace with XML
      virtual void save(const string &path, ofstream& outputfile);
      virtual string getType() const { return "MultiBodySystem"; }
      virtual void plot(const Vec& z, double t, double dt=1); // TODO completely rearrange
      virtual void closePlot();
      /***************************************************/

      /* INHERITED INTERFACE FOR DERIVED CLASS */
      /**
       * \param multibody system to preintegrate
       */
      virtual void preInteg(MultiBodySystem *parent);

      /**
       * \brief solves prox-functions on acceleration level
       */
      virtual int solveConstraints(); 

      /**
       * \brief solves prox-functions on velocity level
       */
      virtual int solveImpacts(double dt = 0); 
      /***************************************************/

      /* GETTER / SETTER */
      void setAccelerationOfGravity(const Vec& g) { grav = g; }
      const Vec& getAccelerationOfGravity() const { return grav; }

      void setProjectDirectory(const string &directoryName_) { directoryName = directoryName_; }
      void setPreInteg(Integrator *preInteg_) { preIntegrator = preInteg_; }

      void setImpact(bool impact_) { impact = impact_; }
      void setSticking(bool sticking_) { sticking = sticking_; }

      void setConstraintSolver(Solver solver_) { contactSolver = solver_; }                         
      void setImpactSolver(Solver solver_) { impactSolver = solver_; }                         
      void setTermination(bool term_) { term = term_; }
      void setStrategy(Strategy strategy_) { strategy = strategy_; }
      void setMaxIter(int iter) { maxIter = iter; }
      void setHighIter(int iter) { highIter = iter; }
      void setNumJacProj(bool numJac_) { numJac = numJac_; }
      void setMaxDampingSteps(int maxDSteps) { maxDampingSteps = maxDSteps; }
      void setLevenbergMarquardtParam(double lmParm_) { lmParm = lmParm_; }
      void setLinAlg(LinAlg linAlg_) { linAlg = linAlg_; }                         

      void setUseOldla(bool flag) { useOldla = flag; }
      void setDecreaseLevels(const Vector<int> &decreaseLevels_) { decreaseLevels = decreaseLevels_; }
      void setCheckTermLevels(const Vector<int> &checkTermLevels_) { checkTermLevels = checkTermLevels_; }
      void setCheckGSize(bool checkGSize_) { checkGSize = checkGSize_; }
      void setLimitGSize(int limitGSize_) { limitGSize = limitGSize_; checkGSize = false; }

      const double getpinf() const { return pinf; };
      void setpinf(const double pinf_) { pinf=pinf_; };
      void setFluid(HydFluid *fl_) { fl=fl_; }
      HydFluid *getFluid(){ return fl; }  

      const Matrix<Sparse, double>& getGs() const { return Gs; }
      Matrix<Sparse, double>& getGs() { return Gs; }
      const SqrMat& getG() const { return G; }
      SqrMat& getG() { return G; }
      const Vec& getb() const { return b; }
      Vec& getb() { return b; }
      const SqrMat& getJprox() const { return Jprox; }
      SqrMat& getJprox() { return Jprox; }

      MultiBodySystem* getMultiBodySystem() { return this; }
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
      Vec deltau(const Vec &zParent, double t, double dt);

      /**
       * \return position difference for current time 
       * \param state
       * \param time
       * \param time step
       */
      Vec deltaq(const Vec &zParent, double t, double dt);

      /**
       * \brief TODO
       * \param TODO
       * \param time
       * \param time step
       */
      Vec deltax(const Vec &zParent, double t, double dt);

      /**
       * \brief initialises state variables
       */
      void initz(Vec& z0);

      /**
       * \return successful flag for function pointer for election of prox-solver on acceleration level
       */
      int (MultiBodySystem::*solveConstraints_)();

      /**
       * \return successful flag for function pointer for election of prox-solver on velocity level
       * \param time step
       */
      int (MultiBodySystem::*solveImpacts_)(double dt);

      /**
       * \return successful solution of contact equations with Cholesky decomposition on acceleration level
       * TODO subsystem?
       */
      int solveConstraintsLinearEquations(); 

      /**
       * \return successful solution of contact equations with Cholesky decomposition on velocity level
       * TODO subsystem?
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
       * \brief update of multibody system for time-stepping integrator
       * \param state
       * \param time
       */
      void update(const Vec &z, double t);

      /**
       * \brief update for event driven integrator for event
       * \param state (return)
       * \param boolean evaluation of stop vector
       * \param time
       */
      virtual void shift(Vec& z, const Vector<int>& jsv, double t);

      /**
       * \brief update for event driven integrator during smooth phase
       * \param state
       * \param differentiated state (return)
       * \param time
       */
      void zdot(const Vec& z, Vec& zd, double t);

      /**
       * \brief evaluation of stop vector
       * \param state
       * \param TODO
       * \param time
       */
      void getsv(const Vec& z, Vec& svExt, double t);

      /**
       * \brief update for event driven integrator during smooth phase
       * \param state
       * \param time
       */
      Vec zdot(const Vec& z, double t) { return (this->*zdot_)(z,t); }

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
      void savela(); // TODO put in subsystem 

      /**
       * load contact force parameter for use as starting value
       */
      void initla(); // TODO put in subsystem

      /** 
       * \brief compute kinetic energy of entire multibodysystem
       */
      double computeKineticEnergy() { return 0.5*trans(u)*M*u; }

      /** 
       * \brief compute potential energy of entire multibody system TODO change
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
      Element* getElement(const string &name); 

      /**
       * \return information for solver including strategy and linear algebra
       */
      string getSolverInfo();

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
       * \param name of frame to find in multibody system
       * TODO concept
       */
      Frame* findFrame( const string &name );
      
      /**
       * \param name of contour to find in multibody system
       *
       * TODO concept
       */
      Contour* findContour( const string &name );

      /**
       * \param path of topology
       * \return multibody system 
       */
      static MultiBodySystem* load(const string &path);
      
      /**
       * \brief save multibody system topology
       * \param path of topology
       * \param multibody system pointer
       */
      static void save(const string &path, MultiBodySystem* mbs);

      /**
       * \brief handler for signals
       * \param TODO
       */
      static void sigTermHandler(int);

      // TODO necessary?
      bool driftCompensation(Vec& z, double t) { return false; } 

      // TODO just for testing
      void setPartialEventDrivenSolver(bool peds_) { peds = peds_; }

      // TODO just for testing
      void writez();
      void readz0();

    protected:
      /**
       * \brief mass matrix
       */
      SymMat MParent;
      
      /**
       * \brief Cholesky decomposition of mass matrix
       */
      SymMat LLMParent;

      /**
       * \brief matrix of linear relation between differentiated positions and velocities
       */
      Mat TParent;

      /**
       * \brief contact force directions
       */
      Mat WParent;

      /**
       * \brief condensed contact force directions
       */
      Mat VParent;

      /**
       * \brief TODO
       */
      Vec wbParent;

      /**
       * \brief contact force parameters
       */
      Vec laParent;

      /**
       * \brief relaxation parameters for contact equations
       */
      Vec rFactorParent;

      /**
       * \brief TODO
       */
      Vec sParent;

      /**
       * \brief residuum of contact equations
       */
      Vec resParent;

      /**
       * \brief relative distances
       */
      Vec gParent;

      /**
       * \brief relative velocities
       */
      Vec gdParent;

      /**
       * \brief differentiated state
       */
      Vec zdParent;

      /**
       * \brief smooth right hand side
       */
      Vec hParent;

      /**
       * \brief nonsmooth right hand side
       */
      Vec rParent;

      /**
       * \brief right hand side of order one parameters
       */
      Vec fParent;

      /**
       * \brief stopvector (rootfunctions for event driven integration
       */
      Vec svParent;

      /**
       * \brief boolean evaluation of stopvector
       */
      Vector<int> jsvParent;
      
      /** 
       * \brief gravitation common for all components
       */
      Vec grav;

      /**
       * \brief sparse mass action matrix
       */
      Matrix<Sparse, double> Gs;

      /**
       * \brief JACOBIAN of contact equations for Newton scheme
       */
      SqrMat Jprox;

      /**
       * \brief mass action matrix
       */
      SqrMat G;

      /**
       * \brief TODO
       */
      Vec b;

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
      Vector<int> decreaseLevels;

      /**
       * \brief TODO
       */
      Vector<int> checkTermLevels;

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
       * \brief name of directory where output is processed
       */
      string directoryName;
      
      /**
       * \brief hydraulic fluid, only for hydraulic systems TODO
       */
      HydFluid *fl;

      /**
       * \brief ambient pressure, only for hydraulic systems TODO
       */
      double pinf;

      /**
       * \brief pre-integration scheme
       */
      Integrator *preIntegrator;

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
       * \brief references to external state
       * \param external state
       */
      void updatezRef(const Vec &ext);

      /**
       * \brief references to differentiated external state
       * \param differentiated external state
       */
      void updatezdRef(const Vec &ext);
      
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
       * \brief create directories for simulation output
       */
      void setDirectory();

      /**
       * \brief function pointer for election of smooth update for event driven integrator
       */
      Vec (MultiBodySystem::*zdot_)(const Vec &zParent, double t);

      /**
       * \brief standard invocation of smooth update for event driven integration without inverse kinetics
       * \param state
       * \param time
       */
      Vec zdotStandard(const Vec &zParent, double t);

      /**
       * \brief invocation of smooth update for event driven integration with inverse kinetics
       * \param state
       * \param time
       */
      Vec zdotResolveConstraints(const Vec &zParent, double t);

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
  };

}

#endif

