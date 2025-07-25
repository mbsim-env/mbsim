/* Copyright (C) 2004-2018 MBSim Development Team
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

#ifndef _DYNAMIC_SYSTEM_SOLVER_H_
#define _DYNAMIC_SYSTEM_SOLVER_H_

#include "mbsim/group.h"
#include "fmatvec/sparse_matrix.h"
#include "mbsim/functions/function.h"
#include "mbsim/environment.h"

#include <atomic>

namespace MBSim {

  class Graph;
  class MBSimEnvironment;
  class MultiDimNewtonMethod;
  class ConstraintResiduum;
  class ConstraintJacobian;

  struct StateTable {
    std::string name;
    char label;
    int number;
    StateTable() = default;
    StateTable(const std::string &name_, char label_, int number_) : name(name_), label(label_), number(number_) { }
  };

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
   */
  class DynamicSystemSolver : public Group {

    class Residuum : public MBSim::Function<fmatvec::Vec(fmatvec::Vec)> {
      public:
        Residuum(MBSim::DynamicSystemSolver *sys_) : sys(sys_) { }
        fmatvec::Vec operator()(const fmatvec::Vec &z);
      private:
        MBSim::DynamicSystemSolver *sys;
    };

    public:

      /**
       * \brief solver for contact equations
       */
      enum Solver { fixedpoint, GaussSeidel, direct, rootfinding, unknownSolver, directNonlinear };

      /**
       * \brief constructor
       * \param name of dynamic system
       */
      DynamicSystemSolver(const std::string &name="");

      /**
       * \brief destructor
       */
      ~DynamicSystemSolver() override;

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
       *     MyBaseClass::init(stage, config); // base must be called
       *     // do something
       *   }
       *   else if(stage==plot) {
       *     // do something
       *     // base need not be called
       *   }
       *   else
       *     MyBaseClass::init(stage, config);
       *
       *   for(int i=0; i<container.size(); i++)
       *     container[i]->init(stage, config);
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
      void init(InitStage stage, const InitConfigSet &config) override;
      using Group::plot;
      /***************************************************/

      /* INHERITED INTERFACE OF DYNAMICSYSTEM */
      int solveConstraintsFixpointSingle() override;
      int solveImpactsFixpointSingle() override;
      int solveConstraintsGaussSeidel() override;
      int solveImpactsGaussSeidel() override;
      int solveConstraintsRootFinding() override;
      int solveImpactsRootFinding() override;
      void checkConstraintsForTermination() override;
      void checkImpactsForTermination() override;
      /***************************************************/

      /* INHERITED INTERFACE OF OBJECTINTERFACE */
      void updateT() override;
      void updateh(int i=0) override;
      void updateM() override;
      void updateLLM() override;
      void updatezd() override;
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
      void updater(int j=0) override;
      void updateJrla(int j=0) override;
      virtual void updaterdt();
      void updatewb() override;
      void updateg() override;
      void updategd() override;
      void updateW(int j=0) override;
      void updateV(int j=0) override;
      virtual void updatebc();
      virtual void updatebi();
      virtual void updatela();
      virtual void updateLa();
      /***************************************************/

      /* INHERITED INTERFACE OF ELEMENT */
      /***************************************************/

      /* GETTER / SETTER */

      void setSmoothSolver(Solver solver_) { smoothSolver = solver_; }
      void setConstraintSolver(Solver solver_) { contactSolver = solver_; }
      void setImpactSolver(Solver solver_) { impactSolver = solver_; }
      const Solver& getSmoothSolver() { return smoothSolver; }
      const Solver& getConstraintSolver() { return contactSolver; }
      const Solver& getImpactSolver() { return impactSolver; }
      void setTermination(bool term_) { term = term_; }
      void setMaximumNumberOfIterations(int iter) { maxIter = iter; }
      void setHighNumberOfIterations(int iter) { highIter = iter; }
      void setNumericalJacobian(bool numJac_) { numJac = numJac_; }
      void setMaximumDampingSteps(int maxDSteps) { maxDampingSteps = maxDSteps; }
      void setLevenbergMarquardtParamater(double lmParm_) { lmParm = lmParm_; }

      void setUseOldla(bool flag) { useOldla = flag; }
      void setDecreaseLevels(const fmatvec::VecInt &decreaseLevels_) { decreaseLevels = decreaseLevels_; }
      void setCheckTermLevels(const fmatvec::VecInt &checkTermLevels_) { checkTermLevels = checkTermLevels_; }
      void setCheckGSize(bool checkGSize_) { checkGSize = checkGSize_; }
      void setLimitGSize(int limitGSize_) { limitGSize = limitGSize_; checkGSize = false; }

      double& getTime() { return t; }
      const double& getTime() const { return t; }
      void setTime(double t_) { t = t_; }

      double getStepSize() const { return dt; }
      void setStepSize(double dt_) { dt = dt_; }

      int getzSize() const { return zSize; }

      fmatvec::Vec& getState() { return z; }
      const fmatvec::Vec& getState() const { return z; }
      void setState(const fmatvec::Vec &z_) { z = z_; }

      const fmatvec::Vec& getzd(bool check=true) const { assert((not check) or (not updzd)); return zd; }
      void setzd(const fmatvec::Vec &zd_) { zd = zd_; }

      const fmatvec::SqrMat& getG(bool check=true) const { assert((not check) or (not updG)); return G; }
      const fmatvec::SparseMat& getGs(bool check=true) const { assert((not check) or (not updG)); return Gs; }
      const fmatvec::Vec& getbc(bool check=true) const { assert((not check) or (not updbc)); return bc; }
      const fmatvec::Vec& getbi(bool check=true) const { assert((not check) or (not updbi)); return bi; }
      const fmatvec::SqrMat& getJprox() const { return Jprox; }
      fmatvec::SqrMat& getG(bool check=true) { assert((not check) or (not updG)); return G; }
      fmatvec::SparseMat& getGs(bool check=true) { assert((not check) or (not updG)); return Gs; }
      fmatvec::Vec& getbc(bool check=true) { assert((not check) or (not updbc)); return bc; }
      fmatvec::Vec& getbi(bool check=true) { assert((not check) or (not updbi)); return bi; }
      fmatvec::SqrMat& getJprox() { return Jprox; }
      const fmatvec::Vec& getr(int j=0, bool check=true) const { assert((not check) or (not updr[j])); return r[j]; }
      fmatvec::Vec& getr(int j=0, bool check=true) { assert((not check) or (not updr[j])); return r[j]; }

      // Jacobian of dr/dla
      const fmatvec::Mat& getJrla(int j=0, bool check=true) const { assert((not check) or (not updJrla[j])); return Jrla[j]; }
      fmatvec::Mat& getJrla(int j=0, bool check=true) { assert((not check) or (not updJrla[j])); return Jrla[j]; }

      const fmatvec::Vec& evaldq() { if(upddq) updatedq(); return dq; }
      const fmatvec::Vec& evaldu() { if(upddu) updatedu(); return du; }
      const fmatvec::Vec& evaldx() { if(upddx) updatedx(); return dx; }
      const fmatvec::Vec& evalzd();
      const fmatvec::SqrMat& evalG() { if(updG) updateG(); return G; }
      const fmatvec::SparseMat& evalGs() { if(updG) updateG(); return Gs; }
      const fmatvec::Vec& evalbc() { if(updbc) updatebc(); return bc; }
      const fmatvec::Vec& evalbi() { if(updbi) updatebi(); return bi; }
      const fmatvec::Vec& evalsv();
      const fmatvec::Vec& evalz0();
      const fmatvec::Vec& evalla() { if(updla) updatela(); return la; }
      const fmatvec::Vec& evalLa() { if(updLa) updateLa(); return La; }

      fmatvec::Vec& getzParent() { return zParent; }
      fmatvec::Vec& getzdParent() { return zdParent; }
      fmatvec::Vec& getlaParent() { return laParent; }
      fmatvec::Vec& getLaParent() { return LaParent; }
//      const fmatvec::Vec& getzParent() const { return zParent; }
//      const fmatvec::Vec& getzdParent() const { return zdParent; }
//      const fmatvec::Mat& getWParent(int i=0) const { return WParent[i]; }
      fmatvec::Mat& getWParent(int i=0) { return WParent[i]; }
      fmatvec::Mat& getVParent(int i=0) { return VParent[i]; }
//      fmatvec::Vec& getlaParent() { return laParent; }
//      fmatvec::Vec& getLaParent() { return LaParent; }
//      const fmatvec::Vec& getgParent() const { return gParent; }
      fmatvec::Vec& getgParent() { return gParent; }
//      const fmatvec::Vec& getgdParent() const { return gdParent; }
      fmatvec::Vec& getgdParent() { return gdParent; }
      fmatvec::Vec& getresParent() { return resParent; }
      fmatvec::Vec& getrFactorParent() { return rFactorParent; }

      void resizezParent(int nz) { zParent.resize(nz); }
      void resizezdParent(int nz) { zdParent.resize(nz); }

      DynamicSystemSolver* getDynamicSystemSolver() { return this; }
      int getMaxIter()  {return maxIter;}
      int getHighIter()  {return highIter;}
      int getIterC()  {return iterc;}
      int getIterI()  {return iteri;}
      /***************************************************/

      /**
       * \brief compute initial condition for links for event driven integrator
       */
      void computeInitialCondition();

      int (DynamicSystemSolver::*solveSmooth_)();

      /**
       * \function pointer for election of prox-solver for contact equations
       * \return iterations of solver
       */
      int (DynamicSystemSolver::*solveConstraints_)();

      /**
       * \brief function pointer for election of prox-solver for impact equations on velocity level
       * \param time step
       * \return iterations of solver
       */
      int (DynamicSystemSolver::*solveImpacts_)();

      /**
       * \brief solution of contact equations with direct linear solver using minimum of least squares
       * \return iterations of solver
       * \todo put in dynamic system? TODO
       */
      int solveConstraintsLinearEquations();

      /**
       * \brief solution of contact equations with direct nonlinear newton solver using minimum of least squares in each iteration step
       * \return iterations of solver
       * \todo put in dynamic system? TODO
       */
      int solveConstraintsNonlinearEquations();

      /**
       * \brief solution of contact equations with Cholesky decomposition on velocity level
       * \param time step-size, if 0 non-impulsive contributions vanish
       * \return iterations of solver
       * \todo put in dynamic system? TODO
       */
      int solveImpactsLinearEquations();

      int solveImpactsNonlinearEquations();

      /**
       * \brief updates mass action matrix
       * \param time
       */
      virtual void updateG();

      /**
       * \brief decrease relaxation factors if mass action matrix is not diagonal dominant
       */
      void decreaserFactors();

      /**
       * \brief update for event driven integrator for event
       * \param state (return)
       * \param boolean evaluation of stop vector
       * \param time
       */
      virtual const fmatvec::Vec& shift(std::optional<std::reference_wrapper<bool>> &&velProjWasCalled={},
                                        std::optional<std::reference_wrapper<bool>> &&posProjWasCalled={});

      /** brief collect status of all set-valued links
       * \param result vector
       * \param time
       */
      void getLinkStatus(fmatvec::VecInt &LinkStatusExt);

      /** brief collect status of all single-valued links
       * \param result vector
       * \param time
       */
      void getLinkStatusReg(fmatvec::VecInt &LinkStatusRegExt);

      /**
       * \brief check if drift compensation on position level is needed
       */
      bool positionDriftCompensationNeeded(double gmax);

      /**
       * \brief check if drift compensation on velocity level is needed
       */
      bool velocityDriftCompensationNeeded(double gdmax);

      /**
       * \brief drift projection for positions
       */
      void projectGeneralizedPositions(int mode, bool fullUpdate=false);

      /**
       * \brief drift projection for positions
       */
      void projectGeneralizedVelocities(int mode);

      /**
       * \brief save contact forces for use as starting value in next time step
       * \todo put in dynamic system TODO
       */
      void savela();

      /**
       * \brief load contact forces for use as starting value
       * \todo put in dynamic system TODO
       */
      void initla();

      /**
       * \brief save contact impulses for use as starting value in next time step
       * \todo put in dynamic system TODO
       */
      void saveLa();

      /**
       * \brief load contact impulses for use as starting value
       * \todo put in dynamic system TODO
       */
      void initLa();

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
       * \return information for solver
       */
      std::string getSolverInfo();

      /**
       * \param specify whether time integration should be stopped in case of no convergence of constraint-problem
       * \param specify whether contact informations should be dropped to file
       */
      void setStopIfNoConvergence(bool flag, bool dropInfo = false) { stopIfNoConvergence = flag; dropContactInfo=dropInfo; }

      bool getStopIfNoConvergence() { return stopIfNoConvergence; }

      /**
       * writes a file with relevant matrices for debugging
       */
      void dropContactMatrices();

      // MBSim signal handler
#ifndef SWIG
      class SignalHandler {
        public:
          SignalHandler();
          ~SignalHandler();
        private:
          using SigHandle = void (*)(int);
          #ifndef _WIN32
          SigHandle oldSigHup;
          #endif
          SigHandle oldSigInt;
          SigHandle oldSigTerm;
      };
#endif

      static void throwIfExitRequested(bool silent=false) {
        if(exitRequest) {
          if(!silent)
            msgStatic(fmatvec::Atom::Error)<<"User requested a exit (throw exception now)."<<std::endl;
          throw std::runtime_error("Exception due to user requested exit.");
        }
      }

      static bool exitRequested(bool silent=false) {
        if(exitRequest && !silent)
          msgStatic(fmatvec::Atom::Error)<<"User requested a exit (caller will handle this request now)."<<std::endl;
        return exitRequest;
      }

      // TODO just for testing
      void setPartialEventDrivenSolver(bool peds_) { peds = peds_; }

      /**
       * \brief writes state to a file
       * \param name of the file
       * \param h5, else ascii
       */
      void writez(std::string fileName, bool formatH5=true);

      /**
       * \brief writes state table to a file
       * \param name of the file
       * \param h5, else ascii
       */
      void writeStateTable(std::string fileName);

      /**
       * \brief reads state from a file
       * \param name of the file
       */
      void readz0(std::string fileName);

      void initializeUsingXML(xercesc::DOMElement *element) override;

      /**
       * \brief set tolerance for projection of generalized position
       * \param tolerance
       */
      void setProjectionTolerance(double tol) { tolProj = tol; }

      /**
       * \brief set tolerance for local none-linear solver (solvers on element level), like the Newton-Solver in JointConstraint
       * \param tolerance
       */
      void setLocalSolverTolerance(double tol) { tolLocalSolver = tol; }
      double getLocalSolverTolerance() { return tolLocalSolver; }

      /**
       * \brief set tolerance for global none-linear solver (solvers on DynamicSystemSolver level), like the Newton-Solver in directNonlinear
       * \param tolerance
       */
      void setDynamicSystemSolverTolerance(double tol) { tolDSS = tol; }
      double getDynamicSystemSolverTolerance() { return tolDSS; }

      /**
       * \brief references to external state
       * \param external state
       */
      void updatezRef(fmatvec::Vec &ext);

      /**mbsim/dynamic_system_solver.cc
       * \brief references to differentiated external state
       * \param differentiated external state
       */
      void updatezdRef(fmatvec::Vec &ext);

      void setAlwaysConsiderContact(bool alwaysConsiderContact_) { alwaysConsiderContact = alwaysConsiderContact_; }

      void setInverseKinetics(bool inverseKinetics_) { inverseKinetics = inverseKinetics_; }
      bool getInverseKinetics() const { return inverseKinetics; }

      void setInitialProjection(bool initialProjection_) { initialProjection = initialProjection_; }
      bool getInitialProjection() const { return initialProjection; }

      void setDetermineEquilibriumState(bool determineEquilibriumState_) { determineEquilibriumState = determineEquilibriumState_; }
      bool getDetermineEquilibriumState() const { return determineEquilibriumState; }

      void setUseConstraintSolverForPlot(bool useConstraintSolverForPlot_) { useConstraintSolverForPlot = useConstraintSolverForPlot_; }
      bool getUseConstraintSolverForPlot() const { return useConstraintSolverForPlot; }

      fmatvec::Mat dhdq(int lb=0, int ub=0);
      fmatvec::Mat dhdu(int lb=0, int ub=0);
      fmatvec::Mat dhdx();
      fmatvec::Vec dhdt();

      void setRootID(int ID) {rootID = ID;}
      int getRootID() const {return rootID;}

      void resetUpToDate() override;

      bool getUpdateT() { return updT; }
      bool getUpdateM() { return updM; }
      bool getUpdateLLM() { return updLLM; }
      bool getUpdateh(int j) { return updh[j]; }
      bool getUpdater(int j) { return updr[j]; }
      bool getUpdateJrla(int j) { return updJrla[j]; }
      bool getUpdaterdt() { return updrdt; }
      bool getUpdateW(int j) { return updW[j]; }
      bool getUpdateV(int j) { return updV[j]; }
      bool getUpdatewb() { return updwb; }
      bool getUpdateg() { return updg; }
      bool getUpdategd() { return updgd; }
      bool getUpdatela() { return updla; }
      bool getUpdateLa() { return updLa; }
      bool getUpdatezd() { return updzd; }
      bool getUpdatedq() { return upddq; }
      bool getUpdatedu() { return upddu; }
      bool getUpdatedx() { return upddx; }
      void setUpdatela(bool updla_) { updla = updla_; }
      void setUpdateLa(bool updLa_) { updLa = updLa_; }
      void setUpdateG(bool updG_) { updG = updG_; }
      void setUpdatebi(bool updbi_) { updbi = updbi_; }
      void setUpdatebc(bool updbc_) { updbc = updbc_; }
      void setUpdatezd(bool updzd_) { updzd = updzd_; }
      void setUpdateW(bool updW_, int i=0) { updW[i] = updW_; }

      /**
       * \brief references to relative distances of dynamic system parent
       * \param vector to be referenced
       */
      void updategRef(fmatvec::Vec &ref) override { Group::updategRef(ref); updg = true; }

      /**
       * \brief references to relative velocities of dynamic system parent
       * \param vector to be referenced
       */
      void updategdRef(fmatvec::Vec &ref) override { Group::updategdRef(ref); updgd = true; }

      /**
       * \brief references to TODO of dynamic system parent
       * \param vector to be referenced
       */      
      void updatewbRef(fmatvec::Vec &ref) override { Group::updatewbRef(ref); updwb = true; }

      /**
       * \brief references to contact force direction matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateWRef(fmatvec::Mat &ref, int i=0) override { Group::updateWRef(ref,i); updW[i] = true; }

      /**
       * \brief references to condensed contact force direction matrix of dynamic system parent
       * \param matrix to be referenced
       * \param index of normal usage and inverse kinetics
       */
      void updateVRef(fmatvec::Mat &ref, int i=0) override { Group::updateVRef(ref,i); updV[i] = true; }

      /**
       * \brief update inverse kinetics constraint forces
       */
      virtual void updatelaInverseKinetics();

      void updatedq() override;
      void updatedu() override;
      void updatedx() override;

      void updateStopVector() override;

      void updateInternalState();

      void plot() override;

      void addEnvironment(Environment* env);

      /** Get the Environment of type Env.
       * Note that this function uses runtime type information to get the correct environment. This may be slow!
       * Instead of calling this method often you should call it once, store the returned pointer and use this pointer.
       * If no Environment of type Env exits a new one (empty one) is added first and than returned. */
      template<class Env>
      Env* getEnvironment();

      /** Get the MBSimEnvironment.
       * This method a convinence method being fast compared to getEnvironment<MBSimEnvironment> */
      MBSimEnvironment* getMBSimEnvironment();

      std::vector<StateTable>& getStateTable() { return tabz; }

      void setCompressionLevel(int level) { compressionLevel=level; }
      void setChunkSize(int size) { chunkSize=size; }
      void setCacheSize(int size) { cacheSize=size; }

      void setqdequ(bool qdequ_) { qdequ = qdequ_; }
      bool getqdequ() { return qdequ; }

    protected:
      /**
       * \brief time
       */
      double t;

      /**
       * \brief step size
       */
      double dt;

      /**
       * \brief size of state vector
       */
      int zSize;

      /**
       * \brief state vector
       */
      fmatvec::Vec z;

      /**
       * \brief derivative of state vector
       */
      fmatvec::Vec zd;

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

      fmatvec::Vec curisParent, nextisParent;

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

      fmatvec::Vec dxParent, dqParent, duParent;

      /**
       * \brief smooth, smooth with respect to objects, smooth with respect to links right hand side
       */
      fmatvec::Vec hParent[2];

      /**
       * \brief nonsmooth right hand side
       */
      fmatvec::Vec rParent[2], rdtParent;

      fmatvec::Mat JrlaParent[2];

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
      fmatvec::Vec bc, bi;

      /**
       * \brief boolean to check for termination of contact equations solution
       */
      bool term;

      /**
       * \brief maximum number of contact iterations, high number of contact iterations for warnings, maximum number of damping steps for Newton scheme
       */
      int maxIter, highIter, maxDampingSteps, iterc, iteri;

      /**
       * \brief Levenberg-Marquard parameter
       */
      double lmParm;

      /**
       * \brief solver for contact equations and impact equations
       */
      Solver smoothSolver, contactSolver, impactSolver;

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
       * \brief TODO, flag for occuring impact and sticking in event driven solver
       */
      bool peds;

      /**
       * \brief Tolerance for projection of generalized position.
       */
      double tolProj;

      /**
       * \brief Tolerance for local none-linear solvers (solvers on element level)
       */
      double tolLocalSolver { 1e-10 };

      /**
       * \brief Tolerance for global none-linear solvers (solvers on DynamicSystemSolver level)
       */
      double tolDSS { 1e-9 };

      /**
       * \brief
       */
      fmatvec::Vec laInverseKineticsParent;
      fmatvec::Mat bInverseKineticsParent;

      /**
       * \brief
       */
      fmatvec::Mat WInverseKineticsParent;

      bool alwaysConsiderContact;
      bool inverseKinetics;
      bool initialProjection;
      bool determineEquilibriumState;
      bool useConstraintSolverForPlot;

      fmatvec::Vec corrParent;

      int rootID;

      double gTol, gdTol, gddTol, laTol, LaTol;

      bool updT, updh[2], updr[2], updJrla[2], updrdt, updM, updLLM, updW[2], updV[2], updwb, updg, updgd, updG, updbc, updbi, updsv, updzd, updla, updLa, upddq, upddu, upddx;

      bool useSmoothSolver;

      bool qdequ;

      std::vector<StateTable> tabz;

      int compressionLevel { H5::File::getDefaultCompression() };
      int chunkSize { H5::File::getDefaultChunkSize() };
      int cacheSize { H5::File::getDefaultCacheSize() };

    private:
      /**
       * \brief handler for user interrupt signal
       */
      static void sigInterruptHandler(int);

      /**
       * \brief set plot feature default values
       */
      void constructor();

      /**
       * \brief boolean signal evaluation for end integration set by user
       */
      static std::atomic<bool> exitRequest;
      static_assert(decltype(exitRequest)::is_always_lock_free);

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

      double facSizeGs;

      std::vector<std::unique_ptr<Environment>> environments;
      MBSimEnvironment *mbsimEnvironment = nullptr;

      bool firstPlot { true };

      std::unique_ptr<MultiDimNewtonMethod> nonlinearConstraintNewtonSolver;
      std::unique_ptr<ConstraintResiduum> constraintResiduum;
      std::unique_ptr<ConstraintJacobian> constraintJacobian;
  };

  template<class Env>
  Env* DynamicSystemSolver::getEnvironment() {
    // get the Environment of type Env
    auto &reqType=typeid(Env);
    for(auto &e : environments) {
      auto &e_=*e;
      if(reqType==typeid(e_))
        return static_cast<Env*>(e.get());
    }
    auto newEnv=new Env;
    addEnvironment(newEnv);
    return newEnv;
  }

}

#endif /* _DYNAMIC_SYSTEM_SOLVER_H_ */
