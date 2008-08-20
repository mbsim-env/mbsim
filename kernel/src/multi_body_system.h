/* Copyright (C) 2004-2006  Martin Förg, Roland Zander

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
 *   mfoerg@users.berlios.de
 *   rzander@users.berlios.de
 *
 */

#ifndef _MULTI_BODY_SYSTEM_H_
#define _MULTI_BODY_SYSTEM_H_

#include<string>
#include "object.h"
#include "hitsphere_link.h"
#include "sparse_matrix.h"

using namespace std;

namespace MBSim {

  class Port;
  class HydFluid;
  class ExtraDynamicInterface;
  class DataInterfaceBase;
  class Integrator;

  /** Solver for contact parameter identification */
  enum Solver {FixedPointTotal, FixedPointSingle, GaussSeidel, LinearEquations, RootFinding};
  /** r-Factor strategies */
  enum Strategy {global, local};
  /** Linear algebra for rootFinding solver */
  enum LinAlg {LUDecomposition,LevenbergMarquardt,PseudoInverse};

  /*! Multibodysystem for holding the global system information */
  class MultiBodySystem : public Object {

    friend class HitSphereLink;

    protected:
    /* INCLUDED MBS PARTS */
    /** vector of included objects */
    vector<Object*> objects;
    /** vector of included links */
    vector<Link*> links;
    /** vector of included single valued links */
    vector<Link*> linkSingleValued;
    /** vector of included set valued links */
    vector<Link*> linkSetValued;
    /** vector of included active set valued links */
    vector<Link*> linkSetValuedActive;
    /** vector of included extra dynamic interfaces */
    vector<ExtraDynamicInterface*> EDI;
    /** vector of included data interface bases */
    vector<DataInterfaceBase*> DIBs;
    /** vector of included hitspheres */
    vector<HitSphereLink*> HSLinks;
    /** ingredients to plot */
    vector<Object*> objects2plot;
    vector<Link*> links2plot;
    vector<Contour*> contours2plot;
    vector<Port*> ports2plot;
    vector<ExtraDynamicInterface*> EDIs2plot;

    /** gravitation common for all components */
    Vec grav;
    /************************************************************/

    /* INTERNAL DATA */
    /** Delassus matrix */
    SymMat G;
    /** Delassus matrix sparse structure */
    Matrix<Sparse, double> Gs;
    /** matrix of force directions */
    Mat W;
    /** numerical Jacobian in Newton */
    SqrMat Jprox;
    /** Delassus vector */
    Vec b;

    Vec dla;
    /** relative velocity for contact iteration */ 
    Vec s;
    /** residuum of fixpoint expression in Newton */
    Vec res;

    /** gaps and gap velocities */
    Vec g, gd;
    /** dimension and index of gaps */
    int gSize, gInd;
    /** constraint forces */
    Vec la;
    /** dimension and index of constraint forces */
    int laSize, laInd;
    /** rFactors */
    Vec rFactor;
    /** dimension and index of rFactor */
    int rFactorSize, rFactorInd;

    int svSize;
    int svInd;
    Vec sv;
    Vector<int> jsv;

    /** Jacobian of smooth right hand side */
    Mat Jh;

    int nHSLinksSetValuedFixed;
    int nHSLinksSingleValuedFixed;

    bool checkGSize;
    int limitGSize;
    /************************************************************/

    /* PARENT DATA */
    /** mass matrix parent */
    SymMat MParent;
    /** Cholesky decomposition of mass matrix parent */
    SymMat LLMParent;
    /** T-matrix parent */
    Mat TParent;
    /** Delassus matrix parent */
    SymMat GParent;
    /** force directions parent */
    Mat WParent;
    /** Delassus vector parent */
    Vec bParent;
    /** constraint forces parent */
    Vec laParent;
    Vec dlaParent;
    /** rFactor parent */
    Vec rFactorParent;
    /** vector for contact iteration parent */
    Vec sParent;
    /** residuum of fixpoint expression parent */
    Vec resParent;
    /** gaps parent */
    Vec gParent;
    /** gap velocities parent */
    Vec gdParent;
    /** differentiated state parent */
    Vec zdParent;
    /** smooth right hand side parent */
    Vec hParent;
    /** set-valued right hand side parent */
    Vec rParent;
    /** control vector parent */
    Vec fParent;
    Vec svParent;
    Vector<int> jsvParent;
    /************************************************************/

    /* CONTACT ITERATION */
    /** maximum and high iteration number for error and warning during constraint calculation */
    int maxIter, highIter, maxDampingSteps;
    /** Levenberg Marquard parameter */
    double lmParm;
    /** termination FLAG for constrained iteration */
    bool term;
    /** constraint solver */
    Solver solver;
    /** rFactor-strategy */
    Strategy strategy;
    /** linear algebra for root finding */
    LinAlg linAlg;
    /** FLAG for stopping the contact iterations, if there is no convergence */
    bool stopIfNoConvergence;
    /** FLAG to detect a change in the time variant configuration */
    bool activeConstraintsChanged;
    /** FLAG for the output of contact matrices in case of no convergence */
    bool dropContactInfo;
    /** should old constraint forces be used to accelerate constraint solver */
    bool useOldla;
    /** numerical Jacobian in Rootfinding? */
    bool numJac;
    /** levels for decreasing rFactors */
    Vector<int> decreaseLevels;
    Vector<int> checkTermLevels;
    /************************************************************/

    /* UPDATES */
    /*! References to differentiated state of parent multibody system mbs */
    void updatezdRef(const Vec &zdExt);
    /*! References to positions of parent multibody system mbs */
    void updateqRef(const Vec &qExt);
    /*! References to differentiated positions of parent multibody system mbs */
    void updateqdRef(const Vec &qdExt);
    /*! References to velocities of parent multibody system mbs */
    void updateuRef(const Vec &uExt);
    /*! References to order one parameters of parent multibody system mbs */
    void updatexRef(const Vec &xExt);
    /*! References to smooth right hand side of parent multibody system mbs */
    void updatehRef(const Vec &hExt);
    /*! References to set-valued right hand side of parent multibody system mbs */
    void updaterRef(const Vec &rExt);
    /*! References to control vector of parent multibody system mbs */
    void updatefRef(const Vec &fExt);
    /*! References to T-matrix of parent multibody system mbs */
    void updateTRef(const Mat &TExt);
    /*! References to mass matrix of parent multibody system mbs */
    void updateMRef(const SymMat &MExt);
    /*! References to Cholesky decomposition of mass matrix of parent multibody system mbs */
    void updateLLMRef(const SymMat &LLMExt);
    void updatesvRef(const Vec &svExt);
    void updatejsvRef(const Vector<int> &jsvExt);

    /*! Updates rFeactors without checking the hierarchy */
    void updaterFactors();
    /*! Updates differentiated state of a multibody system without checking the hierarchy */
    void updatezd(double t);
    /*! Updates order one variables for a multibody system without checking the hierarchy */
    void updatedx(double t, double dt);
    /*! Updates velocity gap for a multibody system without checking the hierarchy */
    void updatedu(double t, double dt);
    /*! Updates position gap for a multibody system without checking the hierarchy */
    void updatedq(double t, double dt);
    /*! Computes constraint forces with respect to EoM */
    void computeConstraintForces(double t);
    /*! Calls updateStopVector for children */
    void updateStopVector(double t);
    /************************************************************/

    /* OUTPUT */
    /** Name of directory where output is processed */
    string directoryName;
    /*! Test and enumerate directories for simulation output */
    void setDirectory();
    /***********************************************************/

    /* HYDRAULICS */
    /** Hydraulic fluid, only for hydraulic systems */
    HydFluid *fl;
    /** Ambient pressure, only for hydraulic systems */
    double pinf;
    /***********************************************************/

    /** preintegrator */
    Integrator *preIntegrator;

    public:
    /*! Constructor */
    MultiBodySystem();
    /*! Constructor */
    MultiBodySystem(const string &projectName);
    /*! Destructor */
    ~MultiBodySystem();

    /*! Return vector of gravitational acceleration in world system */
    const Vec& getGrav() const {return grav;};
    /*! Define vector of gravitational acceleration g in world system */
    void setGrav(const Vec& g);
    /*! Compute kinetic energy of entire multibodysystem, which is the quadratic form \f$\frac{1}{2}\boldsymbol{u}^T\boldsymbol{M}\boldsymbol{u}\f$ for all systems */
    double computeKineticEnergy() { return 0.5*trans(u)*M*u; }
    /* ! Compute potential energy of entire multibody system */
    double computePotentialEnergy();
    /*! Returns an object */
    Object* getObject(const string &name,bool check=true);
    /*! Get MBS */
    MultiBodySystem* getMultiBodySystem() const {return mbs;};
    /*! Returns a link */
    Link* getLink(const string &name,bool check=true);
    /*! Get a port */
    Port* getPort(const string &name,bool check=true);
    /*! Get a contour */
    Contour* getContour(const string &name,bool check=true);
    /*! Returns an extra dynamic interface */
    ExtraDynamicInterface* getEDI(const string &name,bool check=true);
    /*! Returns a data interface base */
    DataInterfaceBase* getDataInterfaceBase(const string &name, bool check=true);
    /*! Returns the pointer to an element name */
    Element* getElement(const string &name); 
    /*! Adds an object to multibody system */
    void addObject(Object *object);
    /*! Adds mbs to multibody system */
    void addMbs(MultiBodySystem* mbs);
    /*! Add a link to multibody system */
    void addLink(Link *connection);
    /*! Add a port */
    void addPort(Port *port, const Vec &WrOP);
    /*! Add a port */
    void addPort(const string &name, const Vec &WrSP);
    /*! Add a contour */
    void addContour(Contour *contour, const Vec &WrOP);
    /*! Add a contour */
    void addContour(Contour *contour, const Vec &WrOP,const SqrMat &AWC);
    /*! Add a ExtraDynamicInterface to multibody system */
    void addEDI(ExtraDynamicInterface *edi_);
    /*! Add a DataInterfaceBase to the DataInterfaceBase-vector */
    void addDataInterfaceBase(DataInterfaceBase* dib_);
    /*! Method to add any element (Link, Object, ExtraDynamicInterface) by dynamic casting */
    void addElement(Element *element_);

    /*! Get mass matrix */
    const SymMat& getM() const {return M;}
    /*! Get mass matrix */
    SymMat& getM() {return M;}
    /*! Get smooth right hand side */
    const Vec& geth() const {return h;}
    /*! Get smooth right hand side */
    Vec& geth() {return h;}
    /*! Get Delassus matrix */
    const SymMat& getG() const {return G;}
    /*! Get Delassus matrix */
    SymMat& getG() {return G;}
    /** Get Delassus matrix in sparse structure */
    const Matrix<Sparse, double>& getGs() const {return Gs;}
    /** Get Delassus matrix in sparse structure */
    Matrix<Sparse, double>& getGs() {return Gs;}
    /*! Get Delassus vector */
    const Vec& getb() const {return b;}
    /*! Get Delassus vector */
    Vec& getb() {return b;}
    /*! Get force direction matrix */
    const Mat& getW() const {return W;}
    /*! Get force direction matrix */
    Mat& getW() {return W;}
    /*! Get contact loads */
    const Vec& getla() const {return la;}
    /*! Get contact loads */
    Vec& getla() {return la;}
    /*! Get number of contact loads */
    int getlaSize() const {return laSize;}
    /*! Set index of contact loads */
    void setlaInd(int ind) {laInd = ind;}
    /*! Get gaps */
    const Vec& getg() const {return g;}
    /*! Get gaps */
    Vec& getg() {return g;}
    /*! Get number of gaps */
    int getgSize() const {return gSize;}
    /*! Set index of gaps */
    void setgInd(int ind) {gInd = ind;}
    /*! Get gap velocities */
    const Vec& getgd() const {return gd;}
    /*! Get gap velocities */
    Vec& getgd() {return gd;}
    /*! Get rFactors */
    const Vec& getrFactor() const {return rFactor;}
    /*! Get rFactors */
    Vec& getrFactor() {return rFactor;}
    /*! Get dimension of rFactor */
    int getrFactorSize() const {return rFactorSize;}
    /*! Set index of rFactor */
    void setrFactorInd(int ind) {rFactorInd = ind;}
    /*! Returns hit sphere of two partners */
    HitSphereLink* getHitSphereLink(Object* obj0, Object* obj1);

    /*! Initialises MBS */
    void init();
    /*! Initialises the state of objects and EDIs */
    void initz(Vec& z0);
    /*! Computes velocity difference for current time with state and time step */
    Vec deltau(const Vec &uParent, double t, double dt);
    /*! Updates position gap for current time with state and time step */
    Vec deltaq(const Vec &zParent, double t, double dt);
    /*! Updates order one gap for current time with state and time step */
    Vec deltax(const Vec &zParent, double t, double dt);
    /*! Updates the state depending structures for multibody system */
    void update(const Vec &zParent, double t);  
    /*! Calls updateKinematics for children */
    void updateKinematics(double t);
    /*! Calls updateT for children */
    void updateT(double t);
    /*! Calls updateM for children */
    void updateM(double t);
    /*! Cholesky factorization of mass matrix for children */
    void facLLM();
    /*! Calls updateh for children */
    void updateh(double t);
    /*! Calls updateW for children */
    void updateW(double t);
    /*! Calls updatew for children */
    void updatew(double t);
    /*! Calls updateGb for children */
    void updateGb(double t);
    /*! Calls updateLinksStage1 for children */
    virtual void updateLinksStage1(double t);
    /*! Calls updateLinksStage2 for children */
    virtual void updateLinksStage2(double t);
    /*! Decide which constraints are active */
    void checkActiveConstraints();
    /*! Set if the number of active constraints has changed */
    void setActiveConstraintsChanged(bool b) {activeConstraintsChanged = b;}	    
    /*! References multibody children to multibody parent states and external states zExt and collects data */
    void updatezRef(const Vec &zExt);
    /*! Calls updater for children */
    void updater(double t);

    /*! Solves prox-functions depending on solver settings */
    int solve(double dt);
    /*! Pointer to constraint solver function */
    int (MultiBodySystem::*solve_)(double dt);
    /*! solves constraint equations with Gauss-Seidel-Splitting scheme */
    int solveGaussSeidel(double dt);
    /*! Solves constraint equations with Cholesky decomposition */
    int solveLinearEquations(double dt); 
    /*! Solves constraint equations with single fixed point iteration */
    int solveFixpointSingle(double dt);
    /*! Solves constraint equations with total fixed point iteration */
    int solveFixpointTotal(double dt);
    /*! Solves constraint equations with generalised NewtonMethod */
    int solveRootFinding(double dt);
    /*! Calculates fixpoint residuum with numeric Jacobian for rootFinding */
    void residualProj(double dt);
    /*! Calculates fixpoint residuum with analytic Jacobian for rootFinding */
    void residualProjJac(double dt);
    using Object::addPort;
    /*! Tests if convergence is satisfied */
    void checkForTermination(double dt);
    /*! 
     * Specify whether time integration should be stopped flag(true) in case of no convergence of constraint-problem
     * or only a warning should be processed (false=default)
     * dropInfo is used for dropping contact informations to file (true)
     */
    void setStopIfNoConvergence(bool flag, bool dropInfo = false) {stopIfNoConvergence = flag; dropContactInfo=dropInfo;}
    /*! Writes a file with relevant matrices for debugging */
    void dropContactMatrices();
    /*! Should constraint forces of last steps be used for better convergence? */
    void setUseOldla(bool flag) {useOldla = flag;}
    /*! Save constraint forces from last step for better convergence */
    void savela();
    /*! Initialises constraint forces with those of last step */
    void initla();
    /*! Set scale factor for flow quantity tolerances tolQ=tol*scaleTolQ */
    void setScaleTolQ(double scaleTolQ);
    /*! Set scale factor for pressure quantity tolerances tolp=tol*scaleTolp */
    void setScaleTolp(double scaleTolp);
    /*! Set tolerance gap velocities have to fulfill during contact iteration */
    void setgdTol(double tol);
    /*! Set tolerance load has to fulfill during contact iteration */
    void setlaTol(double tol);
    /*! Set termination FLAG for constraint iteration */
    void setTermination(bool term_) {term = term_;}
    /*! Set maximum rFactor */
    void setrMax(double rMax);
    /*! Decreases rFactors for links */
    void decreaserFactors();
    /*! Sets numerical evaluation of Jacobian for Newton */
    void setNumJacProj(bool numJac_) {numJac = numJac_;}
    /*! Set maximum number of contact iterations for errors */
    void setMaxIter(int iter) {maxIter = iter;}
    /*! Set high number of contact iterations for warnings */
    void setHighIter(int iter) {highIter = iter;}
    /*! Set maximum damping steps for Newton */
    void setMaxDampingSteps(int maxDSteps) {maxDampingSteps = maxDSteps;}
    /*! Sets the Levenberg Marquardt parameters */
    void setLevenbergMarquardtParam(double lmParm_) {lmParm = lmParm_;}
    /*! Set Solver for treatment of constraint problems */
    void setSolver(Solver solver_) {solver = solver_;}
    /*! Set linear algebra for rootFinding solver */                       
    void setLinAlg(LinAlg linAlg_) {linAlg = linAlg_;}
    /*! Set rFactor strategy */                        
    void setStrategy(Strategy strategy_) {strategy = strategy_;}
    /*! Returns information for solver including strategy and linear algebra */
    string getSolverInfo();
    /*! Get numerical Jacobian in Newton */
    const SqrMat& getJprox() const {return Jprox;}
    /*! Get numerical Jacobian in Newton */
    SqrMat& getJprox() {return Jprox;}
    /*! Projects state, such that constraints are not violated */
    void projectViolatedConstraints(double t);

    /*! Preintegrate MBS */
    virtual void preInteg(MultiBodySystem *parent);
    /*! Set preintegrator */
    void setPreInteg(Integrator *preInteg_) {preIntegrator = preInteg_;}
    /*! Write state to file for preintegration */
    void writez();
    /*! Read state from file for preintegration */
    void readz0();

    /*!Define directory name for simulation output.
     * Default is Element::(fullName+i) with i used for enumeration 
     */
    void setProjectDirectory(const string &directoryName_) {directoryName = directoryName_;}
    /*! Get directory name for simulation output */
    const string& getDirectoryName() {return dirName;}
    /*! Plots interesting data */
    void plot(const Vec& z, double t, double dt=1);
    /*! Plots interesting data */
    void plot(double t, double dt=1);
    /*! Plot parameter file */
    void plotParameters();
    /*! Initialise plot files */
    void initPlotFiles();
    /*! Close plot files */
    void closePlotFiles();
    void plotParameterFiles();
    void initPlotLists();



    /*! Get ambient pressure */
    const double getpinf() const {return pinf;};
    /*! Set ambient pressure */
    void setpinf(const double pinf_) {pinf=pinf_;};
    /*! Set hydraulic fluid for system */
    void setFluid(HydFluid *fl_) {fl=fl_;}
    /*! Get hydraulic fluid of system */
    HydFluid *getFluid(){return fl;}

    /*! Get Jacobian of generalised smooth forces */
    Mat& getJh() {return Jh;}
    /*! Update Jacobian of generalised smooth forces for children */
    void updateJh(double t);

    // Calculates differentiated state for ODE integrator */
    void zdot(const Vec& z, Vec& zd, double t);
    // Calculates differentiated state for ODE integrator */
    Vec zdot(const Vec& z, double t);








    virtual void shift(Vec& z, const Vector<int>& jsv, double t) {}
    bool driftCompensation(Vec& z, double t) { return false; }

    void getsv(const Vec&, Vec&, double);
    Vec& getsv() {return sv;}
    const Vec& getsv() const {return sv;}
    Vector<int>& getjsv() {return jsv;}
    const Vector<int>& getjsv() const {return jsv;}
    int getsvSize() const {return svSize;}
    int getzSize() const {return Object::getzSize();}

    const Vec& getdla() const {return dla;}
    Vec& getdla() {return dla;}
    const Vec& gets() const {return s;}
    Vec& gets() {return s;}
    /*! Get residuum of fixpoint expression */
    const Vec& getres() const {return res;}
    /*! Get residuum of fixpoint expression */
    Vec& getres() {return res;}

    /*! Decreases rFactors in defined steps */
    void setDecreaseLevels(const Vector<int> &decreaseLevels_) {decreaseLevels = decreaseLevels_;}
    void setCheckTermLevels(const Vector<int> &checkTermLevels_) {checkTermLevels = checkTermLevels_;}

    /*! Set, if the size of G and b should be adapted */
    void setCheckGSize(bool checkGSize_) {checkGSize = checkGSize_;}
    void setLimitGSize(int limitGSize_) {limitGSize = limitGSize_; checkGSize = false;}
    /*! Initialise external data base */
    void initDataInterfaceBase();
  };

}

#endif
