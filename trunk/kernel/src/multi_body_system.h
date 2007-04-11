/* Copyright (C) 2004-2006  Martin Förg, Roland Zander, Felix Kahr
 
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

#include <string>
#include <dirent.h>
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

  /** solver for contact parameter identification */
  enum Solver {FixedPointTotal, FixedPointSingle, GaussSeidel, LinearEquations, RootFinding};
  /** r-Factor strategies */
  enum Strategy {global, local};
  /** linear algebra for root-function solver solveN */
  enum LinAlg {LUDecomposition,LevenbergMarquardt,PseudoInverse};

  /*! Comment
   *
   * */
  class MultiBodySystem : public Object {

    friend class HitSphereLink;

    protected:
    Integrator *preIntegrator;
    vector<Object*> objects;
    vector<Link*> links;
    vector<ExtraDynamicInterface*> EDI;
    vector<DataInterfaceBase*> DIBs;

    vector<Link*> linkSingleValued;
    vector<Link*> linkSetValued;
    vector<Link*> linkSetValuedActive;

    vector<HitSphereLink*> HSLinks;

    vector<Object*>                objects2plot;
    vector<Link*>                  links2plot;
    vector<Contour*>               contours2plot;
    vector<Port*>                  ports2plot;
    vector<ExtraDynamicInterface*> EDIs2plot;

    bool term;
    int gSize;
    int gInd;
    SymMat G;
    Matrix<Sparse, double> Gs;
    Mat W;
    SqrMat Jprox;
    Vec w;
    Vec b;
    Vec la;
    Vec dla;
    Vec s;
    Vec res;
    Vec rFactor;
    //Vector<int> rFactorUnsure;
    Vec g, gd;
    int laSize, laInd;
    int rFactorSize, rFactorInd;

    int svSize;
    int svInd;
    Vec sv;
    Vector<int> jsv;

    SymMat MParent;
    SymMat GParent;
    Mat WParent;
    Vec wParent;
    Vec bParent;
    Vec laParent;
    Vec dlaParent;
    Vec rFactorParent;
    Vec sParent;
    Vec resParent;
    Vec gParent;
    Vec gdParent;
    Vec zdParent;
    Vec hParent;
    Vec rParent;
    Vec fParent;
    Vec svParent;
    Vector<int> jsvParent;
    Vec grav;
    bool activeConstraintsChanged;

    int maxIter, highIter, maxDampingSteps;
    double lmParm;
    int warnLevel;
    Solver solver;
    Strategy strategy;
    LinAlg linAlg;
    bool stopIfNoConvergence;
    bool useOldla;
    bool numJac;
    Vector<int> decreaseLevels;
    Vector<int> checkTermLevels;

    int nHSLinksSetValuedFixed;
    int nHSLinksSingleValuedFixed;

    bool checkGSize;
    int limitGSize;

    void updatezRef(const Vec &zExt);
    void updatezdRef(const Vec &zdExt);
    void updateqRef(const Vec &qExt);
    void updateqdRef(const Vec &qdExt);
    void updateuRef(const Vec &uExt);
    void updatexRef(const Vec &xExt);
    void updatehRef(const Vec &hExt);
    void updaterRef(const Vec &rExt);
    void updatefRef(const Vec &fExt);
    void updateMRef(const SymMat &MExt);
    void updatesvRef(const Vec &svExt);
    void updatejsvRef(const Vector<int> &jsvExt);

    void updaterFactors();
    void updateKinematics(double t);
    void updateLinksStage1(double t);
    void updateLinksStage2(double t);
    void updateh(double t);
    void updateG(double t);
    void updater(double t);
    void updatezd(double t);
    void updatedx(double t, double dt);
    void updatedu(double t, double dt);
    void updatedq(double t, double dt);
    void computeConstraintForces(double t);
    void updateStopVector(double t);

    /** Name of directory where output is processed
    */
    string directoryName;
    /*! Test and enumerate directories for simulation output
    */
    void setDirectory();
    DIR* dir_d;

    /** hydraulic fluid, only for hydraulic systems */
    HydFluid *fl;
    /** ambient pressure, only for hydraulic systems */
    double pinf;

    public:
    MultiBodySystem();
    MultiBodySystem(const string &projectName);
    ~MultiBodySystem();

    void addMbs(MultiBodySystem* mbs);

    void init();
    void checkActiveConstraints();
    void setActiveConstraintsChanged(bool b) {activeConstraintsChanged = b;}
    virtual void preInteg(MultiBodySystem *parent);

    /*!
     * return vector of gravitational acceleration in world system
     */
    const Vec& getGrav() const {return grav;};
    /*!
     * define vector of gravitational acceleration in world system
     */
    void setGrav(const Vec& g);
    /*!
     * define MultiBodySystem::directoryName for simulation output. Default is Element::(fullName+i) with i used for enumeration
     */
    void setProjectDirectory(const string &directoryName_) {directoryName = directoryName_;}
    void setPreInteg(Integrator *preInteg_) {preIntegrator = preInteg_;}
    const string& getDirectoryName() {return dirName;}

    // Implementation of the ODERootFindingSystemInterface (Integratores)
    void zdot(const Vec& z, Vec& zd, double t);
    Vec zdot(const Vec& z, double t);
    void getsv(const Vec&, Vec&, double);
    virtual void shift(Vec& z, const Vector<int>& jsv, double t) {}

    void plot(const Vec& z, double t, double dt=1);

    void update(const Vec &zParent, double t);
    Vec deltau(const Vec &uParent, double t, double dt);
    Vec deltaq(const Vec &zParent, double t, double dt);
    Vec deltax(const Vec &zParent, double t, double dt);
    void decreaserFactors();

    void initz(Vec& z0);
    Vec& getsv() {return sv;}
    const Vec& getsv() const {return sv;}
    Vector<int>& getjsv() {return jsv;}
    const Vector<int>& getjsv() const {return jsv;}

    bool driftCompensation(Vec& z, double t) { return false; }

    void savela();
    void initla();

    void addPort(Port *port, const Vec &WrOP);
    Port* getPort(const string &name,bool check=true);
    Contour* getContour(const string &name,bool check=true);


    void addPort(const string &name, const Vec &WrSP);
    void addContour(Contour *contour, const Vec &WrOP);
    void addContour(Contour *contour, const Vec &WrOP,const SqrMat &AWC);

    void plot(double t, double dt=1);
    void plotParameters();
    void initPlotFiles();
    void closePlotFiles();

    /* ! 
       compute kinetic energy of entire multibodysystem, which is the quadratic form \f$\frac{1}{2}\boldsymbol{u}^T\boldsymbol{M}\boldsymbol{u}\f$ for all systems
     */
    double computeKineticEnergy() { return 0.5*trans(u)*M*u; }
    /* ! 
       compute potential energy of entire multibodysystem
       */
    double computePotentialEnergy();

    Object* getObject(const string &name,bool check=true);
    ExtraDynamicInterface* getEDI(const string &name);
    void addObject(Object *object);
    void addLink(Link *connection);
    void addEDI(ExtraDynamicInterface *edi_);
    /** add a data_interface_base to the DataInterfaceBase-vector */
    void addDataInterfaceBase(DataInterfaceBase* dib_);
    /** Method to add any element (Link,Object,ExtraDynamicInterface) by dynamic casting*/
    void addElement(Element *element_);
    /** returns the pointer to a data_interface_base_which is listed in the DataInterfaceBase-vector*/
    DataInterfaceBase* getDataInterfaceBase(const string &name);
    Element* getElement(const string &name); 

    HitSphereLink* getHitSphereLink(Object* obj0, Object* obj1);

    int getzSize() const {return Object::getzSize();}
    int getsvSize() const {return svSize;}
    int getrFactorSize() const {return rFactorSize;}
    int getgSize() const {return gSize;}
    int getlaSize() const {return laSize;}
    void setrFactorInd(int ind) {rFactorInd = ind;}
    void setlaInd(int ind) {laInd = ind;}
    void setgInd(int ind) {gInd = ind;}
    const SymMat& getG() const {return G;}
    SymMat& getG() {return G;}
    const Matrix<Sparse, double>& getGs() const {return Gs;}
    Matrix<Sparse, double>& getGs() {return Gs;}
    const Mat& getW() const {return W;}
    Mat& getW() {return W;}
    const SqrMat& getJprox() const {return Jprox;}
    SqrMat& getJprox() {return Jprox;}
    const Vec& getw() const {return w;}
    Vec& getw() {return w;}
    const Vec& getb() const {return b;}
    Vec& getb() {return b;}
    const Vec& getla() const {return la;}
    Vec& getla() {return la;}
    const Vec& getdla() const {return dla;}
    Vec& getdla() {return dla;}
    const Vec& gets() const {return s;}
    Vec& gets() {return s;}
    const Vec& getres() const {return res;}
    Vec& getres() {return res;}
    const Vec& getg() const {return g;}
    Vec& getg() {return g;}
    const Vec& getgd() const {return gd;}
    Vec& getgd() {return gd;}
    const Vec& geth() const {return h;}
    Vec& geth() {return h;}
    const Vec& getrFactor() const {return rFactor;}
    Vec& getrFactor() {return rFactor;}

    void setTermination(bool term_) {term = term_;}

    /*! set scale factor for Flow Quantity Tolerances tolQ=tol*scaleTolQ */
    void setScaleTolQ(double scaleTolQ);
    /*! set scale factor for Pressure Quantity Tolerances tolp=tol*scaleTolp */
    void setScaleTolp(double scaleTolp);
    void setgdTol(double tol);
    void setlaTol(double tol);
    void setrMax(double rMax);
    void setNumJacProj(bool numJac_) {numJac = numJac_;}
    void setMaxIter(int iter) {maxIter = iter;}
    void setHighIter(int iter) {highIter = iter;}
    void setMaxDampingSteps(int maxDSteps) {maxDampingSteps = maxDSteps;}
    void setLevenbergMarquardtParam(double lmParm_) {lmParm = lmParm_;}
    /*! set Solver for treatment of constraint problems
     * 		  */
    void setSolver(Solver solver_) {solver = solver_;}                         
    void setLinAlg(LinAlg linAlg_) {linAlg = linAlg_;}                         
    void setStrategy(Strategy strategy_) {strategy = strategy_;}
    /*! get info for solver including strategy and linear algebra
     * \return string holding solver name, r-factor strategie and linear algebra
     */
    string getSolverInfo();

    /*! specify wether time integration should be stopped (true) in case of no convergence of constraint-problem
     * or only a waring should be processed (flase=default)
     * \param flag true, false */
    void setStopIfNoConvergence(bool flag) {stopIfNoConvergence = flag;}
    void setUseOldla(bool flag) {useOldla = flag;}
    void setDecreaseLevels(const Vector<int> &decreaseLevels_) {decreaseLevels = decreaseLevels_;}
    void setCheckTermLevels(const Vector<int> &checkTermLevels_) {checkTermLevels = checkTermLevels_;}

    void setCheckGSize(bool checkGSize_) {checkGSize = checkGSize_;}
    void setLimitGSize(int limitGSize_) {limitGSize = limitGSize_; checkGSize = false;}

    MultiBodySystem* getMultiBodySystem() const {return mbs;};

    int solve(double dt); 
    int (MultiBodySystem::*solve_)(double dt);
    int solveGaussSeidel(double dt); 
    int solveLinearEquations(double dt); 
    int solveFixpointSingle(double dt); 
    int solveFixpointTotal(double dt); 
    int solveRootFinding(double dt); 
    void residualProj(double dt); 
    void checkForTermination(double dt); 
    void residualProjJac(double dt);

    using Object::addPort;

    //-----------
    void writez();
    void readz0();

    /*! get ambient pressure
     *  \return pinf
     * */
    const double getpinf() const {return pinf;};
    /*! set ambient pressure
     * \param pinf_
     * */
    void setpinf(const double pinf_) {pinf=pinf_;};
    /*!
     *  set hydraulic fluid for system
     *  \param fl_
     * */
    void setFluid(HydFluid *fl_) {fl=fl_;}
    /*! get hydraulic fluid of system
     *  \return fl
     * */
    HydFluid *getFluid(){return fl;}  

    void initDataInterfaceBase();
  };

}

#endif
