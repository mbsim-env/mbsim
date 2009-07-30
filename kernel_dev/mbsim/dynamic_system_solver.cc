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

#include<config.h>
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/modelling_interface.h"
#include "mbsim/frame.h"
#include "mbsim/contour.h"
#include "mbsim/link.h"
#include "mbsim/tree.h"
#include "mbsim/order_one_dynamics.h"
#include "mbsim/integrators/integrator.h"
#include "mbsim/flexible_body.h"
#include "mbsim/utils/eps.h"
#include "dirent.h"
#include <mbsim/environment.h>
#include <mbsim/objectfactory.h>

#ifndef MINGW
#  include<sys/stat.h>
#else
#  include<io.h>
#  define mkdir(a,b) mkdir(a)
#endif

#include <H5Cpp.h>
#include <hdf5serie/fileserie.h>
#include <hdf5serie/simpleattribute.h>

#ifdef HAVE_ANSICSIGNAL
#  include <signal.h>
#endif

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  bool DynamicSystemSolver::exitRequest=false;

  DynamicSystemSolver::DynamicSystemSolver() : Group("Default"), maxIter(10000), highIter(1000), maxDampingSteps(3), lmParm(0.001), contactSolver(FixedPointSingle), impactSolver(FixedPointSingle), strategy(local), linAlg(LUDecomposition), stopIfNoConvergence(false), dropContactInfo(false), useOldla(true), numJac(false), checkGSize(true), limitGSize(500), warnLevel(0), preIntegrator(NULL), peds(false), impact(false), sticking(false), k(1), reorganizeHierarchy(false) { 

    constructor();
  } 

  DynamicSystemSolver::DynamicSystemSolver(const string &projectName) : Group(projectName), maxIter(10000), highIter(1000), maxDampingSteps(3), lmParm(0.001), contactSolver(FixedPointSingle), impactSolver(FixedPointSingle), strategy(local), linAlg(LUDecomposition), stopIfNoConvergence(false), dropContactInfo(false), useOldla(true), numJac(false), checkGSize(true), limitGSize(500), warnLevel(0), preIntegrator(NULL), peds(false), impact(false), sticking(false), k(1), reorganizeHierarchy(false) { 

    constructor();
  }

  DynamicSystemSolver::~DynamicSystemSolver() {
    if (preIntegrator) delete preIntegrator;
  } 

  void DynamicSystemSolver::init() {
#ifdef _OPENMP
    omp_set_nested(true);
#endif

    Group::preinit();

    if(reorganizeHierarchy) {
      cout <<name << " (special group) preinit():" << endl;

      vector<Object*>  objList;
      vector<Link*>  lnkList;
      vector<OrderOneDynamics*>  oodList;
      buildListOfObjects(objList,true);
      buildListOfLinks(lnkList,true);
      buildListOfOrderOneDynamics(oodList,true);

      vector<ModellingInterface*> modellList;
      buildListOfModels(modellList,true);
      if(modellList.size())
        do {
          modellList[0]->processModellList(modellList,objList,lnkList);
        } while(modellList.size());

      link.clear(); // Alte link-Liste löschen
      orderOneDynamics.clear(); // Alte ood-Liste löschen
      cout << "object List:" << endl;
      for(unsigned int i=0; i<objList.size(); i++) {
        cout << objList[i]->getName() << endl;
        stringstream str;
        str << objList[i]->getName() << "#" << i;
        objList[i]->setName(str.str());
      }
      cout << "link List:" << endl;
      for(unsigned int i=0; i<lnkList.size(); i++) {
        cout << lnkList[i]->getName() << endl;
        stringstream str;
        str << lnkList[i]->getName() << "#" << i;
        lnkList[i]->setName(str.str());
        addLink(lnkList[i]);
      }
      cout << "ood List:" << endl;
      for(unsigned int i=0; i<oodList.size(); i++) {
        cout << oodList[i]->getName() << endl;
        stringstream str;
        str << oodList[i]->getName() << "#" << i;
        oodList[i]->setName(str.str());
        addOrderOneDynamics(oodList[i]);
      }
      // Matrix anlegen, zeigt die Abhängigkeiten der Körper
      SqrMat A(objList.size());
      for(unsigned int i=0; i<objList.size(); i++) {

        Object* parentBody = objList[i]->getObjectDependingOn();

        if(parentBody) { // Körper hat Relativkinematik
          unsigned int j=0;
          bool foundBody = false;
          for(unsigned int k=0; k<objList.size(); k++, j++) {
            if(objList[k] == parentBody) {
              foundBody = true;
              break;
            }
          }

          if(foundBody) {
            A(i,j) = 2; // 2 bedeuted Vorgänger
            A(j,i) = 1; // 1 bedeuted Nachfolger
          }
        }
      }
      // Matrix der Abhängigkeiten
      //cout << "A=" << A << endl;
      // Tree Liste
      vector<Tree*> bufTree;
      int nt = 0;
      // Lege unsichtbare Group an 
      Group* group = new Group("InvisibleGroup");
      object.clear(); // Alte Object-Liste löschen
      dynamicsystem.clear(); // Alte DynamicSystem-Liste löschen
      // Starte Aufbau
      for(int i=0; i<A.size(); i++) {
        double a = max(trans(A).col(i));
        if(a==1) { // Root einer Relativkinematik
          stringstream str;
          str << "InvisibleTree" << nt++;
          // Lege unsichtbaren Tree an 
          Tree *tree = new Tree(str.str());
          bufTree.push_back(tree);
          addToTree(tree, 0, A, i, objList);
        } 
        else if(a==0) // Absolutkinematik
          addObject(objList[i]);
        //group->addObject(objList[i]);
      }

      addDynamicSystem(group);

      for(unsigned int i=0; i<bufTree.size(); i++) {
        addDynamicSystem(bufTree[i]);
      }

      cout << "End of special group preinit()" << endl;
    }

    calcqSize();
    calcuSize(0);
    calcuSize(1);
    sethSize(uSize[0]);
    sethSize(uSize[1],1);
    checkForConstraints(); // TODO for preinit
    setUpLinks();
    setDynamicSystemSolver(this);

    calcxSize();

    calclaSize();
    calcgSize();
    calcgdSize();
    calcrFactorSize();
    calcsvSize();
    svSize += 1; // TODO additional event for drift 

    cout << "qSize = " << qSize <<endl;
    cout << "uSize[0] = " << uSize[0] <<endl;
    cout << "xSize = " << xSize <<endl;
    cout << "gSize = " << gSize <<endl;
    cout << "gdSize = " << gdSize <<endl;
    cout << "laSize = " << laSize <<endl;
    cout << "svSize = " << svSize <<endl;
    cout << "hSize[0] = " << hSize[0] <<endl;

    cout << "uSize[1] = " << uSize[1] <<endl;
    cout << "hSize[1] = " << hSize[1] <<endl;

    if(uSize[0] == uSize[1]) 
      zdot_ = &DynamicSystemSolver::zdotStandard;
    else
      zdot_ = &DynamicSystemSolver::zdotResolveConstraints;

    setlaIndDS(laInd);

    // TODO memory problem with many contacts
    if(laSize>8000)
      laSize=8000;
    MParent.resize(getuSize(1));
    TParent.resize(getqSize(),getuSize());
    LLMParent.resize(getuSize(1));
    WParent.resize(getuSize(1),getlaSize());
    VParent.resize(getuSize(1),getlaSize());
    wbParent.resize(getlaSize());
    laParent.resize(getlaSize());
    rFactorParent.resize(getlaSize());
    sParent.resize(getlaSize());
    if(impactSolver==RootFinding) resParent.resize(getlaSize());
    gParent.resize(getgSize());
    gdParent.resize(getgdSize());
    zdParent.resize(getzSize());
    hParent.resize(getuSize(1));
    hObjectParent.resize(getuSize(1));
    hLinkParent.resize(getuSize(1));
    dhdqObjectParent.resize(getuSize(1),getqSize());
    dhdqLinkParent.resize(getuSize(1),getqSize());
    dhduObjectParent.resize(getuSize(1));
    dhduLinkParent.resize(getuSize(1));
    dhdtObjectParent.resize(getuSize(1));
    dhdtLinkParent.resize(getuSize(1));
    rParent.resize(getuSize());
    fParent.resize(getxSize());
    svParent.resize(getsvSize());
    jsvParent.resize(getsvSize());

    updateMRef(MParent);
    updateTRef(TParent);
    updateLLMRef(LLMParent);

    Group::init();

    updatesvRef(svParent);
    updatejsvRef(jsvParent);
    updatezdRef(zdParent);
    updatelaRef(laParent);
    updategRef(gParent);
    updategdRef(gdParent);
    updatehRef(hParent,hObjectParent,hLinkParent);
    updatedhdqRef(dhdqObjectParent,dhdqLinkParent);
    updatedhduRef(dhduObjectParent,dhduLinkParent);
    updatedhdtRef(dhdtObjectParent,dhdtLinkParent);
    updaterRef(rParent);
    updateWRef(WParent);
    updateVRef(VParent);
    updatewbRef(wbParent);
    updategRef(gParent);
    updategdRef(gdParent);
    updatelaRef(laParent);
    if(impactSolver==RootFinding) updateresRef(resParent);
    updaterFactorRef(rFactorParent);

    // contact solver specific settings
    cout << "  use contact solver \'" << getSolverInfo() << "\' for contact situations" << endl;
    if(contactSolver == GaussSeidel) solveConstraints_ = &DynamicSystemSolver::solveConstraintsGaussSeidel; 
    else if(contactSolver == LinearEquations) {
      solveConstraints_ = &DynamicSystemSolver::solveConstraintsLinearEquations;
      cout << "WARNING: solveLL is only valid for bilateral constrained systems!" << endl;
    }
    else if(contactSolver == FixedPointSingle) solveConstraints_ = &DynamicSystemSolver::solveConstraintsFixpointSingle;
    else if(contactSolver == RootFinding)solveConstraints_ = &DynamicSystemSolver::solveConstraintsRootFinding;
    else throw new MBSimError("ERROR (DynamicSystemSolver::init()): Unknown contact solver");

    // impact solver specific settings
    cout << "  use impact solver \'" << getSolverInfo() << "\' for impact situations" << endl;
    if(impactSolver == GaussSeidel) solveImpacts_ = &DynamicSystemSolver::solveImpactsGaussSeidel; 
    else if(impactSolver == LinearEquations) {
      solveImpacts_ = &DynamicSystemSolver::solveImpactsLinearEquations;
      cout << "WARNING: solveLL is only valid for bilateral constrained systems!" << endl;
    }
    else if(impactSolver == FixedPointSingle) solveImpacts_ = &DynamicSystemSolver::solveImpactsFixpointSingle;
    else if(impactSolver == RootFinding)solveImpacts_ = &DynamicSystemSolver::solveImpactsRootFinding;
    else throw new MBSimError("ERROR (DynamicSystemSolver::init()): Unknown impact solver");

    cout << "  initialising plot-files ..." << endl;
    initPlot();

    cout << "...... done initialising." << endl << endl;

#ifdef HAVE_ANSICSIGNAL
    signal(SIGINT, sigInterruptHandler);
    signal(SIGTERM, sigInterruptHandler);
    signal(SIGABRT, sigAbortHandler);
#endif

  }

  int DynamicSystemSolver::solveConstraintsFixpointSingle() {
    updaterFactors();

    checkConstraintsForTermination();
    if(term) return 0;

    int iter, level = 0;
    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {

      if(level < decreaseLevels.size() && iter > decreaseLevels(level)) {
        level++;
        decreaserFactors();
        cout <<endl<< "Warning: decreasing r-factors at iter = " << iter << endl;
        if(warnLevel>=2) cout <<endl<< "Warning: decreasing r-factors at iter = " << iter << endl;
      }

      Group::solveConstraintsFixpointSingle();

      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkConstraintsForTermination();
        if(term) break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveImpactsFixpointSingle(double dt) {
    updaterFactors();

    checkImpactsForTermination();
    if(term) return 0;

    int iter, level = 0;
    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {

      if(level < decreaseLevels.size() && iter > decreaseLevels(level)) {
        level++;
        decreaserFactors();
        cout <<endl<< "Warning: decreasing r-factors at iter = " << iter << endl;
        if(warnLevel>=2) cout <<endl<< "Warning: decreasing r-factors at iter = " << iter << endl;
      }

      Group::solveImpactsFixpointSingle();

      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkImpactsForTermination();
        if(term) break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveConstraintsGaussSeidel() {
    checkConstraintsForTermination();
    if(term) return 0 ;

    int iter;
    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {
      Group::solveConstraintsGaussSeidel();
      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkConstraintsForTermination();
        if(term) break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveImpactsGaussSeidel(double dt) {
    checkImpactsForTermination();
    if(term) return 0 ;

    int iter;
    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {
      Group::solveImpactsGaussSeidel();
      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkImpactsForTermination();
        if(term) break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveConstraintsRootFinding() {
    updaterFactors();

    int iter;
    int checkTermLevel = 0;

    Group::solveConstraintsRootFinding(); 
    double nrmf0 = nrm2(res);
    Vec res0 = res.copy();

    checkConstraintsForTermination();
    if(term)
      return 0 ;

    DiagMat I(la.size(),INIT,1);
    for(iter=1; iter<maxIter; iter++) {

      if(Jprox.size() != la.size()) Jprox.resize(la.size(),NONINIT);

      if(numJac) {
        double dx, xj;

        for(int j=0; j<la.size(); j++) {
          xj = la(j);

          dx = (epsroot() * 0.5);
          do dx += dx;
          while (xj + dx == la(j));

          la(j) += dx;
          Group::solveConstraintsRootFinding(); 
          la(j) = xj;
          Jprox.col(j) = (res-res0)/dx;
        }
      } 
      else jacobianConstraints();
      Vec dx;
      if(linAlg == LUDecomposition) dx >> slvLU(Jprox,res0);
      else if(linAlg == LevenbergMarquardt) {
        SymMat J = SymMat(JTJ(Jprox) + lmParm*I);
        dx >> slvLL(J,trans(Jprox)*res0);
      }
      else if(linAlg == PseudoInverse) dx >> slvLS(Jprox,res0);
      else throw 5;

      double alpha = 1;       

      Vec La_old = la.copy();

      double nrmf = 1;
      for (int k=0; k<maxDampingSteps; k++) {
        la = La_old - alpha*dx;
        solveConstraintsRootFinding();
        nrmf = nrm2(res);
        if(nrmf < nrmf0) break;

        alpha = 0.5*alpha;  
      }
      nrmf0 = nrmf;
    res0 = res;

    if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
      checkTermLevel++;
      checkConstraintsForTermination();
      if(term) break;
    }
  }
  return iter;
}

int DynamicSystemSolver::solveImpactsRootFinding(double dt) {
  updaterFactors();

  int iter;
  int checkTermLevel = 0;

  Group::solveImpactsRootFinding(); 
  double nrmf0 = nrm2(res);
  Vec res0 = res.copy();

    checkImpactsForTermination();
    if(term)
      return 0 ;

    DiagMat I(la.size(),INIT,1);
    for(iter=1; iter<maxIter; iter++) {

      if(Jprox.size() != la.size()) Jprox.resize(la.size(),NONINIT);

      if(numJac) {
        double dx, xj;

        for(int j=0; j<la.size(); j++) {
          xj = la(j);

          dx = (epsroot() * 0.5);
          do dx += dx;
          while (xj + dx == la(j));

          la(j) += dx;
          Group::solveImpactsRootFinding(); 
          la(j) = xj;
          Jprox.col(j) = (res-res0)/dx;
        }
      } 
      else jacobianImpacts();
      Vec dx;
      if(linAlg == LUDecomposition) dx >> slvLU(Jprox,res0);
      else if(linAlg == LevenbergMarquardt) {
        SymMat J = SymMat(JTJ(Jprox) + lmParm*I);
        dx >> slvLL(J,trans(Jprox)*res0);
      }
      else if(linAlg == PseudoInverse) dx >> slvLS(Jprox,res0);
      else throw 5;

      double alpha = 1;       

      Vec La_old = la.copy();

      double nrmf = 1;
      for (int k=0; k<maxDampingSteps; k++) {
        la = La_old - alpha*dx;
        solveImpactsRootFinding();
        nrmf = nrm2(res);
        if(nrmf < nrmf0) break;

        alpha = 0.5*alpha;  
      }
      nrmf0 = nrmf;
      res0 = res;

      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkImpactsForTermination();
        if(term) break;
      }
    }
    return iter;
  }

  void DynamicSystemSolver::checkConstraintsForTermination() {
    term = true;
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) { 
      (*i)->checkConstraintsForTermination(); 
      if(term == false) return;
    }

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (**i).checkConstraintsForTermination();
      if(term == false) return;
    }
  }

  void DynamicSystemSolver::checkImpactsForTermination() {
    term = true;
    for(vector<DynamicSystem*>::iterator i = dynamicsystem.begin(); i != dynamicsystem.end(); ++i) {
      (*i)->checkImpactsForTermination(); 
      if(term == false) return;
    }

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (**i).checkImpactsForTermination();
      if(term == false) return;
    }
  }

  void DynamicSystemSolver::initPlot() {
    Group::initPlot();
#ifdef HAVE_OPENMBVCPPINTERFACE
    openMBVGrp->initialize();
#endif
  }

  void DynamicSystemSolver::updateh(double t) {
    h.init(0);
    hObject.init(0);
    hLink.init(0);
    Group::updateh(t);
  }

  void DynamicSystemSolver::updatedhdz(double t) {
    h.init(0);
    hObject.init(0);
    hLink.init(0);
    dhdqObject.init(0);
    dhdqLink.init(0);
    dhduObject.init(0);
    dhduLink.init(0);
    dhdtObject.init(0);
    dhdtLink.init(0);
    Group::updatedhdz(t);
  }

  void DynamicSystemSolver::updateM(double t) {
    M.init(0);
    Group::updateM(t);
  }

  void DynamicSystemSolver::updateStateDependentVariables(double t) {
    Group::updateStateDependentVariables(t);

    // if the integrator has not exit after a integratorExitRequest, exit the hard way
    if(integratorExitRequest) {
      cout<<"MBSim: Integrator has not stopped integration! Terminate NOW the hard way!"<<endl;
      H5::FileSerie::deletePIDFiles();
      _exit(1);
    }
    // on exitRequest flush plot files and ask the integrator to exit
    if(exitRequest) {
      cout<<"MBSim: Flushing HDF5 files and ask integrator to terminate!"<<endl;
      H5::FileSerie::flushAllFiles();
      integratorExitRequest=true;
    }

    // flush files ones if requested
    if(H5::FileSerie::getFlushOnes()) H5::FileSerie::flushAllFiles();
  }

  void DynamicSystemSolver::updater(double t) {
    r = V*la;
  }

  void DynamicSystemSolver::updatewb(double t) {
    wb.init(0);
    Group::updatewb(t);
  }

  void DynamicSystemSolver::updateW(double t) {
    W.init(0);
    Group::updateW(t);
  }

  void DynamicSystemSolver::updateV(double t) {
    V = W;
    Group::updateV(t);
  }

  void DynamicSystemSolver::plot(const Vec& zParent, double t, double dt) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }

    if(qd()!=zdParent()) 
      updatezdRef(zdParent);

    updateStateDependentVariables(t);
    updateg(t);
    updategd(t);
    updateJacobians(t);
    updateT(t);
    updateh(t); 
    updateM(t); 
    facLLM(); 
    updateW(t); 

    plot(t,dt);
  }

  void DynamicSystemSolver::closePlot() {
    if(getPlotFeature(plotRecursive)==enabled) {
      Group::closePlot();
    }
  }

  void DynamicSystemSolver::preInteg(DynamicSystemSolver *parent) {
    throw new MBSimError("ERROR (DynamicSystemSolver::preInteg()): Preintegrator not implemented yet.");
    //    if(preIntegrator) {
    ////      setProjectDirectory(name+".preInteg");
    //      setAccelerationOfGravity(parent->getAccelerationOfGravity()); // TODO in preintegration gravitation of MBS parent has to be set already
    //      cout << "Initialisation of " << name << " for Preintegration..."<<endl;
    //      init();  
    //      cout << "Preintegration..."<<endl;
    //      preIntegrator->integrate(*this);
    //      closePlot();
    //      writez();
    //      delete preIntegrator;
    //      preIntegrator=NULL; 
    //      cout << "Finished." << endl;
    //    }  
  }

  int DynamicSystemSolver::solveConstraints() {
    if(la.size()==0) return 0;

    if(useOldla)initla();
    else la.init(0);

    int iter;
    Vec laOld;
    laOld = la;
    iter = (this->*solveConstraints_)(); // solver election
    if(iter >= maxIter) {
      cout << endl;
      cout << "Iterations: " << iter << endl;
      cout << "\nError: no convergence."<< endl;
      if(stopIfNoConvergence) {
        if(dropContactInfo) dropContactMatrices();
        assert(iter < maxIter);
      }
      cout << "Anyway, continuing integration..."<< endl;
    }

    if(warnLevel>=1 && iter>highIter)
      cerr <<endl<< "Warning: high number of iterations: " << iter << endl;

    if(useOldla) savela();

    return iter;
  }

  int DynamicSystemSolver::solveImpacts(double dt) {
    if(la.size()==0) return 0;

    if(useOldla)initla();
    else la.init(0);

    int iter;
    Vec laOld;
    laOld = la;
    iter = (this->*solveImpacts_)(dt); // solver election
    if(iter >= maxIter) {
      cout << endl;
      cout << "Iterations: " << iter << endl;
      cout << "\nError: no convergence."<< endl;
      if(stopIfNoConvergence) {
        if(dropContactInfo) dropContactMatrices();
        assert(iter < maxIter);
      }
      cout << "Anyway, continuing integration..."<< endl;
    }

    if(warnLevel>=1 && iter>highIter)
      cerr <<endl<< "Warning: high number of iterations: " << iter << endl;

    if(useOldla) savela();

    return iter;
  }

  void DynamicSystemSolver::computeInitialCondition() {
    updateStateDependentVariables(0);
    updateg(0);
    checkActiveg();
    updategd(0);
    checkActivegd();
    checkActiveLinks();
    updateJacobians(0);
    calclaSize();
    calcrFactorSize();
    setlaIndDS(laInd);
    updateWRef(WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
    updateVRef(VParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
    updatelaRef(laParent(0,laSize-1));
    updatewbRef(wbParent(0,laSize-1));
    updaterFactorRef(rFactorParent(0,rFactorSize-1));
  }

  Vec DynamicSystemSolver::deltau(const Vec &zParent, double t, double dt) {
    if(q()!=zParent()) updatezRef(zParent);

    updater(t); // TODO update should be outside
    updatedu(t,dt);
    return ud;
  }

  Vec DynamicSystemSolver::deltaq(const Vec &zParent, double t, double dt) {
    if(q()!=zParent()) updatezRef(zParent);
    updatedq(t,dt);

    return qd;
  }

  Vec DynamicSystemSolver::deltax(const Vec &zParent, double t, double dt) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    updatedx(t,dt);
    return xd;
  }

  void DynamicSystemSolver::initz(Vec& z) {
    updatezRef(z);
    Group::initz();
  }

  int DynamicSystemSolver::solveConstraintsLinearEquations() {
    la = slvLS(G,-(trans(W)*slvLLFac(LLM,h) + wb));
    return 1;
  }

  int DynamicSystemSolver::solveImpactsLinearEquations(double dt) {
    la = slvLS(G,-(gd + trans(W)*slvLLFac(LLM,h)*dt));
    return 1;
  }

  void DynamicSystemSolver::updateG(double t) {
    G.resize() = SqrMat(trans(W)*slvLLFac(LLM,V)); 

    if(checkGSize) Gs.resize();
    else if(Gs.cols() != G.size()) {
      static double facSizeGs = 1;
      if(G.size()>limitGSize && facSizeGs == 1) facSizeGs = double(countElements(G))/double(G.size()*G.size())*1.5;
      Gs.resize(G.size(),G.size(),int(G.size()*G.size()*facSizeGs));
    }
    Gs << G;
  }

  void DynamicSystemSolver::decreaserFactors() {

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->decreaserFactors();
  }

  void DynamicSystemSolver::update(const Vec &zParent, double t) {
    if(q()!=zParent()) updatezRef(zParent);

    updateStateDependentVariables(t);
    updateg(t);
    checkActiveg();
    checkActiveLinks();
    if(gActiveChanged()) {

      // checkAllgd(); // TODO necessary?
      calcgdSizeActive();
      calclaSize();
      calcrFactorSize();

      setlaIndDS(laInd);

      updateWRef(WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updateVRef(VParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updatelaRef(laParent(0,laSize-1));
      updategdRef(gdParent(0,gdSize-1));
      if(impactSolver==RootFinding) updateresRef(resParent(0,laSize-1));
      updaterFactorRef(rFactorParent(0,rFactorSize-1));

    }
    updategd(t);

    updateT(t); 
    updateJacobians(t);
    updateh(t); 
    updateM(t); 
    facLLM(); 
    updateW(t); 
    updateV(t); 
    updateG(t); 
  }

  void DynamicSystemSolver::shift(Vec &zParent, const Vector<int> &jsv_, double t) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    jsv = jsv_;

    if(jsv(sv.size()-1)) { // projection
      k++;
      updateStateDependentVariables(t);
      updateg(t);
      updateT(t); 
      updateJacobians(t);
      updateM(t); 
      facLLM(); 
      updateW(t); 
      projectGeneralizedPositions(t);
      updategd(t);
      updateT(t); 
      updateJacobians(t);
      updateM(t); 
      facLLM(); 
      updateW(t); 
      projectGeneralizedVelocities(t);
    }

    updateStateDependentVariables(t);
    updateg(t);
    //updategd(t); // TODO necessary?
    updateT(t); 
    updateJacobians(t);
    //updateh(t); // TODO necessary?
    updateM(t); 
    facLLM(); 
    updateW(t); 
    //updateG(t); // TODO necessary?

    projectGeneralizedPositions(t);

    updateCondition(); // decide which constraints should be added and deleted
    checkActiveLinks(); // list with active links (g<=0)

    if(impact) { // impact

      checkAllgd(); // look at all closed contacts
      calcgdSizeActive();
      updategdRef(gdParent(0,gdSize-1));
      calclaSize();
      calcrFactorSize();
      setlaIndDS(laInd);
      updateWRef(WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updateVRef(VParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updatelaRef(laParent(0,laSize-1));
      updaterFactorRef(rFactorParent(0,rFactorSize-1));

      updateStateDependentVariables(t); // TODO necessary?
      updateg(t); // TODO necessary?
      updategd(t); // important because of updategdRef
      updateJacobians(t);
      updateW(t); // important because of updateWRef
      updateV(t); 
      updateG(t); 

      projectGeneralizedPositions(t);
      b.resize() = gd; // b = gd + trans(W)*slvLLFac(LLM,h)*dt with dt=0
      int iter;
      iter = solveImpacts();
      u += deltau(zParent,t,0);

      //saveActive();
      checkActivegdn(); // which contacts open / close after impact?
      checkActiveLinks(); 

      //calcgdSize(); no invocation allowed 
      //updategdRef(); no invocation allowed 

      calclaSize();
      calcrFactorSize();
      setlaIndDS(laInd);
      updateWRef(WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updateVRef(VParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updatelaRef(laParent(0,laSize-1));
      updatewbRef(wbParent(0,laSize-1));
      updaterFactorRef(rFactorParent(0,rFactorSize-1));

      if(laSize) {

        updateStateDependentVariables(t); // necessary because of velocity change 
        updategd(t); // necessary because of velocity change 
        updateJacobians(t);
        updateh(t); 
        updateW(t); 
        updateV(t); 
        updateG(t); 
        updatewb(t); 
        b.resize() = trans(W)*slvLLFac(LLM,h) + wb;
        int iter;
        iter = solveConstraints();
        checkActivegdd();
        checkActiveLinks();
        calclaSize();
        calcrFactorSize();
        setlaIndDS(laInd);
        updateWRef(WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
        updateVRef(VParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
        updatelaRef(laParent(0,laSize-1));
        updatewbRef(wbParent(0,laSize-1));
        updaterFactorRef(rFactorParent(0,rFactorSize-1));
      }
    } 
    else if(sticking) { // sticking->slip

      calclaSize();
      calcrFactorSize();
      setlaIndDS(laInd);
      updateWRef(WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updateVRef(VParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updatelaRef(laParent(0,laSize-1));
      updatewbRef(wbParent(0,laSize-1));
      updaterFactorRef(rFactorParent(0,rFactorSize-1));

      if(laSize) {
        updateStateDependentVariables(t); // TODO necessary
        updateg(t); // TODO necessary
        updategd(t); // TODO necessary
        updateT(t);  // TODO necessary
        updateJacobians(t);
        updateh(t);  // TODO necessary
        updateM(t);  // TODO necessary
        facLLM();  // TODO necessary 
        updateW(t);  // TODO necessary
        updateV(t);  // TODO necessary
        updateG(t);  // TODO necessary 
        updatewb(t);  // TODO necessary 
        b.resize() = trans(W)*slvLLFac(LLM,h) + wb;
        int iter;
        iter = solveConstraints();
        checkActivegdd();
        checkActiveLinks();
        calclaSize();
        calcrFactorSize();
        setlaIndDS(laInd);
        updateWRef(WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
        updateVRef(VParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
        updatelaRef(laParent(0,laSize-1));
        updatewbRef(wbParent(0,laSize-1));
        updaterFactorRef(rFactorParent(0,rFactorSize-1));
      }
    } 
    else { // contact opens
      checkActiveLinks();
      calclaSize();
      calcrFactorSize();
      setlaIndDS(laInd);
      updateWRef(WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updateVRef(VParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updatelaRef(laParent(0,laSize-1));
      updatewbRef(wbParent(0,laSize-1));
      updaterFactorRef(rFactorParent(0,rFactorSize-1));
    }
    impact = false;
    sticking = false;
    updateStateDependentVariables(t);
    updateg(t);
    //updategd(t); TODO
    updateT(t); 
    updateJacobians(t);
    //updateh(t); TODO
    updateM(t); 
    facLLM(); 
    updateW(t); 
    //updateG(t); TODO

    projectGeneralizedPositions(t);
    updategd(t);
    updateT(t); 
    updateJacobians(t);
    //updateh(t); TODO
    updateM(t); 
    facLLM(); 
    updateW(t); 
    projectGeneralizedVelocities(t);
  }

  void DynamicSystemSolver::zdot(const Vec &zParent, Vec &zdParent, double t) {
    if(qd()!=zdParent()) {
      updatezdRef(zdParent);
    }
    zdot(zParent,t);
  }

  void DynamicSystemSolver::getsv(const Vec& zParent, Vec& svExt, double t) { 
    if(sv()!=svExt()) {
      updatesvRef(svExt);
    }

    if(q()!=zParent())
      updatezRef(zParent);

    if(qd()!=zdParent()) 
      updatezdRef(zdParent);

    updateStateDependentVariables(t);
    updateg(t);
    updategd(t);
    updateT(t); 
    updateJacobians(t);
    updateh(t); 
    updateM(t); 
    facLLM(); 
    if(laSize) {
      updateW(t); 
      updateV(t); 
      updateG(t); 
      updatewb(t); 
      computeConstraintForces(t); 
    }
    updateStopVector(t);
    sv(sv.size()-1) = k*1e-1-t; 
  }

  void DynamicSystemSolver::projectGeneralizedPositions(double t) {
    if(laSize) {
      calcgSizeActive();
      updategRef(gParent(0,gSize-1));
      updateg(t);
      Vec nu(getuSize());
      calclaSizeForActiveg();
      updateWRef(WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      updateW(t);
      Vec corr;
      corr = g;
      corr.init(1e-14);
      SqrMat Gv= SqrMat(trans(W)*slvLLFac(LLM,W)); 
      // TODO: Wv*T check
      while(nrmInf(g-corr) >= 1e-14) {
        Vec mu = slvLS(Gv, -g+trans(W)*nu+corr);
        Vec dnu = slvLLFac(LLM,W*mu)-nu;
        nu += dnu;
        q += T*dnu;
        updateStateDependentVariables(t);
        updateg(t);
      }
      calclaSize();
      updateWRef(WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1)));
      calcgSize();
      updategRef(gParent(0,gSize-1));
    }
  }

  void DynamicSystemSolver::projectGeneralizedVelocities(double t) {
    if(laSize) {
      calcgdSizeActive();
      updategdRef(gdParent(0,gdSize-1));
      updategd(t);
      Vec nu(getuSize());

      SqrMat Gv= SqrMat(trans(W)*slvLLFac(LLM,W)); 
      Vec mu = slvLS(Gv,-gd);
      u += slvLLFac(LLM,W*mu);
      calcgdSize();
      updategdRef(gdParent(0,gdSize-1));
    }
  }

  void DynamicSystemSolver::savela() {
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).savela();
  }

  void DynamicSystemSolver::initla() {
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).initla();
  }

  double DynamicSystemSolver::computePotentialEnergy() {
    double Vpot = 0.0;

    vector<Object*>::iterator i;
    for(i = object.begin(); i != object.end(); ++i) Vpot += (**i).computePotentialEnergy();

    vector<Link*>::iterator ic;
    for(ic = link.begin(); ic != link.end(); ++ic) Vpot += (**ic).computePotentialEnergy();
    return Vpot;
  }

  void DynamicSystemSolver::addElement(Element *element_) {
    Object* object_=dynamic_cast<Object*>(element_);
    Link* link_=dynamic_cast<Link*>(element_);
    OrderOneDynamics* ood_=dynamic_cast<OrderOneDynamics*>(element_);
    if(object_) addObject(object_);
    else if(link_) addLink(link_);
    else if(ood_) addOrderOneDynamics(ood_);
    else{ throw new MBSimError("ERROR (DynamicSystemSolver: addElement()): No such type of Element to add!");}
  }

  Element* DynamicSystemSolver::getElement(const string &name) {
    //   unsigned int i1;
    //   for(i1=0; i1<object.size(); i1++) {
    //     if(object[i1]->getName() == name) return (Element*)object[i1];
    //   }
    //   for(i1=0; i1<object.size(); i1++) {
    //     if(object[i1]->getPath() == name) return (Element*)object[i1];
    //   }
    //   unsigned int i2;
    //   for(i2=0; i2<link.size(); i2++) {
    //     if(link[i2]->getName() == name) return (Element*)link[i2];
    //   }
    //   for(i2=0; i2<link.size(); i2++) {
    //     if(link[i2]->getPath() == name) return (Element*)link[i2];
    //   }
    //   unsigned int i3;
    //   for(i3=0; i3<orderOneDynamics.size(); i3++) {
    //     if(orderOneDynamics[i3]->getName() == name) return (Element*)orderOneDynamics[i3];
    //   }
    //   for(i3=0; i3<orderOneDynamics.size(); i3++) {
    //     if(orderOneDynamics[i3]->getPath() == name) return (Element*)orderOneDynamics[i3];
    //   }
    //   if(!(i1<object.size())||!(i2<link.size())||!(i3<orderOneDynamics.size())) cout << "Error: The DynamicSystemSolver " << this->name <<" comprises no element " << name << "!" << endl; 
    //   assert(i1<object.size()||i2<link.size()||!(i3<orderOneDynamics.size()));
    return NULL;
  }

  string DynamicSystemSolver::getSolverInfo() {
    stringstream info;

    if(impactSolver == GaussSeidel) info << "GaussSeidel";
    else if(impactSolver == LinearEquations) info << "LinearEquations";
    else if(impactSolver == FixedPointSingle) info << "FixedPointSingle";
    else if(impactSolver == FixedPointTotal) info << "FixedPointTotal";
    else if(impactSolver == RootFinding) info << "RootFinding";

    // Gauss-Seidel & solveLL do not depend on the following ...
    if(impactSolver!=GaussSeidel && impactSolver!=LinearEquations) {
      info << "(";

      // r-Factor strategy
      if(strategy==global) info << "global";
      else if(strategy==local) info << "local";

      // linear algebra for RootFinding only
      if(impactSolver == RootFinding) {
        info << ",";
        if(linAlg==LUDecomposition) info << "LU";
        else if(linAlg==LevenbergMarquardt) info << "LM";
        else if(linAlg==PseudoInverse) info << "PI";
      }
      info << ")";
    }
    return info.str();
  }

  void DynamicSystemSolver::dropContactMatrices() {
    cout << "dropping contact matrices to file <dump_matrices.asc>" << endl;
    ofstream contactDrop("dump_matrices.asc");   

    contactDrop << "constraint functions g" << endl << trans(g) << endl << endl;
    contactDrop << endl;
    contactDrop << "mass matrix M" << endl << M << endl << endl;
    contactDrop << "generalized force directions W" << endl << W << endl << endl;
    contactDrop << "Delassus matrix G" << endl << G << endl << endl;
    contactDrop << endl;
    contactDrop << "constraint velocities gp" << endl << trans(gd) << endl << endl;
    contactDrop << "non-holonomic part in gp; b" << endl << trans(b) << endl << endl;
    contactDrop << "Lagrange multipliers la" << endl << trans(la) << endl << endl;
    contactDrop.close();
  }

  void DynamicSystemSolver::initDataInterfaceBase() {
    vector<Link*>::iterator il1;
    for(il1 = link.begin(); il1 != link.end(); ++il1) (*il1)->initDataInterfaceBase(this);
    vector<Object*>::iterator io1;
    for(io1 = object.begin(); io1 != object.end(); ++io1) (*io1)->initDataInterfaceBase(this);
    vector<OrderOneDynamics*>::iterator ie1;
    for(ie1 = orderOneDynamics.begin(); ie1 != orderOneDynamics.end(); ++ie1) (*ie1)->initDataInterfaceBase(this); 
  }

  void DynamicSystemSolver::sigInterruptHandler(int) {
    cout<<"MBSim: Received user interrupt or terminate signal!"<<endl;
    exitRequest=true;
  }

  void DynamicSystemSolver::sigAbortHandler(int) {
    cout<<"MBSim: Received abort signal! Flushing HDF5 files and abort!"<<endl;
    H5::FileSerie::flushAllFiles();
  }

  void DynamicSystemSolver::writez(){
    for(unsigned int i=0; i<object.size(); i++)  {
      object[i]->writeq();
      object[i]->writeu();
      object[i]->writex();
    }
    for(unsigned int i=0; i<orderOneDynamics.size(); i++)  {
      orderOneDynamics[i]->writex();
    }
  }

  void DynamicSystemSolver::readz0(){
    for(unsigned int i=0; i<object.size(); i++)  {
      object[i]->readq0();
      object[i]->readu0();
      object[i]->readx0();
    }
    for(unsigned int i=0; i<orderOneDynamics.size(); i++)  {
      orderOneDynamics[i]->readx0();
    }
  }

  void DynamicSystemSolver::updatezRef(const Vec &zParent) {

    q >> ( zParent(0,qSize-1) );
    u >> ( zParent(qSize,qSize+uSize[0]-1) );
    x >> ( zParent(qSize+uSize[0],qSize+uSize[0]+xSize-1) );

    updateqRef(q);
    updateuRef(u);
    updatexRef(x);
  }

  void DynamicSystemSolver::updatezdRef(const Vec &zdParent) {

    qd >> ( zdParent(0,qSize-1) );
    ud >> ( zdParent(qSize,qSize+uSize[0]-1) );
    xd >> ( zdParent(qSize+uSize[0],qSize+uSize[0]+xSize-1) );

    updateqdRef(qd);
    updateudRef(ud);
    updatexdRef(xd);
  }

  void DynamicSystemSolver::updaterFactors() {
    if(strategy == global) {
      //     double rFac;
      //     if(G.size() == 1) rFac = 1./G(0,0);
      //     else {
      //       Vec eta = eigvalSel(G,1,G.size());
      //       double etaMax = eta(G.size()-1);
      //       double etaMin = eta(0);
      //       int i=1;
      //       while(abs(etaMin) < 1e-8 && i<G.size()) etaMin = eta(i++);
      //       rFac = 2./(etaMax + etaMin);
      //     }
      //     rFactor.init(rFac);

      throw new MBSimError("ERROR (DynamicSystemSolver::updaterFactors()): Global r-Factor strategy not currently not available.");
    }
    else if(strategy == local) Group::updaterFactors();
    else throw new MBSimError("ERROR (DynamicSystemSolver::updaterFactors()): Unknown strategy.");
  }

  void DynamicSystemSolver::computeConstraintForces(double t) {
    la = slvLS(G, -(trans(W)*slvLLFac(LLM,h) + wb)); // slvLS wegen unbestimmten Systemen
  } 

  //  void DynamicSystemSolver::setDirectory() {
  //    int i;
  //    string projectDirectory;
  //
  //    if(false) { // TODO: introduce flag "overwriteDirectory"
  //      for(i=0; i<=99; i++) {
  //        stringstream number;
  //        number << "." << setw(2) << setfill('0') << i;
  //        projectDirectory = directoryName + number.str();
  //        int ret = mkdir(projectDirectory.c_str(),0777);
  //        if(ret == 0) break;
  //      }
  //      cout << "  make directory \'" << projectDirectory << "\' for output processing" << endl;
  //    }
  //    else { // always the same directory
  //      projectDirectory = string(directoryName);
  //
  //      int ret = mkdir(projectDirectory.c_str(),0777);
  //      if(ret == 0) {
  //        cout << "  make directory \'" << projectDirectory << "\' for output processing" << endl;
  //      }
  //      else {
  //        cout << "  use existing directory \'" << projectDirectory << "\' for output processing" << endl;
  //      }
  //    }
  //
  //    if(preIntegrator) {
  //      string preDir="PREINTEG";
  //      int ret=mkdir(preDir.c_str(),0777);
  //      if(ret==0) {
  //        cout << "Make directory " << preDir << " for Preintegration results." << endl;
  //      }
  //      else {
  //        cout << "Use existing directory " << preDir << " for Preintegration results." << endl;
  //      }
  //    }
  //
  //    return;
  //  }

  Vec DynamicSystemSolver::zdotStandard(const Vec &zParent, double t) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    updateStateDependentVariables(t);
    updateg(t);
    updategd(t);
    updateT(t); 
    updateJacobians(t);
    updateh(t); 
    updateM(t); 
    facLLM(); 
    if(laSize) {
      updateW(t); 
      updateV(t); 
      updateG(t); 
      updatewb(t); 
      computeConstraintForces(t); 
    }
    updater(t); 
    updatezd(t);

    return zdParent;
  }

  Vec DynamicSystemSolver::zdotResolveConstraints(const Vec &zParent, double t) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    updateStateDependentVariables(t);
    updateg(t);
    updategd(t);
    updateT(t); 
    resizeJacobians(1);
    updateInverseKineticsJacobians(t);
    updatehRef(hParent,hObjectParent,hLinkParent,1);
    updateh(t); 
    updateMRef(MParent,1);
    updateM(t); 
    updateLLMRef(LLMParent,1);
    facLLM(); 
    if(laSize) {
      updateLLMRef(LLMParent,1);
      updateWRef(WParent(Index(0,hSize[1]-1),Index(0,getlaSize()-1)),1);
      updateW(t); 
      updateVRef(VParent(Index(0,hSize[1]-1),Index(0,getlaSize()-1)),1);
      updateV(t); 
      updateG(t); 
      updatewb(t); 
      computeConstraintForces(t); 
    }
    resizeJacobians(0);
    updateJacobians(t);
    updatehRef(hParent,hObjectParent,hLinkParent);
    updateh(t); 
    updateMRef(MParent);
    updateM(t); 
    updateWRef(WParent);
    updateW(t); 
    updateVRef(VParent);
    updateV(t); 
    updateLLMRef(LLMParent);
    facLLM(); 
    //updater(t); 
    updatezd(t);
    return zdParent;
  }

  void DynamicSystemSolver::constructor() {
    integratorExitRequest=false;
    setPlotFeatureRecursive(plotRecursive, enabled);
    setPlotFeature(separateFilePerDynamicSystem, enabled);
    setPlotFeatureForChildren(separateFilePerDynamicSystem, disabled);
    setPlotFeatureRecursive(state, enabled);
    setPlotFeatureRecursive(stateDerivative, disabled);
    setPlotFeatureRecursive(rightHandSide, disabled);
    setPlotFeatureRecursive(globalPosition, disabled);
    setPlotFeatureRecursive(contact, enabled);
    setPlotFeatureRecursive(energy, disabled);
    setPlotFeatureRecursive(openMBV, enabled);
  }

  void DynamicSystemSolver::initializeUsingXML(TiXmlElement *element) {
    Group::initializeUsingXML(element);
    TiXmlElement *e;
    // search first Environment element
    e=element->FirstChildElement();
    while(e && ObjectFactory::getInstance()->getEnvironment(e)==0)
      e=e->NextSiblingElement();

    Environment *env;
    while((env=ObjectFactory::getInstance()->getEnvironment(e))) {
      env->initializeUsingXML(e);
      e=e->NextSiblingElement();
    }
  }

  void DynamicSystemSolver::addToTree(Tree* tree, Node* node, SqrMat &A, int i, vector<Object*>& objList) {

    Node *nextNode = tree->addObject(node,objList[i]);

    for(int j=0; j<A.cols(); j++)
      if(A(i,j) == 1) // Child node of object i
        addToTree(tree, nextNode, A, j,objList);
  }

}

