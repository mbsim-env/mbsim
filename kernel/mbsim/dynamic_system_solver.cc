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

#include <config.h>
#include <boost/core/demangle.hpp>
#include <boost/scope_exit.hpp>
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/modelling_interface.h"
#include "mbsim/frames/frame.h"
#include "mbsim/contours/contour.h"
#include "mbsim/links/link.h"
#include "mbsim/graph.h"
#include "mbsim/objects/object.h"
#include "mbsim/observers/observer.h"
#include "mbsim/constraints/constraint.h"
#include "mbsim/utils/eps.h"
#include <mbsim/environment.h>
#include <mbsim/objectfactory.h>
#include "mbsim/utils/nonlinear_algebra.h"

#include <hdf5serie/file.h>
#include <hdf5serie/simpleattribute.h>
#include <hdf5serie/simpledataset.h>
#include <limits>

#ifdef HAVE_ANSICSIGNAL
#  include <signal.h>
#endif

#include "openmbvcppinterface/group.h"

//#ifdef _OPENMP
//#include <omp.h>
//#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {
  double tP = 20.0;
  bool gflag = false;

  int baseIndexForPlot = 0;

  atomic<bool> DynamicSystemSolver::exitRequest = false;

  MBSIM_OBJECTFACTORY_REGISTERCLASS(MBSIM, DynamicSystemSolver)

  Vec DynamicSystemSolver::Residuum::operator()(const Vec &z) {
    sys->setState(z);
    sys->resetUpToDate();
    return sys->evalzd();
  }

  DynamicSystemSolver::DynamicSystemSolver(const string &name) : Group(name), t(0), dt(0), maxIter(10000), highIter(1000), maxDampingSteps(3), iterc(0), iteri(0), lmParm(0.001), contactSolver(fixedpoint), impactSolver(fixedpoint), stopIfNoConvergence(false), dropContactInfo(false), useOldla(true), numJac(false), checkGSize(true), limitGSize(500), peds(false), tolProj(1e-12), alwaysConsiderContact(true), inverseKinetics(false), initialProjection(true), determineEquilibriumState(false), useConstraintSolverForSmoothMotion(false), useConstraintSolverForPlot(false), rootID(0), updT(true), updrdt(true), updM(true), updLLM(true), updwb(true), updg(true), updgd(true), updG(true), updbc(true), updbi(true), updsv(true), updzd(true), updla(true), updLa(true), upddq(true), upddu(true), upddx(true), solveDirectly(false), READZ0(false), truncateSimulationFiles(true), facSizeGs(1) {
    for(int i=0; i<2; i++) {
      updh[i] = true;
      updr[i] = true;
      updW[i] = true;
      updV[i] = true;
    }
    constructor();
  }

  DynamicSystemSolver::~DynamicSystemSolver() {
    // Now we also delete the DynamicSystem's which exists before "reorganizing hierarchie" takes place.
    // Note all other containers are readded to DynamicSystemSolver and deleted by the dtor of
    // DynamicSystem (a base class of DynamicSystemSolver).
    for (unsigned int i = 0; i < dynamicsystemPreReorganize.size(); i++)
      delete dynamicsystemPreReorganize[i];
  }

  void DynamicSystemSolver::initialize() {

    std::string InitStageStrings[] = {
      "resolveStringRef",
      "preInit",
      "plotting",
      "unknownStage",
    };

    for (int stage = 0; stage < LASTINITSTAGE; stage++) {
      msg(Info) << "Initializing stage " << stage << "/" << LASTINITSTAGE - 1 << " \"" << InitStageStrings[stage] << "\" " << endl;
      init((InitStage) stage, InitConfigSet());
      msg(Info) << "Done initializing stage " << stage << "/" << LASTINITSTAGE - 1 << endl;
    }
  }

  void DynamicSystemSolver::init(InitStage stage, const InitConfigSet &config) {
    if(stage==resolveStringRef) {
      // Set the ds member variable of all elements recursively (for the user defined elements)
      // This should be one of the very first actions, e.g. to enable element to access the Environment of ds during initialization.
      setDynamicSystemSolver(this);

      Group::init(stage, config);
    }
    else if (stage == unknownStage) {
      vector<Element*> eleList;

      vector<Object*> objList;
      buildListOfObjects(objList);

      vector<Frame*> frmList;
      buildListOfFrames(frmList);

      vector<Contour*> cntList;
      buildListOfContours(cntList);

      vector<Link*> lnkList;
      buildListOfLinks(lnkList);

      vector<Constraint*> crtList;
      buildListOfConstraints(crtList);

      vector<ModellingInterface*> modellList;
      buildListOfModels(modellList);
      if (modellList.size()) {
        do {
          modellList[0]->processModellList(modellList, objList, lnkList);
        } while (modellList.size());
      }

      vector<Link*> iKlnkList;
      buildListOfInverseKineticsLinks(iKlnkList);

      vector<Observer*> obsrvList;
      buildListOfObservers(obsrvList);

      buildListOfDynamicSystems(dynamicsystemPreReorganize);

      clearElementLists();

      /****** reorganize ******/

      for (unsigned int i = 0; i < frmList.size(); i++) {
        frmList[i]->setName("Frame_"+to_string(i)); // just a unique local name
        addFrame(frmList[i]);
      }
      for (unsigned int i = 0; i < cntList.size(); i++) {
        cntList[i]->setName("Contour_"+to_string(i)); // just a unique local name
        addContour(cntList[i]);
      }
      for (unsigned int i = 0; i < lnkList.size(); i++) {
        lnkList[i]->setName("Link_"+to_string(i)); // just a unique local name
        addLink(lnkList[i]);
      }
      for (unsigned int i = 0; i < crtList.size(); i++) {
        crtList[i]->setName("Constraint_"+to_string(i)); // just a unique local name
        addConstraint(crtList[i]);
      }
      for (unsigned int i = 0; i < iKlnkList.size(); i++) {
        iKlnkList[i]->setName("InverseKinetic_"+to_string(i)); // just a unique local name
        addInverseKineticsLink(iKlnkList[i]);
      }
      for (unsigned int i = 0; i < obsrvList.size(); i++) {
        obsrvList[i]->setName("Observer_"+to_string(i)); // just a unique local name
        addObserver(obsrvList[i]);
      }

     for (unsigned int i = 0; i < objList.size(); i++)
       eleList.push_back(objList[i]);
     for (unsigned int i = 0; i < crtList.size(); i++)
       eleList.push_back(crtList[i]);

      /* now objects: these are much more complex since we must build a graph */

      /* matrix of body dependencies */
      SqrMat A(eleList.size(), INIT, 0.);
      for (unsigned int i = 0; i < eleList.size(); i++) {

        vector<Element*> parentElement = eleList[i]->getDependencies();

        for (unsigned int h = 0; h < parentElement.size(); h++) {
          bool foundBody = false;
          unsigned int j;
          for (j = 0; j < eleList.size(); j++) {
            if (eleList[j] == parentElement[h]) {
              foundBody = true;
              break;
            }
          }

          if (foundBody) {
            A(i, j) = 2; // 2 means predecessor
            A(j, i) = 1; // 1 means successor
          }
        }
      }

      vector<Graph*> bufGraph;
      int nt = 0;
      for (int i = 0; i < A.size(); i++) {
        double a = max(A.T().col(i));
        if (a > 0 && fabs(A(i, i) + 1) > epsroot) { // root of relativ kinematics
          Graph *graph = new Graph("InvisibleGraph_"+to_string(nt++));
          addToGraph(graph, A, i, eleList);
          bufGraph.push_back(graph);
        }
        else if (fabs(a) < epsroot) { // absolut kinematics
          Object *obj = dynamic_cast<Object*>(eleList[i]);
          if(obj) {
            eleList[i]->setName("Object_absolute_"+to_string(i)); // just a unique local name
            addObject(obj);
          }
        }
      }

      for (unsigned int i = 0; i < bufGraph.size(); i++)
        addGroup(bufGraph[i]);

      calcqSize();
      setqInd(0);
      for(int i=0; i<2; i++) {
        calcuSize(i);
        setuInd(0, i);
        sethSize(uSize[i], i);
        sethInd(0, i);
      }

      setUpLinks(); // is needed by calcgSize()

      calcxSize();
      setxInd(0);

      zSize = qSize + uSize[0] + xSize;

      calclaInverseKineticsSize();
      calcbInverseKineticsSize();

      calcisSize();
      calclaSize(0);
      calcgSize(0);
      calcgdSize(0);
      calcrFactorSize(0);
      calcsvSize();
      setsvInd(0);

      calcLinkStatusSize();
      calcLinkStatusRegSize();

      msg(Info) << "qSize = " << qSize << endl;
      msg(Info) << "uSize = " << uSize[0] << endl;
      msg(Info) << "xSize = " << xSize << endl;
      msg(Info) << "hSize = " << hSize[0] << endl;
      msg(Info) << "isSize = " << isSize << endl;
      msg(Info) << "gSize = " << gSize << endl;
      msg(Info) << "gdSize = " << gdSize << endl;
      msg(Info) << "laSize = " << laSize << endl;
      msg(Info) << "svSize = " << svSize << endl;
      msg(Info) << "uSize[1] = " << uSize[1] << endl;
      msg(Info) << "hSize[1] = " << hSize[1] << endl;
      msg(Info) << "LinkStatusSize = " << LinkStatusSize << endl;
      msg(Info) << "LinkStatusRegSize = " << LinkStatusRegSize << endl;

      // Set the ds member variable of all elements recursively (this includes now generated elements of the graph)
      setDynamicSystemSolver(this);

      MParent.resize(getuSize(0));
      TParent.resize(getqSize(), getuSize());
      LLMParent.resize(getuSize(0));
      WParent[0].resize(getuSize(0), getlaSize());
      VParent[0].resize(getuSize(0), getlaSize());
      WParent[1].resize(getuSize(1), getlaSize());
      VParent[1].resize(getuSize(1), getlaSize());
      wbParent.resize(getlaSize());
      laParent.resize(getlaSize());
      LaParent.resize(getlaSize());
      rFactorParent.resize(getlaSize());
      sParent.resize(getlaSize());
      if (impactSolver == rootfinding)
        resParent.resize(getlaSize());
      curisParent.resize(getisSize());
      nextisParent.resize(getisSize());
      gParent.resize(getgSize());
      gdParent.resize(getgdSize());
      zParent.resize(getzSize());
      zdParent.resize(getzSize());
      dqParent.resize(getqSize());
      duParent.resize(getuSize());
      dxParent.resize(getxSize());
      hParent[0].resize(getuSize(0));
      hParent[1].resize(getuSize(1));
      rParent[0].resize(getuSize(0));
      rParent[1].resize(getuSize(1));
      rdtParent.resize(getuSize(0));
      svParent.resize(getsvSize());
      jsvParent.resize(getsvSize());
      LinkStatusParent.resize(getLinkStatusSize());
      LinkStatusRegParent.resize(getLinkStatusRegSize());
      WInverseKineticsParent.resize(hSize[1], laInverseKineticsSize);
      bInverseKineticsParent.resize(bInverseKineticsSize, laInverseKineticsSize);
      laInverseKineticsParent.resize(laInverseKineticsSize);
      corrParent.resize(getgdSize());

      updateMRef(MParent);
      updateTRef(TParent);
      updateLLMRef(LLMParent);

      updatesvRef(svParent);
      updatejsvRef(jsvParent);
      updateLinkStatusRef(LinkStatusParent);
      updateLinkStatusRegRef(LinkStatusRegParent);
      updatezRef(zParent);
      updatezdRef(zdParent);
      updatedxRef(dxParent);
      updatedqRef(dqParent);
      updateduRef(duParent);
      updatelaRef(laParent);
      updateLaRef(LaParent);
      updateInternalStateRef(curisParent, nextisParent);
      updategRef(gParent);
      updategdRef(gdParent);
      updatehRef(hParent[0], 0);
      updaterRef(rParent[0], 0);
      updaterdtRef(rdtParent);
      updatehRef(hParent[1], 1);
      updaterRef(rParent[1], 1);
      updateWRef(WParent[0], 0);
      updateWRef(WParent[1], 1);
      updateVRef(VParent[0], 0);
      updateVRef(VParent[1], 1);
      updatewbRef(wbParent);

      updatelaInverseKineticsRef(laInverseKineticsParent);
      updateWInverseKineticsRef(WInverseKineticsParent);
      updatebInverseKineticsRef(bInverseKineticsParent);

      if (impactSolver == rootfinding)
        updateresRef(resParent);
      updaterFactorRef(rFactorParent);

      // contact solver specific settings
      msg(Debug) << "  use contact solver \'" << getSolverInfo() << "\' for contact situations" << endl;
      if (contactSolver == GaussSeidel)
        solveConstraints_ = &DynamicSystemSolver::solveConstraintsGaussSeidel;
      else if (contactSolver == direct) {
        solveConstraints_ = &DynamicSystemSolver::solveConstraintsLinearEquations;
        msg(Info) << "The selected contact solver 'direct' is only valid for bilateral constrained systems!" << endl;
      }
      else if (contactSolver == fixedpoint)
        solveConstraints_ = &DynamicSystemSolver::solveConstraintsFixpointSingle;
      else if (contactSolver == rootfinding)
        solveConstraints_ = &DynamicSystemSolver::solveConstraintsRootFinding;
      else
        throwError("(DynamicSystemSolver::init()): Unknown contact solver");

      // impact solver specific settings
      msg(Debug) << "  use impact solver \'" << getSolverInfo() << "\' for impact situations" << endl;
      if (impactSolver == GaussSeidel)
        solveImpacts_ = &DynamicSystemSolver::solveImpactsGaussSeidel;
      else if (impactSolver == direct) {
        solveImpacts_ = &DynamicSystemSolver::solveImpactsLinearEquations;
        msg(Info) << "The selected impact solver 'direct' is only valid for bilateral constrained systems!" << endl;
      }
      else if (impactSolver == fixedpoint)
        solveImpacts_ = &DynamicSystemSolver::solveImpactsFixpointSingle;
      else if (impactSolver == rootfinding)
        solveImpacts_ = &DynamicSystemSolver::solveImpactsRootFinding;
      else
        throwError("(DynamicSystemSolver::init()): Unknown impact solver");

      Group::init(stage, config);

      setUpObjectsWithNonConstantMassMatrix();
    }
    else if (stage == preInit) {
      if(contactSolver==unknownSolver)
        throwError("(DynamicSystemSolver::init): constraint solver unknown");
      if(impactSolver==unknownSolver)
        throwError("(DynamicSystemSolver::init): impact solver unknown");
      if (inverseKinetics)
        setUpInverseKinetics();
      Group::init(stage, config);
    }
    else if (stage == plotting) {
      firstPlot=true;

      // We do not use getPath here since separateFilePerGroup is only allowed per Group and all parents of Group's
      // are also Group's (DynamicSystem's) -> Skip the Group[...] for each sub path.
      // We can walk to the top here since stage plotting is done before reorganizeHierarchy.
      string fileName="mbsh5";
      const DynamicSystem *ds=this;
      while(ds) {
        fileName=ds->getName()+"."+fileName;
        ds=static_cast<const DynamicSystem*>(ds->getParent());
      }
      // create new plot file (cast needed because of the inadequacy of the HDF5 C++ interface?)
      auto curCompressionLevel=H5::File::getDefaultCompression();
      auto curChunkSize=H5::File::getDefaultChunkSize();
      auto curCacheSize=H5::File::getDefaultCacheSize();
      BOOST_SCOPE_EXIT(curChunkSize, curCompressionLevel, curCacheSize) {
        H5::File::setDefaultCompression(curCompressionLevel);
        H5::File::setDefaultChunkSize(curChunkSize);
        H5::File::setDefaultCacheSize(curCacheSize);
      } BOOST_SCOPE_EXIT_END
      H5::File::setDefaultCompression(compressionLevel);
      H5::File::setDefaultChunkSize(chunkSize);
      H5::File::setDefaultCacheSize(cacheSize);
      hdf5File = std::make_shared<H5::File>(fileName, H5::File::write);

      Group::init(stage, config);

      if (plotFeature[openMBV]) {
        // add MBSimEnvironment OpenMBV objects
        auto envs=getMBSimEnvironment()->getOpenMBVObjects();
        if(!envs.empty()) {
          auto openmbvEnv=OpenMBV::ObjectFactory::create<OpenMBV::Group>();
          openmbvEnv->setName("environments");
          openMBVGrp->addObject(openmbvEnv);
          for(auto env : envs) {
            env->setEnvironment(true);
            openmbvEnv->addObject(env);
          }
        }
        // write openmbv files
        openMBVGrp->write(true, truncateSimulationFiles);
      }
    }
    else
      Group::init(stage, config);
  }

  int DynamicSystemSolver::solveConstraintsFixpointSingle() {
    updaterFactors();

    checkConstraintsForTermination();
    if (term)
      return 0;

    int iter, level = 0;
    int checkTermLevel = 0;

    for (iter = 1; iter <= maxIter; iter++) {

      if (level < decreaseLevels.size() && iter > decreaseLevels(level)) {
        level++;
        decreaserFactors();
        msg(Warn) << "decreasing r-factors at iter = " << iter << endl;
      }

      Group::solveConstraintsFixpointSingle();

      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkConstraintsForTermination();
        if (term)
          break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveImpactsFixpointSingle() {
    updaterFactors();

    checkImpactsForTermination();
    if (term)
      return 0;

    int iter, level = 0;
    int checkTermLevel = 0;

    for (iter = 1; iter <= maxIter; iter++) {

      if (level < decreaseLevels.size() && iter > decreaseLevels(level)) {
        level++;
        decreaserFactors();
        msg(Warn) << "decreasing r-factors at iter = " << iter << endl;
      }

      Group::solveImpactsFixpointSingle();

      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkImpactsForTermination();
        if (term)
          break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveConstraintsGaussSeidel() {
    checkConstraintsForTermination();
    if (term)
      return 0;

    int iter;
    int checkTermLevel = 0;

    for (iter = 1; iter <= maxIter; iter++) {
      Group::solveConstraintsGaussSeidel();
      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkConstraintsForTermination();
        if (term)
          break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveImpactsGaussSeidel() {
    checkImpactsForTermination();
    if (term)
      return 0;

    int iter;
    int checkTermLevel = 0;

    for (iter = 1; iter <= maxIter; iter++) {
      Group::solveImpactsGaussSeidel();
      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkImpactsForTermination();
        if (term)
          break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveConstraintsRootFinding() {
    updaterFactors();

    int iter;
    int checkTermLevel = 0;

    updateresRef(resParent);
    Group::solveConstraintsRootFinding();

    double nrmf0 = nrm2(res);
    Vec res0 = res;

    checkConstraintsForTermination();
    if (term)
      return 0;

    DiagMat I(la.size(), INIT, 1);
    for (iter = 1; iter <= maxIter; iter++) {

      if (Jprox.size() != la.size())
        Jprox.resize(la.size(), NONINIT);

      if (numJac) {
        for (int j = 0; j < la.size(); j++) {
          const double xj = la(j);
          double dx = epsroot / 2.;
          
          do
            dx += dx;
          while (fabs(xj + dx - la(j)) < epsroot);

          la(j) += dx;
          Group::solveConstraintsRootFinding();
          la(j) = xj;
          Jprox.set(j, (res - res0) / dx);
        }
      }
      else
        jacobianConstraints();

      Vec dx = slvLS(Jprox, res0);

      Vec La_old = la;
      double alpha = 1;
      double nrmf = 1;
      for (int k = 0; k < maxDampingSteps; k++) {
        la = La_old - alpha * dx;
        Group::solveConstraintsRootFinding();
        nrmf = nrm2(res);
        if (nrmf < nrmf0)
          break;

        alpha *= .5;
      }
      nrmf0 = nrmf;
      res0 = res;

      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkConstraintsForTermination();
        if (term)
          break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveImpactsRootFinding() {
    updaterFactors();

    int iter;
    int checkTermLevel = 0;
    
    updateresRef(resParent);
    Group::solveImpactsRootFinding();

    double nrmf0 = nrm2(res);
    Vec res0 = res;

    checkImpactsForTermination();
    if (term)
      return 0;

    for (iter = 1; iter <= maxIter; iter++) {

      if (Jprox.size() != La.size())
        Jprox.resize(La.size(), NONINIT);

      if (numJac) {
        for (int j = 0; j < La.size(); j++) {
          const double xj = La(j);
          double dx = .5 * epsroot;
          
          do
            dx += dx;
          while (fabs(xj + dx - La(j)) < epsroot);
          
          La(j) += dx;
          Group::solveImpactsRootFinding();
          La(j) = xj;
          Jprox.set(j, (res - res0) / dx);
        }
      }
      else
        jacobianImpacts();

      Vec dx = slvLS(Jprox, res0);

      Vec La_old = La;
      double alpha = 1.;
      double nrmf = 1;
      for (int k = 0; k < maxDampingSteps; k++) {
        La = La_old - alpha * dx;
        Group::solveImpactsRootFinding();
        nrmf = nrm2(res);
        if (nrmf < nrmf0)
          break;
        alpha *= .5;
      }
      nrmf0 = nrmf;
      res0 = res;

      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkImpactsForTermination();
        if (term)
          break;
      }
    }
    return iter;
  }

  void DynamicSystemSolver::checkConstraintsForTermination() {
    term = true;

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (**i).checkConstraintsForTermination();
      if (term == false) {
        return;
      }
    }
  }

  void DynamicSystemSolver::checkImpactsForTermination() {
    term = true;

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (**i).checkImpactsForTermination();
      if (term == false)
        return;
    }
  }

  void DynamicSystemSolver::updateh(int j) {
    h[j].init(0);
    Group::updateh(j);
    updh[j] = false;
    checkExitRequest(); // updateh is called by all solvers
  }

  Mat DynamicSystemSolver::dhdq(int lb, int ub) {
    throwError("DynamicSystemSolver::dhdq not implemented.");
//    if (lb != 0 || ub != 0) {
//      assert(lb >= 0);
//      assert(ub <= qSize);
//    }
//    else if (lb == 0 && ub == 0) {
//      lb = 0;
//      ub = qSize;
//    }
//    double delta = epsroot;
//    Mat J(hSize[0], qSize, INIT, 0.0);
//    throw;
//    updateg();
//    updategd();
//    updateT();
//    updateh();
//    Vec hOld = h[0];
//    for (int i = lb; i < ub; i++) {
//      double qtmp = q(i);
//      q(i) += delta;
//      updateg();
//      updategd();
//      updateT();
//      updateh();
//      J.col(i) = (h[0] - hOld) / delta;
//      q(i) = qtmp;
//    }
//    h[0] = hOld;
//
//    updateg();
//    updategd();
//    updateT();
//    updateh();
//
//    return J;
  }

  Mat DynamicSystemSolver::dhdu(int lb, int ub) {
    throwError("DynamicSystemSolver::dhdu not implemented.");
//    if (lb != 0 || ub != 0) {
//      assert(lb >= 0);
//      assert(ub <= uSize[0]);
//    }
//    else if (lb == 0 && ub == 0) {
//      lb = 0;
//      ub = uSize[0];
//    }
//    double delta = epsroot;
//    Mat J(hSize[0], uSize[0], INIT, 0.0);
//    throw;
//    updateg();
//    updategd();
//    updateT();
//    updateh();
//    Vec hOld = h[0];
//    for (int i = lb; i < ub; i++) {
//      //msg(Info) << "bin bei i=" << i << endl;
//      double utmp = u(i);
//      u(i) += delta;
//      //updateg();
//      updategd();
//      //updateT();
//      updateh();
//      J.col(i) = (h[0] - hOld) / delta;
//      u(i) = utmp;
//    }
//    h[0] = hOld;
//    updategd();
//    updateh();
//
//    return J;
  }

  Mat DynamicSystemSolver::dhdx() {
    throwError("Internal error");
  }

  Vec DynamicSystemSolver::dhdt() {
    throwError("Internal error");
  }

  void DynamicSystemSolver::updateT() {
    Group::updateT();
    updT = false;
  }

  void DynamicSystemSolver::updateM() {
    Group::updateM();
    updM = false;
  }

  void DynamicSystemSolver::updateLLM() {
    Group::updateLLM();
    updLLM = false;
  }

  void DynamicSystemSolver::updater(int j) {
    r[j] = evalV(j) * evalla(); // cannot be called locally (hierarchically), because this adds some values twice to r for tree structures
    updr[j] = false;
  }

  void DynamicSystemSolver::updaterdt() {
    rdt = evalV() * evalLa(); // cannot be called locally (hierarchically), because this adds some values twice to r for tree structures
    updrdt = false;
  }

  void DynamicSystemSolver::updatewb() {
    wb.init(0);
    Group::updatewb();
    updwb = false;
  }

  void DynamicSystemSolver::updateg() {
    Group::updateg();
    updg = false;
  }

  void DynamicSystemSolver::updategd() {
    Group::updategd();
    updgd = false;
  }

  void DynamicSystemSolver::updateW(int j) {
    W[j].init(0);
    Group::updateW(j);
    updW[j] = false;
  }

  void DynamicSystemSolver::updateV(int j) {
    V[j] = evalW(j);
    Group::updateV(j);
    updV[j] = false;
  }

  void DynamicSystemSolver::updatezd() {
    Group::updatezd();
    updzd = false;
  }

  void DynamicSystemSolver::computeInitialCondition() {
    resetUpToDate();
    checkActive(1);
    checkActive(2);
    calclaSize(3);
    calcrFactorSize(3);
    updateWRef(WParent[0], 0);
    updateVRef(VParent[0], 0);
    updatelaRef(laParent);
    updatewbRef(wbParent);
    updaterFactorRef(rFactorParent);
    if (laSize) {
      checkActive(4);
      // Perform a projection of generalized positions and velocities at time t=0
      if(initialProjection) {
        projectGeneralizedPositions(2,true);
        projectGeneralizedVelocities(2);
      }
    }
    checkActive(5); // final update von gActive, ...
    calclaSize(3); // IH
    calcrFactorSize(3); // IH
    updateWRef(WParent[0]);
    updateVRef(VParent[0]);
    updatelaRef(laParent);
    updatewbRef(wbParent);
    updaterFactorRef(rFactorParent);
    updateWRef(WParent[1], 1);
    updateVRef(VParent[1], 1);
    if(determineEquilibriumState) {
      Residuum f(this);
      MultiDimNewtonMethod newton(&f);
      newton.setLinearAlgebra(1);
      z = newton.solve(z);
      if(newton.getInfo() != 0)
        throwError("(DynamicSystemSolver::computeInitialCondition): computation of equilibrium state failed!");
    }
  }

  int DynamicSystemSolver::solveConstraintsLinearEquations() {
    la = slvLS(evalG(), -evalbc());
    return 1;
  }

  int DynamicSystemSolver::solveImpactsLinearEquations() {
    La = slvLS(evalG(), -evalbi());
    return 1;
  }

  void DynamicSystemSolver::updateG() {
    G <<= evalW().T() * slvLLFac(evalLLM(), evalV());

    if (checkGSize) {
      int k = G.nonZeroElements();
      if(G.size() != Gs.cols() or k != Gs.nonZeroElements())
        Gs.resize(G.size(),G.size(),k,NONINIT);
    }
    else if (Gs.cols() != G.size()) {
      if (G.size() > limitGSize && fabs(facSizeGs - 1) < epsroot)
        facSizeGs = double(G.nonZeroElements()) / double(G.size() * G.size()) * 1.5;
      Gs.resize(G.size(), G.size(), int(G.size() * G.size() * facSizeGs));
    }
    Gs = G;

    updG = false;
  }

  void DynamicSystemSolver::updatebc() {
    bc <<= evalW().T() * slvLLFac(evalLLM(), evalh()) + evalwb();
    updbc = false;
  }

  void DynamicSystemSolver::updatebi() {
    bi <<= evalgd(); // bi = gd + trans(W)*slvLLFac(LLM,h)*dt with dt=0
    updbi = false;
  }

  void DynamicSystemSolver::updatela() {
    if (la.size()) {

      if(solveDirectly)
        la = slvLS(evalG(), -evalbc()); // slvLS because of undetermined system of equations
      else {

        if (useOldla)
          initla();
        else
          la.init(0);

        iterc = (this->*solveConstraints_)(); // solver election
        if (iterc >= maxIter) {
          msg(Warn) << "Iterations: " << iterc << endl;
          msg(Warn) << "Error: no convergence in constraint solver (t=" << t << ")." << endl;
          if (stopIfNoConvergence) {
            if (dropContactInfo)
              dropContactMatrices();
            throwError("Maximum number of iterations reached in constraint solver");
          }
          msg(Warn) << "Anyway, continuing integration..." << endl;
        }

        if (iterc > highIter)
          msg(Warn) << "high number of iterations in constraint solver: " << iterc << endl;

        if (useOldla)
          savela();
      }
    }

    updla = false;
  }

  void DynamicSystemSolver::updateLa() {
    if (La.size()) {

      if (useOldla)
        initLa();
      else
        La.init(0);

      iteri = (this->*solveImpacts_)(); // solver election
      if (iteri >= maxIter) {
        msg(Warn) << "Iterations: " << iteri << endl;
        msg(Warn) << "Error: no convergence in impact solver (t=" << t << ")." << endl;
        if (stopIfNoConvergence) {
          if (dropContactInfo)
            dropContactMatrices();
          throwError("Maximal Number of iterations reached in impact solver");
        }
        msg(Warn) << "Anyway, continuing integration..." << endl;
      }

      if (iteri > highIter)
        msg(Warn) << "high number of iterations in impact solver: " << iteri << endl;

      if (useOldla)
        saveLa();
    }

    updLa = false;
  }

  void DynamicSystemSolver::decreaserFactors() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->decreaserFactors();
  }

  void DynamicSystemSolver::getLinkStatus(VecInt &LinkStatusExt) {
    if (LinkStatusExt.size() < LinkStatusSize)
      LinkStatusExt.resize(LinkStatusSize);
    if (LinkStatus() != LinkStatusExt())
      updateLinkStatusRef(LinkStatusExt);
    updateLinkStatus();
  }

  void DynamicSystemSolver::getLinkStatusReg(VecInt &LinkStatusRegExt) {
    if (LinkStatusRegExt.size() < LinkStatusRegSize)
      LinkStatusRegExt.resize(LinkStatusRegSize);
    if (LinkStatusReg() != LinkStatusRegExt())
      updateLinkStatusRegRef(LinkStatusRegExt);
    updateLinkStatusReg();
  }

  bool DynamicSystemSolver::positionDriftCompensationNeeded(double gmax) {
    // update g to represent only closed contacts
    calcgSize(1);
    updategRef(gParent);
    // get g of closed contacts
    const fmatvec::Vec &g=evalg();
    // check maximal drift
    for(int i=0; i<g.size(); ++i)
      if(abs(g(i))>gmax)
        return true;
    return false;
  }

  bool DynamicSystemSolver::velocityDriftCompensationNeeded(double gdmax) {
    // update gd to represent only closed contacts in stick mode
    calcgdSize(3);
    updategdRef(gdParent);
    // get gd of closed contacts in stick mode
    const fmatvec::Vec &gd=evalgd();
    // check maximal drift
    for(int i=0; i<gd.size(); ++i)
      if(abs(gd(i))>gdmax)
        return true;
    return false;
  }

  void DynamicSystemSolver::projectGeneralizedPositions(int mode, bool fullUpdate) {
    msg(Info) << "System projection of generalized position at t = " << getTime() << " (mode=" << mode << " fullUpdate=" << fullUpdate << ")." << endl;

    int gID = 0;
    int laID = 0;
    int corrID = 0;
    if (mode == 3) { // impact
      gID = 1; // IG
      laID = 4;
      corrID = 1;
    }
    else if (mode == 2) { // opening, slip->stick
      gID = 2; // IB
      laID = 5;
      corrID = 2;
    }
    else if (mode == 1) { // opening
      gID = 2; // IB
      laID = 5;
      corrID = 2;
    }
    else
      throwError("Internal error");

    calcgSize(gID);
    calccorrSize(corrID);
    updatecorrRef(corrParent);
    updategRef(gParent);
    updg = true;
    updatecorr(corrID);
    Vec nu(getuSize());
    calclaSize(laID);
    updateWRef(WParent[0]);
    updW[0] = true;
    SqrMat Gv = SqrMat(evalW().T() * slvLLFac(evalLLM(), evalW()));
    Mat T = evalT();
    int iter = 0;
    bool highIterMsgPrinted=false;
    while (nrmInf(evalg() - corr) >= tolProj) {
      if (++iter >= maxIter) {
        msg(Warn) << "Iterations: " << iter << endl;
        msg(Warn) << "Error: no convergence in projection of generalized positions (t=" << t << ")." << endl;
        if(stopIfNoConvergence)
          throwError("Maximal Number of iterations reached in projection of generalized position");
        msg(Warn) << "Anyway, continuing integration..." << endl;
        break;
      }
      if (iter > highIter && !highIterMsgPrinted) {
        msg(Warn) << "high number of iterations in projection of generalized positions: " << iter << endl;// print only ones
        highIterMsgPrinted=true;
      }
      if(fullUpdate) Gv = SqrMat(evalW().T() * slvLLFac(evalLLM(), evalW()));
      Vec mu = slvLS(Gv, -evalg() + getW(0,false).T() * nu + corr);
      Vec dnu = slvLLFac(getLLM(false), getW(0,false) * mu) - nu;
      nu += dnu;
      q += T * dnu;
      resetUpToDate();
    }
    calclaSize(3);
    updateWRef(WParent[0]);
    calcgSize(0);
    updategRef(gParent);
  }

  void DynamicSystemSolver::projectGeneralizedVelocities(int mode) {
    msg(Info) << "System projection of generalized velocities at t = " << getTime() << " (mode=" << mode << ")." << endl;

    int gdID = 0; // IH
    int corrID = 0;
    if (mode == 3) { // impact
      gdID = 3; // IH
      corrID = 4; // IH
    }
    else if (mode == 2) { // opening, slip->stick
      gdID = 3;
      corrID = 4; // IH
    }
    else if (mode == 1) { // opening and stick->slip
      gdID = 3;
      corrID = 4; // IH
    }
    else
      throwError("Internal error");
    calccorrSize(corrID); // IH
    if (corrSize) {
      calcgdSize(gdID); // IH
      updatecorrRef(corrParent);
      updategdRef(gdParent);
      updgd = true;
      updatecorr(corrID);

      calclaSize(gdID);
      updateWRef(WParent[0]);
      updW[0] = true;

      if (laSize) {
        SqrMat Gv = SqrMat(evalW().T() * slvLLFac(evalLLM(), evalW()));
        Vec mu = slvLS(Gv, -evalgd() + corr);
        u += slvLLFac(getLLM(), getW() * mu);
        resetUpToDate();
      }
    }
  }

  void DynamicSystemSolver::savela() {
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).savela();
  }

  void DynamicSystemSolver::initla() {
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).initla();
  }

  void DynamicSystemSolver::saveLa() {
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).saveLa();
  }

  void DynamicSystemSolver::initLa() {
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).initLa();
  }

  void DynamicSystemSolver::addElement(Element *element_) {
    Object* object_ = dynamic_cast<Object*>(element_);
    Link* link_ = dynamic_cast<Link*>(element_);
    if (object_)
      addObject(object_);
    else if (link_)
      addLink(link_);
    else {
      throwError("(DynamicSystemSolver: addElement()): No such type of Element to add!");
    }
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
    //   for(i3=0; i3<extraDynamic.size(); i3++) {
    //     if(extraDynamic[i3]->getName() == name) return (Element*)extraDynamic[i3];
    //   }
    //   for(i3=0; i3<extraDynamic.size(); i3++) {
    //     if(extraDynamic[i3]->getPath() == name) return (Element*)extraDynamic[i3];
    //   }
    //   if(!(i1<object.size())||!(i2<link.size())||!(i3<extraDynamic.size())) msg(Error) << "Error: The DynamicSystemSolver " << this->name <<" comprises no element " << name << "!" << endl; 
    //   assert(i1<object.size()||i2<link.size()||!(i3<extraDynamic.size()));
    return NULL;
  }

  string DynamicSystemSolver::getSolverInfo() {
    stringstream info;

    if (impactSolver == GaussSeidel)
      info << "GaussSeidel";
    else if (impactSolver == direct)
      info << "direct";
    else if (impactSolver == fixedpoint)
      info << "fixedpoint";
    else if (impactSolver == rootfinding)
      info << "rootfinding";

    return info.str();
  }

  void DynamicSystemSolver::dropContactMatrices() {
    msg(Info) << "dropping contact matrices to file <dump_matrices.asc>" << endl;
    ofstream contactDrop("dump_matrices.asc");

    contactDrop << "constraint functions g" << endl << evalg() << endl << endl;
    contactDrop << endl;
    contactDrop << "mass matrix M" << endl << evalM() << endl << endl;
    contactDrop << "generalized force vector h" << endl << evalh() << endl << endl;
    contactDrop << "generalized force directions W" << endl << evalW() << endl << endl;
    contactDrop << "generalized force directions V" << endl << evalV() << endl << endl;
    contactDrop << "mass action matrix G" << endl << evalG() << endl << endl;
    contactDrop << "vector wb" << endl << evalwb() << endl << endl;
    contactDrop << endl;
    contactDrop << "constraint velocities gp" << endl << evalgd() << endl << endl;
    contactDrop << "non-holonomic part in gp; bc" << endl << bc << endl << endl;
    contactDrop << "non-holonomic part in gp; bi" << endl << bi << endl << endl;
    contactDrop << "Lagrange multipliers la" << endl << la << endl << endl;
    contactDrop << "Lagrange multipliers La" << endl << La << endl << endl;
    contactDrop.close();
  }

  DynamicSystemSolver::SignalHandler::SignalHandler() {
#ifdef HAVE_ANSICSIGNAL
    oldSigInt=signal(SIGINT, sigInterruptHandler);
    oldSigTerm=signal(SIGTERM, sigInterruptHandler);
#endif
  }

  DynamicSystemSolver::SignalHandler::~SignalHandler() {
#ifdef HAVE_ANSICSIGNAL
    signal(SIGINT, oldSigInt);
    signal(SIGTERM, oldSigTerm);
#endif
  }

  void DynamicSystemSolver::sigInterruptHandler(int) {
    exitRequest = true;
  }

  void DynamicSystemSolver::checkExitRequest() {
    if (integratorExitRequest) // if the integrator has not exit after a integratorExitRequest
      throwError("MBSim: Integrator has not stopped integration! Terminate NOW by throwing an exception!");

    if (exitRequest) { // on exitRequest flush plot files and ask the integrator to exit
      msg(Info) << "MBSim: User exit request: Ask integrator to terminate!" << endl;
      integratorExitRequest = true;
    }
  }

  void DynamicSystemSolver::writez(string fileName, bool formatH5) {
    if (formatH5) {
      H5::File file(fileName, H5::File::write);

      Group::writez(&file);
    }
    else {
      ofstream file(fileName);
      file.setf(ios::scientific);
      file.precision(numeric_limits<double>::digits10 + 1);
      for (int i = 0; i < q.size(); i++)
        file << q(i) << endl;
      for (int i = 0; i < u.size(); i++)
        file << u(i) << endl;
      for (int i = 0; i < x.size(); i++)
        file << x(i) << endl;
      for (int i = 0; i < curis.size(); i++)
        file << curis(i) << endl;
      file.close();
    }
  }

  void DynamicSystemSolver::writeStateTable(string fileName) {
    if(tabz.empty()) {
      tabz.resize(zSize+isSize);
      updateStateTable();
    }
    ofstream file(fileName);
    for (size_t i = 0; i < tabz.size(); i++)
      file << tabz[i].name << " " << tabz[i].label << " " << tabz[i].number << endl;
    file.close();
  }

  void DynamicSystemSolver::readz0(string fileName) {
    H5::File file(fileName, H5::File::read);

    DynamicSystem::readz0(&file);

    READZ0 = true;
  }

  void DynamicSystemSolver::updatezRef(Vec &zParent) {
    z.ref(zParent, RangeV(0, getzSize()-1));

    q.ref(z, RangeV(0, qSize - 1));
    u.ref(z, RangeV(qSize, qSize + uSize[0] - 1));
    x.ref(z, RangeV(qSize + uSize[0], qSize + uSize[0] + xSize - 1));

    updateqRef(q);
    updateuRef(u);
    updatexRef(x);
  }

  void DynamicSystemSolver::updatezdRef(Vec &zdParent) {
    zd.ref(zdParent, RangeV(0, getzSize()-1));

    qd.ref(zd, RangeV(0, qSize - 1));
    ud.ref(zd, RangeV(qSize, qSize + uSize[0] - 1));
    xd.ref(zd, RangeV(qSize + uSize[0], qSize + uSize[0] + xSize - 1));

    updateqdRef(qd);
    updateudRef(ud);
    updatexdRef(xd);

    updateudallRef(ud);
  }

  void DynamicSystemSolver::constructor() {
    integratorExitRequest = false;
    plotFeature[plotRecursive] = true;
    plotFeatureForChildren[plotRecursive] = true;
    plotFeature[openMBV] = true;
    plotFeatureForChildren[openMBV] = true;
  }

  void DynamicSystemSolver::initializeUsingXML(DOMElement *element) {
    Group::initializeUsingXML(element);
    DOMElement *e;
    // search first Environment element
    e = E(element)->getFirstElementChildNamed(MBSIM%"environments")->getFirstElementChild();

    while (e) {
      addEnvironment(ObjectFactory::createAndInit<Environment>(e));
      e = e->getNextElementSibling();
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"constraintSolver");
    if (e) {
      std::string str=X()%E(e)->getFirstTextChild()->getData();
      str=str.substr(1,str.length()-2);
      if(str=="fixedpoint") contactSolver=fixedpoint;
      else if(str=="GaussSeidel") contactSolver=GaussSeidel;
      else if(str=="direct") contactSolver=direct;
      else if(str=="rootfinding") contactSolver=rootfinding;
      else contactSolver=unknownSolver;
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"impactSolver");
    if (e) {
      std::string str=X()%E(e)->getFirstTextChild()->getData();
      str=str.substr(1,str.length()-2);
      if(str=="fixedpoint") impactSolver=fixedpoint;
      else if(str=="GaussSeidel") impactSolver=GaussSeidel;
      else if(str=="direct") impactSolver=direct;
      else if(str=="rootfinding") impactSolver=rootfinding;
      else impactSolver=unknownSolver;
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"maximumNumberOfIterations");
    if (e)
      setMaximumNumberOfIterations(E(e)->getText<int>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"highNumberOfIterations");
    if (e)
      setHighNumberOfIterations(E(e)->getText<int>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"numericalJacobian");
    if (e)
      setNumericalJacobian(E(e)->getText<bool>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"stopIfNoConvergence");
    if (e)
      setStopIfNoConvergence(E(e)->getText<bool>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"projectionTolerance");
    if (e)
      setProjectionTolerance(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedRelativePositionTolerance");
    if (e)
      setGeneralizedRelativePositionTolerance(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedRelativeVelocityTolerance");
    if (e)
      setGeneralizedRelativeVelocityTolerance(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedRelativeAccelerationTolerance");
    if (e)
      setGeneralizedRelativeAccelerationTolerance(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedForceTolerance");
    if (e)
      setGeneralizedForceTolerance(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedImpulseTolerance");
    if (e)
      setGeneralizedImpulseTolerance(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedRelativePositionCorrectionValue");
    if (e)
      setGeneralizedRelativePositionCorrectionValue(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"generalizedRelativeVelocityCorrectionValue");
    if (e)
      setGeneralizedRelativeVelocityCorrectionValue(E(e)->getText<double>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"inverseKinetics");
    if (e)
      setInverseKinetics(E(e)->getText<bool>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"initialProjection");
    if (e)
      setInitialProjection(E(e)->getText<bool>());
    e=E(element)->getFirstElementChildNamed(MBSIM%"determineEquilibriumState");
    if (e)
      setDetermineEquilibriumState(E(e)->getText<bool>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"useConstraintSolverForSmoothMotion");
    if (e)
      setUseConstraintSolverForSmoothMotion(E(e)->getText<bool>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"useConstraintSolverForPlot");
    if (e)
      setUseConstraintSolverForPlot(E(e)->getText<bool>());

    e = E(element)->getFirstElementChildNamed(MBSIM%"compressionLevel");
    if(e) setCompressionLevel(E(e)->getText<int>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"chunkSize");
    if(e) setChunkSize(E(e)->getText<int>());
    e = E(element)->getFirstElementChildNamed(MBSIM%"cacheSize");
    if(e) setCacheSize(E(e)->getText<int>());
  }

  void DynamicSystemSolver::addToGraph(Graph* graph, SqrMat &A, int i, vector<Element*>& eleList) {
    Object *obj = dynamic_cast<Object*>(eleList[i]);
    if(obj) {
      eleList[i]->setName("Object_graph_"+to_string(i)); // just a unique local name
      graph->addObject(eleList[i]->computeLevel(), obj);
    }
    A(i, i) = -1;

    for (int j = 0; j < A.cols(); j++)
      if (A(i, j) > 0 && fabs(A(j, j) + 1) > epsroot) // child node of object i
        addToGraph(graph, A, j, eleList);
  }

  const Vec& DynamicSystemSolver::shift() {
    msg(Info) << "System shift at t = " << t << "." << endl;

    solveDirectly = false;

    checkRoot();
    if (getRootID() == 3) { // impact (velocity jump)
      bool saveUseOldLa = useOldla;
      useOldla = false;

      checkActive(6); // decide which contacts have closed
      //msg(Info) << "stoss" << endl;

      calcgdSize(3); // IG
      updategdRef(gdParent);
      calclaSize(3); // IG
      calcrFactorSize(3); // IG
      updateWRef(WParent[0]);
      updateVRef(VParent[0]);
      updatelaRef(laParent);
      updateLaRef(LaParent);
      updaterFactorRef(rFactorParent);

      V[0] = evalW(); //updateV() not allowed here
      updV[0] = false;

      u += evaldu();
      checkActive(3); // neuer Zustand nach Stoss
      resetUpToDate();
      // Projektion:
      // - es müssen immer alle Größen projiziert werden
      // - neuer Zustand ab hier bekannt
      // - Auswertung vor Setzen von gActive und gdActive
      projectGeneralizedPositions(3);
      // Projektion der Geschwindikgeiten erst am Schluss
      //projectGeneralizedVelocities(3);

      if (laSize) {
        calclaSize(3); // IH
        calcrFactorSize(3); // IH
        updateWRef(WParent[0]);
        updateVRef(VParent[0]);
        updatelaRef(laParent);
        updatewbRef(wbParent);
        updaterFactorRef(rFactorParent);

        checkActive(4);
        projectGeneralizedPositions(2);
        projectGeneralizedVelocities(2);
      }
      useOldla = saveUseOldLa;
    }
    else if (getRootID() == 2) { // transition from slip to stick (acceleration jump)
      //msg(Info) << "haften" << endl;
      checkActive(7); // decide which contacts may stick

      calclaSize(3); // IH
      calcrFactorSize(3); // IH
      updateWRef(WParent[0]);
      updateVRef(VParent[0]);
      updatelaRef(laParent);
      updatewbRef(wbParent);
      updaterFactorRef(rFactorParent);

      if (laSize) {
        checkActive(4);

        projectGeneralizedPositions(2);
        projectGeneralizedVelocities(2);
      }
    }
    else if (getRootID() == 1) { // contact opens or transition from stick to slip
      checkActive(8);

      projectGeneralizedPositions(1);
      projectGeneralizedVelocities(1);
    }
    checkActive(5); // final update von gActive, ...
    calclaSize(3); // IH
    calcrFactorSize(3); // IH
    updateWRef(WParent[0]);
    updateVRef(VParent[0]);
    updatelaRef(laParent);
    updatewbRef(wbParent);
    updaterFactorRef(rFactorParent);

    setRootID(0);
    return zParent;
  }

  void DynamicSystemSolver::updatelaInverseKinetics() {
    updateWRef(WParent[1], 1);
    updateVRef(VParent[1], 1);
    int n = evalWInverseKinetics().cols();
    int m1 = WInverseKinetics.rows();
    int m2 = evalbInverseKinetics().rows();
    Mat A(m1 + m2, n);
    Vec b(m1 + m2);
    A.set(RangeV(0, m1 - 1), RangeV(0, n - 1), WInverseKinetics);
    A.set(RangeV(m1, m1 + m2 - 1), RangeV(0, n - 1), bInverseKinetics);
    b.set(RangeV(0, m1 - 1), -evalh(1) - evalr(1));
    laInverseKinetics = slvLL(JTJ(A), A.T() * b);
  }

  void DynamicSystemSolver::updatedq() {
    Group::updatedq();
    upddq = false;
  }

  void DynamicSystemSolver::updatedu() {
    Group::updatedu();
    upddu = false;
  }

  void DynamicSystemSolver::updatedx() {
    Group::updatedx();
    upddx = false;
  }

  void DynamicSystemSolver::updateStopVector() {
    Group::updateStopVector();
    updsv = false;
  }

  void DynamicSystemSolver::updateInternalState() {
    aboutToUpdateInternalState();
    curisParent = nextisParent;
  }

  void DynamicSystemSolver::resetUpToDate() {
    updT = true;
    updh[0] = true;
    updh[1] = true;
    updr[0] = true;
    updr[1] = true;
    updrdt = true;
    updM = true;
    updLLM = true;
    updW[0] = true;
    updW[1] = true;
    updV[0] = true;
    updV[1] = true;
    updwb = true;
    updg = true;
    updgd = true;
    updG = true;
    updbc = true;
    updbi = true;
    updsv = true;
    updzd = true;
    updla = true;
    updLa = true;
    upddq = true;
    upddu = true;
    upddx = true;
    Group::resetUpToDate();
  }

  const Vec& DynamicSystemSolver::evalzd() {
    if(updzd) {
      solveDirectly = not(useConstraintSolverForSmoothMotion);
      updatezd();
    }
    return zd;
  }

  void DynamicSystemSolver::plot() {
    solveDirectly = not(useConstraintSolverForPlot);
    if (inverseKinetics) updatelaInverseKinetics();
    Group::plot();
    if(firstPlot) {
      // we enable SWMR after the frist plot to ensure that readers see at least the initial plot step
      firstPlot=false;
      if(hdf5File)
        hdf5File->enableSWMR();
      if(openMBVGrp)
        openMBVGrp->enableSWMR();
    }
    if(hdf5File)
      hdf5File->flushIfRequested();
    if(openMBVGrp)
      openMBVGrp->flushIfRequested();
  }

  const Vec& DynamicSystemSolver::evalsv() {
    if(updsv) {
      solveDirectly = false;
      updateStopVector();
    }
    return sv;
  }

  const Vec& DynamicSystemSolver::evalz0() {
    initz();
    return z;
  }

  void DynamicSystemSolver::addEnvironment(Environment* env) {
    // check for duplicates of the same type
    auto &curType=typeid(*env);
    for(auto &e : environments) {
      auto &e_=*e;
      if(curType==typeid(e_))
        throwError("An Environment of type "+boost::core::demangle(curType.name())+" already exists in this DynamicSystemSolver.");
    }
    environments.emplace_back(unique_ptr<Environment>(env));
  }

  MBSimEnvironment* DynamicSystemSolver::getMBSimEnvironment() {
    if(!mbsimEnvironment)
      mbsimEnvironment=getEnvironment<MBSimEnvironment>();
    return mbsimEnvironment;
  }

}
