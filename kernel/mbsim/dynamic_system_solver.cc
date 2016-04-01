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
 */

#include<config.h>
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

#include <hdf5serie/file.h>
#include <hdf5serie/simpleattribute.h>
#include <hdf5serie/simpledataset.h>
#include <limits>
#include <boost/lexical_cast.hpp>

#ifdef HAVE_ANSICSIGNAL
#  include <signal.h>
#endif

#ifdef HAVE_OPENMBVCPPINTERFACE
#include "openmbvcppinterface/group.h"
#endif

//#ifdef _OPENMP
//#include <omp.h>
//#endif

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace boost;

namespace MBSim {
  double tP = 20.0;
  bool gflag = false;

  bool DynamicSystemSolver::exitRequest = false;

  MBSIM_OBJECTFACTORY_REGISTERXMLNAME(DynamicSystemSolver, MBSIM%"DynamicSystemSolver")

  DynamicSystemSolver::DynamicSystemSolver(const string &name) : Group(name), t(0), dt(1), maxIter(10000), highIter(1000), maxDampingSteps(3), lmParm(0.001), contactSolver(FixedPointSingle), impactSolver(FixedPointSingle), strategy(local), linAlg(LUDecomposition), stopIfNoConvergence(false), dropContactInfo(false), useOldla(true), numJac(false), checkGSize(true), limitGSize(500), warnLevel(0), peds(false), driftCount(1), flushEvery(100000), flushCount(flushEvery), tolProj(1e-15), alwaysConsiderContact(true), inverseKinetics(false), initialProjection(false), useConstraintSolverForPlot(false), rootID(0), gTol(1e-8), gdTol(1e-10), gddTol(1e-12), laTol(1e-12), LaTol(1e-10), updT(true), updwb(true), updg(true), updgd(true), updG(true), updb(true), READZ0(false), truncateSimulationFiles(true) {
    for(int i=0; i<2; i++) {
      updh[i] = true;
      updr[i] = true;
      updrdt[i] = true;
      updM[i] = true;
      updLLM[i] = true;
      updW[i] = true;
      updV[i] = true;
    }
    constructor();
  }

  DynamicSystemSolver::~DynamicSystemSolver() {
    closePlot();

    // Now we also delete the DynamicSystem's which exists before "reorganizing hierarchie" takes place.
    // Note all other containers are readded to DynamicSystemSolver and deleted by the dtor of
    // DynamicSystem (a base class of DynamicSystemSolver).
    for (unsigned int i = 0; i < dynamicsystemPreReorganize.size(); i++)
      delete dynamicsystemPreReorganize[i];
  }

  void DynamicSystemSolver::initialize() {

    std::string InitStageStrings[] = {
      "Modelbuildup",
      "ResolveXML-Path",
      "PreInit",
      "Resize",
      "plot",
      "reorganizeHierarchy",
      "unknownStage",
    };

#ifdef HAVE_ANSICSIGNAL
    signal(SIGINT, sigInterruptHandler);
    signal(SIGTERM, sigInterruptHandler);
    signal(SIGABRT, sigAbortHandler);
    signal(SIGSEGV, sigSegfaultHandler);
#endif
    for (int stage = 0; stage < LASTINITSTAGE; stage++) {
      msg(Info) << "Initializing stage " << stage << "/" << LASTINITSTAGE - 1 << " \"" << InitStageStrings[stage] << "\" " << endl;
      init((InitStage) stage);
      msg(Info) << "Done initializing stage " << stage << "/" << LASTINITSTAGE - 1 << endl;
    }
  }

  void DynamicSystemSolver::init(InitStage stage) {
    if (stage == reorganizeHierarchy) {
      msg(Info) << name << " (special group) stage==preInit:" << endl;

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
      if (modellList.size())
        do {
          modellList[0]->processModellList(modellList, objList, lnkList);
        } while (modellList.size());

      vector<Link*> iKlnkList;
      buildListOfInverseKineticsLinks(iKlnkList);

      vector<Observer*> obsrvList;
      buildListOfObservers(obsrvList);

      buildListOfDynamicSystems(dynamicsystemPreReorganize);

      clearElementLists();

      /****** reorganize ******/

      for (unsigned int i = 0; i < frmList.size(); i++) {
        frmList[i]->setName("Frame_"+lexical_cast<string>(i)); // just a unique local name
        addFrame(frmList[i]);
      }
      for (unsigned int i = 0; i < cntList.size(); i++) {
        cntList[i]->setName("Contour_"+lexical_cast<string>(i)); // just a unique local name
        addContour(cntList[i]);
      }
      for (unsigned int i = 0; i < lnkList.size(); i++) {
        lnkList[i]->setName("Link_"+lexical_cast<string>(i)); // just a unique local name
        addLink(lnkList[i]);
      }
      for (unsigned int i = 0; i < crtList.size(); i++) {
        crtList[i]->setName("Constraint_"+lexical_cast<string>(i)); // just a unique local name
        addConstraint(crtList[i]);
      }
      for (unsigned int i = 0; i < iKlnkList.size(); i++) {
        iKlnkList[i]->setName("InverseKinematic_"+lexical_cast<string>(i)); // just a unique local name
        addInverseKineticsLink(iKlnkList[i]);
      }
      for (unsigned int i = 0; i < obsrvList.size(); i++) {
        obsrvList[i]->setName("Observer_"+lexical_cast<string>(i)); // just a unique local name
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
        if (a > 0 && fabs(A(i, i) + 1) > epsroot()) { // root of relativ kinematics
          Graph *graph = new Graph("InvisibleGraph_"+lexical_cast<string>(nt++));
          addToGraph(graph, A, i, eleList);
          graph->setPlotFeatureRecursive(plotRecursive, enabled); // the generated invisible graph must always walk through the plot functions
          bufGraph.push_back(graph);
        }
        else if (fabs(a) < epsroot()) { // absolut kinematics
          Object *obj = dynamic_cast<Object*>(eleList[i]);
          if(obj) {
            eleList[i]->setName("Object_absolute_"+lexical_cast<string>(i)); // just a unique local name
            addObject(obj);
          }
        }
      }

      for (unsigned int i = 0; i < bufGraph.size(); i++) {
        addGroup(bufGraph[i]);
      }

      for (unsigned int i = 0; i < eleList.size(); i++) {
        int level = eleList[i]->computeLevel();
        for(int j=elementOrdered.size(); j<=level; j++) {
          vector<Element*> vec;
          elementOrdered.push_back(vec);
        }
        elementOrdered[level].push_back(eleList[i]);
      }

      for (unsigned int i = 0; i < link.size(); i++) {
        if (link[i]->isSingleValued() or (link[i]->isSetValued() and link[i]->hasSmoothPart())) {
          int level = link[i]->computeLevel();
          for(int j=linkOrdered.size(); j<=level; j++) {
            vector<Link*> vec;
            linkOrdered.push_back(vec);
          }
          linkOrdered[level].push_back(link[i]);
        }
      }

      msg(Info) << "End of special group stage==preInit" << endl;

      // after reorganizing a resize is required
      init(resize);

      for (unsigned int i = 0; i < dynamicsystem.size(); i++)
        if (dynamic_cast<Graph*>(dynamicsystem[i]))
          static_cast<Graph*>(dynamicsystem[i])->printGraph();

      msg(Info) << "Content of element graph "<< name << ":" << endl;
      for(unsigned int i=0; i<elementOrdered.size(); i++) {
        msg(Info) << "  Elements in level "<< i << ":"<< endl;
        for(unsigned int j=0; j<elementOrdered[i].size(); j++)
          msg(Info) << "    "<< elementOrdered[i][j]->getName() << " " << elementOrdered[i][j]->getPath() << " " << elementOrdered[i][j]->getType()<<endl;
      }

      msg(Info) << "Content of link graph "<< name << ":" << endl;
      for(unsigned int i=0; i<linkOrdered.size(); i++) {
        msg(Info) << "  Elements in level "<< i << ":"<< endl;
        for(unsigned int j=0; j<linkOrdered[i].size(); j++)
          msg(Info) << "    "<< linkOrdered[i][j]->getName() << " " << linkOrdered[i][j]->getPath() << " " << linkOrdered[i][j]->getType()<<endl;
      }
    }
    else if (stage == resize) {
      calcqSize();
      calcuSize(0);
      calcuSize(1);
      setsvInd(0);
      setqInd(0);
      setuInd(0, 0);
      setuInd(0, 1);
      sethSize(uSize[0], 0);
      sethSize(uSize[1], 1);
      sethInd(0, 0);
      sethInd(0, 1);
      setUpLinks(); // is needed by calcgSize()

      calcxSize();
      setxInd(0);

      calclaInverseKineticsSize();
      calcbInverseKineticsSize();

      calclaSize(0);
      calcgSize(0);
      calcgdSize(0);
      calcrFactorSize(0);
      calcsvSize();
      svSize += 1; // TODO additional event for drift 
      calcLinkStatusSize();
      calcLinkStatusRegSize();

      msg(Info) << "qSize = " << qSize << endl;
      msg(Info) << "uSize[0] = " << uSize[0] << endl;
      msg(Info) << "xSize = " << xSize << endl;
      msg(Info) << "gSize = " << gSize << endl;
      msg(Info) << "gdSize = " << gdSize << endl;
      msg(Info) << "laSize = " << laSize << endl;
      msg(Info) << "svSize = " << svSize << endl;
      msg(Info) << "LinkStatusSize = " << LinkStatusSize << endl;
      msg(Info) << "LinkStatusRegSize = " << LinkStatusRegSize << endl;
      msg(Info) << "hSize[0] = " << hSize[0] << endl;

      msg(Info) << "uSize[1] = " << uSize[1] << endl;
      msg(Info) << "hSize[1] = " << hSize[1] << endl;

      Group::init(stage);
    }
    else if (stage == unknownStage) {
      setDynamicSystemSolver(this);

      MParent[0].resize(getuSize(0));
      MParent[1].resize(getuSize(1));
      TParent.resize(getqSize(), getuSize());
      LLMParent[0].resize(getuSize(0));
      LLMParent[1].resize(getuSize(1));
      WParent[0].resize(getuSize(0), getlaSize());
      VParent[0].resize(getuSize(0), getlaSize());
      WParent[1].resize(getuSize(1), getlaSize());
      VParent[1].resize(getuSize(1), getlaSize());
      wbParent.resize(getlaSize());
      laParent.resize(getlaSize());
      LaParent.resize(getlaSize());
      rFactorParent.resize(getlaSize());
      sParent.resize(getlaSize());
      if (impactSolver == RootFinding)
        resParent.resize(getlaSize());
      gParent.resize(getgSize());
      gdParent.resize(getgdSize());
      zParent.resize(getzSize());
      zdParent.resize(getzSize());
      udParent1.resize(getuSize(1));
      hParent[0].resize(getuSize(0));
      hParent[1].resize(getuSize(1));
      rParent[0].resize(getuSize(0));
      rParent[1].resize(getuSize(1));
      rdtParent[0].resize(getuSize(0));
      rdtParent[1].resize(getuSize(1));
      fParent.resize(getxSize());
      svParent.resize(getsvSize());
      jsvParent.resize(getsvSize());
      LinkStatusParent.resize(getLinkStatusSize());
      LinkStatusRegParent.resize(getLinkStatusRegSize());
      WInverseKineticsParent[0].resize(hSize[0], laInverseKineticsSize);
      WInverseKineticsParent[1].resize(hSize[1], laInverseKineticsSize);
      bInverseKineticsParent.resize(bInverseKineticsSize, laInverseKineticsSize);
      laInverseKineticsParent.resize(laInverseKineticsSize);
      corrParent.resize(getgdSize());

      updateMRef(MParent[0], 0);
      updateMRef(MParent[1], 1);
      updateTRef(TParent);
      updateLLMRef(LLMParent[0], 0);
      updateLLMRef(LLMParent[1], 1);

      Group::init(stage);

      updatesvRef(svParent);
      updatejsvRef(jsvParent);
      updateLinkStatusRef(LinkStatusParent);
      updateLinkStatusRegRef(LinkStatusRegParent);
      updatezRef(zParent);
      updatezdRef(zdParent);
      updateudRef(udParent1, 1);
      updateudallRef(udParent1, 1);
      updatelaRef(laParent);
      updateLaRef(LaParent);
      updategRef(gParent);
      updategdRef(gdParent);
      updatehRef(hParent[0], 0);
      updaterRef(rParent[0], 0);
      updaterdtRef(rdtParent[0], 0);
      updatehRef(hParent[1], 1);
      updaterRef(rParent[1], 1);
      updaterdtRef(rdtParent[1], 1);
      updateWRef(WParent[0], 0);
      updateWRef(WParent[1], 1);
      updateVRef(VParent[0], 0);
      updateVRef(VParent[1], 1);
      updatewbRef(wbParent);

      updatelaInverseKineticsRef(laInverseKineticsParent);
      updateWInverseKineticsRef(WInverseKineticsParent[0], 0);
      updateWInverseKineticsRef(WInverseKineticsParent[1], 1);
      updatebInverseKineticsRef(bInverseKineticsParent);
      updatehRef(hParent[1], 1); // TODO: warum zweifacher Auruf?
      updaterRef(rParent[1], 1);
      updaterdtRef(rdtParent[1], 1);
      updateWRef(WParent[1], 1);
      updateVRef(VParent[1], 1);

      if (impactSolver == RootFinding)
        updateresRef(resParent);
      updaterFactorRef(rFactorParent);

      // contact solver specific settings
      msg(Info) << "  use contact solver \'" << getSolverInfo() << "\' for contact situations" << endl;
      if (contactSolver == GaussSeidel)
        solveConstraints_ = &DynamicSystemSolver::solveConstraintsGaussSeidel;
      else if (contactSolver == LinearEquations) {
        solveConstraints_ = &DynamicSystemSolver::solveConstraintsLinearEquations;
        msg(Warn) << "solveLL is only valid for bilateral constrained systems!" << endl;
      }
      else if (contactSolver == FixedPointSingle)
        solveConstraints_ = &DynamicSystemSolver::solveConstraintsFixpointSingle;
      else if (contactSolver == RootFinding) {
        msg(Warn) << "RootFinding solver is BUGGY at least if there is friction!" << endl;
        solveConstraints_ = &DynamicSystemSolver::solveConstraintsRootFinding;
      }
      else
        THROW_MBSIMERROR("(DynamicSystemSolver::init()): Unknown contact solver");

      // impact solver specific settings
      msg(Info) << "  use impact solver \'" << getSolverInfo() << "\' for impact situations" << endl;
      if (impactSolver == GaussSeidel)
        solveImpacts_ = &DynamicSystemSolver::solveImpactsGaussSeidel;
      else if (impactSolver == LinearEquations) {
        solveImpacts_ = &DynamicSystemSolver::solveImpactsLinearEquations;
        msg(Warn) << "solveLL is only valid for bilateral constrained systems!" << endl;
      }
      else if (impactSolver == FixedPointSingle)
        solveImpacts_ = &DynamicSystemSolver::solveImpactsFixpointSingle;
      else if (impactSolver == RootFinding) {
        msg(Warn) << "RootFinding solver is BUGGY at least if there is friction!" << endl;
        solveImpacts_ = &DynamicSystemSolver::solveImpactsRootFinding;
      }
      else
        THROW_MBSIMERROR("(DynamicSystemSolver::init()): Unknown impact solver");
    }
    else if (stage == modelBuildup) {
      msg(Info) << "  initialising modelBuildup ..." << endl;
      Group::init(stage);
      setDynamicSystemSolver(this);
    }
    else if (stage == preInit) {
      msg(Info) << "  initialising preInit ..." << endl;
      Group::init(stage);
      if (inverseKinetics)
        setUpInverseKinetics();
    }
    else if (stage == plotting) {
      msg(Info) << "  initialising plot-files ..." << endl;
      Group::init(stage);
#ifdef HAVE_OPENMBVCPPINTERFACE
      if (getPlotFeature(plotRecursive) == enabled)
        openMBVGrp->write(true, truncateSimulationFiles);
#endif
      H5::File::reopenAllFilesAsSWMR();
      msg(Info) << "...... done initialising." << endl << endl;
    }
    else
      Group::init(stage);
  }

  int DynamicSystemSolver::solveConstraintsFixpointSingle(double t) {
    updaterFactors(t);

    checkConstraintsForTermination(t);
    if (term)
      return 0;

    int iter, level = 0;
    int checkTermLevel = 0;

    for (iter = 1; iter <= maxIter; iter++) {

      if (level < decreaseLevels.size() && iter > decreaseLevels(level)) {
        level++;
        decreaserFactors();
        msg(Warn) << endl << "decreasing r-factors at iter = " << iter << endl;
        if (warnLevel >= 2)
          msg(Warn) << endl << "decreasing r-factors at iter = " << iter << endl;
      }

      Group::solveConstraintsFixpointSingle(t);

      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkConstraintsForTermination(t);
        if (term)
          break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveImpactsFixpointSingle(double t, double dt) {
    updaterFactors(t);

    checkImpactsForTermination(t,dt);
    if (term)
      return 0;

    int iter, level = 0;
    int checkTermLevel = 0;

    for (iter = 1; iter <= maxIter; iter++) {

      if (level < decreaseLevels.size() && iter > decreaseLevels(level)) {
        level++;
        decreaserFactors();
        msg(Warn) << endl << "decreasing r-factors at iter = " << iter << endl;
        if (warnLevel >= 2)
          msg(Warn) << endl << "decreasing r-factors at iter = " << iter << endl;
      }

      Group::solveImpactsFixpointSingle(t,dt);

      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkImpactsForTermination(t,dt);
        if (term)
          break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveConstraintsGaussSeidel(double t) {
    checkConstraintsForTermination(t);
    if (term)
      return 0;

    int iter;
    int checkTermLevel = 0;

    for (iter = 1; iter <= maxIter; iter++) {
      Group::solveConstraintsGaussSeidel(t);
      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkConstraintsForTermination(t);
        if (term)
          break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveImpactsGaussSeidel(double t, double dt) {
    checkImpactsForTermination(t,dt);
    if (term)
      return 0;

    int iter;
    int checkTermLevel = 0;

    for (iter = 1; iter <= maxIter; iter++) {
      Group::solveImpactsGaussSeidel(t,dt);
      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkImpactsForTermination(t,dt);
        if (term)
          break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveConstraintsRootFinding(double t) {
    updaterFactors(t);

    int iter;
    int checkTermLevel = 0;

    updateresRef(resParent(0, laSize - 1));
    Group::solveConstraintsRootFinding(t);

    double nrmf0 = nrm2(res);
    Vec res0 = res.copy();

    checkConstraintsForTermination(t);
    if (term)
      return 0;

    DiagMat I(la.size(), INIT, 1);
    for (iter = 1; iter < maxIter; iter++) {

      if (Jprox.size() != la.size())
        Jprox.resize(la.size(), NONINIT);

      if (numJac) {
        for (int j = 0; j < la.size(); j++) {
          const double xj = la(j);
          double dx = epsroot() / 2.;
          
          do
            dx += dx;
          while (fabs(xj + dx - la(j)) < epsroot());

          la(j) += dx;
          Group::solveConstraintsRootFinding(t);
          la(j) = xj;
          Jprox.col(j) = (res - res0) / dx;
        }
      }
      else
        jacobianConstraints(t);
      Vec dx;
      if (linAlg == LUDecomposition)
        dx >> slvLU(Jprox, res0);
      else if (linAlg == LevenbergMarquardt) {
        SymMat J = SymMat(JTJ(Jprox) + lmParm * I);
        dx >> slvLL(J, Jprox.T() * res0);
      }
      else if (linAlg == PseudoInverse)
        dx >> slvLS(Jprox, res0);
      else
        THROW_MBSIMERROR("Internal error");

      Vec La_old = la.copy();
      double alpha = 1;
      double nrmf = 1;
      for (int k = 0; k < maxDampingSteps; k++) {
        la = La_old - alpha * dx;
        Group::solveConstraintsRootFinding(t);
        nrmf = nrm2(res);
        if (nrmf < nrmf0)
          break;

        alpha *= .5;
      }
      nrmf0 = nrmf;
      res0 = res;

      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkConstraintsForTermination(t);
        if (term)
          break;
      }
    }
    return iter;
  }

  int DynamicSystemSolver::solveImpactsRootFinding(double t, double dt) {
    updaterFactors(t);

    int iter;
    int checkTermLevel = 0;
    
    updateresRef(resParent(0, laSize - 1));
    Group::solveImpactsRootFinding(t,dt);

    double nrmf0 = nrm2(res);
    Vec res0 = res.copy();

    checkImpactsForTermination(dt);
    if (term)
      return 0;

    DiagMat I(La.size(), INIT, 1);
    for (iter = 1; iter < maxIter; iter++) {

      if (Jprox.size() != La.size())
        Jprox.resize(La.size(), NONINIT);

      if (numJac) {
        for (int j = 0; j < La.size(); j++) {
          const double xj = La(j);
          double dx = .5 * epsroot();
          
          do
            dx += dx;
          while (fabs(xj + dx - La(j)) < epsroot());
          
          La(j) += dx;
          Group::solveImpactsRootFinding(t,dt);
          La(j) = xj;
          Jprox.col(j) = (res - res0) / dx;
        }
      }
      else
        jacobianImpacts(t,dt);
      Vec dx;
      if (linAlg == LUDecomposition)
        dx >> slvLU(Jprox, res0);
      else if (linAlg == LevenbergMarquardt) {
        SymMat J = SymMat(JTJ(Jprox) + lmParm * I);
        dx >> slvLL(J, Jprox.T() * res0);
      }
      else if (linAlg == PseudoInverse)
        dx >> slvLS(Jprox, res0);
      else
        THROW_MBSIMERROR("Internal error");

      Vec La_old = La.copy();
      double alpha = 1.;
      double nrmf = 1;
      for (int k = 0; k < maxDampingSteps; k++) {
        La = La_old - alpha * dx;
        Group::solveImpactsRootFinding(t,dt);
        nrmf = nrm2(res);
        if (nrmf < nrmf0)
          break;
        alpha *= .5;
      }
      nrmf0 = nrmf;
      res0 = res;

      if (checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
        checkTermLevel++;
        checkImpactsForTermination(dt);
        if (term)
          break;
      }
    }
    return iter;
  }

  void DynamicSystemSolver::checkConstraintsForTermination(double t) {
    term = true;

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (**i).checkConstraintsForTermination(t);
      if (term == false) {
        return;
      }
    }
  }

  void DynamicSystemSolver::checkImpactsForTermination(double t, double dt) {
    term = true;

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (**i).checkImpactsForTermination(t,dt);
      if (term == false)
        return;
    }
  }

  void DynamicSystemSolver::updateh(double t, int j) {
    h[j].init(0);
    Group::updateh(t, j);
    updh[j] = false;
    checkExitRequest(); // updateh is called by all solvers
  }

  Mat DynamicSystemSolver::dhdq(double t, int lb, int ub) {
    if (lb != 0 || ub != 0) {
      assert(lb >= 0);
      assert(ub <= qSize);
    }
    else if (lb == 0 && ub == 0) {
      lb = 0;
      ub = qSize;
    }
    double delta = epsroot();
    Mat J(hSize[0], qSize, INIT, 0.0);
    throw;
//    updateStateDependentVariables(t);
    updateg(t);
    updategd(t);
    updateT(t);
    updateh(t);
    Vec hOld = h[0].copy();
    for (int i = lb; i < ub; i++) {
      double qtmp = q(i);
      q(i) += delta;
      //updateStateDependentVariables(t);
      updateg(t);
      updategd(t);
      updateT(t);
      updateh(t);
      J.col(i) = (h[0] - hOld) / delta;
      q(i) = qtmp;
    }
    h[0] = hOld;

    //updateStateDependentVariables(t);
    updateg(t);
    updategd(t);
    updateT(t);
    updateh(t);

    return J;
  }

  Mat DynamicSystemSolver::dhdu(double t, int lb, int ub) {
    if (lb != 0 || ub != 0) {
      assert(lb >= 0);
      assert(ub <= uSize[0]);
    }
    else if (lb == 0 && ub == 0) {
      lb = 0;
      ub = uSize[0];
    }
    double delta = epsroot();
    Mat J(hSize[0], uSize[0], INIT, 0.0);
    throw;
    //updateStateDependentVariables(t);
    updateg(t);
    updategd(t);
    updateT(t);
    updateh(t);
    Vec hOld = h[0].copy();
    for (int i = lb; i < ub; i++) {
      //msg(Info) << "bin bei i=" << i << endl;
      double utmp = u(i);
      u(i) += delta;
      //updateStateDependentVariables(t);
      //updateg(t);
      updategd(t);
      //updateT(t); 
      updateh(t);
      J.col(i) = (h[0] - hOld) / delta;
      u(i) = utmp;
    }
    h[0] = hOld;
    //updateStateDependentVariables(t);
    updategd(t);
    updateh(t);

    return J;
  }

  Mat DynamicSystemSolver::dhdx(double t) {
    THROW_MBSIMERROR("Internal error");
  }

  Vec DynamicSystemSolver::dhdt(double t) {
    THROW_MBSIMERROR("Internal error");
  }

  void DynamicSystemSolver::updateT(double t) {
    Group::updateT(t);
    updT = false;
  }

  void DynamicSystemSolver::updateM(double t, int i) {
    M[i].init(0);
    Group::updateM(t, i);
    updM[i] = false;
  }

  void DynamicSystemSolver::updateLLM(double t, int i) {
    Group::updateLLM(t, i);
    updLLM[i] = false;
  }

//  void DynamicSystemSolver::updateStateDependentVariables(double t) {
//    Group::updateStateDependentVariables(t);
//
//    if (integratorExitRequest) { // if the integrator has not exit after a integratorExitRequest
//      msg(Warn) << "MBSim: Integrator has not stopped integration! Terminate NOW the hard way!" << endl;
//      exit(1);
//    }
//
//    if (exitRequest) { // on exitRequest flush plot files and ask the integrator to exit
//      msg(Info) << "MBSim: Flushing HDF5 files and ask integrator to terminate!" << endl;
//      H5::File::flushAllFiles(); // flush files
//      integratorExitRequest = true;
//    }
//
//    H5::File::flushAllFilesIfRequested(); // flush files if requested by reader process
//  }

  void DynamicSystemSolver::updater(double t, int j) {
    r[j] = getV(t,j) * la; // cannot be called locally (hierarchically), because this adds some values twice to r for tree structures
    updr[j] = false;
  }

  void DynamicSystemSolver::updaterdt(double t, int j) {
    rdt[j] = getV(t,j) * La; // cannot be called locally (hierarchically), because this adds some values twice to r for tree structures
    updrdt[j] = false;
  }

  void DynamicSystemSolver::updatewb(double t) {
    wb.init(0);
    Group::updatewb(t);
    updwb = false;
  }

  void DynamicSystemSolver::updateg(double t) {
    Group::updateg(t);
    updg = false;
  }

  void DynamicSystemSolver::updategd(double t) {
    Group::updategd(t);
    updgd = false;
  }

  void DynamicSystemSolver::updateW(double t, int j) {
    W[j].init(0);
    Group::updateW(t, j);
    updW[j] = false;
  }

  void DynamicSystemSolver::updateV(double t, int j) {
    V[j] = getW(t,j);
    Group::updateV(t, j);
    updV[j] = false;
  }

  void DynamicSystemSolver::closePlot() {
    if (getPlotFeature(plotRecursive) == enabled) {
      Group::closePlot();
    }
  }

  int DynamicSystemSolver::solveConstraints(double t) {
    if (la.size() == 0)
      return 0;

    if (useOldla)
      initla();
    else
      la.init(0);

    int iter;
    Vec laOld;
    laOld << la;
    iter = (this->*solveConstraints_)(t); // solver election
    if (iter >= maxIter) {
      msg(Warn) << "\n";
      msg(Warn) << "Iterations: " << iter << "\n";
      msg(Warn) << "\nError: no convergence." << endl;
      if (stopIfNoConvergence) {
        if (dropContactInfo)
          dropContactMatrices(t);
        THROW_MBSIMERROR("Maximal Number of Iterations reached");
      }
      msg(Warn) << "Anyway, continuing integration..." << endl;
    }

    if (warnLevel >= 1 && iter > highIter)
      msg(Warn) << endl << "high number of iterations: " << iter << endl;

    if (useOldla)
      savela();

    return iter;
  }

  int DynamicSystemSolver::solveImpacts(double t, double dt) {
    if (La.size() == 0)
      return 0;
    double H = 1;
    if (dt > 0)
      H = dt;

    if (useOldla)
      initLa(H);
    else
      La.init(0);
    
    int iter;
    Vec LaOld;
    LaOld << La;
    iter = (this->*solveImpacts_)(t,dt); // solver election
    if (iter >= maxIter) {
      msg(Warn) << "\n";
      msg(Warn) << "Iterations: " << iter << "\n";
      msg(Warn) << "\nError: no convergence." << endl;
      if (stopIfNoConvergence) {
        if (dropContactInfo)
          dropContactMatrices(t);
        THROW_MBSIMERROR("Maximal Number of Iterations reached");
      }
      msg(Warn) << "Anyway, continuing integration..." << endl;
    }

    if (warnLevel >= 1 && iter > highIter)
      msg(Warn) << "high number of iterations: " << iter << endl;

    if (useOldla)
      saveLa(H);

    la = La/dt;

    return iter;
  }

  void DynamicSystemSolver::computeInitialCondition() {
    resetUpToDate();
    updateg(0);
    checkActive(0,1);
    updategd(0);
    checkActive(0,2);
    calclaSize(3);
    calcrFactorSize(3);
    updateWRef(WParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)), 0);
    updateVRef(VParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)), 0);
    updateWRef(WParent[1](Index(0, getuSize(1) - 1), Index(0, getlaSize() - 1)), 1);
    updateVRef(VParent[1](Index(0, getuSize(1) - 1), Index(0, getlaSize() - 1)), 1);
    updatelaRef(laParent(0, laSize - 1));
    updatewbRef(wbParent(0, laSize - 1));
    updaterFactorRef(rFactorParent(0, rFactorSize - 1));
  }

  Vec DynamicSystemSolver::deltau(const Vec &zParent, double t, double dt) {
    if (q() != zParent())
      updatezRef(zParent);

    updatedu(t, dt);
    return ud[0];
  }

  Vec DynamicSystemSolver::deltaq(const Vec &zParent, double t, double dt) {
    if (q() != zParent())
      updatezRef(zParent);
    updatedq(t, dt);

    return qd;
  }

  Vec DynamicSystemSolver::deltax(const Vec &zParent, double t, double dt) {
    if (q() != zParent()) {
      updatezRef(zParent);
    }
    updatedx(t, dt);
    return xd;
  }

  void DynamicSystemSolver::initz(Vec& z) {
    updatezRef(z);
    Group::initz();

    // Perform a projection of generalized positions and velocities at time t=0
    if(initialProjection) { 
      projectGeneralizedPositions(0, 1, true);
      projectGeneralizedVelocities(0, 1);
    }
  }

  int DynamicSystemSolver::solveConstraintsLinearEquations(double t) {
    la = slvLS(getG(t), -b);
    //la = slvLS(G, -(W[0].T() * slvLLFac(LLM[0], h[0]) + wb));
    return 1;
  }

  int DynamicSystemSolver::solveImpactsLinearEquations(double t, double dt) {
    La = slvLS(getG(t), -(getgd(t) + getW(t).T() * slvLLFac(getLLM(t), geth(t)) * dt));
    return 1;
  }

  void DynamicSystemSolver::updateG(double t, int j) {
    G << SqrMat(getW(t,j).T() * slvLLFac(getLLM(t,j), getV(t,j)));

    if (checkGSize)
      ; // Gs.resize();
    else if (Gs.cols() != G.size()) {
      static double facSizeGs = 1;
      if (G.size() > limitGSize && fabs(facSizeGs - 1) < epsroot())
        facSizeGs = double(countElements(G)) / double(G.size() * G.size()) * 1.5;
      Gs.resize(G.size(), int(G.size() * G.size() * facSizeGs));
    }
    Gs << G;

    updG = false;
  }

  void DynamicSystemSolver::updateb(double t) {
    updb = false;
  }

  void DynamicSystemSolver::decreaserFactors() {

    for (vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i)
      (*i)->decreaserFactors();
  }

  void DynamicSystemSolver::resize_(double t) {
      calcgdSize(2); // contacts which stay closed
      calclaSize(2); // contacts which stay closed
      calcrFactorSize(2); // contacts which stay closed

      updateWRef(WParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
      updateVRef(VParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
      updatelaRef(laParent(0, laSize - 1));
      updateLaRef(LaParent(0, laSize - 1));
      updategdRef(gdParent(0, gdSize - 1));
      if (impactSolver == RootFinding)
        updateresRef(resParent(0, laSize - 1));
      updaterFactorRef(rFactorParent(0, rFactorSize - 1));
  }

  void DynamicSystemSolver::update(const Vec &zParent, double t, int options) {
    throw;
  }

  void DynamicSystemSolver::zdot(const Vec &zParent, Vec &zdParent, double t) {
    if (qd() != zdParent()) {
      updatezdRef(zdParent);
    }
    setTime(t);
    zdot(zParent, t);
  }

  void DynamicSystemSolver::getLinkStatus(VecInt &LinkStatusExt, double t) {
    if (LinkStatusExt.size() < LinkStatusSize)
      LinkStatusExt.resize(LinkStatusSize);
    if (LinkStatus() != LinkStatusExt())
      updateLinkStatusRef(LinkStatusExt);
    updateLinkStatus(t);
  }

  void DynamicSystemSolver::getLinkStatusReg(VecInt &LinkStatusRegExt, double t) {
    if (LinkStatusRegExt.size() < LinkStatusRegSize)
      LinkStatusRegExt.resize(LinkStatusRegSize);
    if (LinkStatusReg() != LinkStatusRegExt())
      updateLinkStatusRegRef(LinkStatusRegExt);
    updateLinkStatusReg(t);
  }

  void DynamicSystemSolver::projectGeneralizedPositions(double t, int mode, bool fullUpdate) {
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
      THROW_MBSIMERROR("Internal error");

    calcgSize(gID);
    calccorrSize(corrID);
    updatecorrRef(corrParent(0, corrSize - 1));
    updategRef(gParent(0, gSize - 1));
    updatecorr(corrID);
    Vec nu(getuSize());
    calclaSize(laID);
    updateWRef(WParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
    SqrMat Gv = SqrMat(getW(t).T() * slvLLFac(getLLM(t), getW(t)));
    Mat T = getT(t);
    int iter = 0;
    while (nrmInf(getg(t) - corr) >= tolProj) {
      if (++iter > 500) {
        msg(Warn) << endl << "Error in DynamicSystemSolver: projection of generalized positions failed at t = " << t << "!" << endl;
        break;
      }
      Vec mu = slvLS(Gv, -getg(t) + getW(0,false).T() * nu + corr);
      Vec dnu = slvLLFac(getLLM(0,false), getW(0,false) * mu) - nu;
      nu += dnu;
      q += T * dnu;
      resetUpToDate();
      if(fullUpdate) Gv = SqrMat(getW(t).T() * slvLLFac(getLLM(t), getW(t)));
   }
    calclaSize(3);
    updateWRef(WParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
    calcgSize(0);
    updategRef(gParent(0, gSize - 1));
  }

  void DynamicSystemSolver::projectGeneralizedVelocities(double t, int mode) {
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
      THROW_MBSIMERROR("Internal error");
    calccorrSize(corrID); // IH
    if (corrSize) {
      calcgdSize(gdID); // IH
      updatecorrRef(corrParent(0, corrSize - 1));
      updategdRef(gdParent(0, gdSize - 1));
      updatecorr(corrID);

      calclaSize(gdID);
      updateWRef(WParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));

      if (laSize) {
        SqrMat Gv = SqrMat(getW(t).T() * slvLLFac(getLLM(t), getW(t)));
        Vec mu = slvLS(Gv, -getgd(t) + corr);
        u += slvLLFac(getLLM(), getW() * mu);
        resetUpToDate();
      }
      calclaSize(3);
      updateWRef(WParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
      calcgdSize(1);
      updategdRef(gdParent(0, gdSize - 1));
    }
  }

  void DynamicSystemSolver::savela(double dt) {
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).savela(dt);
  }

  void DynamicSystemSolver::initla(double dt) {
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).initla(dt);
  }

  void DynamicSystemSolver::saveLa(double dt) {
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).saveLa(dt);
  }

  void DynamicSystemSolver::initLa(double dt) {
    for (vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).initLa(dt);
  }

  double DynamicSystemSolver::computePotentialEnergy() {
    double Vpot = 0.0;

    vector<Object*>::iterator i;
    for (i = object.begin(); i != object.end(); ++i)
      Vpot += (**i).computePotentialEnergy();

    vector<Link*>::iterator ic;
    for (ic = link.begin(); ic != link.end(); ++ic)
      Vpot += (**ic).computePotentialEnergy();
    return Vpot;
  }

  void DynamicSystemSolver::addElement(Element *element_) {
    Object* object_ = dynamic_cast<Object*>(element_);
    Link* link_ = dynamic_cast<Link*>(element_);
    if (object_)
      addObject(object_);
    else if (link_)
      addLink(link_);
    else {
      THROW_MBSIMERROR("(DynamicSystemSolver: addElement()): No such type of Element to add!");
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
    //   if(!(i1<object.size())||!(i2<link.size())||!(i3<extraDynamic.size())) cout << "Error: The DynamicSystemSolver " << this->name <<" comprises no element " << name << "!" << endl; 
    //   assert(i1<object.size()||i2<link.size()||!(i3<extraDynamic.size()));
    return NULL;
  }

  string DynamicSystemSolver::getSolverInfo() {
    stringstream info;

    if (impactSolver == GaussSeidel)
      info << "GaussSeidel";
    else if (impactSolver == LinearEquations)
      info << "LinearEquations";
    else if (impactSolver == FixedPointSingle)
      info << "FixedPointSingle";
    else if (impactSolver == FixedPointTotal)
      info << "FixedPointTotal";
    else if (impactSolver == RootFinding)
      info << "RootFinding";

    // Gauss-Seidel & solveLL do not depend on the following ...
    if (impactSolver != GaussSeidel && impactSolver != LinearEquations) {
      info << "(";

      // r-Factor strategy
      if (strategy == global)
        info << "global";
      else if (strategy == local)
        info << "local";

      // linear algebra for RootFinding only
      if (impactSolver == RootFinding) {
        info << ",";
        if (linAlg == LUDecomposition)
          info << "LU";
        else if (linAlg == LevenbergMarquardt)
          info << "LM";
        else if (linAlg == PseudoInverse)
          info << "PI";
      }
      info << ")";
    }
    return info.str();
  }

  void DynamicSystemSolver::dropContactMatrices(double t) {
    msg(Info) << "dropping contact matrices to file <dump_matrices.asc>" << endl;
    ofstream contactDrop("dump_matrices.asc");

    contactDrop << "constraint functions g" << endl << getg(t) << endl << endl;
    contactDrop << endl;
    contactDrop << "mass matrix M" << endl << getM(t) << endl << endl;
    contactDrop << "generalized force vector h" << endl << geth(t) << endl << endl;
    contactDrop << "generalized force directions W" << endl << getW(t) << endl << endl;
    contactDrop << "generalized force directions V" << endl << getV(t) << endl << endl;
    contactDrop << "mass action matrix G" << endl << getG(t) << endl << endl;
    contactDrop << "vector wb" << endl << getwb(t) << endl << endl;
    contactDrop << endl;
    contactDrop << "constraint velocities gp" << endl << getgd(t) << endl << endl;
    contactDrop << "non-holonomic part in gp; b" << endl << b << endl << endl;
    contactDrop << "Lagrange multipliers la" << endl << la << endl << endl;
    contactDrop << "Lagrange multipliers La" << endl << La << endl << endl;
    contactDrop.close();
  }

  void DynamicSystemSolver::sigInterruptHandler(int) {
    msgStatic(Info) << "MBSim: Received user interrupt or terminate signal!" << endl;
    exitRequest = true;
  }

  void DynamicSystemSolver::sigAbortHandler(int) {
    signal(SIGABRT, SIG_DFL);
    msgStatic(Info) << "MBSim: Received abort signal! Flushing HDF5 files (this may crash) and abort!" << endl;
    H5::File::flushAllFiles(); // This call is unsafe, since it may call (signal) unsafe functions. However, we call it here
    raise(SIGABRT);
  }

  void DynamicSystemSolver::sigSegfaultHandler(int) {
    signal(SIGSEGV, SIG_DFL);
    msgStatic(Info) << "MBSim: Received segmentation fault signal! Flushing HDF5 files (this may crash again) and abort!" << endl;
    H5::File::flushAllFiles(); // This call is unsafe, since it may call (signal) unsafe functions. However, we call it here
    raise(SIGSEGV);
  }

  void DynamicSystemSolver::checkExitRequest() {
    if (integratorExitRequest) { // if the integrator has not exit after a integratorExitRequest
      msg(Warn) << "MBSim: Integrator has not stopped integration! Terminate NOW the hard way!" << endl;
      exit(1);
    }

    if (exitRequest) { // on exitRequest flush plot files and ask the integrator to exit
      msg(Info) << "MBSim: Flushing HDF5 files and ask integrator to terminate!" << endl;
      H5::File::flushAllFiles(); // flush files
      integratorExitRequest = true;
    }

    H5::File::flushAllFilesIfRequested(); // flush files if requested by reader process
  }

  void DynamicSystemSolver::writez(string fileName, bool formatH5) {
    if (formatH5) {
      H5::File file(fileName, H5::File::write);

      Group::writez(&file);
    }
    else {
      ofstream file(fileName.c_str());
      file.setf(ios::scientific);
      file.precision(numeric_limits<double>::digits10 + 1);
      for (int i = 0; i < q.size(); i++)
        file << q(i) << endl;
      for (int i = 0; i < u.size(); i++)
        file << u(i) << endl;
      for (int i = 0; i < x.size(); i++)
        file << x(i) << endl;
      file.close();
    }
  }

  void DynamicSystemSolver::readz0(string fileName) {
    H5::File file(fileName, H5::File::read);

    DynamicSystem::readz0(&file);

    READZ0 = true;
  }

  void DynamicSystemSolver::updatezRef(const Vec &zParent) {

    q >> (zParent(0, qSize - 1));
    u >> (zParent(qSize, qSize + uSize[0] - 1));
    x >> (zParent(qSize + uSize[0], qSize + uSize[0] + xSize - 1));

    updateqRef(q);
    updateuRef(u);
    updatexRef(x);
  }

  void DynamicSystemSolver::updatezdRef(const Vec &zdParent) {

    qd >> (zdParent(0, qSize - 1));
    ud[0] >> (zdParent(qSize, qSize + uSize[0] - 1));
    xd >> (zdParent(qSize + uSize[0], qSize + uSize[0] + xSize - 1));

    updateqdRef(qd);
    updateudRef(ud[0]);
    updatexdRef(xd);

    updateudallRef(ud[0]);
  }

  void DynamicSystemSolver::updaterFactors(double t) {
    if (strategy == global) {
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

      THROW_MBSIMERROR("(DynamicSystemSolver::updaterFactors()): Global r-Factor strategy currently not not available.");
    }
    else if (strategy == local)
      Group::updaterFactors(t);
    else
      THROW_MBSIMERROR("(DynamicSystemSolver::updaterFactors()): Unknown strategy.");
  }

  void DynamicSystemSolver::computeConstraintForces(double t) {
    la = slvLS(getG(t), -(getW(t).T() * slvLLFac(getLLM(t), geth(t)) + getwb(t))); // slvLS because of undeterminded system of equations
  }

  void DynamicSystemSolver::constructor() {
    integratorExitRequest = false;
    setPlotFeatureRecursive(plotRecursive, enabled);
    setPlotFeature(separateFilePerGroup, enabled);
    setPlotFeatureForChildren(separateFilePerGroup, disabled);
    setPlotFeatureRecursive(openMBV, enabled);
  }

  void DynamicSystemSolver::initializeUsingXML(DOMElement *element) {
    Group::initializeUsingXML(element);
    DOMElement *e;
    // search first Environment element
    e = E(element)->getFirstElementChildNamed(MBSIM%"environments")->getFirstElementChild();

    while (e) {
      ObjectFactory::createAndInit<Environment>(e);
      e = e->getNextElementSibling();
    }

    e = E(element)->getFirstElementChildNamed(MBSIM%"solverParameters");
    if (e) {
      DOMElement * ee;
      ee = E(e)->getFirstElementChildNamed(MBSIM%"constraintSolver");
      if (ee) {
        Solver solver=FixedPointSingle;
        std::string str=X()%E(ee)->getFirstTextChild()->getData();
        str=str.substr(1,str.length()-2);
        if(str=="FixedPointTotal") solver=FixedPointTotal;
        else if(str=="FixedPointSingle") solver=FixedPointSingle;
        else if(str=="GaussSeidel") solver=GaussSeidel;
        else if(str=="LinearEquations") solver=LinearEquations;
        else if(str=="RootFinding") solver=RootFinding;
        setConstraintSolver(solver);
      }
      ee = E(e)->getFirstElementChildNamed(MBSIM%"impactSolver");
      if (ee) {
        Solver solver=FixedPointSingle;
        std::string str=X()%E(ee)->getFirstTextChild()->getData();
        str=str.substr(1,str.length()-2);
        if(str=="FixedPointTotal") solver=FixedPointTotal;
        else if(str=="FixedPointSingle") solver=FixedPointSingle;
        else if(str=="GaussSeidel") solver=GaussSeidel;
        else if(str=="LinearEquations") solver=LinearEquations;
        else if(str=="RootFinding") solver=RootFinding;
        setImpactSolver(solver);
      }
      ee = E(e)->getFirstElementChildNamed(MBSIM%"numberOfMaximalIterations");
      if (ee)
        setMaxIter(getInt(ee));
      ee = E(e)->getFirstElementChildNamed(MBSIM%"tolerances");
      if (ee) {
        DOMElement * eee;
        eee = E(ee)->getFirstElementChildNamed(MBSIM%"projection");
        if (eee)
          setProjectionTolerance(getDouble(eee));
        eee = E(ee)->getFirstElementChildNamed(MBSIM%"g");
        if (eee)
          setgTol(getDouble(eee));
        eee = E(ee)->getFirstElementChildNamed(MBSIM%"gd");
        if (eee)
          setgdTol(getDouble(eee));
        eee = E(ee)->getFirstElementChildNamed(MBSIM%"gdd");
        if (eee)
          setgddTol(getDouble(eee));
        eee = E(ee)->getFirstElementChildNamed(MBSIM%"la");
        if (eee)
          setlaTol(getDouble(eee));
        eee = E(ee)->getFirstElementChildNamed(MBSIM%"La");
        if (eee)
          setLaTol(getDouble(eee));
      }
    }
    e = E(element)->getFirstElementChildNamed(MBSIM%"inverseKinetics");
    if (e)
      setInverseKinetics(Element::getBool(e));
    e = E(element)->getFirstElementChildNamed(MBSIM%"initialProjection");
    if (e)
      setInitialProjection(Element::getBool(e));
    e = E(element)->getFirstElementChildNamed(MBSIM%"useConstraintSolverForPlot");
    if (e)
      setUseConstraintSolverForPlot(Element::getBool(e));
  }

  DOMElement* DynamicSystemSolver::writeXMLFile(DOMNode *parent) {
    DOMElement *ele0 = Group::writeXMLFile(parent);

//    DOMElement *ele1 = new DOMElement(MBSIM%"environments");
//    MBSimEnvironment::getInstance()->writeXMLFile(ele1);
//    ele0->LinkEndChild(ele1);
//
//    ele1 = new DOMElement(MBSIM%"solverParameters");
//    if (contactSolver != FixedPointSingle) {
//      DOMElement *ele2 = new DOMElement(MBSIM%"constraintSolver");
//      if (contactSolver == FixedPointTotal)
//        ele2->LinkEndChild(new DOMElement(MBSIM%"FixedPointTotal"));
//      else if (contactSolver == FixedPointSingle)
//        ele2->LinkEndChild(new DOMElement(MBSIM%"FixedPointSingle"));
//      else if (contactSolver == GaussSeidel)
//        ele2->LinkEndChild(new DOMElement(MBSIM%"GaussSeidel"));
//      else if (contactSolver == LinearEquations)
//        ele2->LinkEndChild(new DOMElement(MBSIM%"LinearEquations"));
//      else if (contactSolver == RootFinding)
//        ele2->LinkEndChild(new DOMElement(MBSIM%"RootFinding"));
//      ele1->LinkEndChild(ele2);
//    }
//    if (impactSolver != FixedPointSingle) {
//      DOMElement *ele2 = new DOMElement(MBSIM%"impactSolver");
//      if (impactSolver == FixedPointTotal)
//        ele2->LinkEndChild(new DOMElement(MBSIM%"FixedPointTotal"));
//      else if (impactSolver == FixedPointSingle)
//        ele2->LinkEndChild(new DOMElement(MBSIM%"FixedPointSingle"));
//      else if (impactSolver == GaussSeidel)
//        ele2->LinkEndChild(new DOMElement(MBSIM%"GaussSeidel"));
//      else if (impactSolver == LinearEquations)
//        ele2->LinkEndChild(new DOMElement(MBSIM%"LinearEquations"));
//      else if (impactSolver == RootFinding)
//        ele2->LinkEndChild(new DOMElement(MBSIM%"RootFinding"));
//      ele1->LinkEndChild(ele2);
//    }
//    if (maxIter != 10000)
//      addElementText(ele1, MBSIM%"numberOfMaximalIterations", maxIter);
//    DOMElement *ele2 = new DOMElement(MBSIM%"tolerances");
//    if (tolProj > 1e-15)
//      addElementText(ele2, MBSIM%"projection", tolProj);
//    if (gTol > 1e-8)
//      addElementText(ele2, MBSIM%"g", gTol);
//    if (gdTol > 1e-10)
//      addElementText(ele2, MBSIM%"gd", gdTol);
//    if (gddTol > 1e-12)
//      addElementText(ele2, MBSIM%"gdd", gddTol);
//    if (laTol > 1e-12)
//      addElementText(ele2, MBSIM%"la", laTol);
//    if (LaTol > 1e-10)
//      addElementText(ele2, MBSIM%"La", LaTol);
//    ele1->LinkEndChild(ele2);
//    ele0->LinkEndChild(ele1);
//    if (inverseKinetics)
//      addElementText(ele0, MBSIM%"inverseKinetics", inverseKinetics);
//
    return ele0;
  }

  DynamicSystemSolver* DynamicSystemSolver::readXMLFile(const string &filename) {
    shared_ptr<DOMParser> parser=DOMParser::create(false);
    shared_ptr<DOMDocument> doc=parser->parse(filename);
    DOMElement *e = doc->getDocumentElement();
    DynamicSystemSolver *dss = dynamic_cast<DynamicSystemSolver*>(ObjectFactory::createAndInit<Group>(e));
    return dss;
  }

  void DynamicSystemSolver::writeXMLFile(const string &name) {
//    TiXmlDocument doc;
//    TiXmlDeclaration *decl = new TiXmlDeclaration("1.0", "UTF-8", "");
//    doc.LinkEndChild(decl);
//    writeXMLFile(&doc);
//    map<string, string> nsprefix = XMLNamespaceMapping::getNamespacePrefixMapping();
//    unIncorporateNamespace(doc.getFirstElementChild(), nsprefix);
//    doc.SaveFile((name.length() > 10 && name.substr(name.length() - 10, 10) == ".mbsim.xml") ? name : name + ".mbsim.xml");
  }

  void DynamicSystemSolver::addToGraph(Graph* graph, SqrMat &A, int i, vector<Element*>& eleList) {
    Object *obj = dynamic_cast<Object*>(eleList[i]);
    if(obj) {
      eleList[i]->setName("Object_graph_"+lexical_cast<string>(i)); // just a unique local name
      graph->addObject(eleList[i]->computeLevel(), obj);
    }
    A(i, i) = -1;

    for (int j = 0; j < A.cols(); j++)
      if (A(i, j) > 0 && fabs(A(j, j) + 1) > epsroot()) // child node of object i
        addToGraph(graph, A, j, eleList);
  }

  void DynamicSystemSolver::shift(Vec &zParent, const VecInt &jsv_, double t) {
    if(msgAct(Debug))
      msg(Debug) << "System shift at t = " << t << "." << endl;

    resetUpToDate();
    useOldla = false;

    if (q() != zParent()) {
      updatezRef(zParent);
    }
    jsv = jsv_;

    checkRoot(t);
    int maxj = getRootID();
    if (maxj == 3) { // impact (velocity jump)
      checkActive(t,6); // decide which contacts have closed
      //msg(Info) << "stoss" << endl;

      calcgdSize(1); // IG
      updategdRef(gdParent(0, gdSize - 1));
      calclaSize(1); // IG
      calcrFactorSize(1); // IG
      updateWRef(WParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
      updateVRef(VParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
      updatelaRef(laParent(0, laSize - 1));
      updateLaRef(LaParent(0, laSize - 1));
      updaterFactorRef(rFactorParent(0, rFactorSize - 1));

      V[0] = getW(t); //updateV(t) not allowed here
      updV[0] = false;

      b << getgd(t); // b = gd + trans(W)*slvLLFac(LLM,h)*dt with dt=0
      solveImpacts(t);
      u += deltau(zParent, t, 0);
      resetUpToDate();
      checkActive(t,3); // neuer Zustand nach Stoss
      // Projektion:
      // - es müssen immer alle Größen projiziert werden
      // - neuer Zustand ab hier bekannt
      // - Auswertung vor Setzen von gActive und gdActive
      projectGeneralizedPositions(t, 3);
      // Projektion der Geschwindikgeiten erst am Schluss
      //projectGeneralizedVelocities(t,3);

      if (laSize) {

        calclaSize(3); // IH
        calcrFactorSize(3); // IH
        updateWRef(WParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
        updateVRef(VParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
        updatelaRef(laParent(0, laSize - 1));
        updatewbRef(wbParent(0, laSize - 1));
        updaterFactorRef(rFactorParent(0, rFactorSize - 1));

        b << getW(t).T() * slvLLFac(getLLM(t), geth(t)) + getwb(t);
        solveConstraints(t);

        checkActive(t,4);
        projectGeneralizedPositions(t, 2);
        projectGeneralizedVelocities(t, 2);
      }
    }
    else if (maxj == 2) { // transition from slip to stick (acceleration jump)
      //msg(Info) << "haften" << endl;
      checkActive(t,7); // decide which contacts may stick

      calclaSize(3); // IH
      calcrFactorSize(3); // IH
      updateWRef(WParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
      updateVRef(VParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
      updatelaRef(laParent(0, laSize - 1));
      updatewbRef(wbParent(0, laSize - 1));
      updaterFactorRef(rFactorParent(0, rFactorSize - 1));

      if (laSize) {
        b << getW(t).T() * slvLLFac(getLLM(t), geth(t)) + getwb(t);
        solveConstraints(t);

        checkActive(t,4);

        projectGeneralizedPositions(t, 2);
        projectGeneralizedVelocities(t, 2);
      }
    }
    else if (maxj == 1) { // contact opens or transition from stick to slip
      checkActive(t,8);

      projectGeneralizedPositions(t, 1);
      projectGeneralizedVelocities(t, 1);
    }
    checkActive(t,5); // final update von gActive, ...
    calclaSize(3); // IH
    calcrFactorSize(3); // IH
    updateWRef(WParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
    updateVRef(VParent[0](Index(0, getuSize() - 1), Index(0, getlaSize() - 1)));
    updatelaRef(laParent(0, laSize - 1));
    updatewbRef(wbParent(0, laSize - 1));
    updaterFactorRef(rFactorParent(0, rFactorSize - 1));

    setRootID(0);
    useOldla = true;
  }

  void DynamicSystemSolver::getsv(const Vec& zParent, Vec& svExt, double t) {
    resetUpToDate();
    if (sv() != svExt()) {
      updatesvRef(svExt);
    }

    if (q() != zParent())
      updatezRef(zParent);

    if (qd() != zdParent())
      updatezdRef(zdParent);

    if (laSize) {
      b << getW(t).T() * slvLLFac(getLLM(t), geth(t)) + getwb(t);
      solveConstraints(t);
    }
    updateStopVector(t);
    sv(sv.size() - 1) = 1;
  }

  Vec DynamicSystemSolver::zdot(const Vec &zParent, double t) {
    resetUpToDate();
    if (q() != zParent()) {
      updatezRef(zParent);
    }
    if (laSize) computeConstraintForces(t);
    updatezd(t);

    return zdParent;
  }

  void DynamicSystemSolver::plot(const fmatvec::Vec& zParent, double t, double dt) {
    resetUpToDate();
    if (q() != zParent()) {
      updatezRef(zParent);
    }

    if (qd() != zdParent())
      updatezdRef(zdParent);
    setTime(t);
    updateWRef(WParent[1](Index(0, getuSize(1) - 1), Index(0, getlaSize() - 1)), 1);
    updateVRef(VParent[1](Index(0, getuSize(1) - 1), Index(0, getlaSize() - 1)), 1);
    if (laSize) {
      if(useConstraintSolverForPlot) {
        b << getW(t).T() * slvLLFac(getLLM(t), geth(t)) + getwb(t);
        solveConstraints(t);
      }
      else
        computeConstraintForces(t);
    }

    updatezd(t);
    if (true) {
      int n = getWInverseKinetics(t,1).cols();
      int m1 = WInverseKinetics[1].rows();
      int m2 = getbInverseKinetics(t).rows();
      Mat A(m1 + m2, n);
      Vec b(m1 + m2);
      A(Index(0, m1 - 1), Index(0, n - 1)) = WInverseKinetics[1];
      A(Index(m1, m1 + m2 - 1), Index(0, n - 1)) = bInverseKinetics;
      b(0, m1 - 1) = -geth(t,1) - getr(t,1);
      laInverseKinetics = slvLL(JTJ(A), A.T() * b);
    }

    DynamicSystemSolver::plot();

    if (++flushCount > flushEvery) {
      flushCount = 0;
      H5::File::flushAllFiles();
    }
  }

  // TODO: Momentan für TimeStepping benötigt
  void DynamicSystemSolver::plot2(const fmatvec::Vec& zParent, double t, double dt) {
    if (q() != zParent()) {
      updatezRef(zParent);
    }

    if (qd() != zdParent())
      updatezdRef(zdParent);

    setTime(t);

    plot();

    if (++flushCount > flushEvery) {
      flushCount = 0;
      H5::File::flushAllFiles();
    }
  }

  void DynamicSystemSolver::resetUpToDate() {
    updT = true;
    updh[0] = true;
    updh[1] = true;
    updr[0] = true;
    updr[1] = true;
    updrdt[0] = true;
    updrdt[1] = true;
    updM[0] = true;
    updM[1] = true;
    updLLM[0] = true;
    updLLM[1] = true;
    updW[0] = true;
    updW[1] = true;
    updV[0] = true;
    updV[1] = true;
    updwb = true;
    updg = true;
    updgd = true;
    updG = true;
    updb = true;
    Group::resetUpToDate();
  }

  const SqrMat& DynamicSystemSolver::getG(double t) {
    if(updG) updateG(t);
    return G;
  }

  const SparseMat& DynamicSystemSolver::getGs(double t) {
    if(updG) updateG(t);
    return Gs;
  }

  const Vec& DynamicSystemSolver::getb(double t) {
    if(updb) updateb(t);
    return b;
  }

}
