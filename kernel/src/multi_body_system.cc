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
#include <config.h>
#include "multi_body_system.h"
#include "port.h"
#include "contour.h"
#include "link.h"
#include "extra_dynamic_interface.h"
#include "integrator.h"
#include "eps.h"
#ifndef MINGW
#  include <sys/stat.h>
#else
#  include <io.h>
#  define mkdir(a,b) mkdir(a)
#endif

namespace MBSim {

  MultiBodySystem::MultiBodySystem() : Object("Default"), gSize(0), laSize(0), rFactorSize(0), svSize(0), svInd(0), grav(3), activeConstraintsChanged(true), directoryName("Default") , numJac(false), maxIter(10000), highIter(1000), stopIfNoConvergence(false), solver(FixedPointSingle), strategy(local), warnLevel(0), useOldla(true), linAlg(LUDecomposition), maxDampingSteps(3), lmParm(0.001) , preIntegrator(NULL) , nHSLinksSingleValuedFixed(0), nHSLinksSetValuedFixed(0), checkGSize(true), limitGSize(500){
  } 

  MultiBodySystem::MultiBodySystem(const string &projectName) : Object(projectName), gSize(0), laSize(0), rFactorSize(0), svSize(0), svInd(0), grav(3), activeConstraintsChanged(true),  directoryName(projectName) , numJac(false), maxIter(10000), highIter(1000), stopIfNoConvergence(false),solver(FixedPointSingle), strategy(local), warnLevel(0), useOldla(true), linAlg(LUDecomposition), maxDampingSteps(3), lmParm(0.001), preIntegrator(NULL), nHSLinksSingleValuedFixed(0), nHSLinksSetValuedFixed(0), checkGSize(true), limitGSize(500) {
  }

  MultiBodySystem::~MultiBodySystem() {
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i)
      delete *i;
  } 

  void MultiBodySystem::init() {
    cout << endl << "Initialising MultiBodySystem " << fullName << " ......" << endl;
    // Ausgabeverzeichnis festlegen
    setDirectory();

    // Vektor-Dimensionierung
    cout << "  setting dimensions of ..." << endl;
    // Objects
    if(objects.size()>0)  cout << "      Object parameters" << endl;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) {
      (*i)->calcSize();
      (*i)->setqInd(qSize);
      (*i)->setuInd(uSize);
      (*i)->setxInd(xSize);
      qSize += (*i)->getqSize();
      uSize += (*i)->getuSize();
      xSize += (*i)->getxSize();
    }

    // Links
    if(links.size()>0)  cout << "      Link parameters" << endl;

    for(vector<Link*>::iterator il = links.begin(); il != links.end(); ++il) {
      (*il)->calcSize();
      if((*il)->isSetValued()) {
	(*il)->setgInd(gSize);
	(*il)->setlaInd(laSize);
	(*il)->setrFactorInd(rFactorSize);
	gSize += (*il)->getgSize();
	laSize += (*il)->getlaSize();
	rFactorSize += (*il)->getrFactorSize();
      }

      (*il)->setxInd(xSize);
      xSize += (*il)->getxSize();

      (*il)->setsvInd(svSize);
      svSize += (*il)->getsvSize();
    }

    // EDIs
    if(EDI.size()>0)  cout << "      EDI parameters" << endl;

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF!= EDI.end(); ++iF) {
      (*iF)->setxInd(xSize);
      xSize += (*iF)->getxSize();
    }


    // TODO Speicherproblem bei vielen moeglichen Kontakten
    if(laSize>8000)
      laSize=8000;
    MParent.resize(getuSize());
    TParent.resize(getqSize(),getuSize());
    LLMParent.resize(getuSize());
    WParent.resize(getuSize(),getlaSize());
    GParent.resize(getlaSize());
    wParent.resize(getlaSize());
    bParent.resize(getlaSize());
    laParent.resize(getlaSize());
    dlaParent.resize(getlaSize());
    rFactorParent.resize(getlaSize());
    sParent.resize(getlaSize());
    resParent.resize(getlaSize());
    gParent.resize(getgSize());
    gdParent.resize(getlaSize());
    zdParent.resize(getzSize());
    hParent.resize(getuSize());
    rParent.resize(getuSize());
    fParent.resize(getxSize());
    svParent.resize(getsvSize());
    jsvParent.resize(getsvSize());

    updatesvRef(svParent);
    updatejsvRef(jsvParent);
    updatehRef(hParent);
    updaterRef(rParent);
    updateMRef(MParent);
    updateTRef(TParent);
    updateLLMRef(LLMParent);
    G.resize() >> GParent;
    W.resize() >> WParent;
    b.resize() >> bParent;
    w.resize() >> wParent;
    g.resize() >> gParent;

    updatezdRef(zdParent);

    // Init der einzelenen Komponenten
    cout << "  initialising ..." << endl;
    for(int i=0; i<T.cols(); i++)
      T(i,i) = 1;

    if(objects.size()>0)  cout << "      " << objects.size() << " Objects" << endl;
    Object::init();
    for (vector<Object*>::iterator i = objects.begin(); i != objects.end(); ++i)
      (**i).init();

    if(links.size()>0)    cout << "      " << links.size()   << " Links" << endl;
    for (vector<Link*>::iterator i = links.begin(); i != links.end(); ++i) {
      (**i).init();
      if(!(*i)->getHitSphereCheck()) {
	if((*i)->isSetValued()) {
	  nHSLinksSetValuedFixed++;
	  linkSetValued.push_back(*i);
	} else {
	  nHSLinksSingleValuedFixed++;
	  linkSingleValued.push_back(*i);
	}
      }
    }

    if(EDI.size()>0)    cout << "      " << EDI.size()   << " EDIs" << endl;
    for (vector<ExtraDynamicInterface*>::iterator i=EDI.begin(); i !=EDI.end(); ++i)
      (**i).init();

    // HitSphereLink
    if(HSLinks.size()>0) cout << "  building " << HSLinks.size() << " HitSphereLinks between Objects" << endl;
    for (vector<HitSphereLink*>::iterator i = HSLinks.begin(); i != HSLinks.end(); ++i)
      (**i).init();


    //   // complete the inituialisation by updating to initial state
    //   double t0 = 0.0;
    //   updateKinematics(t0);
    //   updateh(t0);
    //   // -----------
    for(vector<Link*>::iterator ic = links.begin(); ic != links.end(); ++ic) {
      if((*ic)->isSetValued()) 
	(**ic).updategRef();
    }
    checkActiveConstraints();

    // solver specific things
    cout << "  use solver \'" << getSolverInfo() << "\' for contact situations" << endl;
    if(solver == GaussSeidel) {
      solve_ = &MultiBodySystem::solveGaussSeidel;
    } else if(solver == LinearEquations) {
      solve_ = &MultiBodySystem::solveLinearEquations;
      cout << "Warnung: der gewählte Solver (solveLL) ist nur für die Berechnung von ausschließlich zweiseitig gebundenen Systemen zu verwenden!!!" <<endl;
    } else if(solver == FixedPointSingle) {
      solve_ = &MultiBodySystem::solveFixpointSingle;
    } else if(solver == FixedPointTotal) {
      solve_ = &MultiBodySystem::solveFixpointTotal;
    } else if(solver == RootFinding) {
      solve_ = &MultiBodySystem::solveRootFinding;
    } else {
      cout << "Error: unknown solver" << endl;
      throw 5;
    }

    //if(plotting) {
    cout << "  initialising plot-files ..." << endl;
    initPlotFiles();
    cout << "  writing parameter-files ..." << endl;
    plotParameters();
    //}

    cout << "...... done initialising." << endl << endl;
  }

  /*!
   * create directories for outputs
   */
  void MultiBodySystem::setDirectory() {
    int i;
    string projectDirectory;

    if(directoryName == name) { // fortlaufend nummerierte Verzeichnisse
      for(i=0; i<=99; i++) {

        stringstream number;
        number << "." << setw(2) << setfill('0') << i;
        projectDirectory = directoryName + number.str();
        int ret=mkdir(projectDirectory.c_str(),0777);
        if(ret==0) break; // Wenn anlegbar, dann ist es ein neues Verz.
      }

      cout << "  make directory \'" << projectDirectory << "\' for output processing" << endl;
    }
    else { // hart immer ins gleiche Verzeichnis
       projectDirectory = string(directoryName);

       int ret=mkdir(projectDirectory.c_str(),0777);
       if(ret==0) {
         cout << "  make directory \'" << projectDirectory << "\' for output processing" << endl;
       }
       else {
         cout << "  use existing directory \'" << projectDirectory << "\' for output processing" << endl;
       }
    }

    dirName = projectDirectory+"/";

    if(preIntegrator) {
      string preDir="PREINTEG";
      int ret=mkdir(preDir.c_str(),0777);
      if(ret==0) {
        cout << "Make directory " << preDir << " for Preintegration results." << endl;
      }
      else {
        cout << "Use existing directory " << preDir << " for Preintegration results." << endl;
      }
    }

    return;
  }

  void MultiBodySystem::checkActiveConstraints() {
    if(activeConstraintsChanged) {
      laSize = 0;
      rFactorSize = 0;

      linkSetValuedActive.clear();

      vector<Link*>::iterator ic;
      for(ic = linkSetValued.begin(); ic != linkSetValued.end(); ++ic) {
	if((*ic)->isActive()) {
	  linkSetValuedActive.push_back(*ic);
	  (*ic)->setlaInd(laSize);
	  (*ic)->setrFactorInd(rFactorSize);
	  laSize += (*ic)->getlaSize();
	  rFactorSize += (*ic)->getrFactorSize();
	}
      }

      W.resize() >> WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1));
      G.resize() >> GParent(Index(0,getlaSize()-1));
      b.resize() >> bParent(Index(0,getlaSize()-1));

      la.resize() >> laParent(Index(0,getlaSize()-1));
      gd.resize() >> gdParent(Index(0,getlaSize()-1));
      s.resize() >> sParent(Index(0,getlaSize()-1));
      // TODO Nur bei Newton
      res.resize() >> resParent(Index(0,getlaSize()-1));
      rFactor.resize() >> rFactorParent(Index(0,getrFactorSize()-1));
      for(ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) 
	(**ic).updateRef();

      activeConstraintsChanged = false;
      W.init(0);
      G.init(0);
    }
  }

  Vec MultiBodySystem::zdot(const Vec &zParent, double t) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    updateKinematics(t);
    updateLinksStage1(t);
    //  checkActiveConstraints();
    updateLinksStage2(t);

    updateh(t); 
    updateG(t); 
    computeConstraintForces(t); // Berechnet die Zwangskrafte aus der Bewegungsgleichung
    updater(t); 
    updatezd(t); // Läuft du

    return zdParent;
  }

  void MultiBodySystem::zdot(const Vec &zParent, Vec &zdParent, double t) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    if(qd()!=zdParent()) {
      updatezdRef(zdParent);
    }
    updateKinematics(t);
    updateLinksStage1(t);
    //  checkActiveConstraints();
    updateLinksStage2(t);

    updateh(t); 
    updateG(t); 
    computeConstraintForces(t); // Berechnet die Zwangskrafte aus der Bewegungsgleichung
    updater(t); 
    updatezd(t); // Läuft du
  }


  void MultiBodySystem::updatezRef(const Vec &zParent) {

    q >> ( zParent(0,qSize-1) );
    u >> ( zParent(qSize,qSize+uSize-1) );
    x >> ( zParent(qSize+uSize,qSize+uSize+xSize-1) );

    vector<Object*>::iterator i;     
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updatezRef();

    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic)
      (**ic).updatexRef();

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF != EDI.end(); ++iF)
      (**iF).updatexRef();
  }

  void MultiBodySystem::updateqdRef(const Vec &qdExt) {

    qd >> qdExt;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updateqdRef();
  }

  void MultiBodySystem::updateqRef(const Vec &qParent) {

    q >> qParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updateqRef();
  }

  void MultiBodySystem::updateuRef(const Vec &uParent) {

    u >> uParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updateuRef();
  }

  void MultiBodySystem::updatexRef(const Vec &xParent) {

    x >> xParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updatexRef();

    vector<Link*>::iterator i2;
    for(i2 = links.begin(); i2 != links.end(); ++i2) 
      (**i2).updatexRef();

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF != EDI.end(); ++iF) 
      (**iF).updatexRef();
  }


  void MultiBodySystem::updatezdRef(const Vec &zdParent) {

    qd >> ( zdParent(0,qSize-1) );
    ud >> ( zdParent(qSize,qSize+uSize-1) );
    xd >> ( zdParent(qSize+uSize,qSize+uSize+xSize-1) );

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updatezdRef();

    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic)
      (**ic).updatexdRef();

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF != EDI.end(); ++iF)
      (**iF).updatexdRef();
  }

  void MultiBodySystem::updateMRef(const SymMat &MParent) {

    M >> MParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updateMRef();
  }

  void MultiBodySystem::updateTRef(const Mat &TParent) {

    T >> TParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updateTRef();
  }

  void MultiBodySystem::updateLLMRef(const SymMat &LLMParent) {

    LLM >> LLMParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updateLLMRef();
  }


  void MultiBodySystem::updatehRef(const Vec &hParent) {

    h >> hParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i)
      (**i).updatehRef();
  }

  void MultiBodySystem::updaterRef(const Vec &hParent) {

    r >> rParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i)
      (**i).updaterRef();
  }

  void MultiBodySystem::updatefRef(const Vec &fParent) {

    f >> fParent;

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updatefRef();
  }


  void MultiBodySystem::updatesvRef(const Vec &svExt) {

    sv >> svExt;

    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic) 
      (**ic).updatesvRef();
  }

  void MultiBodySystem::updatejsvRef(const Vector<int> &jsvExt) {

    jsv >> jsvExt;

    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic) 
      (**ic).updatejsvRef();
  }

  void MultiBodySystem::plot(const Vec& zParent, double t, double dt) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }

    if(qd()!=zdParent()) 
      updatezdRef(zdParent);

    updateKinematics(t);
    updateLinksStage1(t);
    updateLinksStage2(t);
    // TODO nötig für ODE-Integration und hohem plotLevel
   // for(vector<Link*>::iterator iL = linkSetValued.begin(); iL != linkSetValued.end(); ++iL) 
    //  (*iL)->updateStage2(t);
    updateh(t); 
    //updateG(t); 
    //computeConstraintForces(t); 
    //updater(t); 
    //updatezd(t); 


    plot(t,dt);
  }

  void MultiBodySystem::initz(Vec& z) {
    updatezRef(z);
    for(vector<Object*>::iterator i = objects.begin(); i != objects.end(); ++i) 
      (**i).initz();
    for(vector<ExtraDynamicInterface*>::iterator iF = EDI.begin(); iF != EDI.end(); ++iF) 
      (**iF).initz();
  }

  double MultiBodySystem::computePotentialEnergy() {
    double Vpot = 0.0;

    //      cout << endl << "Objects:" << endl;
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) {
      //  	cout << (**i).getFullName() << endl;
      Vpot += (**i).computePotentialEnergy();
    }

    //      cout << endl << "Links:" << endl;
    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic) {
      // 	cout << (**ic).getFullName() << endl;
      Vpot += (**ic).computePotentialEnergy();
    }
    //      cout << " -------- " << endl;

    return Vpot;
  }

  void MultiBodySystem::getsv(const Vec& zParent, Vec& svExt, double t) {  // PASST SCHO
    if(sv()!=svExt()) {
      updatesvRef(svExt);
      //sv.init(1);
    }
    if(q()!=zParent()) {
      updatezRef(zParent);
    }

    if(qd()!=zdParent()) 
      updatezdRef(zdParent);
    updateKinematics(t);
    updateLinksStage1(t);
    updateLinksStage2(t);
    updateh(t); 
    updateG(t); 
    computeConstraintForces(t); // Berechnet die Zwangskrafte aus der Bewegungsgleichung
    updateStopVector(t);
  }

  void MultiBodySystem::setGrav(const Vec& g) {
    grav = g;
  }

  void MultiBodySystem::update(const Vec &zParent, double t) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    updateKinematics(t);
    updateLinksStage1(t);
    checkActiveConstraints();
    updateLinksStage2(t);
    updateh(t); 
    updateG(t); 

  }

  void MultiBodySystem::updaterFactors() {
    if(strategy == global) {
      double rFac;
      if(G.size() == 1) {
	rFac = 1./G(0,0);
      } else {
	Vec eta = eigvalSel(G,1,G.size());
	double etaMax = eta(G.size()-1);
	double etaMin = eta(0);
	int i=1;
	while(abs(etaMin) < 1e-8 && i<G.size()) 
	  //while(etaMin < 1e-8 && i<G.size()) 
	  etaMin = eta(i++);
	rFac = 2./(etaMax + etaMin);
      }
      rFactor.init(rFac);

    } else if(strategy == local) {

      for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
	(**i).updaterFactors();

      // } else if(strategy == optimal) {
      // //  cout << "Strategy vorübergehend deaktiviert" << endl;
      // //  throw 5;
      //   if(rFactor.size() == 1) {
      //     rFactor(0) = 1.0/G(0,0);
      //   } else {
      //   for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      //    (**i).updaterFactors();
      //   //rFactor.init(1);

      // //  if(G.size()>=3)  {
      //   MyZfkt zfkt(this);
      //   TAMOEBAOptimizer OPTI(&zfkt);   
      //   //Vec State(rFactor.size(),INIT,1);
      //   //Vec State = rFactor.copy();

      //   OPTI.setInitialState(rFactor);
      //   OPTI.optimize();

      //   rFactor = OPTI.getx();
      //   if(OPTI.getNumberOfIterations() > 450)
      //     cout << "WARNING WARNING, high iterations during optimization" << endl;
      //   if(min(rFactor) <= 0) {
      //  
      //  cout << "WARNING WARNING, using local Strategy" << endl;
      //   for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      //     (**i).updaterFactors();

      // // cout << "WARNING WARNING, using IFFCO, as" << endl;
      // //    cout << rFactor << endl;
      // //    rFactor.init(1);
      // //    IFFCOOptimizer OPTI(&zfkt);   
      // //    Vec UBounds(rFactor.size(),INIT,2);
      // //    Vec LBounds(rFactor.size(),INIT,0.000001);
      // //    OPTI.setStateBounds(UBounds,LBounds);
      // //    OPTI.setInitialState(rFactor);
      // //    OPTI.setProfitMagnitude(0);
      // //    OPTI.setMaxCuts(3);
      // //    OPTI.setMinhnMaxh(0.001,0.5);
      // //    OPTI.activateFortranOutput();
      // //    OPTI.optimize();

      // //    rFactor = OPTI.getx();
      //   }

      //   //   DiagMat E(G.size(),INIT,1.);
      //   //    DiagMat R(G.size());
      //   //     for(int i=0; i<G.size(); i++) {
      //   //	R(i) = r(i);
      //   // }
      //   //SqrMat A = SqrMat(E-R*G);
      //   //cout << trans(eigval(A)) <<endl;

      //   // if((**i).isActive()) {
      //   //(**i).updaterFactors();
      //   // Vector<int> unsure = (**i).getrFactorUnsure();
      //   // if(max(unsure)==1) {
      //   //cout << unsure << endl;
      //   //cout << G << endl;
      //   //      }
      //   //}
      //   //}
      //   }
  } else {
    cout << "Unknown strategy" << endl;
    throw 5;
  }

  }

  void MultiBodySystem::decreaserFactors() {
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (*i)->decreaserFactors();
  }

  void MultiBodySystem::updateStopVector(double t) {

    vector<Link*>::iterator ic;
    for(ic = links.begin(); ic != links.end(); ++ic)
      (*ic)->updateStopVector(t); 
  }   

  void MultiBodySystem::updateKinematics(double t) {

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (*i)->updateKinematics(t);
  }

  void MultiBodySystem::updateLinksStage1(double t) {

    if(!HSLinks.empty()) {
      linkSingleValued.erase(linkSingleValued.begin()+nHSLinksSingleValuedFixed,linkSingleValued.end());
      linkSetValued.erase(linkSetValued.begin()+nHSLinksSetValuedFixed,linkSetValued.end());
      for(vector<HitSphereLink*>::iterator iHS = HSLinks.begin(); iHS != HSLinks.end(); ++iHS)
	(*iHS)->checkActive();
    }

    for(vector<Link*>::iterator iL = linkSingleValued.begin(); iL != linkSingleValued.end(); ++iL) 
      (*iL)->updateStage1(t);
    for(vector<Link*>::iterator iL = linkSetValued.begin(); iL != linkSetValued.end(); ++iL)  
      (*iL)->updateStage1(t);
    for(vector<ExtraDynamicInterface*>::iterator iF = EDI.begin(); iF != EDI.end(); ++iF) 
      (*iF)->updateStage1(t);
  }

  void MultiBodySystem::updateLinksStage2(double t) {

    for(vector<Link*>::iterator iL = linkSingleValued.begin(); iL != linkSingleValued.end(); ++iL) 
      (*iL)->updateStage2(t);
    for(vector<Link*>::iterator iL = linkSetValuedActive.begin(); iL != linkSetValuedActive.end(); ++iL) {
      (*iL)->updateStage2(t);
    }
  }

  void MultiBodySystem::updateh(double t) {

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updateh(t);
  }

  void MultiBodySystem::updateG(double t) {
    G.init(0);
    b.init(0);
    w.init(0);

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updateG(t);

    if(checkGSize)
      Gs.resize();
    else if(Gs.cols() != G.size()) {
      static double facSizeGs = 1;
      if(G.size()>limitGSize && facSizeGs == 1) 
	facSizeGs = double(countElements(G))/double(G.size()*G.size())*1.5;
      Gs.resize(G.size(),G.size(),int(G.size()*G.size()*facSizeGs));
    }
    Gs << G;
  }

  void MultiBodySystem::updater(double t) {

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updater(t);
  }
  void MultiBodySystem::updatezd(double t) {

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updatezd(t);

    vector<Link*>::iterator il;
    for(il = links.begin(); il!= links.end(); ++il)
      (**il).updatexd(t);

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF!= EDI.end(); ++iF)
      (**iF).updatexd(t);
  }

  void MultiBodySystem::updatedu(double t, double dt) {

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updatedu(t,dt);
  }

  void MultiBodySystem::updatedx(double t, double dt) {
    // TODO Liste eingrenzen
    vector<Object*>::iterator io;
    for(io = objects.begin(); io != objects.end(); ++io) 
      (**io).updatedx(t,dt);

    // TODO Liste eingrenzen
    vector<Link*>::iterator il;
    for(il = links.begin(); il!= links.end(); ++il)
      (**il).updatedx(t,dt);

    vector<ExtraDynamicInterface*>::iterator iF;
    for(iF = EDI.begin(); iF!= EDI.end(); ++iF)
      (**iF).updatedx(t,dt);
  }

  void MultiBodySystem::updatedq(double t, double dt) {

    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) 
      (**i).updatedq(t,dt);
  }

  Vec MultiBodySystem::deltax(const Vec &zParent, double t, double dt) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    updatedx(t,dt);
    return xd;
  }

  Vec MultiBodySystem::deltaq(const Vec &zParent, double t, double dt) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    updatedq(t,dt);
    return qd;
  }

  Vec MultiBodySystem::deltau(const Vec &zParent, double t, double dt) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    // TODO updater auslagern
    updater(t); 
    updatedu(t,dt);
    // cout <<"Zeit : " << t  <<" "<< ud << endl;
    return ud;
  }

  void MultiBodySystem::initPlotFiles() {
    Object::initPlotFiles();

    // plot-Listen aufbauen
    //  vector<Object*>::iterator i;
    for(vector<Object*>::iterator i = objects.begin(); i != objects.end(); ++i) {
      if((**i).getPlotLevel()>0) objects2plot.push_back((*i));
      for(vector<Contour*>::iterator i4 = (**i).contour.begin(); i4!= (**i).contour.end(); ++i4)
	if((**i4).getPlotLevel()>0) contours2plot.push_back((*i4));
      for(vector<Port*>::iterator i5 = (**i).port.begin(); i5!= (**i).port.end(); ++i5)
	// jetzt in Port::init()      (**i5).setFullName((**i).getFullName()+ "."+ (**i5).getFullName());  // etl. problematisch bei Namens-Aufloesungen durch Vergleiche: pruefen
	if((**i5).getPlotLevel()>0) ports2plot.push_back((*i5));
    }

    for(vector<Link*>::iterator i2 = links.begin(); i2!= links.end(); ++i2)
      if((**i2).getPlotLevel()>0) links2plot.push_back((*i2));

    for(vector<Contour*>::iterator i4 = contour.begin(); i4!= contour.end(); ++i4)
      if((**i4).getPlotLevel()>0) contours2plot.push_back((*i4));
    for(vector<Port*>::iterator i5 = port.begin(); i5!= port.end(); ++i5)
      if((**i5).getPlotLevel()>0) ports2plot.push_back((*i5));

    for(vector<ExtraDynamicInterface*>::iterator i3 = EDI.begin(); i3!= EDI.end(); ++i3)
      if((**i3).getPlotLevel()>0) EDIs2plot.push_back((*i3));

    // die eigentliche Initialisierung

    for(vector<Object*>::iterator                i = objects2plot.begin();  i != objects2plot.end(); ++i)  (**i).initPlotFiles();
    for(vector<Link*>::iterator                  i = links2plot.begin();    i != links2plot.end(); ++i)    (**i).initPlotFiles();
    for(vector<Contour*>::iterator               i = contours2plot.begin(); i != contours2plot.end(); ++i) (**i).initPlotFiles();
    for(vector<Port*>::iterator                  i = ports2plot.begin();    i != ports2plot.end(); ++i)    (**i).initPlotFiles();
    for(vector<ExtraDynamicInterface*>::iterator i = EDIs2plot.begin();     i != EDIs2plot.end(); ++i)     (**i).initPlotFiles();

    // cout << "members of plot lists" << endl; 
    //   for(vector<Object*>::iterator                i = objects2plot.begin();  i != objects2plot.end(); ++i)  cout << (**i).getFullName() << endl;
    // cout << endl;
    //   for(vector<Link*>::iterator                  i = links2plot.begin();    i != links2plot.end(); ++i)    cout << (**i).getFullName() << endl;
    // cout << endl;
    //   for(vector<Contour*>::iterator               i = contours2plot.begin(); i != contours2plot.end(); ++i) cout << (**i).getFullName() << endl;
    // cout << endl;
    //   for(vector<Port*>::iterator                  i = ports2plot.begin();    i != ports2plot.end(); ++i)    cout << (**i).getFullName() << endl;
    // cout << endl;
    //   for(vector<ExtraDynamicInterface*>::iterator i = EDIs2plot.begin();     i != EDIs2plot.end(); ++i)     cout << (**i).getFullName() << endl;

    //  Object::initPlotFiles();
    //  vector<Object*>::iterator i;
    //  for(i = objects.begin(); i != objects.end(); ++i) {
    //    (**i).initPlotFiles();
    //    vector<Contour*>::iterator i4; // todo: listen contour2plot und port2plot anlegen, nur diese in plot() auswerten ...
    //    for(i4 = (**i).contour.begin(); i4!= (**i).contour.end(); ++i4)
    //      (**i4).initPlotFiles();
    //    vector<Port*>::iterator i5;
    //    for(i5 = (**i).port.begin(); i5!= (**i).port.end(); ++i5) {
    //      (**i5).setFullName((**i).getFullName()+ "."+ (**i5).getFullName());  // etl. problematisch bei Namens-Aufloesungen durch Vergleiche: pruefen
    //      (**i5).initPlotFiles();
    //    }
    //  }
    //
    //  vector<Link*>::iterator i2;
    //  for(i2 = links.begin(); i2!= links.end(); ++i2)
    //    (**i2).initPlotFiles();
    //
    //  vector<Contour*>::iterator i4; // todo: listen contour2plot und port2plot anlegen, nur diese in plot() auswerten ...
    //  for(i4 = contour.begin(); i4!= contour.end(); ++i4)
    //    (**i4).initPlotFiles();
    //  vector<Port*>::iterator i5;
    //  for(i5 = port.begin(); i5!= port.end(); ++i5)
    //    (**i5).initPlotFiles();
    // 
    //  vector<ExtraDynamicInterface*>::iterator i3;
    //  for(i3 = EDI.begin(); i3!= EDI.end(); ++i3)
    //    (**i3).initPlotFiles();


    // Energieterme
    if(plotLevel>=3) {
      plotfile <<"# " << plotNr++ << ": T" << endl;
      plotfile <<"# " << plotNr++ << ": V" << endl;
      plotfile <<"# " << plotNr++ << ": E" << endl;
    }   
  }
  void MultiBodySystem::closePlotFiles() {
    Object::closePlotFiles();
    vector<Object*>::iterator i;
    for(i = objects.begin(); i != objects.end(); ++i) {
      (**i).closePlotFiles();
      vector<Contour*>::iterator i4;
      for(i4 = (**i).contour.begin(); i4!= (**i).contour.end(); ++i4)
	(**i4).closePlotFiles();
    }
    vector<Link*>::iterator i2;
    for(i2 = links.begin(); i2!= links.end(); ++i2)
      (**i2).closePlotFiles();
    vector<ExtraDynamicInterface*>::iterator i3;
    for(i3 = EDI.begin(); i3!= EDI.end(); ++i3)
      (**i3).closePlotFiles();
  }

  void MultiBodySystem::plotParameters() {
    parafile << "MultibodySystem: \t" << fullName << endl;
    parafile << "solver: \t\t" << getSolverInfo() << endl;

    // all Objects of MultibodySystem
    if(objects.size()>0) {
      parafile << "\nObjects:" << endl;
      for(vector<Object*>::iterator i = objects2plot.begin();  i != objects2plot.end();  ++i)
	parafile << "#  " << (**i).getName() << endl;
    }
    // all Ports to environment
    if(port.size()>0) {
      parafile << "\nenvironmental ports:" << endl;
      for(vector<Port*>::iterator i = port.begin();  i != port.end();  ++i) {
	Vec WrOPtemp = (**i).getWrOP();
	parafile << "#  KrSP: (port:  name= "<<(**i).getName()<<",  ID= "<<(**i).getID()<<") = (" << WrOPtemp(0) <<","<< WrOPtemp(1) <<","<< WrOPtemp(2) << ")" << endl;
      } 
    }
    // all Contours of environment
    if(contour.size()>0) {
      parafile << "\nenvironmental contours:" << endl;
      for(vector<Contour*>::iterator i = contour.begin();  i != contour.end();  ++i)
	parafile << "#  " << (**i).getName() << endl;
    }

    for(vector<Object*>::iterator                i = objects2plot.begin();  i != objects2plot.end();  ++i) (**i).plotParameters();
    for(vector<Link*>::iterator                  i = links2plot.begin();    i != links2plot.end();    ++i) (**i).plotParameters();
    for(vector<Contour*>::iterator               i = contours2plot.begin(); i != contours2plot.end(); ++i) (**i).plotParameters();
    for(vector<Port*>::iterator                  i = ports2plot.begin();    i != ports2plot.end();    ++i) (**i).plotParameters();
  }

  void MultiBodySystem::plot(double t, double dt) {

    for(vector<Object*>::iterator                i = objects2plot.begin();  i != objects2plot.end(); ++i)  (**i).plot(t,dt);
    for(vector<Link*>::iterator                  i = links2plot.begin();    i != links2plot.end(); ++i)    (**i).plot(t,dt);
    for(vector<Contour*>::iterator               i = contours2plot.begin(); i != contours2plot.end(); ++i) (**i).plot(t,dt);
    for(vector<Port*>::iterator                  i = ports2plot.begin();    i != ports2plot.end(); ++i)    (**i).plot(t,dt);
    for(vector<ExtraDynamicInterface*>::iterator i = EDIs2plot.begin();     i != EDIs2plot.end(); ++i)     (**i).plot(t,dt);

    //    /* member-objects */
    //   Object::plot(t,dt);
    //   vector<Object*>::iterator i;
    //   for(i = objects.begin(); i != objects.end(); ++i) {
    //     (**i).plot(t,dt);
    //     /* member-contours of objects*/
    //     vector<Contour*>::iterator i4; // todo: listen contour2plot und port2plot anlegen, nur diese in plot() auswerten ...
    //     for(i4 = (**i).contour.begin(); i4!= (**i).contour.end(); ++i4)
    //       (**i4).plot(t,dt);
    //     vector<Port*>::iterator i5;
    //     for(i5 = (**i).port.begin(); i5!= (**i).port.end(); ++i5)
    //       (**i5).plot(t,dt);
    //   }
    //   /* member-links */
    //   vector<Link*>::iterator i2;
    //   for(i2 = links.begin(); i2!= links.end(); ++i2)
    //     (**i2).plot(t,dt);
    //   /* member-contours*/
    //   vector<Contour*>::iterator i4; // todo: listen contour2plot und port2plot anlegen, nur diese in plot() auswerten ...
    //   for(i4 = contour.begin(); i4!= contour.end(); ++i4)
    //     (**i4).plot(t,dt);
    //   vector<Port*>::iterator i5;
    //   for(i5 = port.begin(); i5!= port.end(); ++i5)
    //     (**i5).plot(t,dt);
    //   /* member-EDIs*/
    //   vector<ExtraDynamicInterface*>::iterator i3;
    //   for(i3 = EDI.begin(); i3!= EDI.end(); ++i3)
    //     (**i3).plot(t,dt);

    Object::plot(t,dt);

    /* Gesamtsystem Energien */
    if(plotLevel>=3) {
      double Ttemp = this->computeKineticEnergy();
      double Vtemp = this->computePotentialEnergy();
      plotfile<<" "<< Ttemp;
      plotfile<<" "<< Vtemp;
      plotfile<<" "<< Ttemp + Vtemp;
    }
  }

  void MultiBodySystem::computeConstraintForces(double t) {
    la = slvLL(G, -(w+b));
  }

  void MultiBodySystem::projectViolatedConstraints(double t) {
    if(laSize) {
    Vec nu(uSize);
    int gASize = 0;
    for(int i = 0; i<linkSetValuedActive.size(); i++) {
      gASize += linkSetValuedActive[i]->getgSize();
    }
    SymMat Gv(gASize,NONINIT);
    Mat Wv(W.rows(),gASize,NONINIT);
    Vec gv(gASize,NONINIT);
    int gAIndi = 0;
    for(int i = 0; i<linkSetValuedActive.size(); i++) {
      Index I1 = Index(linkSetValuedActive[i]->getlaInd(),linkSetValuedActive[i]->getlaInd()+linkSetValuedActive[i]->getgSize()-1);
      Index Iv = Index(gAIndi,gAIndi+linkSetValuedActive[i]->getgSize()-1);
      Wv(Index(0,Wv.rows()-1),Iv) = W(Index(0,W.rows()-1),I1);
      gv(Iv) = g(linkSetValuedActive[i]->getgIndex());

      Gv(Iv) = G(I1);
      int gAIndj = 0;
      for(int j = 0; j<i; j++) {
	Index Jv = Index(gAIndj,gAIndj+linkSetValuedActive[j]->getgSize()-1);
	Index J1 = Index(linkSetValuedActive[j]->getlaInd(),linkSetValuedActive[j]->getlaInd()+linkSetValuedActive[j]->getgSize()-1);
	Gv(Jv,Iv) = G(J1,I1);
	gAIndj+=linkSetValuedActive[j]->getgSize();
      }
      gAIndi+=linkSetValuedActive[i]->getgSize();
    }
    while(nrmInf(gv) >= 1e-8) {
      Vec mu = slvLL(Gv, -gv+trans(Wv)*nu);
      Vec dnu = slvLLFac(LLM,Wv*mu- M*nu);
      nu += dnu;
      q += T*dnu;
      updateKinematics(t);
      updateLinksStage1(t);
      int gAIndi = 0;
      for(int i = 0; i<linkSetValuedActive.size(); i++) {
	Index I1 = Index(linkSetValuedActive[i]->getlaInd(),linkSetValuedActive[i]->getlaInd()+linkSetValuedActive[i]->getgSize()-1);
	Index Iv = Index(gAIndi,gAIndi+linkSetValuedActive[i]->getgSize()-1);
	gv(Iv) = g(linkSetValuedActive[i]->getgIndex());
	gAIndi+=linkSetValuedActive[i]->getgSize();
      }
    }
    }
  }

  void MultiBodySystem::savela() {
    vector<Link*>::iterator ic;
    for(ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) 
      (**ic).savela();
  }

  void MultiBodySystem::initla() {
    vector<Link*>::iterator ic;
    for(ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) 
      (**ic).initla();
  }

  Port* MultiBodySystem::getPort(const string &name,bool check) {

    int i;
    for(i=0; i<port.size(); i++) {
      if(port[i]->getName() == name || port[i]->getFullName()== name)
	return port[i];
    }
    if(check){
      if(!(i<port.size())) cout << "Error: The MultiBodySystem " << this->name <<" comprises no port " << name << "!" << endl; 
      assert(i<port.size());
    }
    else return NULL;
  }
  Contour* MultiBodySystem::getContour(const string &name,bool check) {
    int i;
    for(i=0; i<contour.size(); i++) {
      if(contour[i]->getName() == name || contour[i]->getFullName()== name)
	return contour[i];
    }
    if(check){
      if(!(i<contour.size())) cout << "Error: The MultiBodySystem " << this->name <<" comprises no contour " << name << "!" << endl; 
      assert(i<contour.size());
    }
    else return NULL;
  }



  void MultiBodySystem::addPort(const string &name, const Vec &WrOP) {
    Port *port = new Port(name);
    addPort(port,WrOP);
  }

  void MultiBodySystem::addPort(Port* port, const Vec &WrOP) {
    Object::addPort(port);
    port->setWrOP(WrOP);
  }

  void MultiBodySystem::addContour(Contour* contour, const Vec &WrOP) {
    Object::addContour(contour);
    contour->setWrOP(WrOP);
  }

  void MultiBodySystem::addContour(Contour* contour, const Vec &WrOP, const SqrMat &AWC) {
    Object::addContour(contour);
    contour->setWrOP(WrOP);
    contour->setAWC(AWC);
  }
  void MultiBodySystem::preInteg(MultiBodySystem *parent){
    if(preIntegrator){
      setProjectDirectory(name+".preInteg");
      setGrav(parent->getGrav()); //TODO bedeutet, dass fuer Vorintegration der gravitationsvektor im MBS parent schon gesetzt sein muss.
      cout << "Initialisation of " << name << " for Preintegration..."<<endl;
      init();  
      cout << "Preintegration..."<<endl;
      preIntegrator->integrate(*this);
      closePlotFiles();
      writez();
      cout << "Finished." << endl;
    }  
  }

  void MultiBodySystem::writez(){
    for(int i=0; i<objects.size(); i++)  {
      objects[i]->writeq();
      objects[i]->writeu();
      objects[i]->writex();
    }
    for(int i=0; i<EDI.size(); i++)  {
      EDI[i]->writex();
    }
  }

  void MultiBodySystem::readz0(){
    for(int i=0; i<objects.size(); i++)  {
      objects[i]->readq0();
      objects[i]->readu0();
      objects[i]->readx0();
    }
    for(int i=0; i<EDI.size(); i++)  {
      EDI[i]->readx0();
    }
  }

  void MultiBodySystem::addMbs(MultiBodySystem* mbs) {

    for(int i=0; i<mbs->port.size(); i++) {
      Object::addPort(mbs->port[i]);
    }
    for(int i=0; i<mbs->contour.size(); i++) {
      Object::addContour(mbs->contour[i]);
    }
    for(int i=0; i<mbs->objects.size(); i++) {
      addObject(mbs->objects[i]);
    }
    for(int i=0; i<mbs->links.size(); i++) {
      addLink(mbs->links[i]);
    }
    for(int i=0; i<mbs->EDI.size(); i++) {
      addEDI(mbs->EDI[i]);
    }
    for(int i=0; i<mbs->DIBs.size(); i++) {
      addDataInterfaceBase(mbs->DIBs[i]);
    }
  }

  Object* MultiBodySystem::getObject(const string &name, bool check) {
    int i;
    for(i=0; i<objects.size(); i++) {
      //cout << objects[i]->getName() << " " << name << endl;
      if(objects[i]->getName() == name)
	return objects[i];
    }
    for(i=0; i<objects.size(); i++) {
      //cout << objects[i]->getFullName() << " " << name << endl;
      if(objects[i]->getFullName() == name)
	return objects[i];
    }
    if(check){
      if(!(i<objects.size())) cout << "Error: The MultiBodySystem " << this->name <<" comprises no object " << name << "!" << endl; 
      assert(i<objects.size());
    }
    else return NULL;
  }

  Element* MultiBodySystem::getElement(const string &name) {
    int i1;
    for(i1=0; i1<objects.size(); i1++) {
      if(objects[i1]->getName() == name)
	return (Element*)objects[i1];
    }
    for(i1=0; i1<objects.size(); i1++) {
      if(objects[i1]->getFullName() == name)
	return (Element*)objects[i1];
    }
    int i2;
    for(i2=0; i2<links.size(); i2++) {
      if(links[i2]->getName() == name)
	return (Element*)links[i2];
    }
    for(i2=0; i2<links.size(); i2++) {
      if(links[i2]->getFullName() == name)
	return (Element*)links[i2];
    }
    int i3;
    for(i3=0; i3<EDI.size(); i3++) {
      if(EDI[i3]->getName() == name)
	return (Element*)EDI[i3];
    }
    for(i3=0; i3<EDI.size(); i3++) {
      if(EDI[i3]->getFullName() == name)
	return (Element*)EDI[i3];
    }
    if(!(i1<objects.size())||!(i2<links.size())||!(i3<EDI.size())) cout << "Error: The MultiBodySystem " << this->name <<" comprises no element " << name << "!" << endl; 
    assert(i1<objects.size()||i2<links.size()||!(i3<EDI.size()));
  }

  ExtraDynamicInterface* MultiBodySystem::getEDI(const string &name) {
    int i;
    for(i=0; i<EDI.size(); i++) {
      if(EDI[i]->getName() == name)
	return EDI[i];
    }
    for(i=0; i<EDI.size(); i++) {
      if(EDI[i]->getFullName() == name)
	return EDI[i];
    }
    if(!(i<EDI.size())) cout << "Error: The MultiBodySystem " << this->name <<" comprises no EDI " << name << "!" << endl; 
    assert(i<EDI.size());
  }    

  void MultiBodySystem::addObject(Object *object) {
    objects.push_back(object);
    object->setMbs(this);
    object->setFullName(getFullName()+"."+object->getFullName());

  }

  DataInterfaceBase* MultiBodySystem::getDataInterfaceBase(const string &name_) {
    int i;
    for(i=0; i<DIBs.size(); i++) {
      if(DIBs[i]->getName() == name_ || DIBs[i]->getName()== fullName+"."+name_)
	return DIBs[i];
    }
    if(!(i<DIBs.size())) cout << "Error: The MultiBodySystem " << name <<" comprises no DIB " << name_ << "!" << endl; 
    assert(i<DIBs.size());
  }

  void MultiBodySystem::addDataInterfaceBase(DataInterfaceBase* dib_){
    DIBs.push_back(dib_);
    dib_->setName(getFullName()+"."+dib_->getName());
  }


  HitSphereLink* MultiBodySystem::getHitSphereLink(Object* obj0, Object* obj1) {
    // test for existing HitSphereLinks
    for(vector<HitSphereLink*>::iterator hsl = HSLinks.begin();hsl < HSLinks.end();hsl++)
      if((*hsl)->getObject(0) == obj0 && (*hsl)->getObject(1) == obj1 || (*hsl)->getObject(0) == obj1 && (*hsl)->getObject(1) == obj0)
	return  (*hsl);

    //     cout << "Creating new HitSphereLink for " << obj0->getName() << "<->" << obj1->getName() << endl;

    // create new if none is found
    HitSphereLink *HSLink = new HitSphereLink();
    HSLinks.push_back(HSLink);
    return HSLink;
  }

  void MultiBodySystem::addLink(Link *link) {
    links.push_back(link);
    link->setMbs(this);
    link->setFullName(getFullName()+"."+link->getFullName());

  }
  void MultiBodySystem::addEDI(ExtraDynamicInterface *edi_) {
    EDI.push_back(edi_);
    edi_->setMbs(this);
    edi_->setFullName(getFullName()+"."+edi_->getFullName());
  }
  void MultiBodySystem::addElement(Element *element_) {
    MultiBodySystem* mbs_=dynamic_cast<MultiBodySystem*>(element_);
    if(mbs_) addMbs(mbs_);
    else {
      Object* object_=dynamic_cast<Object*>(element_);
      Link* link_=dynamic_cast<Link*>(element_);
      ExtraDynamicInterface* edi_=dynamic_cast<ExtraDynamicInterface*>(element_);
      if(object_) addObject(object_);
      else if(link_) addLink(link_);
      else if(edi_) addEDI(edi_);
      else{ cout << "Error: MultiBodySystem: addElement(): No such type of Element to add!"<<endl; throw 50;}
    }
  }

  void MultiBodySystem::setlaTol(double tol) {
    vector<Link*>::iterator i;
    for(i = links.begin(); i!= links.end(); ++i)
      (**i).setlaTol(tol);
  }

  void MultiBodySystem::setgdTol(double tol) {
    vector<Link*>::iterator i;
    for(i = links.begin(); i!= links.end(); ++i)
      (**i).setgdTol(tol);
  }

  void MultiBodySystem::setScaleTolQ(double scaleTolQ) {
    vector<Link*>::iterator i;
    for(i = links.begin(); i!= links.end(); ++i)
      (**i).setScaleTolQ(scaleTolQ);
  }

  void MultiBodySystem::setScaleTolp(double scaleTolp) {
    vector<Link*>::iterator i;
    for(i = links.begin(); i!= links.end(); ++i)
      (**i).setScaleTolp(scaleTolp);
  }

  void MultiBodySystem::setrMax(double rMax) {
    vector<Link*>::iterator i;
    for(i = links.begin(); i!= links.end(); ++i)
      (**i).setrMax(rMax);
  }

  int MultiBodySystem::solveLinearEquations(double dt) {

    la = slvLL(G, -(getgd() + getb()*dt));
    return 1;
  }

  int MultiBodySystem::solveGaussSeidel(double dt) {

    s = getgd() + getb()*dt ;

    checkForTermination(dt);
    if(term)
      return 0 ;

    int iter;
    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {

      for(vector<Link*>::iterator ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) {
	(**ic).solveGS(dt);
      }
      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
	checkTermLevel++;
	checkForTermination(dt);
	if(term)
	  break;
      }
    }

    return iter;
  }

  int MultiBodySystem::solveFixpointTotal(double dt) {

    updaterFactors();

    Vec s0 = getgd() + getb()*dt ;
    s = s0;

    checkForTermination(dt);
    if(term)
      return 0 ;

    int iter, level = 0;
    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {
      double *a = getGs()();
      int *ia = getGs().Ip();
      int *ja = getGs().Jp();
      for(int i=0; i < G.size(); i++) {
	for(int j=ia[i]; j<ia[1+i]; j++)
	  s(i) += a[j]*la(ja[j]);
      }

      if(level < decreaseLevels.size() && iter > decreaseLevels(level)) {
	level++;
	decreaserFactors();
	if(warnLevel>=2) 
	  cout <<endl<< "Warning: decreasing r-factors at iter = " << iter<<endl;
      }

      for(vector<Link*>::iterator ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) 
	(*ic)->projectJ(dt);

      s = s0;
      checkForTermination(dt);
      if(term)
	break;
    }

    return iter;
  }

  int MultiBodySystem::solveFixpointSingle(double dt) {

    updaterFactors();

    s = getgd() + getb()*dt ;

    checkForTermination(dt);
    if(term)
      return 0 ;

    int iter, level = 0;
    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {

      if(level < decreaseLevels.size() && iter > decreaseLevels(level)) {
	level++;
	decreaserFactors();
	cout <<endl<< "Warning: decreasing r-factors at iter = " << iter<<endl;
	if(warnLevel>=2)
	  cout <<endl<< "Warning: decreasing r-factors at iter = " << iter<<endl;
      }
      for(vector<Link*>::iterator ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) 
      {
	(*ic)->projectGS(dt);
      }

      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
	checkTermLevel++;
	checkForTermination(dt);
	if(term)
	  break;
      }
    }

    return iter;
  }


  void MultiBodySystem::residualProj(double dt) {

    for(vector<Link*>::iterator ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) {
      (**ic).residualProj(dt);
    }
  }

  void MultiBodySystem::checkForTermination(double dt) {

    term = true;
    for(vector<Link*>::iterator ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) {
      (**ic).checkForTermination(dt);
      if(term == false)
	return;
    }
  }

  void MultiBodySystem::residualProjJac(double dt) {

    for(vector<Link*>::iterator ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) {
      (**ic).residualProjJac(dt);
    }
  }

  int MultiBodySystem::solve(double dt) {

    if(la.size()==0)
      return 0;

    if(useOldla)
      initla();
    else
      la.init(0);

    int iter;
    Vec laOld;
    laOld = la;
    iter = (this->*solve_)(dt);
    if(iter >= maxIter) {
      cout << endl;
      cout << "Iterations: " << iter << endl;
      cout << "\nError: no convergence."<<endl;
      if(stopIfNoConvergence) {
	if(dropContactInfo) dropContactMatrices();
	assert(iter < maxIter);
      }
      cout << "Anyway, continuing integration..."<<endl;
    }

    if(warnLevel>=1 && iter>highIter)
      cerr <<endl<< "Warning: high number of iterations: " << iter<<endl;

    if(useOldla)
      savela();

    return iter;
  }

  int MultiBodySystem::solveRootFinding(double dt) {

    updaterFactors();

    s = getgd() + getb()*dt;
    int iter;
    int prim = 0;
    int checkTermLevel = 0;

    residualProj(dt); 
    double nrmf0 = nrm2(res);
    Vec res0 = res.copy();

    checkForTermination(dt);
    if(term)
      return 0 ;

    DiagMat I(la.size(),INIT,1);
    for(iter=1; iter<maxIter; iter++) {

      if(Jprox.size() != la.size())
	Jprox.resize(la.size(),NONINIT);

      if(numJac) {
	double dx, xj;

	for(int j=0; j<la.size(); j++) {
	  xj = la(j);

	  dx = (epsroot() * 0.5);
	  do {                   
	    dx += dx;
	  } while (xj + dx == la(j));

	  la(j)+=dx;
	  residualProj(dt);
	  la(j)=xj;
	  Jprox.col(j) = (res-res0)/dx;
	}
      } else {
	residualProjJac(dt);
      }
      Vec dx;
      if(linAlg == LUDecomposition)
	dx >> slvLU(Jprox,res0);
      else if(linAlg == LevenbergMarquardt) {
	SymMat J = SymMat(JTJ(Jprox) + lmParm*I);
	dx >> slvLL(J,trans(Jprox)*res0);
      } else if(linAlg == PseudoInverse) {
	dx >> slvLS(Jprox,res0);
      } else
	throw 5;

      double alpha = 1;       

      Vec La_old = la.copy();

      double nrmf;
      for (int k=0; k<maxDampingSteps; k++) {
	la = La_old - alpha*dx;
	residualProj(dt);
	nrmf = nrm2(res);
	if(nrmf < nrmf0)
	  break;

	alpha = 0.5*alpha;  
      }
      nrmf0 = nrmf;
      res0 = res;

      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
	checkTermLevel++;
	checkForTermination(dt);
	if(term)
	  break;
      }
    }
    return iter;
  }

  void MultiBodySystem::dropContactMatrices() {
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

  string MultiBodySystem::getSolverInfo() {
    stringstream info;

    // Solver-Name
    if(solver == GaussSeidel)
      info << "GaussSeidel";
    else if(solver == LinearEquations)
      info << "LinearEquations";
    else if(solver == FixedPointSingle)
      info << "FixedPointSingle";
    else if(solver == FixedPointTotal)
      info << "FixedPointTotal";
    else if(solver == RootFinding)
      info << "RootFinding";

    // Gauss-Seidel & solveLL do not depend on the following ...
    if(solver!=GaussSeidel && solver!=LinearEquations) {
      info << "(";

      // r-Factor strategy
      if(strategy==global)
	info << "global";
      else if(strategy==local)
	info << "local";

      // linear algebra for solveN only
      if(solver == RootFinding) {
	info << ",";
	if(linAlg==LUDecomposition)
	  info << "LU";
	else if(linAlg==LevenbergMarquardt)
	  info << "LM";
	else if(linAlg==PseudoInverse)
	  info << "PI";
      }

      info << ")";
    }

    return info.str();
  }

  void MultiBodySystem::initDataInterfaceBase() {
    vector<Link*>::iterator il1;
    for(il1 = links.begin(); il1 != links.end(); ++il1) (*il1)->initDataInterfaceBase(this);
    vector<Object*>::iterator io1;
    for(io1 = objects.begin(); io1 != objects.end(); ++io1) (*io1)->initDataInterfaceBase(this);
    vector<ExtraDynamicInterface*>::iterator ie1;
    for(ie1 = EDI.begin(); ie1 != EDI.end(); ++ie1) (*ie1)->initDataInterfaceBase(this); 
  }

}
