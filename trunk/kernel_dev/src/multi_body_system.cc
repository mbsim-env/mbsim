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
#include<config.h>
#include "multi_body_system.h"
#include "coordinate_system.h"
#include "contour.h"
#include "link.h"
#include "extra_dynamic_interface.h"
#include "integrator.h"
#include "body_flexible.h"
#include "eps.h"
#include "dirent.h"
#ifndef MINGW
#  include<sys/stat.h>
#else
#  include<io.h>
#  define mkdir(a,b) mkdir(a)
#endif

namespace MBSim {

  MultiBodySystem::MultiBodySystem() :                          Group("Default"),    grav(3), activeConstraintsChanged(true), maxIter(10000), highIter(1000), maxDampingSteps(3), lmParm(0.001), warnLevel(0), solver(FixedPointSingle), strategy(local), linAlg(LUDecomposition), stopIfNoConvergence(false), dropContactInfo(false), useOldla(true), numJac(false), checkGSize(true), limitGSize(500), directoryName("Default"), preIntegrator(NULL)  {

  } 

  MultiBodySystem::MultiBodySystem(const string &projectName) : Group(projectName),  grav(3), activeConstraintsChanged(true), maxIter(10000), highIter(1000), maxDampingSteps(3), lmParm(0.001), warnLevel(0), solver(FixedPointSingle), strategy(local), linAlg(LUDecomposition), stopIfNoConvergence(false), dropContactInfo(false), useOldla(true), numJac(false), checkGSize(true), limitGSize(500), directoryName("Default") , preIntegrator(NULL)  {

  }

  MultiBodySystem::~MultiBodySystem() {
    if (preIntegrator) delete preIntegrator;
  } 

  void MultiBodySystem::init() 
  {
    cout << endl << "Initialising MultiBodySystem " << name << " ......" << endl;
    setDirectory(); // output directory

    // Vektor-Dimensionierung
    cout << "  setting dimensions of objects, links and EDI..." << endl;

    Subsystem::calcSize();
    Subsystem::calchSize();
    Subsystem::calclaSize();

    // TODO Speicherproblem bei vielen moeglichen Kontakten
    if(laSize>8000)
      laSize=8000;
    MParent.resize(getuSize());
    TParent.resize(getqSize(),getuSize());
    LLMParent.resize(getuSize());
    WParent.resize(getuSize(),getlaSize());
    GParent.resize(getlaSize());
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
    updateMRef(MParent);
    updateTRef(TParent);
    updateLLMRef(LLMParent);
    G.resize() >> GParent;
    W.resize() >> WParent;
    b.resize() >> bParent;
    g.resize() >> gParent;

    updatezdRef(zdParent);

    Jh.resize(getuSize(),getzSize());

    // Init der einzelenen Komponenten
    cout << "  initialising ..." << endl;

    Subsystem::init();
 //   if(object.size()>0)  cout << "      " << object.size() << " Objects" << endl;
 //   Object::init();
 //   for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i)
 //     (**i).init();

//    if(link.size()>0)    cout << "      " << link.size()   << " Links" << endl;
//    for (vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) {
//      (**i).init();
//      if(!(*i)->getHitSphereCheck()) {
//	if((*i)->isSetValued()) {
//	  nHSLinkSetValuedFixed++;
//	  linkSetValued.push_back(*i);
//	} else {
//	  nHSLinkSingleValuedFixed++;
//	  linkSingleValued.push_back(*i);
//	}
//      }
//    }

 //   if(EDI.size()>0)    cout << "      " << EDI.size()   << " EDIs" << endl;
 //   for (vector<ExtraDynamicInterface*>::iterator i=EDI.begin(); i !=EDI.end(); ++i)
 //     (**i).init();

 //   // HitSphereLink
 //   if(HSLink.size()>0) cout << "  building " << HSLink.size() << " HitSphereLinks between Objects" << endl;
 //   for (vector<HitSphereLink*>::iterator i = HSLink.begin(); i != HSLink.end(); ++i)
 //     (**i).init();

    for(vector<Link*>::iterator ic = link.begin(); ic != link.end(); ++ic) {
      if((*ic)->isSetValued()) 
	(**ic).updategRef();
    }
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updategRef();

    checkActiveConstraints();

    updatehRef(hParent);
    updaterRef(rParent);

    // solver specific settings
    cout << "  use solver \'" << getSolverInfo() << "\' for contact situations" << endl;
    if(solver == GaussSeidel) solve_ = &MultiBodySystem::solveGaussSeidel;
    else if(solver == LinearEquations) {
      solve_ = &MultiBodySystem::solveLinearEquations;
      cout << "WARNING: solveLL is only valid for bilateral constrained systems!" << endl;
    }
    else if(solver == FixedPointSingle) solve_ = &MultiBodySystem::solveFixpointSingle;
    else if(solver == FixedPointTotal) solve_ = &MultiBodySystem::solveFixpointTotal;
    else if(solver == RootFinding)solve_ = &MultiBodySystem::solveRootFinding;
    else {
      cout << "Error: unknown solver" << endl;
      throw 5;
    }

    // if(plotting) {
    cout << "  initialising plot-files ..." << endl;
    initPlotFiles();
    // }

    cout << "...... done initialising." << endl << endl;
  }

  void MultiBodySystem::setDirectory() 
  {
    // SETDIRECTORY creates directories for outputs

    int i;
    string projectDirectory;

    if(false) { // TODO: introduce flag "overwriteDirectory"
      for(i=0; i<=99; i++) {
	stringstream number;
	number << "." << setw(2) << setfill('0') << i;
	projectDirectory = directoryName + number.str();
	int ret = mkdir(projectDirectory.c_str(),0777);
	if(ret == 0) break;
      }
      cout << "  make directory \'" << projectDirectory << "\' for output processing" << endl;
    }
    else { // always the same directory
      projectDirectory = string(directoryName);

      int ret = mkdir(projectDirectory.c_str(),0777);
      if(ret == 0) {
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

      Subsystem::checkActiveConstraints();

      W.resize() >> WParent(Index(0,getuSize()-1),Index(0,getlaSize()-1));
      G.resize() >> GParent(Index(0,getlaSize()-1));
      b.resize() >> bParent(Index(0,getlaSize()-1));
      r.resize() >> rParent(Index(0,getuSize()-1));
      la.resize() >> laParent(Index(0,getlaSize()-1));
      gd.resize() >> gdParent(Index(0,getlaSize()-1));
      s.resize() >> sParent(Index(0,getlaSize()-1));
      // TODO Nur bei Newton
      res.resize() >> resParent(Index(0,getlaSize()-1));
      rFactor.resize() >> rFactorParent(Index(0,getrFactorSize()-1));

      for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
	(**i).updateRef();
      for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
	(*i)->updateRef();

      activeConstraintsChanged = false;
    }
  }

  //Vec MultiBodySystem::zdot_alt(const Vec &zParent, double t) {
  //  if(q()!=zParent()) {
  //    updatezRef(zParent);
  //  }
  //  updateKinematics(t);
  //  updateLinksStage1(t);
  //  updateLinksStage2(t);

  //  updateT(t); 
  //  updateh(t); 
  //  updateM(t); 
  //  facLLM(); 
  //  if(laSize) {
  //    updateW(t); 
  //    updateG(t); 
  //    updateb(t); 
  //    computeConstraintForces(t); 
  //    updater(t); 
  //  }
  //  updatezd(t);
  //  return zdParent;
  //}

  Vec MultiBodySystem::zdot(const Vec &zParent, double t) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    updateKinematics(t);
    updateLinksStage1(t);
    updateLinksStage2(t);

    updateT(t); 
    updateh(t); 
    updateM(t); 
    facLLM(); 
    if(laSize) {
      updateW(t); 
      updateG(t); 
      updateb(t); 
      computeConstraintForces(t); 
      updater(t); 
    }
    updatezd(t);
    return zdParent;
  }

  void MultiBodySystem::zdot(const Vec &zParent, Vec &zdParent, double t) {
    if(qd()!=zdParent()) {
      updatezdRef(zdParent);
    }
    zdot(zParent,t);
  }

  void MultiBodySystem::updatezRef(const Vec &zParent) {

    q >> ( zParent(0,qSize-1) );
    u >> ( zParent(qSize,qSize+uSize-1) );
    x >> ( zParent(qSize+uSize,qSize+uSize+xSize-1) );

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatezRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexRef();

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexRef();
  }

  void MultiBodySystem::updateqRef(const Vec &qParent) {

    q >> qParent;

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateqRef();
  }

  void MultiBodySystem::updateuRef(const Vec &uParent) {

    u >> uParent;

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateuRef();
  }

  void MultiBodySystem::updatexRef(const Vec &xParent) {

    x >> xParent;

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatexRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexRef();

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexRef();
  }

  void MultiBodySystem::updatezdRef(const Vec &zdParent) {

    qd >> ( zdParent(0,qSize-1) );
    ud >> ( zdParent(qSize,qSize+uSize-1) );
    xd >> ( zdParent(qSize+uSize,qSize+uSize+xSize-1) );

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatezdRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i)
      (**i).updatexdRef();

    for(vector<ExtraDynamicInterface*>::iterator i = EDI.begin(); i!= EDI.end(); ++i) 
      (**i).updatexdRef();
  }

  void MultiBodySystem::updateqdRef(const Vec &qdExt) {

    qd >> qdExt;

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateqdRef();
  }

  void MultiBodySystem::updateMRef(const SymMat &MParent) {

    M >> MParent;

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateMRef();
  }

  void MultiBodySystem::updateTRef(const Mat &TParent) {

    T >> TParent;

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (*i)->updateTRef();
  }

  void MultiBodySystem::updateLLMRef(const SymMat &LLMParent) {

    LLM >> LLMParent;

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updateLLMRef();
  }

  void MultiBodySystem::updatehRef(const Vec &hParent) {

    h >> hParent;

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatehRef();

    for(vector<Link*>::iterator i = linkSingleValued.begin(); i != linkSingleValued.end(); ++i)
      (**i).updatehRef();
  }

  void MultiBodySystem::updaterRef(const Vec &rParent) {

    r >> rParent;

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updaterRef();

    for(vector<Link*>::iterator i = linkSetValued.begin(); i != linkSetValued.end(); ++i)
      (**i).updaterRef();
  }

  void MultiBodySystem::updatefRef(const Vec &fParent) {

    f >> fParent;

    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) 
      (**i).updatefRef();
  }

  void MultiBodySystem::updatesvRef(const Vec &svExt) {

    sv >> svExt;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatesvRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatesvRef();
  }

  void MultiBodySystem::updatejsvRef(const Vector<int> &jsvExt) {

    jsv >> jsvExt;

    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->updatejsvRef();

    for(vector<Link*>::iterator i = link.begin(); i != link.end(); ++i) 
      (**i).updatejsvRef();
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
    updateM(t); 
    //updateG(t); 
    //computeConstraintForces(t); 
    //updater(t); 
    //updatezd(t); 


    plot(t,dt);
  }

  void MultiBodySystem::initz(Vec& z) 
  {
    // INITZ initialises the state of object and EDIs for whole multibody system
    updatezRef(z);
    for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) (**i).initz();
    for(vector<ExtraDynamicInterface*>::iterator iF = EDI.begin(); iF != EDI.end(); ++iF) (**iF).initz();
  }

  double MultiBodySystem::computePotentialEnergy()
  {
    // COMPUTEPOTENTIALENERGY computes potential energy
    double Vpot = 0.0;

    vector<Object*>::iterator i;
    for(i = object.begin(); i != object.end(); ++i) Vpot += (**i).computePotentialEnergy();

    vector<Link*>::iterator ic;
    for(ic = link.begin(); ic != link.end(); ++ic) Vpot += (**ic).computePotentialEnergy();
    return Vpot;
  }

  void MultiBodySystem::getsv(const Vec& zParent, Vec& svExt, double t) 
  {  // PASST SCHO
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
    if(linkSetValued.size()) {
      updateW(t); 
      updateG(t); 
      updateb(t); 
      computeConstraintForces(t); // Berechnet die Zwangskrafte aus der Bewegungsgleichung
    }
    updateStopVector(t);
  }

  void MultiBodySystem::setAccelerationOfGravity(const Vec& g) {
    // SETGRAV sets gravitation
    grav = g;
  }

  void MultiBodySystem::update(const Vec &zParent, double t) {
    // UPDATE updates the position depending structures for multibody system

    if(q()!=zParent()) updatezRef(zParent);

    updateKinematics(t);
    updateLinksStage1(t);
    checkActiveConstraints();
    updateLinksStage2(t);
    updateT(t); 
    updateh(t); 
    updateM(t); 
    facLLM(); 
    updateW(t); 
    updateG(t); 
    b = trans(W)*slvLLFac(LLM,h); 
    //updateb(t);
  }

  void MultiBodySystem::updaterFactors() {
    // UPDATERFACTORS updates r-factors for children
    if(strategy == global) {
      double rFac;
      if(G.size() == 1) rFac = 1./G(0,0);
      else {
	Vec eta = eigvalSel(G,1,G.size());
	double etaMax = eta(G.size()-1);
	double etaMin = eta(0);
	int i=1;
	while(abs(etaMin) < 1e-8 && i<G.size()) etaMin = eta(i++);
	rFac = 2./(etaMax + etaMin);
      }
      rFactor.init(rFac);

    }
    else if(strategy == local) {
      Subsystem::updaterFactors();
    } 
    else {
      cout << "Unknown strategy" << endl;
      throw 5;
    }
  }

  void MultiBodySystem::decreaserFactors() 
  {
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) (*i)->decreaserFactors();
  }


  void MultiBodySystem::updateM(double t) {
    M.init(0);
    Group::updateM(t);
  }

  void MultiBodySystem::updateh(double t) {
    h.init(0);
    Group::updateh(t);
  }

  void MultiBodySystem::updateW(double t) {
    W.init(0);
    Group::updateW(t);
  }

  void MultiBodySystem::updateG(double t) 
  {

    G = SymMat(trans(W)*slvLLFac(LLM,W)); 

    if(checkGSize) Gs.resize();
    else if(Gs.cols() != G.size()) {
      static double facSizeGs = 1;
      if(G.size()>limitGSize && facSizeGs == 1) facSizeGs = double(countElements(G))/double(G.size()*G.size())*1.5;
      Gs.resize(G.size(),G.size(),int(G.size()*G.size()*facSizeGs));
    }
    Gs << G;

    //G.init(0);
    //for(vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) (**i).updateG(t);

    //if(checkGSize) Gs.resize();
    //else if(Gs.cols() != G.size()) {
    //  static double facSizeGs = 1;
    //  if(G.size()>limitGSize && facSizeGs == 1) facSizeGs = double(countElements(G))/double(G.size()*G.size())*1.5;
    //  Gs.resize(G.size(),G.size(),int(G.size()*G.size()*facSizeGs));
    //}
    //Gs << G;
  }

  void MultiBodySystem::updateb(double t) {
    b = trans(W)*slvLLFac(LLM,h); 
    Group::updateb(t);
  }

  void MultiBodySystem::updater(double t) {
    r.init(0);
    Group::updater(t);
  }

  Vec MultiBodySystem::deltax(const Vec &zParent, double t, double dt) {
    if(q()!=zParent()) {
      updatezRef(zParent);
    }
    updatedx(t,dt);
    return xd;
  }

  Vec MultiBodySystem::deltaq(const Vec &zParent, double t, double dt) {
    // DELTAQ updates the position gap for the multibody system
    // INPUT 	zParent current state
    //			t		current time
    //			dt		time step

    if(q()!=zParent()) updatezRef(zParent);
    updatedq(t,dt);

    return qd;
  }

  Vec MultiBodySystem::deltau(const Vec &zParent, double t, double dt) {
    // DELTAU updates the velocity gap for the multibody system
    // INPUT 	zParent current state
    //			t		current time
    //			dt		time step

    if(q()!=zParent()) updatezRef(zParent);

    // TODO update auslagern
    updater(t); 
    updatedu(t,dt);
    // cout <<"Zeit : " << t  <<" "<< ud << endl;
    return ud;
  }

  void MultiBodySystem::save(const string &path, ofstream& outputfile) {
    Group::save(path,outputfile);
    outputfile << "# Acceleration of gravity:" << endl;
    outputfile << grav << endl << endl;;
  }

  void MultiBodySystem::load(const string &path, ifstream& inputfile) {

    Group::load(path, inputfile);

    string dummy;

    getline(inputfile,dummy); // #  Acceleration of gravity:
    inputfile >> grav;
    getline(inputfile,dummy); // # Rest of line
    getline(inputfile,dummy); // # Newline
  }

  void MultiBodySystem::save(const string &path, MultiBodySystem* mbs) {

    string model = path + "/" + mbs->getName() + ".mdl";

    ofstream outputfile(model.c_str(), ios::binary);

    mbs->save(path, outputfile);

    outputfile.close();
  }

  MultiBodySystem* MultiBodySystem::load(const string &path) {
    DIR* dir = opendir(path.c_str());
    dirent *first;
    first = readdir(dir); // .
    first = readdir(dir); // ..
    string name;
    while(first){
      first = readdir(dir);
      if(first) {
	name= first->d_name;
	unsigned int s = name.rfind(".mdl");
	if(s<name.size()) {
	  string buf = name.substr(0,s);
	  if(buf.find(".")>buf.size())
	    break;
	}
      }
    }
    closedir(dir);

    string model = path + "/" + name;

    ifstream inputfile(model.c_str(), ios::binary);

    MultiBodySystem* mbs = new MultiBodySystem("NoName");

    mbs->load(path, inputfile);

    inputfile.close();

    return mbs;
  }

  void MultiBodySystem::computeConstraintForces(double t) {
    la = slvLL(G, -b);
  }

  void MultiBodySystem::projectViolatedConstraints(double t) 
  {
    // PROJECTVIOLATEDCONSTRAINTS projects state, such that constraints are not violated

    if(laSize) {
      Vec nu(uSize);
      int gASize = 0;
      for(unsigned int i = 0; i<linkSetValuedActive.size(); i++) gASize += linkSetValuedActive[i]->getgSize();
      SymMat Gv(gASize,NONINIT);
      Mat Wv(W.rows(),gASize,NONINIT);
      Vec gv(gASize,NONINIT);
      int gAIndi = 0;
      for(unsigned int i = 0; i<linkSetValuedActive.size(); i++) {
	Index I1 = Index(linkSetValuedActive[i]->getlaInd(),linkSetValuedActive[i]->getlaInd()+linkSetValuedActive[i]->getgSize()-1);
	Index Iv = Index(gAIndi,gAIndi+linkSetValuedActive[i]->getgSize()-1);
	Wv(Index(0,Wv.rows()-1),Iv) = W(Index(0,W.rows()-1),I1);
	gv(Iv) = g(linkSetValuedActive[i]->getgIndex());

	Gv(Iv) = G(I1);
	int gAIndj = 0;
	for(unsigned int j = 0; j<i; j++) {
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
	for(unsigned int i = 0; i<linkSetValuedActive.size(); i++) {
	  Index I1 = Index(linkSetValuedActive[i]->getlaInd(),linkSetValuedActive[i]->getlaInd()+linkSetValuedActive[i]->getgSize()-1);
	  Index Iv = Index(gAIndi,gAIndi+linkSetValuedActive[i]->getgSize()-1);
	  gv(Iv) = g(linkSetValuedActive[i]->getgIndex());
	  gAIndi+=linkSetValuedActive[i]->getgSize();
	}
      }
    }
  }

  void MultiBodySystem::savela() {
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).savela();
  }

  void MultiBodySystem::initla() {
    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) 
      (**i).initla();
  }

  void MultiBodySystem::preInteg(MultiBodySystem *parent){
    if(preIntegrator){
      setProjectDirectory(name+".preInteg");
      setAccelerationOfGravity(parent->getGrav()); //TODO bedeutet, dass fuer Vorintegration der gravitationsvektor im MBS parent schon gesetzt sein muss.
      cout << "Initialisation of " << name << " for Preintegration..."<<endl;
      init();  
      cout << "Preintegration..."<<endl;
      preIntegrator->integrate(*this);
      closePlotFiles();
      writez();
      delete preIntegrator;
      preIntegrator=NULL; 
      cout << "Finished." << endl;
    }  
  }

  void MultiBodySystem::writez(){
    for(unsigned int i=0; i<object.size(); i++)  {
      object[i]->writeq();
      object[i]->writeu();
      object[i]->writex();
    }
    for(unsigned int i=0; i<EDI.size(); i++)  {
      EDI[i]->writex();
    }
  }

  void MultiBodySystem::readz0(){
    for(unsigned int i=0; i<object.size(); i++)  {
      object[i]->readq0();
      object[i]->readu0();
      object[i]->readx0();
    }
    for(unsigned int i=0; i<EDI.size(); i++)  {
      EDI[i]->readx0();
    }
  }

  void MultiBodySystem::addElement(Element *element_) {
    Object* object_=dynamic_cast<Object*>(element_);
    Link* link_=dynamic_cast<Link*>(element_);
    ExtraDynamicInterface* edi_=dynamic_cast<ExtraDynamicInterface*>(element_);
    if(object_) addObject(object_);
    else if(link_) addLink(link_);
    else if(edi_) addEDI(edi_);
    else{ cout << "Error: MultiBodySystem: addElement(): No such type of Element to add!"<<endl; throw 50;}
  }

  Element* MultiBodySystem::getElement(const string &name) {
 //   // GETELEMENT returns element
 //   unsigned int i1;
 //   for(i1=0; i1<object.size(); i1++) {
 //     if(object[i1]->getName() == name) return (Element*)object[i1];
 //   }
 //   for(i1=0; i1<object.size(); i1++) {
 //     if(object[i1]->getFullName() == name) return (Element*)object[i1];
 //   }
 //   unsigned int i2;
 //   for(i2=0; i2<link.size(); i2++) {
 //     if(link[i2]->getName() == name) return (Element*)link[i2];
 //   }
 //   for(i2=0; i2<link.size(); i2++) {
 //     if(link[i2]->getFullName() == name) return (Element*)link[i2];
 //   }
 //   unsigned int i3;
 //   for(i3=0; i3<EDI.size(); i3++) {
 //     if(EDI[i3]->getName() == name) return (Element*)EDI[i3];
 //   }
 //   for(i3=0; i3<EDI.size(); i3++) {
 //     if(EDI[i3]->getFullName() == name) return (Element*)EDI[i3];
 //   }
 //   if(!(i1<object.size())||!(i2<link.size())||!(i3<EDI.size())) cout << "Error: The MultiBodySystem " << this->name <<" comprises no element " << name << "!" << endl; 
 //   assert(i1<object.size()||i2<link.size()||!(i3<EDI.size()));
    return NULL;
  }


  void MultiBodySystem::setlaTol(double tol) {
    vector<Link*>::iterator i;
    for(i = link.begin(); i!= link.end(); ++i)
      (**i).setlaTol(tol);
  }

  void MultiBodySystem::setgdTol(double tol) {
    vector<Link*>::iterator i;
    for(i = link.begin(); i!= link.end(); ++i)
      (**i).setgdTol(tol);
  }

  void MultiBodySystem::setScaleTolQ(double scaleTolQ) {
    vector<Link*>::iterator i;
    for(i = link.begin(); i!= link.end(); ++i)
      (**i).setScaleTolQ(scaleTolQ);
  }

  void MultiBodySystem::setScaleTolp(double scaleTolp) {
    vector<Link*>::iterator i;
    for(i = link.begin(); i!= link.end(); ++i)
      (**i).setScaleTolp(scaleTolp);
  }

  void MultiBodySystem::setrMax(double rMax) {
    vector<Link*>::iterator i;
    for(i = link.begin(); i!= link.end(); ++i)
      (**i).setrMax(rMax);
  }

  int MultiBodySystem::solveLinearEquations(double dt)
  {
    // SOLVELINEAREQUATIONS solves constraint equations with Cholesky decomposition
    la = slvLL(G,-(getgd() + getb()*dt));
    return 1;
  }

  int MultiBodySystem::solveGaussSeidel(double dt) 
  {
    // SOLVEGAUSSSEIDEL solves constraint equations with Gauss-Seidel scheme
    s = getgd() + getb()*dt ;

    checkForTermination(dt);
    if(term) return 0 ;

    int iter;
    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {
      for(vector<Link*>::iterator ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) (**ic).solveGS(dt);
      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
	checkTermLevel++;
	checkForTermination(dt);
	if(term) break;
      }
    }
    return iter;
  }

  int MultiBodySystem::solveFixpointTotal(double dt)
  {
    // SOLVEFIXEDPOINTTOTAL solves constraint equations with total step iteration

    updaterFactors();

    Vec s0 = getgd() + getb()*dt ;
    s = s0;

    checkForTermination(dt);
    if(term) return 0 ;

    int iter, level = 0;
    //    int checkTermLevel = 0;

    for(iter = 1; iter<=maxIter; iter++) {
      double *a = getGs()();
      int *ia = getGs().Ip();
      int *ja = getGs().Jp();
      for(int i=0; i < G.size(); i++) {
	for(int j=ia[i]; j<ia[1+i]; j++) s(i) += a[j]*la(ja[j]);
      }

      if(level < decreaseLevels.size() && iter > decreaseLevels(level)) {
	level++;
	decreaserFactors();
	if(warnLevel>=2) cout <<endl<< "Warning: decreasing r-factors at iter = " << iter<<endl;
      }

      for(vector<Link*>::iterator ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) (*ic)->projectJ(dt);

      s = s0;
      checkForTermination(dt);
      if(term) break;
    }
    return iter;
  }

  int MultiBodySystem::solveFixpointSingle(double dt)
  {
    // SOLVEFIXEDPOINTSINGLE solves constraint equations with single step iteration

    updaterFactors();

    s = getgd() + getb()*dt;

    checkForTermination(dt);
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

      Subsystem::solveFixpointSingle(dt);

      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
	checkTermLevel++;
	checkForTermination(dt);
	if(term) break;
      }
    }
    return iter;
  }


  void MultiBodySystem::residualProj(double dt) 
  {

    for(vector<Link*>::iterator ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) (**ic).residualProj(dt);
  }

  void MultiBodySystem::checkForTermination(double dt) 
  {

    term = true;
    for(vector<Subsystem*>::iterator i = subsystem.begin(); i != subsystem.end(); ++i) 
      (*i)->checkForTermination(dt); 

    for(vector<Link*>::iterator i = linkSetValuedActive.begin(); i != linkSetValuedActive.end(); ++i) {
      (**i).checkForTermination(dt);
      if(term == false) return;
    }
  }

  void MultiBodySystem::residualProjJac(double dt) {

    for(vector<Link*>::iterator ic = linkSetValuedActive.begin(); ic != linkSetValuedActive.end(); ++ic) {
      (**ic).residualProjJac(dt);
    }
  }

  int MultiBodySystem::solve(double dt) 
  {
    // SOLVE solves prox-functions depending on solver settings
    // INPUT t	time

    if(la.size()==0) return 0;

    if(useOldla)initla();
    else la.init(0);

    int iter;
    Vec laOld;
    laOld = la;
    iter = (this->*solve_)(dt); // solver election
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

  int MultiBodySystem::solveRootFinding(double dt)
  {
    // SOLVEROOTFINDING solves constraint equations with general Newton method

    updaterFactors();

    s = getgd() + getb()*dt;
    int iter;
    //    int prim = 0;
    int checkTermLevel = 0;

    residualProj(dt); 
    double nrmf0 = nrm2(res);
    Vec res0 = res.copy();

    checkForTermination(dt);
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
	  residualProj(dt);
	  la(j) = xj;
	  Jprox.col(j) = (res-res0)/dx;
	}
      } 
      else residualProjJac(dt);
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
	residualProj(dt);
	nrmf = nrm2(res);
	if(nrmf < nrmf0) break;

	alpha = 0.5*alpha;  
      }
      nrmf0 = nrmf;
      res0 = res;

      if(checkTermLevel >= checkTermLevels.size() || iter > checkTermLevels(checkTermLevel)) {
	checkTermLevel++;
	checkForTermination(dt);
	if(term) break;
      }
    }
    return iter;
  }

  void MultiBodySystem::dropContactMatrices() 
  {
    // DROPCONTACTMATRICES writes a file with relevant matrices for debugging

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

  string MultiBodySystem::getSolverInfo() 
  {
    // GETSOLVERINFO returns solver information

    stringstream info;

    // Solver-Name
    if(solver == GaussSeidel) info << "GaussSeidel";
    else if(solver == LinearEquations) info << "LinearEquations";
    else if(solver == FixedPointSingle) info << "FixedPointSingle";
    else if(solver == FixedPointTotal) info << "FixedPointTotal";
    else if(solver == RootFinding) info << "RootFinding";

    // Gauss-Seidel & solveLL do not depend on the following ...
    if(solver!=GaussSeidel && solver!=LinearEquations) {
      info << "(";

      // r-Factor strategy
      if(strategy==global) info << "global";
      else if(strategy==local) info << "local";

      // linear algebra for RootFinding only
      if(solver == RootFinding) {
	info << ",";
	if(linAlg==LUDecomposition) info << "LU";
	else if(linAlg==LevenbergMarquardt) info << "LM";
	else if(linAlg==PseudoInverse) info << "PI";
      }
      info << ")";
    }
    return info.str();
  }

  void MultiBodySystem::initDataInterfaceBase() 
  {
    vector<Link*>::iterator il1;
    for(il1 = link.begin(); il1 != link.end(); ++il1) (*il1)->initDataInterfaceBase(this);
    vector<Object*>::iterator io1;
    for(io1 = object.begin(); io1 != object.end(); ++io1) (*io1)->initDataInterfaceBase(this);
    vector<ExtraDynamicInterface*>::iterator ie1;
    for(ie1 = EDI.begin(); ie1 != EDI.end(); ++ie1) (*ie1)->initDataInterfaceBase(this); 
  }

  void MultiBodySystem::updateJh(double t) 
  {
    Jh.init(0.0);
    for (vector<Object*>::iterator i = object.begin(); i != object.end(); ++i) (**i).updateJh(t);
  }


  CoordinateSystem* MultiBodySystem::findCoordinateSystem(const string &name) {

    istringstream stream(name);

    char dummy[10000];
    vector<string> l;
    do {
      stream.getline(dummy,10000,'.');
      l.push_back(dummy);
    } while(!stream.eof());

    if(l.size() == 1)
      throw 5;

    if(l.size() == 2)
      return getCoordinateSystem(l[1]);

    Subsystem *sys = this;
    for(unsigned int i=1; i<l.size()-2; i++) {
      sys = static_cast<Subsystem*>(sys->getObject(l[i]));
    }
    return sys->getObject(l[l.size()-2])->getCoordinateSystem(l[l.size()-1]);
  }

  Contour* MultiBodySystem::findContour(const string &name) {

    istringstream stream(name);

    char dummy[10000];
    vector<string> l;
    do {
      stream.getline(dummy,10000,'.');
      l.push_back(dummy);
    } while(!stream.eof());

    if(l.size() == 1)
      throw 5;

    if(l.size() == 2)
      return getContour(l[1]);

    Subsystem *sys = this;
    for(unsigned int i=1; i<l.size()-2; i++) {
      sys = static_cast<Subsystem*>(sys->getObject(l[i]));
    }
    return sys->getObject(l[l.size()-2])->getContour(l[l.size()-1]);
  }

}
