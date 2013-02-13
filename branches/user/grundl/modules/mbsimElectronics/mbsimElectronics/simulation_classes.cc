#include "simulation_classes.h"
#include "mbsimControl/signal_.h"

using namespace fmatvec;
using namespace std;
using namespace MBSim;


namespace MBSimElectronics {

  void ElectronicObject::init(InitStage stage) {
    if(stage==preInit) {
      Object::init(stage);
      if(branch)
	dependency.push_back(branch);
    } 
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);

      if(getPlotFeature(plotRecursive)==enabled) {
	plotColumns.push_back("Charge");
	plotColumns.push_back("Current");

	Object::init(stage);
      }
    }
    else
      Object::init(stage);
  }

  void ElectronicObject::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(Q);
      plotVector.push_back(I);
      Object::plot(t,dt);
    }
  }

  void ElectronicLink::calcgSize() {
    gSize = 1;
  }

  void ElectronicLink::calcgdSize() {
    gdSize = 1;
  }

  void ElectronicLink::calclaSize() {
    laSize = 1;
  }

  void ElectronicLink::calclaSizeForActiveg() {
    calclaSize();
  }

  void ElectronicLink::calcgSizeActive() {
    calcgSize();
  }

  void ElectronicLink::calcgdSizeActive() {
    calcgdSize();
  }

  void ElectronicLink::init(InitStage stage) {
    if(stage==unknownStage) {
      h.push_back(Vec(branch->getJacobian().cols()));
      hLink.push_back(Vec(branch->getJacobian().cols()));
      r.push_back(Vec(branch->getJacobian().cols()));
      W.push_back(Mat(branch->getJacobian().cols(),laSize));
      V.push_back(Mat(branch->getJacobian().cols(),laSize));
  }
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);

      if(getPlotFeature(plotRecursive)==enabled) {
	plotColumns.push_back("Charge");
	plotColumns.push_back("Current");
	plotColumns.push_back("Voltage");
	plotColumns.push_back("la(0)");

	Link::init(stage);
      }
    }
    else if(stage==resize) {
      Link::init(stage);

      g.resize(1);
      gd.resize(1);
      gdn.resize(1);
      la.resize(1);
    }
    else
      Link::init(stage);
  }

  void ElectronicLink::updateg(double t) {
    Q = branch->getCharge()(0)*vz;
    g(0) = Q;
  }

  void ElectronicLink::updategd(double t) {
    I = branch->getCurrent()(0)*vz;
    gd(0) = I;
  }

  void ElectronicLink::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(Q);
      plotVector.push_back(I);
      plotVector.push_back(la(0)/(isSetValued()?dt:1.));
      plotVector.push_back(la(0)/(isSetValued()?dt:1.));
      Link::plot(t,dt);
    }
  }

  void ElectronicLink::updateW(double t) {

    W[0] += branch->getJacobian().T()*vz;
  }

  void ElectronicLink::updatehRef(const fmatvec::Vec &hParent, const fmatvec::Vec &hLinkParent, int j) {
    fmatvec::Index I = fmatvec::Index(branch->getParent()->gethInd(parent,j),branch->getParent()->gethInd(parent,j)+branch->getJacobian().cols()-1);
    h[0]>>hParent(I);
    hLink[0]>>hLinkParent(I);
  } 

  void ElectronicLink::updaterRef(const Vec &rParent, int j) {
    fmatvec::Index I = fmatvec::Index(branch->getParent()->gethInd(parent,j),branch->getParent()->gethInd(parent,j)+branch->getJacobian().cols()-1);
    r[0]>>rParent(I);
  } 

  void ElectronicLink::updateWRef(const fmatvec::Mat& WParent, int j) {
    Index J = Index(laInd,laInd+laSize-1);
    Index I = Index(branch->getParent()->gethInd(parent,j),branch->getParent()->gethInd(parent,j)+branch->getJacobian().cols()-1);
    W[0].resize()>>WParent(I,J);
 }

  void ElectronicLink::updateVRef(const fmatvec::Mat& ref, int j) {
  }

  void Mesh::init(InitStage stage) {
    if(stage==unknownStage) {
      T(0,0) = 1;
    }
    else if(stage==preInit) {
      Object::init(stage);
      if(precessor)
	dependency.push_back(precessor);
    } 
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);

      if(getPlotFeature(plotRecursive)==enabled) {
	Object::init(stage);
      }
    }
    else
      Object::init(stage);
  }

  void Branch::updateStateDependentVariables(double t) {
    Q(0) = 0;
    I(0) = 0;
    for(int i=0; i<mesh.size(); i++) {
      Q(0) += J(0,mesh[i]->getuInd())*mesh[i]->getq()(0);
      I(0) += J(0,mesh[i]->getuInd())*mesh[i]->getu()(0);
    }
  }

  void Branch::setvz(int vz_, Mesh* mesh_) {
    for(int i=0; i<mesh.size(); i++)
      if(mesh_ == mesh[i])
	vz[i] = vz_;
  }

  void Branch::init(InitStage stage) {
    if(stage==MBSim::unknownStage) {
      if(J.cols() == 0) {
	J.resize(1,gethSize());
	for(int i=0; i<mesh.size(); i++)
	  J(0,mesh[i]->getuInd()) = vz[i]; 
      }
    }
    else
      Object::init(stage);
  }

  void Branch::addConnectedBranch(Branch *branch) {
    connectedBranch.push_back(branch);
  }

  void Branch::buildTreeBranches(Branch* callingBranch, vector<Branch*> &treeBranch, unsigned int nmax) {
    if(getStartTerminal()->getFlag() != -1 || getEndTerminal()->getFlag() != -1) {
      treeBranch.push_back(this);
      getStartTerminal()->setFlag(-1);
      getEndTerminal()->setFlag(-1);
    }
    for(unsigned int i=0; i<connectedBranch.size(); i++)
      if(connectedBranch[i] != callingBranch) {
	if(connectedBranch[i]->getFlag()==0) {
	  connectedBranch[i]->setFlag(1);

	  if(treeBranch.size() < nmax)
	    connectedBranch[i]->buildTreeBranches(this,treeBranch,nmax);
	}
      }
  }

  void Branch::buildMeshes(Terminal *callingTerminal, Branch* callingBranch, Mesh* currentMesh, bool &foundMesh) {
    if(callingBranch == 0) {
      callingBranch = this;
    } 
    if(getFlag() == 3)
      setFlag(4);
    Terminal *sT;
    sT = callingTerminal == getEndTerminal() ? getStartTerminal() : getEndTerminal();
    for(unsigned int i=0; i<sT->getNumberOfConnectedBranches(); i++) {
      if(sT->getBranch(i) != this) { // nur vorwärts laufen
	if(sT->getBranch(i)->getFlag() == 3) // Branch ist TreeBranch
	  sT->getBranch(i)->buildMeshes(sT, callingBranch, currentMesh, foundMesh);
	else if(sT->getBranch(i) == callingBranch)
	  foundMesh = true;
	if(foundMesh) {
	  if(getFlag()==4)
	    setFlag(3);
	  connect(currentMesh);
	  return;
	}
      }
    }
  }

  Inductor::Inductor(const string &name) : ElectronicObject(name), L(1) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Inductor::updateStateDependentVariables(double t) {
    Q = branch->getCharge()(0)*vz;
    I = branch->getCurrent()(0)*vz;
  }

  void Inductor::updateM(double t) {
    M += L*JTJ(branch->getJacobian());
  }

  Diode::Diode(const string &name) : ElectronicLink(name), sv(false) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Diode::updateh(double t) {
    double slope=1e4;
    la(0) = (I >= 0) ? 0 : -slope*I;
    h[0] += branch->getJacobian().T()*la(0)*vz; 
    hLink[0] += branch->getJacobian().T()*la(0)*vz; 
  }

  void Diode::checkImpactsForTermination() {

    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    gdn(0) = b(laIndDS);
    //cout << gd(0) << " " << gdn(0) << endl;
    for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++) {
      gdn(0) += a[j]*laMBS(ja[j]);
    }

    //cout <<endl<< gd(0) << " "<< gdn(0) <<" "  << la(0)  <<endl;
    if(gdn(0) >= -gdTol && fabs(la(0)) <= LaTol)
      ; 
    else if(la(0) >= -LaTol && fabs(gdn(0)) <= gdTol)
      ;
    else {
      ds->setTermination(false);
      return;
    }
  }

  void Diode::solveImpactsGaussSeidel() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]+1; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);

      if(gdn(0) >= 0)
	la(0) = 0;
      else 
	la(0) = -gdn(0)/a[ia[laIndDS]];
  }

  Switch::Switch(const string &name) : ElectronicLink(name), sv(false) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Switch::updateW(double t) {
    U0 = voltageSignal->getSignal(t)(0);
    ElectronicLink::updateW(t);
  }

  void Switch::updateh(double t) {
    double gdLim = 0.01;
    U0 = voltageSignal->getSignal(t)(0);
    if(fabs(I) < gdLim)
      la(0) = -U0*I/gdLim;
    else
      la(0) = I>0?-U0:U0;
    h[0] += branch->getJacobian().T()*la(0)*vz; 
    hLink[0] += branch->getJacobian().T()*la(0)*vz; 
  }

  void Switch::checkImpactsForTermination() {

    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

    //double U0 = voltageSignal->getSignal(t)(0);

    gdn(0) = b(laIndDS);
    for(int j=ia[laIndDS]; j<ia[laIndDS+1]; j++)
      gdn(0) += a[j]*laMBS(ja[j]);

     if(fabs(la(0) + gdn(0)/fabs(gdn(0))*fabs(U0)) <= laTol)
       ;
    else if(fabs(la(0)) <= fabs(U0)+laTol && fabs(gdn(0)) <= gdTol)
      ;
    else {
      ds->setTermination(false);
      return;
    }
  }

  void Switch::solveImpactsGaussSeidel() {
    double *a = ds->getGs()();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &laMBS = ds->getla();
    Vec &b = ds->getb();

      gdn(0) = b(laIndDS);
      for(int j=ia[laIndDS]+1; j<ia[laIndDS+1]; j++)
        gdn(0) += a[j]*laMBS(ja[j]);

      double sdG = -gdn(0)/a[ia[laIndDS]];
      if(fabs(sdG)<=U0) 
	la(0) = sdG;
      else 
	la(0) = U0<=sdG ? U0 : -U0;
  }

  Resistor::Resistor(const string &name) : ElectronicLink(name), R(1) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Resistor::updateh(double t) {
    la(0) = -R*I; 

    h[0] += branch->getJacobian().T()*la(0)*vz; 
    hLink[0] += branch->getJacobian().T()*la(0)*vz; 
  }

  double Resistor::computeVoltage(double t) {
    return -R*I;  
  }

  Capacitor::Capacitor(const string &name) : ElectronicLink(name), C(1) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Capacitor::updateh(double t) {
    la(0) = -Q/C; 
    h[0] += branch->getJacobian().T()*la(0)*vz;
    hLink[0] += branch->getJacobian().T()*la(0)*vz;
  }

  VoltageSource::VoltageSource(const string &name) : ElectronicLink(name) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void VoltageSource::updateh(double t) {
    la(0) = voltageSignal->getSignal(t)(0);
    h[0] += branch->getJacobian().T()*la(0)*vz;
    hLink[0] += branch->getJacobian().T()*la(0)*vz;
  }

}


