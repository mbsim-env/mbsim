#include <config.h>
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
    else if(stage==plotting) {
      updatePlotFeatures();

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
      plotVector.push_back(getCharge(t));
      plotVector.push_back(getCurrent(t));
      Object::plot(t,dt);
    }
  }

  void ElectronicLink::calcgSize(int j) {
    gSize = 1;
  }

  void ElectronicLink::calcgdSize(int j) {
    gdSize = 1;
  }

  void ElectronicLink::calclaSize(int j) {
    laSize = 1;
  }

  void ElectronicLink::init(InitStage stage) {
    if(stage==unknownStage) {
      h[0].push_back(Vec(branch->getJacobian(0).cols()));
      r[0].push_back(Vec(branch->getJacobian(0).cols()));
      W[0].push_back(Mat(branch->getJacobian(0).cols(),laSize));
      V[0].push_back(Mat(branch->getJacobian(0).cols(),laSize));
      h[1].push_back(Vec(branch->getJacobian(1).cols()));
      r[1].push_back(Vec(branch->getJacobian(1).cols()));
      W[1].push_back(Mat(branch->getJacobian(1).cols(),laSize));
      V[1].push_back(Mat(branch->getJacobian(1).cols(),laSize));
  }
    else if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
	plotColumns.push_back("Charge");
	plotColumns.push_back("Current");
	plotColumns.push_back("Voltage");

	Link::init(stage);
      }
    }
    else if(stage==resize) {
      Link::init(stage);

      g.resize(1);
      gd.resize(1);
      gdn.resize(1);
      laSV.resize(1);
      laMV.resize(1);
    }
    else
      Link::init(stage);
  }

  void ElectronicLink::updateg(double t) {
    g(0) = getCharge(t);
  }

  void ElectronicLink::updategd(double t) {
    gd(0) = getCurrent(t);
  }

  void ElectronicLink::updateGeneralizedSetValuedForces(double t) {
    laMV = la;
    updlaMV = false;
  }

  void ElectronicLink::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(getCharge(t));
      plotVector.push_back(getCurrent(t));
      plotVector.push_back(getGeneralizedForce(t)(0));
      Link::plot(t,dt);
    }
  }

  void ElectronicLink::updateW(double t, int j) {
    W[j][0] += branch->getJacobian(j).T()*vz;
  }

  void ElectronicLink::updatehRef(const fmatvec::Vec &hParent, int j) {
    fmatvec::Index I = fmatvec::Index(branch->gethInd(j),branch->gethInd(j)+branch->getJacobian(j).cols()-1);
    h[j][0]>>hParent(I);
  } 

  void ElectronicLink::updaterRef(const Vec &rParent, int j) {
    fmatvec::Index I = fmatvec::Index(branch->gethInd(j),branch->gethInd(j)+branch->getJacobian(j).cols()-1);
    r[j][0]>>rParent(I);
  } 

  void ElectronicLink::updateWRef(const fmatvec::Mat& WParent, int j) {
    Index J = Index(laInd,laInd+laSize-1);
    Index I = Index(branch->gethInd(j),branch->gethInd(j)+branch->getJacobian(j).cols()-1);
    W[j][0].resize()>>WParent(I,J);
  }

  void ElectronicLink::updateVRef(const fmatvec::Mat& ref, int j) {
  }

  void ElectronicLink::updateh(double t, int j) {
    h[j][0] += branch->getJacobian(j).T()*getGeneralizedSingleValuedForce(t)(0)*vz;
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
    else if(stage==plotting) {
      updatePlotFeatures();

      if(getPlotFeature(plotRecursive)==enabled) {
	Object::init(stage);
      }
    }
    else
      Object::init(stage);
  }

  void Branch::updateCharge(double t) {
    Q(0) = 0;
    for(size_t i=0; i<mesh.size(); i++)
      Q(0) += J[0](0,mesh[i]->gethSize()-1)*mesh[i]->getq()(0);
    updQ = false;
  }

  void Branch::updateCurrent(double t) {
    I(0) = 0;
    for(size_t i=0; i<mesh.size(); i++)
      I(0) += J[0](0,mesh[i]->gethSize()-1)*mesh[i]->getu()(0);
    updI = false;
  }

  void Branch::setvz(double vz_, Mesh* mesh_) {
    for(size_t i=0; i<mesh.size(); i++)
      if(mesh_ == mesh[i])
	vz[i] = vz_;
  }

  void Branch::init(InitStage stage) {
    if(stage==unknownStage) {
      if(J[0].cols() == 0) {
	J[0].resize(1,gethSize(0));
	for(size_t i=0; i<mesh.size(); i++)
	  J[0](0,mesh[i]->gethSize()-1) = vz[i]; 
      }
      if(J[1].cols() == 0) {
	J[1].resize(1,gethSize(1),INIT,1);
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
    for(int i=0; i<sT->getNumberOfConnectedBranches(); i++) {
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

  void Inductor::updateM(double t, int i) {
    M[i] += L*JTJ(branch->getJacobian(i));
  }

  Diode::Diode(const string &name) : ElectronicLink(name), sv(false) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Diode::updateGeneralizedSingleValuedForces(double t) {
    double slope=1e4;
    double I = getCurrent(t);
    laSV(0) = (I >= 0) ? 0 : -slope*I;
    updlaSV = false;
  }

  void Diode::checkImpactsForTermination(double t, double dt) {

    const double *a = ds->getGs(t)();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &LaMBS = ds->getLa();
    Vec &b = ds->getb(false);

    gdn(0) = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++) {
      gdn(0) += a[j]*LaMBS(ja[j]);
    }

    if(gdn(0) >= -gdTol && fabs(La(0)) <= LaTol);
    else if(La(0) >= -LaTol && fabs(gdn(0)) <= gdTol);
    else {
      ds->setTermination(false);
      return;
    }
  }

  void Diode::solveImpactsGaussSeidel(double t, double dt) {
    const double *a = ds->getGs(t)();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &LaMBS = ds->getLa();
    Vec &b = ds->getb(false);

    gdn(0) = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn(0) += a[j]*LaMBS(ja[j]);

    if(gdn(0) >= 0)
      La(0) = 0;
    else
      La(0) = -gdn(0)/a[ia[laInd]];
  }

  Switch::Switch(const string &name) : ElectronicLink(name), sv(false) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Switch::init(InitStage stage) {
    if(stage==preInit) {
      ElectronicLink::init(stage);
      addDependency(voltageSignal);
    }
    else
      ElectronicLink::init(stage);
  }

  void Switch::updateGeneralizedSingleValuedForces(double t) {
    double gdLim = 0.01;
    double U0 = (*voltageSignal)(t)(0);
    double I = getCurrent(t);
    if(fabs(I) < gdLim)
      laSV(0) = -U0*I/gdLim;
    else
      laSV(0) = I>0?-U0:U0;
    updlaSV = false;
  }

  void Switch::checkImpactsForTermination(double t, double dt) {

    const double *a = ds->getGs(t)();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &LaMBS = ds->getLa();
    Vec &b = ds->getb(false);

    double U0 = (*voltageSignal)(t)(0);

    gdn(0) = b(laInd);
    for(int j=ia[laInd]; j<ia[laInd+1]; j++)
      gdn(0) += a[j]*LaMBS(ja[j]);

    if(fabs(La(0) + gdn(0)/fabs(gdn(0))*fabs(U0)) <= LaTol)
      ;
    else if(fabs(La(0)) <= fabs(U0)+LaTol && fabs(gdn(0)) <= gdTol)
      ;
    else {
      ds->setTermination(false);
      return;
    }
  }

  void Switch::solveImpactsGaussSeidel(double t, double dt) {
    const double *a = ds->getGs(t)();
    int *ia = ds->getGs().Ip();
    int *ja = ds->getGs().Jp();
    Vec &LaMBS = ds->getLa();
    Vec &b = ds->getb(false);

    double U0 = (*voltageSignal)(t)(0);

    gdn(0) = b(laInd);
    for(int j=ia[laInd]+1; j<ia[laInd+1]; j++)
      gdn(0) += a[j]*LaMBS(ja[j]);

    double sdG = -gdn(0)/a[ia[laInd]];
    if(fabs(sdG)<=U0)
      La(0) = sdG;
    else
      La(0) = U0<=sdG ? U0 : -U0;
  }

  Resistor::Resistor(const string &name) : ElectronicLink(name), R(1) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Resistor::updateGeneralizedSingleValuedForces(double t) {
    laSV(0) = -R*getCurrent(t);
    updlaSV = false;
  }

  Capacitor::Capacitor(const string &name) : ElectronicLink(name), C(1) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Capacitor::updateGeneralizedSingleValuedForces(double t) {
    laSV(0) = -getCharge(t)/C;
    updlaSV = false;
  }

  VoltageSource::VoltageSource(const string &name) : ElectronicLink(name) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void VoltageSource::updateGeneralizedSingleValuedForces(double t) {
    laSV(0) = (*voltageSignal)(t)(0);
    updlaSV = false;
  }

}


