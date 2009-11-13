#include "simulation_classes.h"

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

  void ElectronicLink::init(InitStage stage) {
    if(stage==unknownStage) {
      h.push_back(fmatvec::Vec(branch->getJacobian().cols()));
      hLink.push_back(fmatvec::Vec(branch->getJacobian().cols()));
    }
    else if(stage==MBSim::plot) {
      updatePlotFeatures(parent);

      if(getPlotFeature(plotRecursive)==enabled) {
	plotColumns.push_back("Charge");
	plotColumns.push_back("Current");
	plotColumns.push_back("Voltage");

	Link::init(stage);
      }
    }
    else
      Link::init(stage);
  }

  void ElectronicLink::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(Q);
      plotVector.push_back(I);
      plotVector.push_back(U);
      Link::plot(t,dt);
    }
  }

  void ElectronicLink::updatehRef(const fmatvec::Vec &hParent, const fmatvec::Vec &hLinkParent, int j) {
    fmatvec::Index I = fmatvec::Index(branch->getParent()->gethInd(parent,j),branch->getParent()->gethInd(parent,j)+branch->getJacobian().cols()-1);
    h[0]>>hParent(I);
    hLink[0]>>hLinkParent(I);
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


  Diode::Diode(const string &name) : ElectronicLink(name) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Diode::updateh(double t) {
    Q = branch->getCharge()(0)*vz;
    I = branch->getCurrent()(0)*vz;
    double slope=1e4;
    U = (I >= 0) ? 0 : slope*I;
    h[0] -= trans(branch->getJacobian())*U*vz; 
    hLink[0] -= trans(branch->getJacobian())*U*vz; 
  }

  Switch::Switch(const string &name) : ElectronicLink(name) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Switch::updateh(double t) {
    Q = branch->getCharge()(0)*vz;
    I = branch->getCurrent()(0)*vz;
    double gdLim = 0.01;
    double U0 = (*voltageSignal)(t)(0);
    if(fabs(I) < gdLim)
      U = -U0*I/gdLim;
    else
      U = I>0?-U0:U0;
    U*=-1;
    h[0] -= trans(branch->getJacobian())*U*vz; 
    hLink[0] -= trans(branch->getJacobian())*U*vz; 
  }

  Resistor::Resistor(const string &name) : ElectronicLink(name), R(1) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Resistor::updateh(double t) {
    Q = branch->getCharge()(0)*vz;
    I = branch->getCurrent()(0)*vz;
    U = R*I; 

    h[0] -= trans(branch->getJacobian())*U*vz; 
    hLink[0] -= trans(branch->getJacobian())*U*vz; 
  }

  double Resistor::computeU(double t) {
    return R*branch->getCurrent()(0)*vz; // TODO Vorzeichen?
  }

  Capacitor::Capacitor(const string &name) : ElectronicLink(name), C(1) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Capacitor::updateh(double t) {
    Q = branch->getCharge()(0)*vz;
    I = branch->getCurrent()(0)*vz;
    U = Q/C; 
    h[0] -= trans(branch->getJacobian())*U*vz;
    hLink[0] -= trans(branch->getJacobian())*U*vz;
  }

  VoltageSource::VoltageSource(const string &name) : ElectronicLink(name) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void VoltageSource::updateh(double t) {
    Q = branch->getCharge()(0)*vz;
    I = branch->getCurrent()(0)*vz;
    U = (*voltageSignal)(t)(0);
    h[0] -= trans(branch->getJacobian())*U*vz;
    hLink[0] -= trans(branch->getJacobian())*U*vz;
  }

}


