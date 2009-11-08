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
      plotVector.push_back(branch->getCharge()(0));
      plotVector.push_back(branch->getCurrent()(0));
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

	Link::init(stage);
      }
    }
    else
      Link::init(stage);
  }

  void ElectronicLink::plot(double t, double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(branch->getCharge()(0));
      plotVector.push_back(branch->getCurrent()(0));
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

  void Branch::init(InitStage stage) {
    if(stage==MBSim::unknownStage) {
      if(J.cols() == 0) {
	J.resize(1,gethSize());
	for(int i=0; i<mesh.size(); i++)
	  J(0,mesh[i]->getuInd()) = i==0?1:1; 
      }
    }
    else
      Object::init(stage);
  }

  void Branch::addConnectedBranch(Branch *branch) {
    connectedBranch.push_back(branch);
  }

  void Branch::buildTreeBranches(Branch* callingBranch, vector<Branch*> &treeBranch, unsigned int nmax) {
    //cout << "   insert " << this->getName() << endl;
    treeBranch.push_back(this);
    for(unsigned int i=0; i<connectedBranch.size(); i++)
      if(connectedBranch[i] != callingBranch) {
	if(connectedBranch[i]->getFlag()==0) {
	  connectedBranch[i]->setFlag(1);
	  //treeBranch.push_back(connectedBranch[i]);
	  //cout << "   insert " << connectedBranch[i]->getName() << endl;
	  //cout << nmax << endl;
	  //cout << treeBranch.size() << endl;

	  if(treeBranch.size() < nmax)
	    connectedBranch[i]->buildTreeBranches(this,treeBranch,nmax);
	}
      }
  }

  void Branch::buildMeshes(Terminal *callingTerminal, Branch* callingBranch, Mesh* currentMesh, bool &foundMesh) {
    cout << "begin " << getName() << endl;
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
	  cout << "connect " << name << " with " << currentMesh->getName() << endl;
	  connect(currentMesh);
	  return;
	}
      }
    }
    cout << " end " << getName() << endl;
  }

  Inductor::Inductor(const string &name) : ElectronicObject(name), L(1) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
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
    double slope=1e4;
    double Is = (branch->getCurrent()(0) >= 0) ? 0 : slope*branch->getCurrent()(0);
    h[0] -= trans(branch->getJacobian())*Is; // TODO Vorzeichen?
    hLink[0] -= trans(branch->getJacobian())*Is; // TODO Vorzeichen?
  }

  Switch::Switch(const string &name) : ElectronicLink(name) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Switch::updateh(double t) {
    double Is = branch->getCurrent()(0);
    double gdLim = 0.01;
    double U0 = (*voltageSignal)(t)(0);
    double U;
    if(fabs(Is) < gdLim)
      U = -U0*Is/gdLim;
    else
      U = Is>0?-U0:U0;
    U*=-1;
    h[0] -= trans(branch->getJacobian())*U; // TODO Vorzeichen?
    hLink[0] -= trans(branch->getJacobian())*U; // TODO Vorzeichen?
  }

  Resistor::Resistor(const string &name) : ElectronicLink(name), R(1) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Resistor::updateh(double t) {
    h[0] -= trans(branch->getJacobian())*R*branch->getCurrent(); // TODO Vorzeichen?
    hLink[0] -= trans(branch->getJacobian())*R*branch->getCurrent(); // TODO Vorzeichen?
  }

  double Resistor::computeU(double t) {
    return -R*branch->getCurrent()(0); // TODO Vorzeichen?
  }

  Capacitor::Capacitor(const string &name) : ElectronicLink(name), C(1) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void Capacitor::updateh(double t) {
    h[0] -= trans(branch->getJacobian())*branch->getCharge()/C; // TODO Vorzeichen?
    hLink[0] -= trans(branch->getJacobian())*branch->getCharge()/C; // TODO Vorzeichen?
  }

  VoltageSource::VoltageSource(const string &name) : ElectronicLink(name) {
    addTerminal("A");
    addTerminal("B");
    connectTerminal(terminal[0],terminal[1]);
  }

  void VoltageSource::updateh(double t) {
    h[0] -= trans(branch->getJacobian())*(*voltageSignal)(t);
    hLink[0] -= trans(branch->getJacobian())*(*voltageSignal)(t);
  }

   void Resistor::init(InitStage stage) {
    if(stage==MBSim::plot) {
     // if(getPlotFeature(plotRecursive)==enabled) {
	plotColumns.push_back("Voltage drop");

	ElectronicLink::init(stage);
     // }
    }
    else
      ElectronicLink::init(stage);
  }

  void Resistor::plot(double t, double dt) {
    //if(getPlotFeature(plotRecursive)==enabled) {
      plotVector.push_back(R*branch->getCurrent()(0));

      ElectronicLink::plot(t,dt);
   // }
  }

   //void Branch::buildMeshes(Branch* callingBranch, Mesh* currentMesh, bool &foundMesh) {
  //  cout << "begin " << getName() << endl;
  //  if(callingBranch == 0) {
  //    callingBranch = this;
  //  } 
  //  if(getFlag() == 3)
  //    setFlag(4);
  //  for(unsigned int i=0; i<connectedBranch.size(); i++) {
  //    cout << i << endl;
  //   // cout << " treeBranch? " << (connectedBranch[i]->getFlag()==3) <<endl;
  //   // cout << " found? " << (connectedBranch[i] == callingBranch) <<endl;
  //   // cout << " sondern: " << connectedBranch[i]->getName() <<endl;
  //    if(callingBranch)
  //      //cout << " und: " << callingBranch->getName() <<endl;
  //      if(connectedBranch[i]->getFlag() == 3)
  //	connectedBranch[i]->buildMeshes(callingBranch,currentMesh,foundMesh);
  //      else if(connectedBranch[i] == callingBranch)
  //	foundMesh = true;
  //    if(foundMesh) {
  //      if(getFlag()==4)
  //	setFlag(3);
  //      cout << "connect " << name << " with " << currentMesh->getName() << endl;
  //      connect(currentMesh);
  //      return;
  //    }
  //  }
  //  //cout << " end " << getName() << endl;
  //}
  //

  //vector<Mesh*> Branch::buildMeshes(Branch* callingBranch, Mesh* currentMesh, bool &foundMesh) {
  //  vector<Mesh*> mesh;
  //  if(callingBranch == 0) {
  //    currentMesh = new Mesh("Name");
  //    //this->connect(currentMesh);
  //    mesh.push_back(currentMesh);
  //  }
  //  for(unsigned int i=0; i<connectedBranch.size(); i++) {
  //    if(!foundMesh) {
  //      if(connectedBranch[i] != callingBranch) {
  //	if(connectedBranch[i]->getFlag()==3) {
  //	  connectedBranch[i]->buildMeshes(this,currentMesh,foundMesh);
  //	}
  //	else if(connectedBranch[i]->getFlag()==2) {
  //	  foundMesh = true;
  //	  cout << "found mesh" << endl;
  //	}
  //      }
  //    }
  //  }
  //  if(foundMesh)
  //    connect(currentMesh);
  //  return mesh;
  //}

}


