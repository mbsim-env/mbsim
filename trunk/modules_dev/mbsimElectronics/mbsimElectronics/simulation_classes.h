#ifndef _SIMULATION_CLASSES_H
#define _SIMULATION_CLASSES_H

#include "mbsim/tree.h"
#include "mbsim/dynamic_system_solver.h"
#include "mbsim/object.h"
#include "mbsim/link.h"
#include "mbsim/userfunction.h"
#include "mbsim/mbsim_event.h"
#include "modelling_classes.h"
#include <string>


class Mesh : public MBSim::Object {
  protected:
    Object* precessor;
  public:
    Mesh(const std::string &name) : MBSim::Object(name), precessor(0) {}
    void calcqSize() { qSize = 1;}
    void calcuSize(int j) { uSize[0] = 1; uSize[1] = 1;}
    void updateStateDependentVariables(double t) {};
    void updateJacobians(double t) {};
    void updateInverseKineticsJacobians(double t) {};
    void init(MBSim::InitStage stage);
    Object* getObjectDependingOn() const {return precessor;}
    void setPrecessor(Object* obj) {precessor = obj;}
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group* getOpenMBVGrp() { return 0; }
#endif
};

class Branch : public MBSim::Object {
  protected:
    fmatvec::Mat J;
    fmatvec::Vec Q, I;
    std::vector<Mesh*> mesh;
    Terminal *startTerminal, *endTerminal;
    std::vector<Branch*> connectedBranch;
    int flag;
    Object* precessor;
  public:
    Branch(const std::string &name) : Object(name), flag(0), precessor(0) {}
    void updateStateDependentVariables(double t);
    void updateJacobians(double t) {};
    void updateInverseKineticsJacobians(double t) {};
    const fmatvec::Mat& getJacobian() const {return J;}
    fmatvec::Mat& getJacobian() {return J;}
    const fmatvec::Vec& getCurrent() const {return I;}
    fmatvec::Vec& getCurrent() {return I;}
    const fmatvec::Vec& getCharge() const {return Q;}
    fmatvec::Vec& getCharge() {return Q;}
    void connect(Mesh *mesh_) {mesh.push_back(mesh_);}
    void clearMeshList() {mesh.clear();}
    int getNumberOfConnectedMeshes() const {return mesh.size();}
    Mesh* getMesh(int i) {return mesh[i];}
    void init(MBSim::InitStage stage);
    void setStartTerminal(Terminal* p) {startTerminal = p; p->addConnectedBranch(this);}
    void setEndTerminal(Terminal* p) {endTerminal = p; p->addConnectedBranch(this);}
    Terminal* getStartTerminal() {return startTerminal;}
    Terminal* getEndTerminal() {return endTerminal;}
    void addConnectedBranch(Branch* branch);
    void buildTreeBranches(Branch* callingBranch, std::vector<Branch*> &treeBranch, unsigned int nmax);
//    std::vector<Mesh*> buildMeshes(Branch* callingBranch, Mesh* currentMesh, bool &flag);
    void buildMeshes(Terminal* callingTerminal, Branch* callingBranch, Mesh* currentMesh, bool &foundMesh);
    void setFlag(int f) { flag = f; }
    int getFlag() const { return flag; }
    void plot(double t, double dt);
    Object* getObjectDependingOn() const {return precessor;}
    void setPrecessor(Object* obj) {precessor = obj;}
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group* getOpenMBVGrp() { return 0; }
#endif
};

class ElectronicLink : public MBSim::Link, public ElectronicComponent {
  protected:
  public:
    ElectronicLink(const std::string &name) : Link(name) {}

    void updateg(double t) {}
    void updategd(double t) {}
    virtual std::string getName() const {return Link::getName();}
    virtual void setName(std::string name) {Link::setName(name);}
    bool isActive() const {return true;}
    bool gActiveChanged() {return true;}
    void init(MBSim::InitStage stage);
    void updatehRef(const fmatvec::Vec &hParent, const fmatvec::Vec &hLinkParent, int j=0);

    /* INHERITED INTERFACE OF LINKINTERFACE */
    virtual void updater(double t) { throw new MBSim::MBSimError("ERROR (ElectronicLink::updater): Not implemented!"); }
    /*****************************************************/
    
    /* INHERITED INTERFACE OF LINK */
    virtual void updateWRef(const fmatvec::Mat& ref, int j) { throw new MBSim::MBSimError("ERROR (ElectronicLink::updateWRef): Not implemented!"); }
    virtual void updateVRef(const fmatvec::Mat& ref, int j) { throw new MBSim::MBSimError("ERROR (ElectronicLink::updateVRef): Not implemented!"); }
    virtual void updaterRef(const fmatvec::Vec& ref) { throw new MBSim::MBSimError("ERROR (ElectronicLink::updaterRef): Not implemented!"); }
    virtual void updatedhdqRef(const fmatvec::Mat& ref, int i=0) {}
    virtual void updatedhduRef(const fmatvec::SqrMat& ref, int i=0) {}
    virtual void updatedhdtRef(const fmatvec::Vec& ref, int i=0) {}
    /*****************************************************/
};

class Resistor : public ElectronicLink {
  protected:
    double R;
  public:
    Resistor(const std::string &name);
    void updateh(double t);
    void setResistance(double R_) { R = R_;}
    double computeU(double t);
    void init(MBSim::InitStage stage);
    void plot(double t, double dt);
};

class Capacitor : public ElectronicLink {
  protected:
    double C;
  public:
    Capacitor(const std::string &name);
    void updateh(double t);
    void setCapacity(double C_) { C = C_;}
};

class VoltageSource : public ElectronicLink {
  protected:
    MBSim::UserFunction *voltageSignal;
  public:
    VoltageSource(const std::string &name);
    void updateh(double t);
    void setVoltageSignal(MBSim::UserFunction* f) {voltageSignal = f;}
};

class Diode : public ElectronicLink {
  protected:
  public:
    Diode(const std::string &name);
    void updateh(double t);
};

class Switch : public ElectronicLink {
  protected:
    MBSim::UserFunction *voltageSignal;
  public:
    Switch(const std::string &name);
    void updateh(double t);
    void setVoltageSignal(MBSim::UserFunction* f) {voltageSignal = f;}
};

class ElectronicObject : public MBSim::Object, public ElectronicComponent {
  protected:
  public:
    ElectronicObject(const std::string &name) : Object(name) {}
    void updateStateDependentVariables(double t) {};
    void updateJacobians(double t) {};
    void updateInverseKineticsJacobians(double t) {};
    virtual std::string getName() const {return Object::getName();}
    virtual void setName(std::string name) {Object::setName(name);}
    Object* getObjectDependingOn() const {return branch;}
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::Group* getOpenMBVGrp() { return 0; }
#endif
};

class Inductor : public ElectronicObject {
  protected:
    double L;
  public:
    Inductor(const std::string &name);
    void updateM(double t); 
    void setInductance(double L_) { L = L_;}
};

#endif
