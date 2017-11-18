#ifndef _SIMULATION_CLASSES_H
#define _SIMULATION_CLASSES_H

#include "mbsim/dynamic_system_solver.h"
#include "mbsim/objects/object.h"
#include "mbsim/links/link.h"
#include "mbsim/functions/function.h"
#include "mbsim/mbsim_event.h"
#include "modelling_classes.h"
#include <string>
//#include <mbsim/utils/function.h>

//namespace MBSimControl {
//  class Signal;
//}

namespace MBSimElectronics {

  class Branch;
  class Mesh : public MBSim::Object {
    protected:
      Object* precessor;
      std::vector<Branch*> branch;
    public:
      Mesh(const std::string &name) : MBSim::Object(name), precessor(nullptr) {}
      void calcqSize() override { qSize = 1;}
      void calcuSize(int j) override { uSize[j] = (j==0) ? 1 : 0;}
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setPrecessor(Object* obj) {precessor = obj;}
      void addBranch(Branch* branch_) {branch.push_back(branch_);}
      int getNumberOfBranches() {return branch.size();}
      Branch* getBranch(int i) {return branch[i];}
      std::shared_ptr<OpenMBV::Group> getOpenMBVGrp() override { return std::shared_ptr<OpenMBV::Group>(); }
  };

  class Branch : public MBSim::Object {
    protected:
      fmatvec::Mat J[2];
      fmatvec::Vec Q, I;
      std::vector<Mesh*> mesh;
      std::vector<double> vz;
      Terminal *startTerminal, *endTerminal;
      std::vector<Branch*> connectedBranch;
      int flag;
      Object* precessor;
      bool updQ, updI;
    public:
      Branch(const std::string &name) : Object(name), Q(1), I(1), flag(0), precessor(nullptr), updQ(true), updI(true) { }
      void calcuSize(int j) override { uSize[j] = (j==0) ? 0 : 1;}
      void updateCharge();
      void updateCurrent();
      const fmatvec::Mat& getJacobian(int j, bool check=true) const { return J[j];}
      fmatvec::Mat& getJacobian(int j, bool check=true) { return J[j];}
      const fmatvec::Vec& getCurrent(bool check=true) const { assert((not check) or (not updI)); return I; }
      fmatvec::Vec& getCurrent(bool check=true) { assert((not check) or (not updI)); return I; }
      const fmatvec::Vec& evalCurrent() { if(updI) updateCurrent(); return I; }
      const fmatvec::Vec& getCharge(bool check=true) const { assert((not check) or (not updQ)); return Q; }
      fmatvec::Vec& getCharge(bool check=true) { assert((not check) or (not updQ)); return Q; }
      const fmatvec::Vec& evalCharge() { if(updQ) updateCharge(); return Q; }
      void setvz(double vz_, Mesh* mesh);
      void connect(Mesh *mesh_) {mesh.push_back(mesh_);mesh_->addBranch(this);vz.push_back(0);}
      void clearMeshList() {mesh.clear();}
      int getNumberOfConnectedMeshes() const {return mesh.size();}
      Mesh* getMesh(int i) {return mesh[i];}
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void setStartTerminal(Terminal* p) {startTerminal = p; p->addConnectedBranch(this);}
      void setEndTerminal(Terminal* p) {endTerminal = p; p->addConnectedBranch(this);}
      Terminal* getStartTerminal() {return startTerminal;}
      Terminal* getEndTerminal() {return endTerminal;}
      void addConnectedBranch(Branch* branch);
      void buildTreeBranches(Branch* callingBranch, std::vector<Branch*> &treeBranch, unsigned int nmax);
      void buildMeshes(Terminal* callingTerminal, Branch* callingBranch, Mesh* currentMesh, bool &foundMesh);

      void setFlag(int f) { flag = f; }
      int getFlag() const { return flag; }
      void setPrecessor(Object* obj) {precessor = obj;}
      void resetUpToDate() override { Object::resetUpToDate(); updQ = true; updI = true; }
      std::shared_ptr<OpenMBV::Group> getOpenMBVGrp() override { return std::shared_ptr<OpenMBV::Group>(); }
  };

  class ElectronicLink : public MBSim::Link, public ElectronicComponent {
    protected:
      fmatvec::Vec gdn, gdd;
    public:
      ElectronicLink(const std::string &name) : Link(name) {}
      Element* getParent() override {return parent;}
      const Element* getParent() const override {return parent;}
      void setParent(Element* parent_) override {parent = parent_;}

      void calcSize() override { ng = 1; ngd = 1; nla = 1; }

      void calcgSize(int j) override;
      void calcgdSize(int j) override;
      void calclaSize(int j) override;
      void updateg() override;
      void updategd() override;
      void updateW(int j=0) override;
      void updateh(int j=0) override;
      void plot() override;
      std::string getName() const override {return Link::getName();}
      void setName(std::string name) override {Link::setName(name);}
      bool isActive() const override {return true;}
      bool gActiveChanged() override {return true;}
      bool isSingleValued() const override { return true; }
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      void updatehRef(const fmatvec::Vec &hParent, int j=0) override;
      void updaterRef(const fmatvec::Vec &rParent, int j=0) override;

      /* INHERITED INTERFACE OF LINKINTERFACE */
      virtual void updater() { THROW_MBSIMERROR("(ElectronicLink::updater): Not implemented!"); }
      /*****************************************************/

      /* INHERITED INTERFACE OF LINK */
      void updateWRef(const fmatvec::Mat& ref, int j) override; 
      void updateVRef(const fmatvec::Mat& ref, int j) override;
      virtual void updatedhdqRef(const fmatvec::Mat& ref, int i=0) {}
      virtual void updatedhduRef(const fmatvec::SqrMat& ref, int i=0) {}
      virtual void updatedhdtRef(const fmatvec::Vec& ref, int i=0) {}
      void resetUpToDate() override { Link::resetUpToDate(); updQ = true; updI = true; }
      /*****************************************************/
  };

  class Resistor : public ElectronicLink {
    protected:
      double R;
    public:
      Resistor(const std::string &name);
      void setResistance(double R_) { R = R_;}
      void updateGeneralizedForces() override;
  };

  class Capacitor : public ElectronicLink {
    protected:
      double C;
    public:
      Capacitor(const std::string &name);
      void setCapacity(double C_) { C = C_;}
      void updateGeneralizedForces() override;
  };

  class VoltageSource : public ElectronicLink {
    protected:
      MBSim::Function<fmatvec::VecV(double)> *voltageSignal;
    public:
      VoltageSource(const std::string &name);
      ~VoltageSource() override { delete voltageSignal; }
      void setVoltageSignal(MBSim::Function<fmatvec::VecV(double)> *func) {voltageSignal = func;}
      void updateGeneralizedForces() override;
  };

  class Diode : public ElectronicLink {
    protected:
      bool sv;
   public:
      Diode(const std::string &name);
      void setSetValued(bool flag) {sv = flag;}
      void updateGeneralizedForces() override;
      bool isSetValued() const override {return sv;}
      bool isSingleValued() const override {return not sv;}
      void checkImpactsForTermination() override;
      void solveImpactsGaussSeidel() override;
  };

  class Switch : public ElectronicLink {
    protected:
      MBSim::Function<fmatvec::VecV(double)> *voltageSignal;
      bool sv;
    public:
      Switch(const std::string &name);
      ~Switch() override { delete voltageSignal; }
      void setSetValued(bool flag) {sv = flag;}
      bool isSetValued() const override {return sv;}
      bool isSingleValued() const override {return not sv;}
      void updateGeneralizedForces() override;
      void checkImpactsForTermination() override;
      void solveImpactsGaussSeidel() override;
      void setVoltageSignal(MBSim::Function<fmatvec::VecV(double)> *func) {voltageSignal = func;}
  };

  class ElectronicObject : public MBSim::Object, public ElectronicComponent {
    protected:
    public:
      ElectronicObject(const std::string &name) : Object(name) {}
      void calcSize() override { nq = 0; nu = 1; }
      void init(InitStage stage, const MBSim::InitConfigSet &config) override;
      Element* getParent() override {return parent;}
      const Element* getParent() const override {return parent;}
      void setParent(Element* parent_) override {parent = parent_;}
      void plot() override;
      std::string getName() const override {return Object::getName();}
      void setName(std::string name) override {Object::setName(name);}
      void resetUpToDate() override { Object::resetUpToDate(); updQ = true; updI = true; }
      std::shared_ptr<OpenMBV::Group> getOpenMBVGrp() override { return std::shared_ptr<OpenMBV::Group>(); }
  };

  class Inductor : public ElectronicObject {
    protected:
      double L;
    public:
      Inductor(const std::string &name);
      void updateM() override;
      void setInductance(double L_) { L = L_;}
  };

}

#endif
