#ifndef _SIMULATION_CLASSES_H
#define _SIMULATION_CLASSES_H

#include "mbsim/tree.h"
#include "mbsim/object.h"
#include "mbsim/link.h"
#include "mbsim/userfunction.h"
#include "mbsim/mbsim_event.h"
#include "modeling_classes.h"
#include <string>

// Just a test voltage signal
class Signal : public MBSim::UserFunction {
  public:
    fmatvec::Vec operator()(double t) {
      fmatvec::Vec U(1);
      U(0) = 3;
      return U;
    }
};

class Mesh : public MBSim::Object {
  protected:
  public:
    Mesh(const std::string &name) : MBSim::Object(name) {}
    void calcqSize() { qSize = 1;}
    void calcuSize(int j) { uSize[0] = 1; uSize[1] = 1;}
    void updateStateDependentVariables(double t) {};
    void updateJacobians(double t) {};
    void updateSecondJacobians(double t) {};
    void init();
#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Group* getAMVisGrp() { return 0; }
#endif
};

class Wire : public MBSim::Object {
  protected:
    fmatvec::Mat J;
    fmatvec::Vec Q, I;
    vector<Mesh*> mesh;
  public:
    Wire(const std::string &name) : Object(name) {}
    void updateStateDependentVariables(double t);
    void updateJacobians(double t) {};
    void updateSecondJacobians(double t) {};
    const fmatvec::Mat& getJacobian() const {return J;}
    fmatvec::Mat& getJacobian() {return J;}
    const fmatvec::Vec& getCurrent() const {return I;}
    fmatvec::Vec& getCurrent() {return I;}
    const fmatvec::Vec& getCharge() const {return Q;}
    fmatvec::Vec& getCharge() {return Q;}
    void connect(Mesh *mesh_) {mesh.push_back(mesh_);}
    void init();
#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Group* getAMVisGrp() { return 0; }
#endif
};

class ElectricalLink : public MBSim::Link {
  protected:
    std::vector<Wire*> wire;
  public:
    ElectricalLink(const std::string &name);
    void updateg(double t) {}
    void updategd(double t) {}
    void connect(Wire *wire_) {wire.push_back(wire_);}
    bool isActive() const {return true;}
    bool gActiveChanged() {return true;}
    void init();
    void updatehRef(const fmatvec::Vec &hParent, int j=0);

    /* INHERITED INTERFACE OF LINKINTERFACE */
    virtual void updater(double t) { throw new MBSim::MBSimError("ERROR (ElectricalLink::updater): Not implemented!"); }
    /*****************************************************/
    
    /* INHERITED INTERFACE OF LINK */
    virtual void updateWRef(const fmatvec::Mat& ref, int j) { throw new MBSim::MBSimError("ERROR (ElectricalLink::updateWRef): Not implemented!"); }
    virtual void updateVRef(const fmatvec::Mat& ref, int j) { throw new MBSim::MBSimError("ERROR (ElectricalLink::updateVRef): Not implemented!"); }
    virtual void updaterRef(const fmatvec::Vec& ref) { throw new MBSim::MBSimError("ERROR (ElectricalLink::updaterRef): Not implemented!"); }
    /*****************************************************/
};

class Resistor : public ElectricalLink {
  protected:
    double R;
  public:
    Resistor(const std::string &name);
    void updateh(double t);
    void setResistance(double R_) { R = R_;}
};

class Capacitor : public ElectricalLink {
  protected:
    double C;
  public:
    Capacitor(const std::string &name);
    void updateh(double t);
    void setCapacity(double C_) { C = C_;}
};

class VoltageSource : public ElectricalLink {
  protected:
    MBSim::UserFunction *voltageSignal;
  public:
    VoltageSource(const std::string &name);
    void updateh(double t);
    void setVoltageSignal(MBSim::UserFunction* f) {voltageSignal = f;}
};

class Inductor : public MBSim::Object {
  protected:
    double L;
    std::vector<Wire*> wire;
  public:
    Inductor(const std::string &name);
    void updateStateDependentVariables(double t) {};
    void updateJacobians(double t) {};
    void updateSecondJacobians(double t) {};
    void updateM(double t); 
    void setInductance(double L_) { L = L_;}
    void connect(Wire *wire_) {wire.push_back(wire_);}
#ifdef HAVE_AMVISCPPINTERFACE
      AMVis::Group* getAMVisGrp() { return 0; }
#endif
};

class ElectricalCircuit : public MBSim::Tree {
  protected:
    std::vector<Pin*> pin;
    std::vector<Component*> comp;
  public:
    ElectricalCircuit(const std::string &name) : Tree(name) {}
    void addComponent(Component *comp);
    Component* getComponent(const std::string &name, bool check=true);
    void addPin(Pin *pin);
    void addPin(const std::string &str);
    Pin* getPin(const std::string &name, bool check=true);
    void preinit();
    void init();
    void facLLM();
    void buildListOfPins(std::vector<Pin*> &pin, bool recursive = true);
};

#endif
