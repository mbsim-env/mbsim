#ifndef _SIMULATION_CLASSES_H
#define _SIMULATION_CLASSES_H

#include "mbsim/tree.h"
#include "mbsim/object.h"
#include "mbsim/link.h"
#include "mbsim/userfunction.h"
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
    Mesh(const string &name) : MBSim::Object(name) {}
    void calcqSize() { qSize = 1;}
    void calcuSize(int j) { uSize[0] = 1; uSize[1] = 1;}
    void updateKinematics(double t) {};
    void updateJacobians(double t) {};
    void updateSecondJacobians(double t) {};
    void init();
};

class Wire : public MBSim::Object {
  protected:
    fmatvec::Mat J;
    fmatvec::Vec Q, I;
  public:
    Wire(const string &name) : Object(name) {}
    void updateKinematics(double t);
    void updateJacobians(double t) {};
    void updateSecondJacobians(double t) {};
    const fmatvec::Mat& getJacobian() const {return J;}
    fmatvec::Mat& getJacobian() {return J;}
    const fmatvec::Vec& getCurrent() const {return I;}
    fmatvec::Vec& getCurrent() {return I;}
    const fmatvec::Vec& getCharge() const {return Q;}
    fmatvec::Vec& getCharge() {return Q;}
};

class ElectricalLink : public MBSim::Link {
  protected:
    vector<Wire*> wire;
  public:
    ElectricalLink(const string &name);
    void updateg(double t) {}
    void updategd(double t) {}
    void connect(Wire *wire_) {wire.push_back(wire_);}
    bool isActive() const {return true;}
    bool gActiveChanged() {return true;}
    void init();
    void updatehRef(const fmatvec::Vec &hParent, int j=0);
};

class Resistor : public ElectricalLink {
  protected:
    double R;
  public:
    Resistor(const string &name);
    void updateh(double t);
    void setResistance(double R_) { R = R_;}
};

class Capacitor : public ElectricalLink {
  protected:
    double C;
  public:
    Capacitor(const string &name);
    void updateh(double t);
    void setCapacity(double C_) { C = C_;}
};

class VoltageSource : public ElectricalLink {
  protected:
    MBSim::UserFunction *voltageSignal;
  public:
    VoltageSource(const string &name);
    void updateh(double t);
    void setVoltageSignal(MBSim::UserFunction* f) {voltageSignal = f;}
};

class Inductor : public MBSim::Object {
  protected:
    double L;
    vector<Wire*> wire;
  public:
    Inductor(const string &name);
    void updateKinematics(double t) {};
    void updateJacobians(double t) {};
    void updateSecondJacobians(double t) {};
    void updateM(double t); 
    void setInductance(double L_) { L = L_;}
    void connect(Wire *wire_) {wire.push_back(wire_);}
};

class ElectricalCircuit : public MBSim::Tree {
  protected:
    vector<Pin*> pin;
    vector<Component*> comp;
  public:
    ElectricalCircuit(const string &name) : Tree(name) {}
    void addComponent(Component *comp);
    Component* getComponent(const string &name, bool check=true);
    void addPin(Pin *pin);
    void addPin(const string &str);
    Pin* getPin(const string &name, bool check=true);
    void preinit();
    void buildListOfPins(std::vector<Pin*> &pin, bool recursive = true);
};

#endif
