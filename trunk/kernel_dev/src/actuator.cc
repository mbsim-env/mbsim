#include "actuator.h"
#include "data_interface_base.h"
#include "multi_body_system.h"
#include "coordinate_system.h"

namespace MBSim {

  Actuator::Actuator(const string &name) : Link(name,false), func(0), KOSYID(1) {
 }

  Actuator::~Actuator() {
    delete func;
  }

  void Actuator::calclaSize() {
    Link::calclaSize();
    laSize = forceDir.cols()+momentDir.cols(); // cols = columns
  }

  void Actuator::init() {
    Link::init();
    IT = Index(0,forceDir.cols()-1);
    IR = Index(forceDir.cols(),forceDir.cols()+momentDir.cols()-1);
    if(forceDir.cols()) 
      Wf = forceDir;
    else {
      forceDir.resize(3,0);
      Wf.resize(3,0);
    }
    if(momentDir.cols())
      Wm = momentDir;
    else {
      momentDir.resize(3,0);
      Wm.resize(3,0);
    }
  }

  void Actuator::connect(CoordinateSystem *port0, CoordinateSystem* port1) {
    Link::connect(port0,0);
    Link::connect(port1,1);
  }

  void Actuator::setKOSY(int id) {
    KOSYID = id;
    assert(KOSYID >= 0);
    assert(KOSYID <= 2);
  }

  void Actuator::setUserFunction(DataInterfaceBase *func_) {
    cout<<"!!!HINT!!!You are using the obsolete Method setUserFunction in Actuator "<<name<<endl;
    cout<<"Use Method setSignal instead! This Method will be removed."<<endl;
    func = func_;
  }

  void Actuator::setSignal(DataInterfaceBase *func_) {
    func = func_;
  }

  void Actuator::setForceDirection(const Mat &fd) {
    assert(fd.rows() == 3);

    forceDir = fd;

    for(int i=0; i<fd.cols(); i++)
      forceDir.col(i) = forceDir.col(i)/nrm2(fd.col(i));
  }

  void Actuator::setMomentDirection(const Mat &md) {
    assert(md.rows() == 3);

    momentDir = md;

    for(int i=0; i<md.cols(); i++)
      momentDir.col(i) = momentDir.col(i)/nrm2(md.col(i));
  }

  void Actuator::initDataInterfaceBase(MultiBodySystem *parentmbs) {
    if(DIBRefs.size()==1){
      DataInterfaceBase* in_=parentmbs->getDataInterfaceBase(DIBRefs[0]); 
      setSignal(in_); 
    }
  }

  void Actuator::updateh(double t) {
    la = (*func)(t); // Größe der Kraft/Moment, kommt von Benutzer über UserFunc
    // Kraft-/Momentrichtungen auf Weltsystem umrechnen
    if(KOSYID) {
      Wf = port[KOSYID-1]->getOrientation()*forceDir;
      Wm = port[KOSYID-1]->getOrientation()*momentDir;
    }
    WF[0] = Wf*la(IT);
    WF[1] = -WF[0];
    WM[0] = Wm*la(IR);
    WM[1] = -WM[0];

    h[0] += trans(port[0]->getJacobianOfTranslation())*WF[0] + trans(port[0]->getJacobianOfRotation())*WM[0];
    h[1] += trans(port[1]->getJacobianOfTranslation())*WF[1] + trans(port[1]->getJacobianOfRotation())*WM[1];
  }

}
