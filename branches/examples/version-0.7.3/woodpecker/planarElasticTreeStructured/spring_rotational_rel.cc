#include "spring_rotational_rel.h"
#include "port.h"
#include "body_rigid_rel.h"

namespace MBSim {

  SpringRotationalRel::SpringRotationalRel(const string &name) : LinkPort(name,false), KMdir(3), WMdir(3), c(0), d(0) {

    for(int i=0; i<2 ; i++) {
      load.push_back(Vec(6));
      WM[i] >> load[i](Index(3,5));
    }

    KMdir(0,0) = 1;
    WMdir(0,0) = 1;
  }

  void SpringRotationalRel::calcSize() {
    LinkPort::calcSize();
    gSize = KMdir.cols();
    laSize = gSize;
    rFactorSize = 0;
    xSize = 0;
  }

  void SpringRotationalRel::init() {
    LinkPort::init();
  }

  void SpringRotationalRel::updateStage1(double t) {

    WMdir = body->getAWK()*KMdir;
    g(0) = body->getq()(0);
    gd(0) = body->getu()(0);
  } 

  void SpringRotationalRel::updateStage2(double t) {

    la(0) = (c*g(0) + d*gd(0));

    WM[0] = WMdir*la(0);
    WM[1] = -WM[0];
  } 

  void SpringRotationalRel::setMomentDirection(const Vec &md) {
    assert(md.rows() == 3);

    KMdir = md/nrm2(md);
  }

  void SpringRotationalRel::connect(Port *port1, Port *port2) {
    body = dynamic_cast<BodyRigidRel*>(port2->getObject());

    assert(body);

    LinkPort::connect(port1,0);
    LinkPort::connect(port2,1);
  }

  double SpringRotationalRel::computePotentialEnergy() {
    return 0.5 * g(0) * c * g(0);
  }

}
