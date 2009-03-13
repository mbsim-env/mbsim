#include "springs.h"
#include "frame.h"
#ifdef HAVE_AMVIS
#include "coilspring.h"
using namespace AMVis;
#endif


namespace MBSim {

  Spring::Spring(const string &name) : Link(name) {
  }

  void Spring::init() {
    Link::init();

    g.resize(1);
    gd.resize(1);
    la.resize(1);
  }

  /*void Spring::initPlotFiles() {

    Link::initPlotFiles();

#ifdef HAVE_AMVIS
    if (coilspringAMVis) {
      coilspringAMVis->writeBodyFile();
    }
#endif
  }*/

  void Spring::updateg(double t) {
    Vec WrP0P1=port[1]->getPosition() - port[0]->getPosition();
    forceDir = WrP0P1/nrm2(WrP0P1);
    g(0) = trans(forceDir)*WrP0P1;
  } 

  void Spring::updategd(double t) {
    gd(0) = trans(forceDir)*(port[1]->getVelocity() - port[0]->getVelocity());  
  }    

  void Spring::updateh(double t) {
    la(0) = (cT*(g(0)-l0) + dT*gd(0));
    WF[0] = forceDir*la;
    WF[1] = -WF[0];
    for(unsigned int i=0; i<port.size(); i++)
      h[i] += trans(port[i]->getJacobianOfTranslation())*WF[i];
  }    

  void Spring::connect(Frame *port0, Frame* port1) {
    Link::connect(port0,0);
    Link::connect(port1,1);
  }

  /*void Spring::plot(double t,double dt) {
    Link::plot(t,dt);

#ifdef HAVE_AMVIS
    if (coilspringAMVis) {
      Vec WrOToPoint;
      Vec WrOFromPoint;

      WrOFromPoint = port[0]->getPosition();
      WrOToPoint   = port[1]->getPosition();
      if (coilspringAMVisUserFunctionColor) {
	double color;
	color = ((*coilspringAMVisUserFunctionColor)(t))(0);
	if (color>1) color=1;
	if (color<0) color=0;
	coilspringAMVis->setColor(color);
      } 
      coilspringAMVis->setTime(t); 
      coilspringAMVis->setFromPoint(WrOFromPoint(0), WrOFromPoint(1), WrOFromPoint(2));
      coilspringAMVis->setToPoint(WrOToPoint(0), WrOToPoint(1), WrOToPoint(2));
      coilspringAMVis->appendDataset(0);
    }
#endif
  }*/

}
