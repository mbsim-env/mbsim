#include "springs.h"
#include "mbsim/frame.h"
#include "mbsim/dynamic_system.h"
#ifdef HAVE_AMVIS
#include "coilspring.h"
using namespace AMVis;
#endif

using namespace std;
using namespace fmatvec;

namespace MBSim {

  Spring::Spring(const string &name) : Link(name) {
  }

  void Spring::init() {
    Link::init();

    g.resize(1);
    gd.resize(1);
    la.resize(1);
  }

  void Spring::initPlot() {
    updatePlotFeatures(parent);

    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_AMVISCPPINTERFACE
      if(coilspringAMVis) {
        coilspringAMVis->setName(name);
        parent->getAMVisGrp()->addObject(coilspringAMVis);
      }
      Link::initPlot();
#endif
    }
  }

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
    Link::connect(port0);
    Link::connect(port1);
  }

  void Spring::plot(double t,double dt) {
    if(getPlotFeature(plotRecursive)==enabled) {
#ifdef HAVE_AMVISCPPINTERFACE
      if (coilspringAMVis) {
        Vec WrOToPoint;
        Vec WrOFromPoint;

        WrOFromPoint = port[0]->getPosition();
        WrOToPoint   = port[1]->getPosition();
        vector<double> data;
        data.push_back(t); 
        data.push_back(WrOFromPoint(0));
        data.push_back(WrOFromPoint(1));
        data.push_back(WrOFromPoint(2));
        data.push_back(WrOToPoint(0));
        data.push_back(WrOToPoint(1));
        data.push_back(WrOToPoint(2));
        data.push_back(0);
        coilspringAMVis->append(data);
      }
#endif
      Link::plot(t,dt);
    }
  }

}

