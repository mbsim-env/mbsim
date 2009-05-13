#ifndef _LINEARSPRINGDAMPER_H_
#define _LINEARSPRINGDAMPER_H_

#include "mbsim/link_mechanics.h"
#ifdef HAVE_OPENMBVCPPINTERFACE
#include <openmbvcppinterface/coilspring.h>
#endif

namespace MBSim {

  class LinearSpringDamper : public LinkMechanics {
    protected:
      double l0, cT, dT;
      fmatvec::Vec forceDir;
#ifdef HAVE_OPENMBVCPPINTERFACE
      OpenMBV::CoilSpring *coilspringOpenMBV;
#endif
    public:
      LinearSpringDamper(const std::string &name);
      void init();
      void initPlot();
      void plot(double t,double dt=1); 
      void updateg(double t);
      void updategd(double t); 
      void updateh(double t); 
      void setUnloadedLength(double l0_) {l0=l0_;}
      void setStiffness(double c) {cT = c;}
      void setDamping(double d) {dT = d;}
      bool isActive() const {return true;}
      bool gActiveChanged() {return false;}
      virtual void connect(FrameInterface *frame1, FrameInterface* frame2);
#ifdef HAVE_OPENMBVCPPINTERFACE
      void setOpenMBVSpring(OpenMBV::CoilSpring *spring_) {coilspringOpenMBV=spring_;}
#endif
      virtual void initializeUsingXML(TiXmlElement *element);
  };

}

#endif
