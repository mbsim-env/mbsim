#ifndef _SPRING_ROTATIONAL_REL_H_
#define _SPRING_ROTATIONAL_REL_H_

#include "link.h"         

namespace MBSim {

  class BodyRigidRel;

  /*! massless linear rotational spring between ports
  */
  class SpringRotationalRel : public LinkPort {

    private:
      const BodyRigidRel *body;
      Vec KMdir, WMdir, WM[2];
      double c, d;

      void updateStage1(double t);
      void updateStage2(double t);

    public:

      SpringRotationalRel(const string &name);

      void init();
      void calcSize();

      void connect(Port *port1, Port* port2);
      //void connect(Port *port);

      void setMomentDirection(const Vec& md);
      void setStiffness(double c_) {c = c_;}
      void setDamping(double d_) {d = d_;}

      double computePotentialEnergy();
  };          

}

#endif
