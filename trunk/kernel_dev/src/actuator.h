#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include "link.h"

namespace MBSim {

  class DataInterfaceBase; // Vorwärtsdeklaration bekannt machen der Klasse

  /*! Comment
   *
   * */
  class Actuator : public Link {

    protected:
      Index IT, IR;
      Mat forceDir, momentDir; // Richtungen (noch Kosy-unabhängig)
      Mat Wf, Wm; // Richtungen im Weltsystem 
      Mat WF[2], WM[2]; // 2 Richtungen im Weltsystem (für jeden Körper eine)

      DataInterfaceBase *func;
      int KOSYID; // Welches KOSY

      void updateg(double t) {}
      void updategd(double t) {}

    public: 
      Actuator(const string &name);
      ~Actuator();
      bool isActive() const {return true;}
      bool gActiveChanged() {return false;}

      void calclaSize();
      void init();

      void initPlotFiles() {}
      void plot(double t, double dt=1) {}
      void setKOSY(int);
      void setUserFunction(DataInterfaceBase *func_);
      void setSignal(DataInterfaceBase *func_);
      void connect(CoordinateSystem *port1, CoordinateSystem *port2);
      void setForceDirection(const Mat& fd);
      void setMomentDirection(const Mat& md);

      void initDataInterfaceBase(MultiBodySystem *parentmbs);

      void updateh(double t);
  };

}

#endif
