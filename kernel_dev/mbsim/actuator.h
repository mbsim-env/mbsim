/* Copyright (C) 2004-2008  Martin Förg
 
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA

 *
 * Contact:
 *   mfoerg@users.berlios.de
 *
 */

#ifndef _ACTUATOR_H_
#define _ACTUATOR_H_

#include <mbsim/link.h>

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

      void setKOSY(int);
      void setUserFunction(DataInterfaceBase *func_);
      void setSignal(DataInterfaceBase *func_);
      void connect(Frame *port1, Frame *port2);
      void setForceDirection(const Mat& fd);
      void setMomentDirection(const Mat& md);

      void initDataInterfaceBase(MultiBodySystem *parentmbs);

      void updateh(double t);
  };

}

#endif
