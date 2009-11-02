/* Copyright (C) 2004-2006 Mathias Bachmayer
 
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
 *    mfoerg@users.berlios.de
 *
 */

#ifndef _DIB_H_
#define _DIB_H_

#include "fmatvec.h"

namespace MBSim {

  /*! DataInterfaceBase
  */
  class DataInterfaceBase { // Data Interface Base
    protected:
      double SigSize;// angedacht fuer Fähigkeit Signale zu muxen
      //virtual fmatvec::Vec y; // hinfällig wenn Ports etc. instantan über Dimension verfügen
      std::string name;
    public:
      DataInterfaceBase() {
        fprintf(stderr,"The class DataInterfaceBase in deprecated. It must be moved to mbsimControl and the sensors must be derived from MBSim::Link!!!\n");
      }
      virtual ~DataInterfaceBase() {}
      const std::string& getName() {return name;}
      void setName(const std::string& name_) {name=name_;}
      virtual fmatvec::Vec operator()(double t) = 0;
      double  getSigSize(){return SigSize;}
      virtual void update(double t) {}
  };

}

#endif
