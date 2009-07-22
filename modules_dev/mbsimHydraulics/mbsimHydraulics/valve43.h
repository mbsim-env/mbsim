/* Copyright (C) 2004-2009 MBSim Development Team
 *
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
 * Contact: schneidm@users.berlios.de
 */

#ifndef  _VALVE43_H_
#define  _VALVE43_H_

#include "mbsim/group.h"

namespace MBSim {

  class FuncTable;
  class DataInterfaceBase;
  class HydLine;

  struct valve43Datastruct {
    std::string Name;
    std::string NameLineP;
    std::string NameLineA;
    std::string NameLineB;
    std::string NameLineT;
    MBSim::FuncTable * relAreaAT;
    MBSim::FuncTable * relAreaPA;
    MBSim::FuncTable * relAreaPB;
    MBSim::FuncTable * relAreaBT;
    MBSim::DataInterfaceBase * signal;
    double lValve;
    double dValve;
    double lLine;
    double dLine;
    double alphaValve;
    double minRelArea;
    valve43Datastruct() {
      Name="Valve43";
      NameLineP="lP";
      NameLineA="lA";
      NameLineB="lB";
      NameLineT="lT";
      relAreaAT=0;
      relAreaPA=0;
      relAreaPB=0;
      relAreaBT=0;
      signal=0;
      lValve=0;
      dValve=0;
      lLine=0;
      dLine=0;
      alphaValve=0;
      minRelArea=0.05;
    };
  };

  class Valve43 : public MBSim::Group {
    public:
      Valve43(valve43Datastruct v, bool setvalued);
  };

}

#endif   /* ----- #ifndef _VALVE43_H_  ----- */

