/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _ANALYZER__H_
#define _ANALYZER__H_

#include "solver.h"
#include "solver_property_dialog.h"

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
  class DOMNode;
}

namespace MBSimGUI {

  class Analyzer : public Solver {
    MBSIMGUI_OBJECTFACTORY_CLASS(Analyzer, Solver, MBSIMCONTROL%"Analyzer", "Analyzer");
  };

  class LinearSystemAnalyzer : public Analyzer {
    MBSIMGUI_OBJECTFACTORY_CLASS(LinearSystemAnalyzer, Analyzer, MBSIMCONTROL%"LinearSystemAnalyzer", "Linear system analyzer");
    public:
      PropertyDialog* createPropertyDialog() override { return new LinearSystemAnalyzerPropertyDialog(this); }
  };
}

#endif
