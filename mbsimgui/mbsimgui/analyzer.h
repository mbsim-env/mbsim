/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
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
    public:
      Analyzer() = default;
  };

  class LinearSystemAnalyzer : public Analyzer {
    public:
      MBXMLUtils::FQN getXMLType() const override { return MBSIMCONTROL%"LinearSystemAnalyzer"; }
      QString getType() const override { return "Linear system analyzer"; }
      PropertyDialog* createPropertyDialog() override { return new LinearSystemAnalyzerPropertyDialog(this); }
  };
}

#endif
