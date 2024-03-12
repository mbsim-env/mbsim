/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2024 Martin Förg

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

#ifndef _DYNAMIC_SYSTEM_SOLVER_PROPERTY_DIALOG_H_
#define _DYNAMIC_SYSTEM_SOLVER_PROPERTY_DIALOG_H_

#include "group_property_dialog.h"

namespace MBSimGUI {

  class ExtWidget;
  class CommentWidget;
  class Element;
  class PlotAttributeStore;

  class DynamicSystemSolverPropertyDialog : public GroupPropertyDialog {
    protected:
      ExtWidget *environments, *constraintSolver, *impactSolver, *maxIter, *highIter, *numericalJacobian, *stopIfNoConvergence, *projection, *gTol, *gdTol, *gddTol, *laTol, *LaTol, *gCorr, *gdCorr, *inverseKinetics, *initialProjection, *determineEquilibriumState, *useConstraintSolverForSmoothMotion, *useConstraintSolverForPlot, *compressionLevel, *chunkSize, *cacheSize;

    public:
      DynamicSystemSolverPropertyDialog(Element *solver);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
  };

}

#endif
