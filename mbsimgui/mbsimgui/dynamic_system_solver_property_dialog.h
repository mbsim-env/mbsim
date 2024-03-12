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
