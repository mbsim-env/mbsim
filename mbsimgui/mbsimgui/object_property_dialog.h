#ifndef _OBJECT_PROPERTY_DIALOG_H_
#define _OBJECT_PROPERTY_DIALOG_H_

#include "element_property_dialog.h"

namespace MBSimGUI {

  class ObjectPropertyDialog : public ElementPropertyDialog {

    public:
      ObjectPropertyDialog(Element *object);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      virtual void resizeGeneralizedPosition() { }
      virtual void resizeGeneralizedVelocity() { }
    protected:
      ExtWidget *q0, *u0, *R;
      void updateWidget() override { resizeGeneralizedPosition(); resizeGeneralizedVelocity(); }
  };

  class BodyPropertyDialog : public ObjectPropertyDialog {

    public:
      BodyPropertyDialog(Element *body);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
  };

  class RigidBodyPropertyDialog : public BodyPropertyDialog {

    public:
      RigidBodyPropertyDialog(Element *body);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      void resizeGeneralizedPosition() override;
      void resizeGeneralizedVelocity() override;
      int getqRelSize() const; 
      int getuRelSize() const { return getqRelSize(); }
    protected:
      ExtWidget *K, *mass, *inertia, *frameForInertiaTensor, *translation, *rotation, *translationDependentRotation, *generalizedVelocityOfRotation, *ombv, *ombvFrameRef, *weightArrow, *jointForceArrow, *jointMomentArrow;
  };

  class GenericFlexibleFfrBodyPropertyDialog : public BodyPropertyDialog {

    public:
      GenericFlexibleFfrBodyPropertyDialog(Element *body_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      void resizeGeneralizedPosition() override;
      void resizeGeneralizedVelocity() override;
      int getqRelSize() const;
      int getuRelSize() const { return getqRelSize(); }
      virtual int getqERelSize() const { return 0; }
    protected:
      ExtWidget *translation, *rotation, *translationDependentRotation, *generalizedVelocityOfRotation;
  };

  class FlexibleFfrBodyPropertyDialog : public GenericFlexibleFfrBodyPropertyDialog {

    public:
      FlexibleFfrBodyPropertyDialog(Element *body_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      int getqERelSize() const override;
    protected:
      ExtWidget *mass, *rdm, *rrdm, *Pdm, *rPdm, *PPdm, *Ke, *De, *beta, *mDamping, *Knl1, *Knl2, *ksigma0, *ksigma1, *K0t, *K0r, *K0om, *nodeNumbers, *r, *A, *Phi, *Psi, *sigmahel, *sigmahen, *sigma0, *K0F, *K0M, *ombv, *visuNodes, *ombvColorRepresentation, *plotNodes;
      void updateWidget() override;
  };

  class ExternalFlexibleFfrBodyPropertyDialog : public GenericFlexibleFfrBodyPropertyDialog {

    public:
      ExternalFlexibleFfrBodyPropertyDialog(Element *body_);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
      int getqERelSize() const override;
    protected:
      ExtWidget *inputDataFile, *ombv, *visuNodes, *plotNodes;
  };

}

#endif
