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
