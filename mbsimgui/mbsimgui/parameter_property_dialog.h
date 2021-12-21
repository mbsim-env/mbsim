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

#ifndef _PARAMETER_PROPERTY_DIALOG_H_
#define _PARAMETER_PROPERTY_DIALOG_H_

#include "property_dialog.h"

namespace MBSimGUI {

  class Parameter;
  class ExtWidget;

  class ParameterPropertyDialog : public PropertyDialog {

    public:
      ParameterPropertyDialog(Parameter *parameter_);
      virtual xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent);
      virtual xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr);
      void toWidget() override;
      void fromWidget() override;
    protected:
      Parameter *parameter;
      ExtWidget *name;
  };

  class StringParameterPropertyDialog : public ParameterPropertyDialog {

    public:
      StringParameterPropertyDialog(Parameter *parameter);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *value;
  };

  class ScalarParameterPropertyDialog : public ParameterPropertyDialog {

    public:
      ScalarParameterPropertyDialog(Parameter *parameter);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *value;
  };

  class VectorParameterPropertyDialog : public ParameterPropertyDialog {

    public:
      VectorParameterPropertyDialog(Parameter *parameter);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *value;
  };

  class MatrixParameterPropertyDialog : public ParameterPropertyDialog {

    public:
      MatrixParameterPropertyDialog(Parameter *parameter);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *value;
  };

  class ImportParameterPropertyDialog : public ParameterPropertyDialog {

    public:
      ImportParameterPropertyDialog(Parameter *parameter);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *parent) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *element, xercesc::DOMNode *ref=nullptr) override;
    protected:
      ExtWidget *value;
  };

}

#endif
