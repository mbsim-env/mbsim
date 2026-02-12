/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2016 Martin Förg

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

#ifndef _SPECIAL_WIDGETS_H_
#define _SPECIAL_WIDGETS_H_

#include "widget.h"
#include "custom_widgets.h"

class QCheckBox;

namespace MBSimGUI {

  class ExtWidget;

  class DofWidget : public Widget {
    public:
      DofWidget(QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      QString getDof();
      void setDof(const QString &dof);
    private:
      std::vector<QCheckBox*> dof;
  };

  class BoundaryConditionWidget : public Widget {
    public:
      BoundaryConditionWidget(QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      std::vector<int> getNodes() const;
      std::vector<int> getDof() const;
    private:
      ExtWidget *nodes, *dof;
  };

  class BoundaryConditionWidgetFactory : public WidgetFactory {
    public:
      BoundaryConditionWidgetFactory(QWidget *parent_) : parent(parent_) { }
      Widget* createWidget(int i=0) override;
      MBXMLUtils::FQN getXMLName(int i=0) const override { return MBSIMFLEX%"boundaryNodeNumbers"; }
    protected:
      QWidget *parent;
  };

  class FiniteElementsDataWidget : public Widget {
    public:
      FiniteElementsDataWidget(QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      QString getType() const;
      QString getElementsFile() const;
    private:
      ExtWidget *type, *elements;
  };

  class FiniteElementsDataWidgetFactory : public WidgetFactory {
    public:
      FiniteElementsDataWidgetFactory(QWidget *parent_) : parent(parent_) { }
      Widget* createWidget(int i=0) override;
      MBXMLUtils::FQN getXMLName(int i=0) const override { return MBSIMFLEX%"elementType"; }
    protected:
      QWidget *parent;
  };

  class CMSDataWidget : public Widget {
    public:
      CMSDataWidget(QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      std::vector<int> getNodes() const;
      std::vector<double> getWeights() const;
      bool getReduceToSingleNode() const;
      std::vector<int> getDof() const;
      int getSingleNodeNumber() const;
      std::vector<double> getPositionOfReferenceNode() const;
    private:
      ExtWidget *nodes, *weights, *rtsn, *dof, *snn, *prf;
  };

  class CMSDataWidgetFactory : public WidgetFactory {
    public:
      CMSDataWidgetFactory(QWidget *parent_) : parent(parent_) { }
      Widget* createWidget(int i=0) override;
      MBXMLUtils::FQN getXMLName(int i=0) const override { return MBSIMFLEX%"interfaceNodeNumbers"; }
    protected:
      QWidget *parent;
  };

  class DistributedLoadsWidget : public Widget {
    public:
      DistributedLoadsWidget(QWidget *parent);
      xercesc::DOMElement* initializeUsingXML(xercesc::DOMElement *element) override;
      xercesc::DOMElement* writeXMLFile(xercesc::DOMNode *parent, xercesc::DOMNode *ref=nullptr) override;
      std::vector<int> getElements() const;
      int getFaceNumber() const;
      int getSingleNodeNumber() const;
    private:
      ExtWidget *elements, *fn, *snn;
  };

  class DistributedLoadsWidgetFactory : public WidgetFactory {
    public:
      DistributedLoadsWidgetFactory(QWidget *parent_) : parent(parent_) { }
      Widget* createWidget(int i=0) override;
      MBXMLUtils::FQN getXMLName(int i=0) const override { return MBSIMFLEX%"elementNumbers"; }
    protected:
      QWidget *parent;
  };
}

#endif
