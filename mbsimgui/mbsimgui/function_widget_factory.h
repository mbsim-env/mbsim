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

#ifndef _FUNCTION_WIDGET_FACTORY_H_
#define _FUNCTION_WIDGET_FACTORY_H_

#include "widget.h"
#include "namespace.h"

namespace MBSimGUI {

  class Element;

  class FunctionWidgetFactory2 : public WidgetFactory {
    public:
      FunctionWidgetFactory2(Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class TranslationWidgetFactory2 : public WidgetFactory {
    public:
      TranslationWidgetFactory2(Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class TranslationWidgetFactory3 : public WidgetFactory {
    public:
      TranslationWidgetFactory3(Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class RotationWidgetFactory2 : public WidgetFactory {
    public:
      RotationWidgetFactory2(Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class RotationWidgetFactory3 : public WidgetFactory {
    public:
      RotationWidgetFactory3(Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class SymbolicFunctionWidgetFactory1 : public WidgetFactory {
    public:
      SymbolicFunctionWidgetFactory1(const QStringList &var_, Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QStringList var;
  };

  class SymbolicFunctionWidgetFactory2 : public WidgetFactory {
    public:
      SymbolicFunctionWidgetFactory2(const QStringList &var_, Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QStringList var;
  };

  class SymbolicFunctionWidgetFactory3 : public WidgetFactory {
    public:
      SymbolicFunctionWidgetFactory3(const QStringList &var_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QStringList var;
  };

  class TranslationWidgetFactory4 : public WidgetFactory {
    public:
      TranslationWidgetFactory4(Element *parent_, const MBXMLUtils::NamespaceURI &uri=MBSIM);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class RotationWidgetFactory4 : public WidgetFactory {
    public:
      RotationWidgetFactory4(Element *parent_, const MBXMLUtils::NamespaceURI &uri=MBSIM);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class TabularFunctionWidgetFactory : public WidgetFactory {
    public:
      TabularFunctionWidgetFactory();
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class TwoDimensionalTabularFunctionWidgetFactory : public WidgetFactory {
    public:
      TwoDimensionalTabularFunctionWidgetFactory();
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class FourierFunctionWidgetFactory : public WidgetFactory {
    public:
      FourierFunctionWidgetFactory();
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class ConstraintWidgetFactory : public WidgetFactory {
    public:
      ConstraintWidgetFactory(Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class ConnectRigidBodiesWidgetFactory : public WidgetFactory {
    public:
      ConnectRigidBodiesWidgetFactory(Element *parent);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
  };

  class SpringDamperWidgetFactory : public WidgetFactory {
    public:
      SpringDamperWidgetFactory(Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class PlanarContourFunctionWidgetFactory : public WidgetFactory {
    public:
      PlanarContourFunctionWidgetFactory(Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class SpatialContourFunctionWidgetFactory : public WidgetFactory {
    public:
      SpatialContourFunctionWidgetFactory(Element *parent_);
      QWidget* createWidget(int i=0);
      QString getName(int i=0) const { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const { return xmlName[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

}

#endif
