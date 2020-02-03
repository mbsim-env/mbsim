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

#include "function_widget.h"
#include "namespace.h"

namespace MBSimGUI {

  class Element;

  class Function1ArgWidgetFactory : public WidgetFactory {
    public:
      Function1ArgWidgetFactory(Element *element_, const QString &argName_, int argDim_, FunctionWidget::VarType argType_, int retDim_, FunctionWidget::VarType retType_, QWidget *parent_, int index_=3);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getDefaultIndex() const override { return index; }
      int getFallbackIndex() const override { return 18; }
    protected:
      Element *element;
      QString argName;
      int argDim;
      FunctionWidget::VarType argType;
      int retDim;
      FunctionWidget::VarType retType;
      QWidget *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      int index;
  };

  class Function2ArgWidgetFactory : public WidgetFactory {
    public:
      Function2ArgWidgetFactory(Element *element_, const QStringList &argName, const std::vector<int> argDim_, const std::vector<FunctionWidget::VarType> &argType, int retDim_, FunctionWidget::VarType retType_, QWidget *parent_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getFallbackIndex() const override { return 3; }
    protected:
      Element *element;
      QStringList argName;
      std::vector<int> argDim;
      std::vector<FunctionWidget::VarType> argType;
      int retDim;
      FunctionWidget::VarType retType;
      QWidget *parent;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class StateDependentTranslationWidgetFactory : public WidgetFactory {
    public:
      StateDependentTranslationWidgetFactory(Element *element_, QWidget *parent_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getDefaultIndex() const override { return 5; }
      int getFallbackIndex() const override { return 13; }
    protected:
      Element *element;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
  };

  class TimeDependentTranslationWidgetFactory : public WidgetFactory {
    public:
      TimeDependentTranslationWidgetFactory(Element *element_, QWidget *parent_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getFallbackIndex() const override { return 5; }
    protected:
      Element *element;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
  };

  class StateDependentRotationWidgetFactory : public WidgetFactory {
    public:
      StateDependentRotationWidgetFactory(Element *element_, QWidget *parent_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getDefaultIndex() const override { return 1; }
      int getFallbackIndex() const override { return 11; }
    protected:
      Element *element;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
  };

  class TimeDependentRotationWidgetFactory : public WidgetFactory {
    public:
      TimeDependentRotationWidgetFactory(Element *element_, QWidget *parent_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getFallbackIndex() const override { return 2; }
    protected:
      Element *element;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
  };

  class TranslationWidgetFactory : public WidgetFactory {
    public:
      TranslationWidgetFactory(Element *element_, const MBXMLUtils::NamespaceURI &uri, QWidget *parent_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getDefaultIndex() const override { return 1; }
    protected:
      Element *element;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
  };

  class RotationWidgetFactory : public WidgetFactory {
    public:
      RotationWidgetFactory(Element *element_, const MBXMLUtils::NamespaceURI &uri, QWidget *parent_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      Element *element;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
  };

  class CompositeFunctionWidgetFactory : public WidgetFactory {
    public:
      CompositeFunctionWidgetFactory(WidgetFactory *factory_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      WidgetFactory *factory;
  };

  class TabularFunctionWidgetFactory : public WidgetFactory {
    public:
      TabularFunctionWidgetFactory(int retDim_, FunctionWidget::VarType retType_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      int retDim;
      FunctionWidget::VarType retType;
  };

  class TwoDimensionalTabularFunctionWidgetFactory : public WidgetFactory {
    public:
      TwoDimensionalTabularFunctionWidgetFactory();
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class FourierFunctionWidgetFactory : public WidgetFactory {
    public:
      FourierFunctionWidgetFactory();
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

  class ConstraintWidgetFactory : public WidgetFactory {
    public:
      ConstraintWidgetFactory(Element *element_, QWidget *parent_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getDefaultIndex() const override { return 1; }
    protected:
      Element *element;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
  };

  class SpringDamperWidgetFactory : public WidgetFactory {
    public:
      SpringDamperWidgetFactory(Element *element_, bool varSize_, QWidget *parent_);
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getDefaultIndex() const override { return 1; }
      int getFallbackIndex() const override { return 4; }
    protected:
      Element *element;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      bool varSize;
      QWidget *parent;
  };

  class PlanarContourFunctionWidgetFactory : public WidgetFactory {
    public:
      PlanarContourFunctionWidgetFactory(Element *element_, QWidget *parent_, const QString &var_="eta");
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getDefaultIndex() const override { return 5; }
      int getFallbackIndex() const override { return 6; }
    protected:
      Element *element;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
      QString var;
  };

  class SpatialContourFunctionWidgetFactory : public WidgetFactory {
    public:
      SpatialContourFunctionWidgetFactory(Element *element_, QWidget *parent_, const QString &var="zeta");
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getDefaultIndex() const override { return 2; }
      int getFallbackIndex() const override { return 3; }
    protected:
      Element *element;
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
      QWidget *parent;
      QString var;
  };

  class GravityFunctionWidgetFactory : public WidgetFactory {
    public:
      GravityFunctionWidgetFactory();
      QWidget* createWidget(int i=0) override;
      QString getName(int i=0) const override { return name[i]; }
      MBXMLUtils::FQN getXMLName(int i=0) const override { return xmlName[i]; }
      int getSize() const override { return name.size(); }
      int getDefaultIndex() const override { return 1; }
      int getFallbackIndex() const override { return 3; }
    protected:
      std::vector<QString> name;
      std::vector<MBXMLUtils::FQN> xmlName;
  };

}

#endif
