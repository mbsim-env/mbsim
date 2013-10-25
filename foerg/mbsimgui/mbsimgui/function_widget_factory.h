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

class Element;

class FunctionWidgetFactory2 : public WidgetFactory {
  public:
    FunctionWidgetFactory2() : name(FunctionWidgetFactory2::getNames()) { }
    QWidget* createWidget(int i=0);
    static std::vector<QString> getNames();
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

class TranslationWidgetFactory2 : public WidgetFactory {
  public:
    TranslationWidgetFactory2() : name(TranslationWidgetFactory2::getNames()) { }
    QWidget* createWidget(int i=0);
    static std::vector<QString> getNames();
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

class TranslationWidgetFactory3 : public WidgetFactory {
  public:
    TranslationWidgetFactory3() : name(TranslationWidgetFactory3::getNames()) { }
    QWidget* createWidget(int i=0);
    static std::vector<QString> getNames();
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

class RotationWidgetFactory2 : public WidgetFactory {
  public:
    RotationWidgetFactory2() : name(RotationWidgetFactory2::getNames())  { }
    QWidget* createWidget(int i=0);
    static std::vector<QString> getNames();
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

class RotationWidgetFactory3 : public WidgetFactory {
  public:
    RotationWidgetFactory3() : name(RotationWidgetFactory3::getNames())  { }
    QWidget* createWidget(int i=0);
    static std::vector<QString> getNames();
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

class SymbolicFunctionWidgetFactory2 : public WidgetFactory {
  public:
    SymbolicFunctionWidgetFactory2(const QStringList &var_) : name(SymbolicFunctionWidgetFactory2::getNames()), var(var_) { }
    QWidget* createWidget(int i=0);
    static std::vector<QString> getNames();
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
    QStringList var;
};

class TranslationWidgetFactory4 : public WidgetFactory {
  public:
    TranslationWidgetFactory4();
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

class RotationWidgetFactory4 : public WidgetFactory {
  public:
    RotationWidgetFactory4();
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

class TabularFunctionWidgetFactory : public WidgetFactory {
  public:
    TabularFunctionWidgetFactory();
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

class ConstraintWidgetFactory : public WidgetFactory {
  public:
    ConstraintWidgetFactory();
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

class ConnectFramesWidgetFactory : public WidgetFactory {
  public:
    ConnectFramesWidgetFactory(Element *element);
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
    Element *element;
};

class SpringDamperWidgetFactory : public WidgetFactory {
  public:
    SpringDamperWidgetFactory();
    QWidget* createWidget(int i=0);
    QString getName(int i=0) const { return name[i]; }
    int getSize() const { return name.size(); }
  protected:
    std::vector<QString> name;
};

#endif
