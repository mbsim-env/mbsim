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

#ifndef _FUNCTION_PROPERTY_FACTORY_H_
#define _FUNCTION_PROPERTY_FACTORY_H_

#include "property.h"
#include <vector>

namespace MBSimGUI {

  class Element;

  //class FunctionPropertyFactory : public PropertyFactory {
  //  public:
  //    FunctionPropertyFactory(const std::string &omit_, int n_, const std::FQN &xmlName_="", int mode_=1) : omit(omit_), n(n_), xmlName(xmlName_), mode(mode_) { }
  //    Property* createProperty(int i=0);
  //  protected:
  //    std::string omit, xmlName;
  //    int n, mode;
  //};
  //
  //class TranslationPropertyFactory : public PropertyFactory {
  //  public:
  //    TranslationPropertyFactory(const std::string &omit_, int n_, const std::FQN &xmlName_="", int mode_=1) : omit(omit_), n(n_), xmlName(xmlName_), mode(mode_) { }
  //    Property* createProperty(int i=0);
  //  protected:
  //    std::string omit, xmlName;
  //    int n, mode;
  //};
  //
  //class RotationPropertyFactory : public PropertyFactory {
  //  public:
  //    RotationPropertyFactory(const std::string &omit_, int n_, const std::FQN &xmlName_="", int mode_=1) : omit(omit_), n(n_), xmlName(xmlName_), mode(mode_) { }
  //    Property* createProperty(int i=0);
  //  protected:
  //    std::string omit, xmlName;
  //    int n, mode;
  //};

  class FunctionPropertyFactory2 : public PropertyFactory {
    public:
      FunctionPropertyFactory2(Element *parent_) : parent(parent_), name(FunctionPropertyFactory2::getNames()) { }
      PropertyInterface* createProperty(int i=0);
      static std::vector<MBXMLUtils::FQN> getNames();
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class TranslationPropertyFactory2 : public PropertyFactory {
    public:
      TranslationPropertyFactory2(Element *parent_) : parent(parent_), name(TranslationPropertyFactory2::getNames()) { }
      PropertyInterface* createProperty(int i=0);
      static std::vector<MBXMLUtils::FQN> getNames();
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class TranslationPropertyFactory3 : public PropertyFactory {
    public:
      TranslationPropertyFactory3(Element *parent_) : parent(parent_), name(TranslationPropertyFactory3::getNames()) { }
      PropertyInterface* createProperty(int i=0);
      static std::vector<MBXMLUtils::FQN> getNames();
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class RotationPropertyFactory2 : public PropertyFactory {
    public:
      RotationPropertyFactory2(Element *parent_) : parent(parent_), name(RotationPropertyFactory2::getNames()) { }
      PropertyInterface* createProperty(int i=0);
      static std::vector<MBXMLUtils::FQN> getNames();
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class RotationPropertyFactory3 : public PropertyFactory {
    public:
      RotationPropertyFactory3(Element *parent_) : parent(parent_), name(RotationPropertyFactory3::getNames()) { }
      PropertyInterface* createProperty(int i=0);
      static std::vector<MBXMLUtils::FQN> getNames();
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class SymbolicFunctionPropertyFactory2 : public PropertyFactory {
    public:
      SymbolicFunctionPropertyFactory2(Element *parent_, const std::string &ext_, const std::vector<std::string> &var_) : parent(parent_), name(SymbolicFunctionPropertyFactory2::getNames()), ext(ext_), var(var_) { }
      PropertyInterface* createProperty(int i=0);
      static std::vector<MBXMLUtils::FQN> getNames();
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
      std::string ext;
      std::vector<std::string> var;
  };

  class SymbolicFunctionPropertyFactory3 : public PropertyFactory {
    public:
      SymbolicFunctionPropertyFactory3(Element *parent_, const std::string &ext_, const std::vector<std::string> &var_) : parent(parent_), name(SymbolicFunctionPropertyFactory3::getNames()), ext(ext_), var(var_) { }
      PropertyInterface* createProperty(int i=0);
      static std::vector<MBXMLUtils::FQN> getNames();
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
      std::string ext;
      std::vector<std::string> var;
  };

  class TranslationPropertyFactory4 : public PropertyFactory {
    public:
      TranslationPropertyFactory4(Element *parent_);
      PropertyInterface* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class RotationPropertyFactory4 : public PropertyFactory {
    public:
      RotationPropertyFactory4(Element *parent_);
      PropertyInterface* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class TabularFunctionPropertyFactory : public PropertyFactory {
    public:
      TabularFunctionPropertyFactory(Element *parent_);
      PropertyInterface* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class TwoDimensionalTabularFunctionPropertyFactory : public PropertyFactory {
    public:
      TwoDimensionalTabularFunctionPropertyFactory(Element *parent_);
      PropertyInterface* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class FourierFunctionPropertyFactory : public PropertyFactory {
    public:
      FourierFunctionPropertyFactory(Element *parent_);
      PropertyInterface* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class ConstraintPropertyFactory : public PropertyFactory {
    public:
      ConstraintPropertyFactory(Element *parent_);
      PropertyInterface* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class ConnectFramesPropertyFactory : public PropertyFactory {
    public:
      ConnectFramesPropertyFactory(Element *parent_);
      PropertyInterface* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class SpringDamperPropertyFactory: public PropertyFactory {
    public:
      SpringDamperPropertyFactory(Element *parent_);
      PropertyInterface* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class PlanarContourFunctionPropertyFactory : public PropertyFactory {
    public:
      PlanarContourFunctionPropertyFactory(Element *parent_);
      PropertyInterface* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

  class SpatialContourFunctionPropertyFactory : public PropertyFactory {
    public:
      SpatialContourFunctionPropertyFactory(Element *parent_);
      PropertyInterface* createProperty(int i=0);
      MBXMLUtils::FQN getName(int i=0) const { return name[i]; }
      int getSize() const { return name.size(); }
    protected:
      Element *parent;
      std::vector<MBXMLUtils::FQN> name;
  };

}

#endif
