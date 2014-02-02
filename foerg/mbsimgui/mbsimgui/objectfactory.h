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

#include <set>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <xercesc/util/XercesDefs.hpp>

class Element;
class Frame;
class Contour;
class Group;
class Object;
class Link;
class Observer;
class Integrator;
class Parameter;
class Environment;
class OMBVBodyProperty;
class FunctionProperty;
class PhysicalProperty;

namespace XERCES_CPP_NAMESPACE {
  class DOMElement;
}

class ObjectFactoryBase {
  protected:
    ObjectFactoryBase() {}
    virtual ~ObjectFactoryBase() {}
    typedef std::pair<std::string, std::string> P_NSPRE;
    typedef std::map<std::string, std::string> M_NSPRE;
    typedef std::pair<double, P_NSPRE> P_PRINSPRE;
  public:
    typedef std::multimap<double, P_NSPRE> MM_PRINSPRE;
    virtual Frame* createFrame(xercesc::DOMElement *element, Element *parent) { return NULL; }
    virtual Contour* createContour(xercesc::DOMElement *element, Element *parent) { return NULL; }
    virtual Group* createGroup(xercesc::DOMElement *element, Element *parent) { return NULL; }
    virtual Object* createObject(xercesc::DOMElement *element, Element *parent) { return NULL; }
    virtual Link* createLink(xercesc::DOMElement *element, Element *parent) { return NULL; }
    virtual Observer* createObserver(xercesc::DOMElement *element, Element *parent) { return NULL; }
    virtual Integrator* createIntegrator(xercesc::DOMElement *element) { return NULL; }
    virtual Parameter* createParameter(xercesc::DOMElement *element) { return NULL; }
    virtual Environment *getEnvironment(xercesc::DOMElement *element) { return NULL; }
    virtual MM_PRINSPRE& getPriorityNamespacePrefix() {
      static MM_PRINSPRE ret;
      return ret;
    }
};

class ObjectFactory : public ObjectFactoryBase {
  protected:
    ObjectFactory() {}
    virtual ~ObjectFactory() {}
  private:
    static ObjectFactory *instance;
    std::set<ObjectFactoryBase*> factories;
  public:
    static ObjectFactory* getInstance() { return instance?instance:instance=new ObjectFactory; }
    void registerObjectFactory(ObjectFactoryBase *fac) { factories.insert(fac); }
    void unregisterObjectFactory(ObjectFactory *fac) { factories.erase(fac); }

    Frame* createFrame(xercesc::DOMElement *element, Element *parent);
    Contour* createContour(xercesc::DOMElement *element, Element *parent);
    Group* createGroup(xercesc::DOMElement *element, Element *parent);
    Object* createObject(xercesc::DOMElement *element, Element *parent);
    Link* createLink(xercesc::DOMElement *element, Element *parent);
    Observer* createObserver(xercesc::DOMElement *element, Element *parent);
    Integrator* createIntegrator(xercesc::DOMElement *element);
    Parameter* createParameter(xercesc::DOMElement *element);
    Environment *getEnvironment(xercesc::DOMElement *element);
    M_NSPRE getNamespacePrefixMapping();
};

class MBSimObjectFactory : protected ObjectFactoryBase  {
  private:
    static MBSimObjectFactory *instance;
    MBSimObjectFactory() {}
  public:
    // This static function must be called before the ObjectFactory is used to create
    // objects from MBSimObjectFactory
    static void initialize();
  protected:
    Frame* createFrame(xercesc::DOMElement *element, Element *parent);
    Contour* createContour(xercesc::DOMElement *element, Element *parent);
    Group* createGroup(xercesc::DOMElement *element, Element *parent);
    Object* createObject(xercesc::DOMElement *element, Element *parent);
    Link* createLink(xercesc::DOMElement *element, Element *parent);
    Observer* createObserver(xercesc::DOMElement *element, Element *parent);
    Integrator* createIntegrator(xercesc::DOMElement *element);
    Parameter* createParameter(xercesc::DOMElement *element);
    Environment* getEnvironment(xercesc::DOMElement *element);
};

class OMBVBodyFactory {
  protected:
    std::vector<std::string> names;
  public:
    OMBVBodyFactory();
    OMBVBodyProperty* createBody(int i, const std::string &ID); 
    OMBVBodyProperty* createBody(const std::string &name, const std::string &ID); 
    OMBVBodyProperty* createBody(xercesc::DOMElement *element, const std::string &ID); 
    const std::string& getName(int i) const { return names[i]; }
    int size() const { return names.size(); }
};

class FunctionFactory {
  protected:
    std::vector<std::string> names;
  public:
    virtual FunctionProperty* createFunction(int i) = 0; 
    FunctionProperty* createFunction(const std::string &name); 
    FunctionProperty* createFunction(xercesc::DOMElement *element); 
    const std::string& getName(int i) const { return names[i]; }
    int size() const { return names.size(); }
};

class FunctionFactory1 : public FunctionFactory {
  public:
    FunctionFactory1();
    FunctionProperty* createFunction(int i); 
    using FunctionFactory::createFunction;
};

class FunctionFactory2 : public FunctionFactory {
  public:
    FunctionFactory2();
    FunctionProperty* createFunction(int i); 
};

class FunctionFactory3 : public FunctionFactory {
  public:
    FunctionFactory3();
    FunctionProperty* createFunction(int i); 
};

class VariableFactory {
  protected:
    std::vector<std::string> names;
  public:
    VariableFactory();
    PhysicalProperty* createVariable(int i); 
    PhysicalProperty* createVariable(const std::string &name); 
    PhysicalProperty* createVariable(xercesc::DOMElement *element); 
    const std::string& getName(int i) const { return names[i]; }
    int size() const { return names.size(); }
};
