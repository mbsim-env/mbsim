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
#include <iostream>
#include <map>
class Element;
class Frame;
class Contour;
class Group;
class Object;
class ExtraDynamic;
class Link;
class Observer;
class Integrator;
class Parameter;
class Environment;
class TreeItem;
namespace MBXMLUtils {
  class TiXmlElement;
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
    virtual Frame* createFrame(MBXMLUtils::TiXmlElement *element, Element *parent) { return NULL; }
    virtual Contour* createContour(MBXMLUtils::TiXmlElement *element, Element *parent) { return NULL; }
    virtual Group* createGroup(MBXMLUtils::TiXmlElement *element, Element *parent) { return NULL; }
    virtual Object* createObject(MBXMLUtils::TiXmlElement *element, Element *parent) { return NULL; }
    virtual ExtraDynamic* createExtraDynamic(MBXMLUtils::TiXmlElement *element, Element *parent) { return NULL; }
    virtual Link* createLink(MBXMLUtils::TiXmlElement *element, Element *parent) { return NULL; }
    virtual Observer* createObserver(MBXMLUtils::TiXmlElement *element, Element *parent) { return NULL; }
    virtual Integrator* createIntegrator(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Parameter* createParameter(MBXMLUtils::TiXmlElement *element) { return NULL; }
    virtual Environment *getEnvironment(MBXMLUtils::TiXmlElement *element) { return NULL; }
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

    Frame* createFrame(MBXMLUtils::TiXmlElement *element, Element *parent);
    Contour* createContour(MBXMLUtils::TiXmlElement *element, Element *parent);
    Group* createGroup(MBXMLUtils::TiXmlElement *element, Element *parent);
    Object* createObject(MBXMLUtils::TiXmlElement *element, Element *parent);
    ExtraDynamic* createExtraDynamic(MBXMLUtils::TiXmlElement *element, Element *parent);
    Link* createLink(MBXMLUtils::TiXmlElement *element, Element *parent);
    Observer* createObserver(MBXMLUtils::TiXmlElement *element, Element *parent);
    Integrator* createIntegrator(MBXMLUtils::TiXmlElement *element);
    Parameter* createParameter(MBXMLUtils::TiXmlElement *element);
    Environment *getEnvironment(MBXMLUtils::TiXmlElement *element);
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
    Frame* createFrame(MBXMLUtils::TiXmlElement *element, Element *parent);
    Contour* createContour(MBXMLUtils::TiXmlElement *element, Element *parent);
    Group* createGroup(MBXMLUtils::TiXmlElement *element, Element *parent);
    Object* createObject(MBXMLUtils::TiXmlElement *element, Element *parent);
    ExtraDynamic* createExtraDynamic(MBXMLUtils::TiXmlElement *element, Element *parent);
    Link* createLink(MBXMLUtils::TiXmlElement *element, Element *parent);
    Observer* createObserver(MBXMLUtils::TiXmlElement *element, Element *parent);
    Integrator* createIntegrator(MBXMLUtils::TiXmlElement *element);
    Parameter* createParameter(MBXMLUtils::TiXmlElement *element);
    Environment *getEnvironment(MBXMLUtils::TiXmlElement *element);
};

