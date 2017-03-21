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

#ifndef _OBJECTFACTORY__H_
#define _OBJECTFACTORY__H_

#include <set>
#include <iostream>
#include <map>
#include <xercesc/dom/DOMElement.hpp>

namespace MBSimGUI {

  class Element;
  class Frame;
  class Contour;
  class Group;
  class Object;
  class Link;
  class Constraint;
  class Observer;
  class Parameter;
  class Environment;
  class Solver;
  class TreeItem;

  class ObjectFactoryBase {
    protected:
      ObjectFactoryBase() {}
      virtual ~ObjectFactoryBase() {}
      typedef std::pair<std::string, std::string> P_NSPRE;
      typedef std::map<std::string, std::string> M_NSPRE;
      typedef std::pair<double, P_NSPRE> P_PRINSPRE;
    public:
      typedef std::multimap<double, P_NSPRE> MM_PRINSPRE;
      virtual Frame* createFrame(xercesc::DOMElement *element) { return NULL; }
      virtual Contour* createContour(xercesc::DOMElement *element) { return NULL; }
      virtual Group* createGroup(xercesc::DOMElement *element) { return NULL; }
      virtual Object* createObject(xercesc::DOMElement *element) { return NULL; }
      virtual Link* createLink(xercesc::DOMElement *element) { return NULL; }
      virtual Constraint* createConstraint(xercesc::DOMElement *element) { return NULL; }
      virtual Observer* createObserver(xercesc::DOMElement *element) { return NULL; }
      virtual Solver* createSolver(xercesc::DOMElement *element) { return NULL; }
      virtual Parameter* createParameter(xercesc::DOMElement *element) { return NULL; }
      virtual Environment *getEnvironment(xercesc::DOMElement *element) { return NULL; }
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

      Frame* createFrame(xercesc::DOMElement *element);
      Contour* createContour(xercesc::DOMElement *element);
      Group* createGroup(xercesc::DOMElement *element);
      Object* createObject(xercesc::DOMElement *element);
      Link* createLink(xercesc::DOMElement *element);
      Constraint* createConstraint(xercesc::DOMElement *element);
      Observer* createObserver(xercesc::DOMElement *element);
      Solver* createSolver(xercesc::DOMElement *element);
      Parameter* createParameter(xercesc::DOMElement *element);
      Environment *getEnvironment(xercesc::DOMElement *element);
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
      Frame* createFrame(xercesc::DOMElement *element);
      Contour* createContour(xercesc::DOMElement *element);
      Group* createGroup(xercesc::DOMElement *element);
      Object* createObject(xercesc::DOMElement *element);
      Link* createLink(xercesc::DOMElement *element);
      Constraint* createConstraint(xercesc::DOMElement *element);
      Observer* createObserver(xercesc::DOMElement *element);
      Solver* createSolver(xercesc::DOMElement *element);
      Parameter* createParameter(xercesc::DOMElement *element);
      Environment *getEnvironment(xercesc::DOMElement *element);
  };

}

#endif
