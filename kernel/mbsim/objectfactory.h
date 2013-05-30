/* Copyright (C) 2004-2010 MBSim Development Team
 *
 * This library is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU Lesser General Public 
 * License as published by the Free Software Foundation; either 
 * version 2.1 of the License, or (at your option) any later version. 
 *  
 * This library is distributed in the hope that it will be useful, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * Lesser General Public License for more details. 
 *  
 * You should have received a copy of the GNU Lesser General Public 
 * License along with this library; if not, write to the Free Software 
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
 *
 * Contact: friedrich.at.gc@googlemail.com
 */

#ifndef _MBSIM_OBJECTFACTORY_H_
#define _MBSIM_OBJECTFACTORY_H_

#include <map>
#include <stdexcept>
#include <typeinfo>
#include "mbxmlutilstinyxml/tinyxml.h"
#ifdef HAVE_BOOST_TYPE_TRAITS_HPP
# include <boost/static_assert.hpp>
# include <boost/type_traits.hpp>
#endif
#include "mbsim/utils/utils.h"

namespace MBSim {

/** A object factory.
 * A object facroty which creates any object derived from BaseType.
 */
template<class BaseType>
class ObjectFactory {

  public:

    /** Register the class CreateType which the XML element name name by the object factory.
     * You should not use this function directly but
     * see also the macro MBSIM_OBJECTFACTORY_REGISTERXMLNAME.  */
    template<class CreateType>
    static void registerXMLName(const std::string &name) {
      registerXMLName(name, &allocate<CreateType>, &deallocate);
    }

    /** Register the class CreateType which the XML element name name by the object factory.
     * You should not use this function directly but
     * see also the macro MBSIM_OBJECTFACTORY_REGISTERXMLNAMEASSINGLETON. */
    template<class CreateType>
    static void registerXMLNameAsSingleton(const std::string &name) {
      registerXMLName(name, &singleton<CreateType>, NULL);
    }

    /** Create an object corresponding to the XML element element and return a pointer of type BaseType.
     * This function returns a new object or a singleton object dependent on the registration of the created object. */
    static BaseType* create(const MBXMLUtils::TiXmlElement *element) {
      // just call the create<TYPE> function with TYPE = BaseType
      return create<BaseType>(element);
    }

    /** Create an object corresponding to the XML element element and return a pointer of type ContainerType.
     * Throws if the created object is not of type ContainerType.
     * This function returns a new object or a singleton object dependent on the registration of the created object. */
    template<class ContainerType>
    static ContainerType* create(const MBXMLUtils::TiXmlElement *element) {
#ifdef HAVE_BOOST_TYPE_TRAITS_HPP
      // just check if ContainerType is derived from BaseType if not throw a compile error if boost is avaliable
      // if boost is not avaliable a runtime error will occure later. (so it does not care if boost is not available)
      BOOST_STATIC_ASSERT_MSG((boost::is_convertible<ContainerType*, BaseType*>::value),
        "In MBSim::ObjectFactory<BaseType>::create<ContainerType>(...) ContainerType must be derived from BaseType.");
#endif
      // return NULL if no input is supplied
      if(element==NULL) return NULL;
      // loop over all all registred types corresponding to element->ValueStr()
      std::pair<MapIt, MapIt> range=instance().registeredType.equal_range(element->ValueStr());
      for(MapIt it=range.first; it!=range.second; it++) {
        // allocate a new object OR get singleton object using the allocate function pointer
        BaseType *ele=it->second.first();
        // try to cast ele up to ContainerType
        ContainerType *ret=dynamic_cast<ContainerType*>(ele);
        // if possible, return it
        if(ret)
          return ret;
        // if not possible, deallocate newly created (wrong) object OR do nothing for
        // singleton objects (is maybe reused later) and continue searching
        else
          it->second.second(ele);
      }
      // no matching element found: throw error
      throw std::runtime_error("No class named "+element->ValueStr()+" found which is of type "+
                               demangleSymbolName(typeid(ContainerType).name())+".");
    }

  private:

    // a pointer to a function allocating an object
    typedef BaseType* (*allocateFkt)();
    // a pointer to a function deallocating an object
    typedef void (*deallocateFkt)(BaseType *obj);

    // convinence typedefs
    typedef std::multimap<std::string, std::pair<allocateFkt, deallocateFkt> > Map;
    typedef typename Map::iterator MapIt;

    // private ctor
    ObjectFactory() {}

    static void registerXMLName(const std::string &name, allocateFkt alloc, deallocateFkt dealloc) {
      // check if name was already registred with the same &allocate<CreateType>: if yes return and do not add it twice
      std::pair<MapIt, MapIt> range=instance().registeredType.equal_range(name);
      for(MapIt it=range.first; it!=range.second; it++)
        if(it->second.first==alloc)
          return;
      // name is not registred with &allocate<CreateType>: register it
      instance().registeredType.insert(std::make_pair(name, std::make_pair(alloc, dealloc)));
    }

    // create an singleton instance of the object factory.
    // only declaration here and defition and explicit instantation for all BaseType in objectfactory.cc (required for Windows)
    static ObjectFactory<BaseType>& instance();

    // a multimap of all registered types
    Map registeredType;

    // a wrapper to allocate an object of type CreateType
    template<class CreateType>
    static BaseType* allocate() {
      return new CreateType;
    }

    // a wrapper to deallocate an object created by allocate
    static void deallocate(BaseType *obj) {
      delete obj;
    }

    // a wrapper to get an singleton object of type CreateType (Must have the same signature as allocate()
    template<class CreateType>
    static BaseType* singleton() {
      return CreateType::getInstance();
    }

};

/** Helper function for automatic class registration for ObjectFactory.
 * You should not use this class directly but
 * use the macro MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY. */
template<class BaseType, class CreateType>
class ObjectFactoryRegisterXMLNameHelper {

  public:

    /** ctor registring the new type */
    ObjectFactoryRegisterXMLNameHelper(const std::string &name) {
      ObjectFactory<BaseType>::template registerXMLName<CreateType>(name);
    };

};

/** Helper function for automatic class registration for ObjectFactory.
 * You should not use this class directly but
 * use the macro MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORYASSINGLETON. */
template<class BaseType, class CreateType>
class ObjectFactoryRegisterXMLNameHelperAsSingleton {

  public:

    /** ctor registring the new type */
    ObjectFactoryRegisterXMLNameHelperAsSingleton(const std::string &name) {
      ObjectFactory<BaseType>::template registerXMLNameAsSingleton<CreateType>(name);
    };

};

}

#define MBSIM_OBJECTFACTORY_CONCAT1(X, Y) X##Y
#define MBSIM_OBJECTFACTORY_CONCAT(X, Y) MBSIM_OBJECTFACTORY_CONCAT1(X, Y)
#define MBSIM_OBJECTFACTORY_APPENDLINE(X) MBSIM_OBJECTFACTORY_CONCAT(X, __LINE__)

/** Use this macro somewhere at the class definition of ThisType to register it by the ObjectFactory.
 * BaseType is the base of ThisType and also the template parameter of ObjectFactory.
 * ThisType must have a public default ctor and a public dtor. */
#define MBSIM_OBJECTFACTORY_REGISTERXMLNAME(BaseType, ThisType, name) \
  static MBSim::ObjectFactoryRegisterXMLNameHelper<BaseType,ThisType> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariable)(name);

/** Use this macro somewhere at the class definition of ThisType to register it by the ObjectFactory (as a singleton).
 * BaseType is the base of ThisType and also the template parameter of ObjectFactory.
 * ThisType must have a public ThisType* getInstance() function and should not have a public dtor. */
#define MBSIM_OBJECTFACTORY_REGISTERXMLNAMEASSINGLETON(BaseType, ThisType, name) \
  static MBSim::ObjectFactoryRegisterXMLNameHelperAsSingleton<BaseType,ThisType> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariableAsSingleTon)(name);

#endif
