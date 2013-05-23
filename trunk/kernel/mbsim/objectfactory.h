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
     * see also the macro MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY. */
    template<class CreateType>
    static void registerXMLName(std::string name) {
      // add to registeredType with allocate and deallocate function
      std::pair<typename std::map<std::string, std::pair<allocateFkt, deallocateFkt> >::iterator,bool> ret=
        instance().registeredType.insert(std::make_pair(name, std::make_pair(&allocate<CreateType>, &deallocate)));
      // error if already registred
      if(!ret.second && ret.first->second.first!=&allocate<CreateType>)
        throw std::runtime_error("A class named "+name+" is already registered by another class.");
    }

    /** Register the class CreateType which the XML element name name by the object factory (as a singleton class).
     * You should not use this function directly but
     * see also the macro MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORYASSINGLETON. */
    template<class CreateType>
    static void registerXMLNameAsSingleton(std::string name) {
      // add to registeredType with allocate function but no deallocate function
      std::pair<typename std::map<std::string, std::pair<allocateFkt, deallocateFkt> >::iterator,bool> ret=
        instance().registeredType.insert(std::make_pair(name, std::make_pair(&singleton<CreateType>, static_cast<deallocateFkt>(NULL))));
      // error if already registred
      if(!ret.second)
        throw std::runtime_error("A class named "+name+" is already registered by another class.");
    }

    /** Create an object corresponding to the XML element element and return a pointer of type BaseType.
     * This function returns a new object or a singleton object dependent on the registration of the created object. */
    static BaseType* create(MBXMLUtils::TiXmlElement *element) {
      // just call the create<TYPE> function with TYPE = BaseType
      return create<BaseType>(element);
    }

    /** Create an object corresponding to the XML element element and return a pointer of type ContainerType.
     * Throws if the created object is not of type ContainerType.
     * This function returns a new object or a singleton object dependent on the registration of the created object. */
    template<class ContainerType>
    static ContainerType* create(MBXMLUtils::TiXmlElement *element) {
#ifdef HAVE_BOOST_TYPE_TRAITS_HPP
      BOOST_STATIC_ASSERT_MSG((boost::is_convertible<ContainerType*, BaseType*>::value),
        "In MBSim::ObjectFactory<BaseType>::create<ContainerType>(...) ContainerType must be derived from BaseType.");
#endif
      // return NULL if not input is supplied
      if(element==NULL) return NULL;
      // search in registeredType for a entry corresponding to element->ValueStr()
      typename std::map<std::string, std::pair<allocateFkt, deallocateFkt> >::iterator it=
        instance().registeredType.find(element->ValueStr());
      // error if not found (not registred)
      if(it==instance().registeredType.end())
        throw std::runtime_error("No class named "+element->ValueStr()+" found when creating a object of type "+
                                 demangleSymbolName(typeid(ContainerType).name())+".");
      // allocate a new object OR get singleton object using the allocate function pointer
      BaseType *ele=it->second.first();
      // try to cast it up to ContainerType
      ContainerType *ret=dynamic_cast<ContainerType*>(ele);
      // if not possible => error
      if(ret==NULL) {
        // deallocate newly created (wrong) object OR do nothing for singleton objects (is maybe reused later)
        it->second.second(ele);
        throw std::runtime_error("Can not cast class named "+element->ValueStr()+" to "+
                                 demangleSymbolName(typeid(ContainerType).name())+".");
      }
      // retrun created or singleton object
      return ret;
    }

  private:

    // a pointer to a funtion allocation an object
    typedef BaseType* (*allocateFkt)();
    // a pointer to a funtion deallocation an object
    typedef void (*deallocateFkt)(BaseType *obj);

    // private ctor
    ObjectFactory() {}

    // create an singelton instance of the object factory
    static ObjectFactory<BaseType>& instance() {
      static ObjectFactory<BaseType> of;
      return of;
    }

    // a map of registered types
    std::map<std::string, std::pair<allocateFkt, deallocateFkt> > registeredType;

    // a wrapper to allocate an object of type CreateType
    template<class CreateType>
    static BaseType* allocate() {
      return new CreateType;
    }

    // a wrapper to deallocate an object created by allocate
    static void deallocate(BaseType *obj) {
      delete obj;
    }

    // a wrapper to get an singelton object of type CreateType
    template<class CreateType>
    static BaseType* singleton() {
      return CreateType::getInstance();
    }

};

/** Helper function for automatic class registration for ObjectFactory.
 * You should not use this class directly but
 * use the macro MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY. */
template<class BaseType, class CreateType>
class RegisterXMLNameAtObjectFactory {

  public:

    /** ctor registring the new type */
    RegisterXMLNameAtObjectFactory(std::string name) {
      ObjectFactory<BaseType>::template registerXMLName<CreateType>(name);
    };

};

/** Helper function for automatic class registration for ObjectFactory (of singleton objects).
 * You should not use this class directly but
 * use the macro MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORYASSINGLETON. */
template<class BaseType, class CreateType>
class RegisterXMLNameAtObjectFactoryAsSingleton {

  public:

    /** ctor registring the new type */
    RegisterXMLNameAtObjectFactoryAsSingleton(std::string name) {
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
#define MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY(BaseType, ThisType, name) \
  static MBSim::RegisterXMLNameAtObjectFactory<BaseType,ThisType> MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistreationDummyVariable)(name);

/** Use this macro somewhere at the class definition of ThisType to register it by the ObjectFactory (as a singelton).
 * BaseType is the base of ThisType and also the template parameter of ObjectFactory.
 * ThisType must have a public ThisType* getInstance() function and should not have a public dtor. */
#define MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORYASSINGLETON(BaseType, ThisType, name) \
  static MBSim::RegisterXMLNameAtObjectFactoryAsSingleton<BaseType,ThisType> MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistreationDummyVariableAsSingleTon)(name);

#endif
