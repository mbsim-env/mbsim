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

#include <boost/tuple/tuple_io.hpp>
#include <vector>
#include <stdexcept>
#include <typeinfo>
#ifdef HAVE_BOOST_TYPE_TRAITS_HPP
# include <boost/static_assert.hpp>
# include <boost/type_traits.hpp>
#endif
#include "mbsim/utils/utils.h"
#include <mbsim/mbsim_event.h>
#include <mbxmlutilshelper/utils.h>

#define COMMA ,

namespace MBSim {

class ObjectFactoryNoObject : public MBXMLUtils::DOMEvalException {
  public:
    ObjectFactoryNoObject(const xercesc::DOMElement *e_, const std::type_info &cppType_) throw() :
      MBXMLUtils::DOMEvalException("Unable to create an object with name <"+MBXMLUtils::X()%e_->getTagName()+"> which is of the requested type "+
        MBXMLUtils::demangleSymbolName(cppType_.name())+".", e_) {
    }
    virtual ~ObjectFactoryNoObject() throw() {}
};

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
    static void registerXMLName(const MBXMLUtils::FQN &name) {
      registerXMLName(name, &allocate<CreateType>, &deallocate);
    }

    /** Register the class CreateType which the XML element name name by the object factory.
     * You should not use this function directly but
     * see also the macro MBSIM_OBJECTFACTORY_REGISTERXMLNAMEASSINGLETON. */
    template<class CreateType>
    static void registerXMLNameAsSingleton(const MBXMLUtils::FQN &name) {
      registerXMLName(name, &getSingleton<CreateType>, &deallocateSingleton);
    }

    /** Deregister the class CreateType.
     * You should not use this function directly but
     * see also the macro MBSIM_OBJECTFACTORY_REGISTERXMLNAME.  */
    template<class CreateType>
    static void deregisterXMLName(const MBXMLUtils::FQN &name) {
      deregisterXMLName(name, &allocate<CreateType>);
    }

    /** Deregister the class CreateType.
     * You should not use this function directly but
     * see also the macro MBSIM_OBJECTFACTORY_REGISTERXMLNAMEASSINGLETON. */
    template<class CreateType>
    static void deregisterXMLNameAsSingleton(const MBXMLUtils::FQN &name) {
      deregisterXMLName(name, &getSingleton<CreateType>);
    }

    /** Create and initialize an object corresponding to the XML element element and return a pointer of type ContainerType.
     * Throws if the created object is not of type ContainerType or no object can be create without errors.
     * This function returns a new object or a singleton object dependent on the registration of the created object. */
    template<class ContainerType>
    static ContainerType* createAndInit(const xercesc::DOMElement *element) {
      size_t useMatchNr=1;
      std::vector<std::string> error;
      do {
        std::pair<ContainerType*, DeallocateFkt> p=std::pair<ContainerType*, DeallocateFkt>(NULL, &deallocateSingleton);
        try {
          p=create<ContainerType>(element, useMatchNr++);
          p.first->initializeUsingXML(const_cast<xercesc::DOMElement*>(element));
          return p.first;
        }
        catch(const ObjectFactoryNoObject &ex) {
          // delete the created object using the provided deallocater (calls delete or nothing for singletons)
          p.second(p.first);
          // save exception, only if this is the first exception during this run (this error has a very low priority)
          if(error.size()==0)
            error.push_back(ex.what());
          // break if useMatchNr number is larger then the factory size: nothing found which can be initialized.
          if(useMatchNr>instance().registeredType.size()) {
            std::string message;
            // collect the message of all previous errors and throw them all!
            for(std::vector<std::string>::iterator it=error.begin(); it!=error.end(); it++) {
              std::vector<std::string>::iterator it2=it; it2++;
              message+=*it+(it2!=error.end()?"\n":"");
            }
            throw MBSimError(message);
          }
        }
        catch(const MBSimError &ex) {
          // delete the created object using the provided deallocater (calls delete or nothing for singletons)
          p.second(p.first);
          // save exception: it is rethrow (including others) alter if everything fails
          error.push_back(ex.what());
        }
      }
      while(true); // loop until we are able to create and init a object without errors
    }

  private:

    // a pointer to a function allocating an object
    typedef BaseType* (*AllocateFkt)();
    // a pointer to a function deallocating an object
    typedef void (*DeallocateFkt)(BaseType *obj);

    /** Create an object corresponding to the XML element element and return a pointer of type ContainerType.
     * Throws if the created object is not of type ContainerType.
     * This function returns a new object or a singleton object dependent on the registration of the created object. */
    template<class ContainerType>
    static std::pair<ContainerType*, DeallocateFkt> create(const xercesc::DOMElement *element, size_t useMatchNr=1) {
#ifdef HAVE_BOOST_TYPE_TRAITS_HPP
      // just check if ContainerType is derived from BaseType if not throw a compile error if boost is avaliable
      // if boost is not avaliable a runtime error will occure later. (so it does not care if boost is not available)
      BOOST_STATIC_ASSERT_MSG((boost::is_convertible<ContainerType*, BaseType*>::value),
        "In MBSim::ObjectFactory<BaseType>::create<ContainerType>(...) ContainerType must be derived from BaseType.");
#endif
      // throw error if NULL is supplied as element
      if(element==NULL) throw MBSimError("Internal error: NULL argument specified.");
      // loop over all all registred types corresponding to element->ValueStr()
      size_t matchNr=1;
      for(VectorIt it=instance().registeredType.begin(); it!=instance().registeredType.end(); it++) {
        // skip type with wrong key value get<0>()
        
        if(it->template get<0>()!=MBXMLUtils::E(element)->getTagName()) continue;

        // allocate a new object OR get singleton object using the allocate function pointer
        BaseType *ele=it->template get<1>()();
        // try to cast ele up to ContainerType
        ContainerType *ret=dynamic_cast<ContainerType*>(ele);
        // if possible (and it is the useMatchNr'st element, return it
        if(ret && (matchNr++)==useMatchNr)
          return std::pair<ContainerType*, DeallocateFkt>(ret, it->template get<2>());
        // if not possible, deallocate newly created (wrong) object OR do nothing for
        // singleton objects (is maybe reused later) and continue searching
        else
          it->template get<2>()(ele);
      }
      // no matching element found: throw error
      throw ObjectFactoryNoObject(element, typeid(ContainerType));
    }

    // convinence typedefs
    typedef boost::tuple<MBXMLUtils::FQN, AllocateFkt, DeallocateFkt> VectorContent;
    typedef std::vector<VectorContent> Vector;
    typedef typename Vector::iterator VectorIt;

    // private ctor
    ObjectFactory() {}

    static void registerXMLName(const MBXMLUtils::FQN &name, AllocateFkt alloc, DeallocateFkt dealloc) {
      // check if name was already registred with the same &allocate<CreateType>: if yes return and do not add it twice
      for(VectorIt it=instance().registeredType.begin(); it!=instance().registeredType.end(); it++) {
        // skip type with wrong key value get<0>()
        if(it->template get<0>()!=name) continue;

        if(it->template get<1>()==alloc)
          return;
      }
      // name is not registred with &allocate<CreateType>: register it
      instance().registeredType.push_back(VectorContent(name, alloc, dealloc));
    }

    static void deregisterXMLName(const MBXMLUtils::FQN &name, AllocateFkt alloc) {
      // dereg the element which as a name of 'name' AND a alloc function of 'alloc'
      for(VectorIt it=instance().registeredType.begin(); it!=instance().registeredType.end(); it++)
        if(it->template get<0>()==name && it->template get<1>()==alloc) {
          instance().registeredType.erase(it);
          return;
        }
    }

    // create an singleton instance of the object factory.
    // only declaration here and defition and explicit instantation for all BaseType in objectfactory.cc (required for Windows)
    static ObjectFactory<BaseType>& instance();

    // a vector of all registered types
    Vector registeredType;

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
    static BaseType* getSingleton() {
      return CreateType::getInstance();
    }

    // a wrapper to "deallocate" an singleton object (Must have the same signature as deallocate()
    static void deallocateSingleton(BaseType *obj) {
      // just do nothing for singletons
    }

};

/** Helper function for automatic class registration for ObjectFactory.
 * You should not use this class directly but
 * use the macro MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY. */
template<class BaseType, class CreateType>
class ObjectFactoryRegisterXMLNameHelper {

  public:

    /** ctor registring the new type */
    ObjectFactoryRegisterXMLNameHelper(const MBXMLUtils::FQN &name_) : name(name_) {
      ObjectFactory<BaseType>::template registerXMLName<CreateType>(name);
    };

    /** dtor deregistring the type */
    ~ObjectFactoryRegisterXMLNameHelper() {
      ObjectFactory<BaseType>::template deregisterXMLName<CreateType>(name);
    };

  private:
    MBXMLUtils::FQN name;

};

/** Helper function for automatic class registration for ObjectFactory.
 * You should not use this class directly but
 * use the macro MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORYASSINGLETON. */
template<class BaseType, class CreateType>
class ObjectFactoryRegisterXMLNameHelperAsSingleton {

  public:

    /** ctor registring the new type */
    ObjectFactoryRegisterXMLNameHelperAsSingleton(const MBXMLUtils::FQN &name_) : name(name_) {
      ObjectFactory<BaseType>::template registerXMLNameAsSingleton<CreateType>(name);
    };

    /** dtor deregistring the type */
    ~ObjectFactoryRegisterXMLNameHelperAsSingleton() {
      ObjectFactory<BaseType>::template deregisterXMLNameAsSingleton<CreateType>(name);
    };

  private:
    MBXMLUtils::FQN name;

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
