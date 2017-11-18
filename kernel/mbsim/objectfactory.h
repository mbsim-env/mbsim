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

#include "objectfactory_part.h"
#include <utility>
#include <vector>
#include <functional>
#include <stdexcept>
#include <typeinfo>
#include "mbsim/utils/utils.h"
#include <mbsim/mbsim_event.h>
#include <xercesc/dom/DOMAttr.hpp>
#include <mbxmlutilshelper/utils.h>
#include "fmatvec/atom.h"
#include <boost/version.hpp>
#if BOOST_VERSION >= 105600
  #include <boost/core/demangle.hpp>
#else
  #include <cxxabi.h>
  #ifndef BOOST_CORE_DEMANGLE_REPLACEMENT
  #define BOOST_CORE_DEMANGLE_REPLACEMENT
  namespace boost {
    namespace core {
      inline std::string demangle(const std::string &name) {
        int status;
        char* retc=abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status);
        if(status!=0) throw std::runtime_error("Cannot demangle c++ symbol.");
        std::string ret(retc);
        free(retc);
        return ret;
      }
    }
  }
  #endif
#endif

#define COMMA ,

namespace MBSim {

/* A tree of DOMEvalException's which store the tree of failed objects
 * creations of the object factory */
class DOMEvalExceptionStack : public MBXMLUtils::DOMEvalException {
  public:
    DOMEvalExceptionStack(const xercesc::DOMElement *element) : MBXMLUtils::DOMEvalException("", element) {}
    DOMEvalExceptionStack(const DOMEvalExceptionStack &src) : MBXMLUtils::DOMEvalException(src), exVec(src.exVec) {}
    ~DOMEvalExceptionStack() noexcept override = default;
    void add(const std::string &type, const std::shared_ptr<MBXMLUtils::DOMEvalException> &ex);
    const char* what() const noexcept override;
    std::vector<std::pair<std::string, std::shared_ptr<MBXMLUtils::DOMEvalException> > > &getExceptionVector();
  protected:
    std::vector<std::pair<std::string, std::shared_ptr<MBXMLUtils::DOMEvalException> > > exVec;
    mutable std::string whatStr;
    void generateWhat(std::stringstream &str, const std::string &indent) const;
};

/* Just to distinguish a wrong type (not castable) error of the object factory
 * from others */
class DOMEvalExceptionWrongType : public MBXMLUtils::DOMEvalException {
  public:
    DOMEvalExceptionWrongType(const std::string &type, const xercesc::DOMElement *element) :
      MBXMLUtils::DOMEvalException(type, element) {}
    DOMEvalExceptionWrongType(const DOMEvalExceptionWrongType &src)  = default;
    ~DOMEvalExceptionWrongType() noexcept override = default;
};

/** A object factory.
 * A object factory which creates any object derived from fmatvec::Atom.
 */
class ObjectFactory {

  friend void registerClass_internal(const MBXMLUtils::FQN &name, const AllocateBase *alloc, const DeallocateBase *dealloc);
  friend void deregisterClass_internal(const MBXMLUtils::FQN &name, const AllocateBase *alloc);

  public:

    /** Create and initialize an object corresponding to the XML element element and return a pointer of type ContainerType.
     * Throws if the created object is not of type ContainerType or no object can be create without errors.
     * This function returns a new object or a singleton object dependent on the registration of the created object. */
    template<class ContainerType>
    static ContainerType* createAndInit(const xercesc::DOMElement *element) {
      // just check if ContainerType is derived from fmatvec::Atom if not throw a compile error
      static_assert(std::is_convertible<ContainerType*, fmatvec::Atom*>::value,
        "In MBSim::ObjectFactory::create<ContainerType>(...) ContainerType must be derived from fmatvec::Atom.");
      // throw error if NULL is supplied as element
      if(element==nullptr)
        throw MBSimError("Internal error: NULL argument specified.");
      // get all allocate functions for the given name
      MBXMLUtils::FQN fqn=MBXMLUtils::E(element)->getTagName();
      auto nameIt=instance().registeredType.find(fqn);
      if(nameIt==instance().registeredType.end())
        throw MBXMLUtils::DOMEvalException("Internal error: No objects of name {"+fqn.first+"}"+fqn.second+" registred", element);
      DOMEvalExceptionStack allErrors(static_cast<xercesc::DOMElement*>(element->getParentNode()));
      // try to create and init a object which each of the allocate function
      for(auto & allocDeallocIt : nameIt->second) {
        // create element
        fmatvec::Atom *ele=(*allocDeallocIt.first)();
        // try to cast the element to ContainerType
        auto *ret=dynamic_cast<ContainerType*>(ele);
        if(!ret) {
          // cast not possible -> deallocate again and try next
          allErrors.add(boost::core::demangle(typeid(*ele).name()),
                        std::make_shared<DOMEvalExceptionWrongType>(
                        boost::core::demangle(typeid(ContainerType).name()), element));
          (*allocDeallocIt.second)(ele); 
          continue;
        }
        try {
          ret->initializeUsingXML(const_cast<xercesc::DOMElement*>(element));
          return ret;
        }
        catch(DOMEvalExceptionStack &ex) {
          allErrors.add(boost::core::demangle(typeid(*ele).name()), std::make_shared<DOMEvalExceptionStack>(ex));
        }
        catch(MBXMLUtils::DOMEvalException &ex) {
          allErrors.add(boost::core::demangle(typeid(*ele).name()), std::make_shared<MBXMLUtils::DOMEvalException>(ex));
        }
        catch(std::exception &ex) { // handles also MBSimError
          allErrors.add(boost::core::demangle(typeid(*ele).name()),
                        std::make_shared<MBXMLUtils::DOMEvalException>(ex.what(), element));
        }
        catch(...) {
          allErrors.add(boost::core::demangle(typeid(*ele).name()),
                        std::make_shared<MBXMLUtils::DOMEvalException>("Unknwon exception", element));
        }
        (*allocDeallocIt.second)(ele);
      }
      // if all failed -> return errors of all trys
      throw allErrors;
    }

  private:

    // convinence typedefs
    typedef std::pair<const AllocateBase*, const DeallocateBase*> AllocDeallocPair;
    typedef std::vector<AllocDeallocPair> AllocDeallocVector;
    typedef AllocDeallocVector::iterator AllocDeallocVectorIt;
    typedef std::map<MBXMLUtils::FQN, AllocDeallocVector> NameMap;
    typedef NameMap::iterator NameMapIt;

    // private ctor
    ObjectFactory() = default;

    // create an singleton instance of the object factory.
    static ObjectFactory& instance();

    // a vector of all registered types
    NameMap registeredType;
};

// a wrapper to allocate an object of type CreateType
template<class CreateType>
struct Allocate : public AllocateBase {
  // create a new object of type CreateType using new
  fmatvec::Atom* operator()() const override {
    return new CreateType;
  }
  // check if this Allocator allocates the same object as other.
  // This is the case if the type of this template class matches
  // the type of other.
  bool operator==(const AllocateBase& other) const override {
    return typeid(*this)==typeid(other);
  }
};

// a wrapper to deallocate an object created by allocate
struct Deallocate : public DeallocateBase {
  // deallocate a object using delete
  void operator()(fmatvec::Atom *obj) const override {
    delete obj;
  }
};

// a wrapper to get an singleton object of type CreateType (Must have the same signature as allocate()
template<class CreateType>
struct GetSingleton : public AllocateBase {
  // create a new singelton object of type CreateType using CreateType::getInstance
  fmatvec::Atom* operator()() const override {
    return CreateType::getInstance();
  }
  // check if this Allocator returns the same object as other.
  // This is the case if the type of this template class matches
  // the type of other.
  bool operator==(const AllocateBase& other) const override {
    return typeid(*this)==typeid(other);
  }
};

// a wrapper to "deallocate" an singleton object (Must have the same signature as deallocate()
struct DeallocateSingleton : public DeallocateBase {
  // deallocate a singleton object -> just do nothing
  void operator()(fmatvec::Atom *obj) const override {
    // just do nothing for singletons
  }
};

/** Helper function for automatic class registration for ObjectFactory.
 * You should not use this class directly but
 * use the macro MBSIM_OBJECTFACTORY_REGISTERCLASS. */
template<class CreateType>
class ObjectFactoryRegisterClassHelper {

  public:

    /** ctor registring the new type */
    ObjectFactoryRegisterClassHelper(MBXMLUtils::FQN name_) : name(std::move(name_)) {
      MBSim::registerClass_internal(name, new Allocate<CreateType>(), new Deallocate());
    }

    /** dtor deregistring the type */
    ~ObjectFactoryRegisterClassHelper() {
      MBSim::deregisterClass_internal(name, new Allocate<CreateType>());
    }

  private:
    MBXMLUtils::FQN name;

};

/** Helper function for automatic class registration for ObjectFactory.
 * You should not use this class directly but
 * use the macro MBSIM_OBJECTFACTORY_REGISTERCLASSASSINGLETON. */
template<class CreateType>
class ObjectFactoryRegisterClassHelperAsSingleton {

  public:

    /** ctor registring the new type */
    ObjectFactoryRegisterClassHelperAsSingleton(MBXMLUtils::FQN name_) : name(std::move(name_)) {
      MBSim::registerClass_internal(name, new GetSingleton<CreateType>(), new DeallocateSingleton());
    }

    /** dtor deregistring the type */
    ~ObjectFactoryRegisterClassHelperAsSingleton() {
      MBSim::deregisterClass_internal(name, new GetSingleton<CreateType>());
    }

  private:
    MBXMLUtils::FQN name;

};

/** Helper function for automatic enum registration for ObjectFactory.
 * You should not use this class directly but
 * use the macro MBSIM_OBJECTFACTORY_REGISTERENUM. */
template<class EnumType>
class EnumFactory {

  template<class EV> friend void registerEnum_internal(const MBXMLUtils::FQN &name, const EV& value);
  template<class EV> friend void deregisterEnum_internal(const MBXMLUtils::FQN &name);

  public:

    /** ctor registring the new enum */
    EnumFactory(const EnumType &enumVar, MBXMLUtils::FQN fqn_) : fqn(std::move(fqn_)) {
      registerEnum_internal<EnumType>(fqn, enumVar);
    }

    /** dtor deregistring the enum */
    ~EnumFactory() {
      deregisterEnum_internal<EnumType>(fqn);
    }

    /** get an enum value given by the string enumStr **/
    static const EnumType& get(const MBXMLUtils::FQN &fqn, const xercesc::DOMElement *e=nullptr) {
      auto it=regmap().find(fqn);
      if(it==regmap().end())
        throw MBXMLUtils::DOMEvalException("No enumeration value named {"+fqn.first+"}"+fqn.second+" registred", e);
      return it->second.get();
    }

  private:
    MBXMLUtils::FQN fqn;

    // return the register map of the enum factory.
    // (note: this function must not be defined in this file to avoid any imlicit instantation)
    static std::map<MBXMLUtils::FQN, std::reference_wrapper<const EnumType>>& regmap();
};

template<class EV>
void registerEnum_internal(const MBXMLUtils::FQN &name, const EV& value) {
  EnumFactory<EV>::regmap().insert(std::make_pair(name, std::ref(value)));
}

template<class EV>
void deregisterEnum_internal(const MBXMLUtils::FQN &name) {
  EnumFactory<EV>::regmap().erase(name);
}

// fix local xml name (remove template and namespace)
std::string fixXMLLocalName(std::string name);

}

#define MBSIM_OBJECTFACTORY_CONCAT1(X, Y) X##Y
#define MBSIM_OBJECTFACTORY_CONCAT(X, Y) MBSIM_OBJECTFACTORY_CONCAT1(X, Y)
#define MBSIM_OBJECTFACTORY_APPENDLINE(X) MBSIM_OBJECTFACTORY_CONCAT(X, __LINE__)

/** Use this macro somewhere at the class definition of Class to register it by the ObjectFactory.
 * fmatvec::Atom is the base of Class and also the template parameter of ObjectFactory.
 * Class must have a public default ctor and a public dtor and a getXMLFQN() static member function. */
#define MBSIM_OBJECTFACTORY_REGISTERCLASS(NS, Class) \
  static MBSim::ObjectFactoryRegisterClassHelper<Class> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariable)(NS%MBSim::fixXMLLocalName(#Class));

/** Use this macro somewhere at the class definition of Class to register it by the ObjectFactory (as a singleton).
 * fmatvec::Atom is the base of Class and also the template parameter of ObjectFactory.
 * Class must have a public Class* getInstance() function and should not have a public dtor and a getXMLFQN() static member function. */
#define MBSIM_OBJECTFACTORY_REGISTERCLASSASSINGLETON(NS, Class) \
  static MBSim::ObjectFactoryRegisterClassHelperAsSingleton<Class> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariable)(NS%MBSim::fixXMLLocalName(#Class));

/** Same as MBSIM_OBJECTFACTORY_REGISTERCLASS but also explicitly instantiates the template class Class.
 * Please note that template member functions of Class must be explicitly instantated by hand. */
#define MBSIM_OBJECTFACTORY_REGISTERCLASS_AND_INSTANTIATE(NS, Class) \
  template class Class; \
  static MBSim::ObjectFactoryRegisterClassHelper<Class> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariable)(NS%MBSim::fixXMLLocalName(#Class));

/** Same as MBSIM_OBJECTFACTORY_REGISTERCLASSASSINGLETON but also explicitly instantiates the template class Class.
 * Please note that template member functions of Class must be explicitly instantated by hand. */
#define MBSIM_OBJECTFACTORY_REGISTERCLASSASSINGLETON_AND_INSTANTIATE(NS, Class) \
  template class Class; \
  static MBSim::ObjectFactoryRegisterClassHelperAsSingleton<Class> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariable)(NS%MBSim::fixXMLLocalName(#Class));

/** Use this macro somewhere at the definition of PlotFeatureEnum enumName to register it by the ObjectFactory. */
#define MBSIM_OBJECTFACTORY_REGISTERENUM(EnumType, NS, enumName) \
  static MBSim::EnumFactory<EnumType> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariable)(enumName, NS%#enumName);

#endif
