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
#include "fmatvec/atom.h"

#define COMMA ,

namespace MBSim {

/* A tree of DOMEvalException's which store the tree of failed objects
 * creations of the object factory */
class DOMEvalExceptionStack : public MBXMLUtils::DOMEvalException {
  public:
    DOMEvalExceptionStack(const xercesc::DOMElement *element) : MBXMLUtils::DOMEvalException("", element) {}
    DOMEvalExceptionStack(const DOMEvalExceptionStack &src) : MBXMLUtils::DOMEvalException(src), exVec(src.exVec) {}
    ~DOMEvalExceptionStack() throw() {}
    void add(const std::string &type, const boost::shared_ptr<MBXMLUtils::DOMEvalException> &ex);
    const char* what() const throw();
    std::vector<std::pair<std::string, boost::shared_ptr<MBXMLUtils::DOMEvalException> > > &getExceptionVector();
  protected:
    std::vector<std::pair<std::string, boost::shared_ptr<MBXMLUtils::DOMEvalException> > > exVec;
    mutable std::string whatStr;
    void generateWhat(std::stringstream &str, const std::string &indent) const;
};

/* Just to distinguish a wrong type (not castable) error of the object factory
 * from others */
class DOMEvalExceptionWrongType : public MBXMLUtils::DOMEvalException {
  public:
    DOMEvalExceptionWrongType(const std::string &type, const xercesc::DOMElement *element) :
      MBXMLUtils::DOMEvalException(type, element) {}
    DOMEvalExceptionWrongType(const DOMEvalExceptionWrongType &src) : MBXMLUtils::DOMEvalException(src) {}
    ~DOMEvalExceptionWrongType() throw() {}
};

/** A object factory.
 * A object factory which creates any object derived from fmatvec::Atom.
 */
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
#ifdef HAVE_BOOST_TYPE_TRAITS_HPP
      // just check if ContainerType is derived from fmatvec::Atom if not throw a compile error if boost is avaliable
      // if boost is not avaliable a runtime error will occure later. (so it does not care if boost is not available)
      BOOST_STATIC_ASSERT_MSG((boost::is_convertible<ContainerType*, fmatvec::Atom*>::value),
        "In MBSim::ObjectFactory::create<ContainerType>(...) ContainerType must be derived from fmatvec::Atom.");
#endif
      // throw error if NULL is supplied as element
      if(element==NULL)
        throw MBSimError("Internal error: NULL argument specified.");
      // get all allocate functions for the given name
      MBXMLUtils::FQN fqn=MBXMLUtils::E(element)->getTagName();
      NameMapIt nameIt=instance().registeredType.find(fqn);
      if(nameIt==instance().registeredType.end())
        throw MBXMLUtils::DOMEvalException("Internal error: No objects of name {"+fqn.first+"}"+fqn.second+" registred", element);
      DOMEvalExceptionStack allErrors(static_cast<xercesc::DOMElement*>(element->getParentNode()));
      // try to create and init a object which each of the allocate function
      for(AllocDeallocVectorIt allocDeallocIt=nameIt->second.begin(); allocDeallocIt!=nameIt->second.end(); ++allocDeallocIt) {
        // create element
        fmatvec::Atom *ele=allocDeallocIt->first();
        // try to cast the element to ContainerType
        ContainerType *ret=dynamic_cast<ContainerType*>(ele);
        if(!ret) {
          // cast not possible -> deallocate again and try next
          allErrors.add(boost::core::demangle(typeid(*ele).name()),
                        boost::make_shared<DOMEvalExceptionWrongType>(
                        boost::core::demangle(typeid(ContainerType).name()), element));
          allocDeallocIt->second(ele); 
          continue;
        }
        try {
          ret->initializeUsingXML(const_cast<xercesc::DOMElement*>(element));
          return ret;
        }
        catch(DOMEvalExceptionStack &ex) {
          allErrors.add(boost::core::demangle(typeid(*ele).name()), boost::make_shared<DOMEvalExceptionStack>(ex));
        }
        catch(MBXMLUtils::DOMEvalException &ex) {
          allErrors.add(boost::core::demangle(typeid(*ele).name()), boost::make_shared<MBXMLUtils::DOMEvalException>(ex));
        }
        catch(std::exception &ex) { // handles also MBSimError
          allErrors.add(boost::core::demangle(typeid(*ele).name()),
                        boost::make_shared<MBXMLUtils::DOMEvalException>(ex.what(), element));
        }
        catch(...) {
          allErrors.add(boost::core::demangle(typeid(*ele).name()),
                        boost::make_shared<MBXMLUtils::DOMEvalException>("Unknwon exception", element));
        }
        allocDeallocIt->second(ele);
      }
      // if all failed -> return errors of all trys
      throw allErrors;
    }

  private:

    // a pointer to a function allocating an object
    typedef fmatvec::Atom* (*AllocateFkt)();
    // a pointer to a function deallocating an object
    typedef void (*DeallocateFkt)(fmatvec::Atom *obj);

    // convinence typedefs
    typedef std::vector<std::pair<AllocateFkt, DeallocateFkt> > AllocDeallocVector;
    typedef AllocDeallocVector::iterator AllocDeallocVectorIt;
    typedef std::map<MBXMLUtils::FQN, AllocDeallocVector> NameMap;
    typedef NameMap::iterator NameMapIt;

    // private ctor
    ObjectFactory() {}

    static void registerXMLName(const MBXMLUtils::FQN &name, AllocateFkt alloc, DeallocateFkt dealloc);

    static void deregisterXMLName(const MBXMLUtils::FQN &name, AllocateFkt alloc);

    // create an singleton instance of the object factory.
    // only declaration here and defition and explicit instantation for all fmatvec::Atom in objectfactory.cc (required for Windows)
    static ObjectFactory& instance();

    // a vector of all registered types
    NameMap registeredType;

    // a wrapper to allocate an object of type CreateType
    template<class CreateType>
    static fmatvec::Atom* allocate() {
      return new CreateType;
    }

    // a wrapper to deallocate an object created by allocate
    static void deallocate(fmatvec::Atom *obj) {
      delete obj;
    }

    // a wrapper to get an singleton object of type CreateType (Must have the same signature as allocate()
    template<class CreateType>
    static fmatvec::Atom* getSingleton() {
      return CreateType::getInstance();
    }

    // a wrapper to "deallocate" an singleton object (Must have the same signature as deallocate()
    static void deallocateSingleton(fmatvec::Atom *obj) {
      // just do nothing for singletons
    }

};

/** Helper function for automatic class registration for ObjectFactory.
 * You should not use this class directly but
 * use the macro MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORY. */
template<class CreateType>
class ObjectFactoryRegisterXMLNameHelper {

  public:

    /** ctor registring the new type */
    ObjectFactoryRegisterXMLNameHelper(const MBXMLUtils::FQN &name_) : name(name_) {
      ObjectFactory::template registerXMLName<CreateType>(name);
    };

    /** dtor deregistring the type */
    ~ObjectFactoryRegisterXMLNameHelper() {
      ObjectFactory::template deregisterXMLName<CreateType>(name);
    };

  private:
    MBXMLUtils::FQN name;

};

/** Helper function for automatic class registration for ObjectFactory.
 * You should not use this class directly but
 * use the macro MBSIM_REGISTER_XMLNAME_AT_OBJECTFACTORYASSINGLETON. */
template<class CreateType>
class ObjectFactoryRegisterXMLNameHelperAsSingleton {

  public:

    /** ctor registring the new type */
    ObjectFactoryRegisterXMLNameHelperAsSingleton(const MBXMLUtils::FQN &name_) : name(name_) {
      ObjectFactory::template registerXMLNameAsSingleton<CreateType>(name);
    };

    /** dtor deregistring the type */
    ~ObjectFactoryRegisterXMLNameHelperAsSingleton() {
      ObjectFactory::template deregisterXMLNameAsSingleton<CreateType>(name);
    };

  private:
    MBXMLUtils::FQN name;

};

}

#define MBSIM_OBJECTFACTORY_CONCAT1(X, Y) X##Y
#define MBSIM_OBJECTFACTORY_CONCAT(X, Y) MBSIM_OBJECTFACTORY_CONCAT1(X, Y)
#define MBSIM_OBJECTFACTORY_APPENDLINE(X) MBSIM_OBJECTFACTORY_CONCAT(X, __LINE__)

/** Use this macro somewhere at the class definition of ThisType to register it by the ObjectFactory.
 * fmatvec::Atom is the base of ThisType and also the template parameter of ObjectFactory.
 * ThisType must have a public default ctor and a public dtor. */
#define MBSIM_OBJECTFACTORY_REGISTERXMLNAME(ThisType, name) \
  static MBSim::ObjectFactoryRegisterXMLNameHelper<ThisType> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariable)(name);

/** Use this macro somewhere at the class definition of ThisType to register it by the ObjectFactory (as a singleton).
 * fmatvec::Atom is the base of ThisType and also the template parameter of ObjectFactory.
 * ThisType must have a public ThisType* getInstance() function and should not have a public dtor. */
#define MBSIM_OBJECTFACTORY_REGISTERXMLNAMEASSINGLETON(ThisType, name) \
  static MBSim::ObjectFactoryRegisterXMLNameHelperAsSingleton<ThisType> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariableAsSingleTon)(name);

/** Same as MBSIM_OBJECTFACTORY_REGISTERXMLNAME but also explicitly instantiates the template class ThisType.
 * Please note that template member functions of ThisType must be explicitly instantated by hand. */
#define MBSIM_OBJECTFACTORY_REGISTERXMLNAME_AND_INSTANTIATE(ThisType, name) \
  template class ThisType; \
  static MBSim::ObjectFactoryRegisterXMLNameHelper<ThisType> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariable)(name);

/** Same as MBSIM_OBJECTFACTORY_REGISTERXMLNAMEASSINGLETON but also explicitly instantiates the template class ThisType.
 * Please note that template member functions of ThisType must be explicitly instantated by hand. */
#define MBSIM_OBJECTFACTORY_REGISTERXMLNAMEASSINGLETON_AND_INSTANTIATE(ThisType, name) \
  template class ThisType; \
  static MBSim::ObjectFactoryRegisterXMLNameHelperAsSingleton<ThisType> \
    MBSIM_OBJECTFACTORY_APPENDLINE(objectFactoryRegistrationDummyVariableAsSingleTon)(name);

#endif
