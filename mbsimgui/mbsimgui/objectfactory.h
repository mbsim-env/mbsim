/*
    MBSimGUI - A fronted for MBSim.
    Copyright (C) 2012 Martin Förg

  This library is free software; you can redistribute it and/or 
  modify it under the terms of the GNU Lesser General Public 
  License as published by the Free Software Foundation; either 
  version 2.1 of the License, or (at your option) any later version. 
   
  This library is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
  Lesser General Public License for more details. 
   
  You should have received a copy of the GNU Lesser General Public 
  License along with this library; if not, write to the Free Software 
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
*/

#ifndef _OBJECTFACTORY__H_
#define _OBJECTFACTORY__H_

#include <QString>
#include <map>
#include <vector>
#include <functional>
#include <xercesc/dom/DOMElement.hpp>
#include <mbxmlutilshelper/dom.h>
#include <boost/fusion/container/map.hpp>
#include <boost/mpl/map.hpp>
#include <boost/mpl/at.hpp>
#include <boost/fusion/include/at_key.hpp>
#include <boost/fusion/mpl.hpp>
#include <boost/fusion/include/for_each.hpp>

// Some X-Macros to do task for all ...
// ... tree container types
#define MBSIMGUI_TREE_CONTAINERS \
  X(Group) \
  X(Contour) \
  X(FixedRelativeFrame) \
  X(NodeFrame) \
  X(Object) \
  X(Link) \
  X(Constraint) \
  X(Observer) \
  X(Solver) \
  X(Parameter)
// ... widget container types
#define MBSIMGUI_WIDGET_CONTAINERS \
  X(GeneralizedImpactLawWidget) \
  X(FrictionImpactLawWidget) \
  X(FrictionForceLawWidget) \
  X(GeneralizedForceLawWidget) \
  X(TyreModelWidget) \
  X(EnvironmentWidget) \
  X(FunctionWidget) \
  X(GravityFunctionWidget) \
  X(OMBVRigidBodyWidget) \
  X(OMBVObjectWidget)
// ... all container types
#define MBSIMGUI_ALL_CONTAINERS \
  MBSIMGUI_TREE_CONTAINERS \
  MBSIMGUI_WIDGET_CONTAINERS

class QWidget;

namespace MBSimGUI {

  class Element;
  class UnknownObject;
  class UnknownConstraint;
  class UnknownContour;
  class UnknownFixedRelativeFrame;
  class UnknownNodeFrame;
  class UnknownLink;
  class UnknownObserver;
  class UnknownGroup;
  class DOPRI5Integrator;
  class UnknownSolver;
  template<class Container> class UnknownWidget;
  class GeneralizedPositionConstraint;
  class Line;
  class SpringDamper;
  class RigidBody;
  class RigidBodyObserver;
  class MBSimEnvironmentWidget;
  class BilateralConstraintWidget;
  class PlanarCoulombFrictionWidget;
  class BilateralImpactWidget;
  class PlanarCoulombImpactWidget;
  class MagicFormulaSharpWidget;
  // class forward declarations
  #define X(Type) \
    class Type;
  MBSIMGUI_ALL_CONTAINERS
  #undef X

  //! Use this macro in every class which is derived from ObjectFactoryBase to define all
  //! static and member functions needed by the object factory.
  //! Class is the name of the current class.
  //! Super is the name of the base class of the current class.
  //! xmlType is the MBXMLUtils::FQN of the current class.
  //! typeName is the human readable name of the current class.
  #define MBSIMGUI_OBJECTFACTORY_CLASS(/*Type*/ Class, /*Type*/ Super, /*MBXMLUtils::FQN*/ xmlType, /*std::string*/ typeName) \
    public: \
      static MBXMLUtils::FQN getXMLTypeStatic() { return xmlType; } \
      static QString getTypeStatic() { return typeName; } \
      static const std::vector<std::pair<QString, std::reference_wrapper<const std::type_info>>>& getTypePathStatic() { \
        static std::vector<std::pair<QString, std::reference_wrapper<const std::type_info>>> ret; \
        if(!ret.empty()) \
          return ret; \
        ret=Super::getTypePathStatic(); \
        ret.emplace_back(getTypeStatic(), typeid(Class)); \
        return ret; \
      } \
      virtual MBXMLUtils::FQN getXMLType() const override { return getXMLTypeStatic(); } \
      virtual QString getType() const override { return getTypeStatic(); } \
    private:

  //! This is the base class for all classes which can be created by the ObjectFactory.
  class ObjectFactoryBase {
    public:
      // the following static and member functions are overwritten by each class using the macro MBSIMGUI_OBJECTFACTORY_CLASS.
      static const std::vector<std::pair<QString, std::reference_wrapper<const std::type_info>>>& getTypePathStatic() {
        static std::vector<std::pair<QString, std::reference_wrapper<const std::type_info>>> ret;
        return ret;
      };
      virtual MBXMLUtils::FQN getXMLType() const = 0;
      virtual QString getType() const = 0;
      virtual ~ObjectFactoryBase() = default;
  };

  //! A object factory which is used to create (all, excluding Functions) objects in MBSimGUI.
  //! New classes can be registered externally, even from libraries loaded at runtime. Hence, plugins are possible.
  class ObjectFactory {
    private:
      // just a abbreviation for shorter lines in this file
      template<class T1, class T2> using P = boost::mpl::pair<T1, T2>;
    public:
      // Its a singleton class.
      ObjectFactory(const ObjectFactory&) = delete;
      ObjectFactory(ObjectFactory&&) = delete;
      ObjectFactory& operator=(const ObjectFactory&) = delete;
      ObjectFactory& operator=(ObjectFactory&&) = delete;

      //! Get the instance of the object factory (its a singleton class)
      static ObjectFactory& getInstance();

      //! Create an object corresponding to the XML element element.
      //! Returns the object of type Container*.
      //! If the object is not of this container type (dynamic_cast fails), throws.
      //! If no class for the XML element is registered returns the corresponding "unknown" object of the container Container.
      //! If there is no such "unknown" object, throws.
      template<class Container>
      Container* create(xercesc::DOMElement *element, Element *e=nullptr, QWidget *pw=nullptr) const;

      //! For each registered class this object factory stores the following functions.
      struct Funcs {
        std::function<ObjectFactoryBase*(Element *, QWidget *)> ctor; //!< creation function.
        std::function<QString()> getType; //!< the human readable type name.
        std::function<MBXMLUtils::FQN()> getXMLType; //!< the XML element name (including namespace) of the corresponding XML element.
        std::function<const std::vector<std::pair<QString, std::reference_wrapper<const std::type_info>>>&()> getTypePath; //!< the human readable type name path of this class.
        const std::type_info& typeInfo; //!< the type_info of the class
      };

      //! Returns a list of all registered classed for the container Container.
      template<class Container>
      const std::vector<const Funcs*>& getAllTypesForContainer() const;

      // Register the class T. Use MBSIMGUI_REGOBJECTFACTORY(T).
      template<class T>
      void registerClass();

      // A map from an container type to the default and unknown type of this container.
      using MapContainerToDefaultAndUnknown = boost::mpl::map<
        // container type [key]         default type of container      unknown type of container
        P<Constraint                , P<void                         , UnknownConstraint>>,
        P<Contour                   , P<void                         , UnknownContour>>,
        P<FixedRelativeFrame        , P<void                         , UnknownFixedRelativeFrame>>,
        P<NodeFrame                 , P<void                         , UnknownNodeFrame>>,
        P<Link                      , P<void                         , UnknownLink>>,
        P<Object                    , P<void                         , UnknownObject>>,
        P<Observer                  , P<void                         , UnknownObserver>>,
        P<Group                     , P<void                         , UnknownGroup>>,
        P<Solver                    , P<DOPRI5Integrator             , UnknownSolver>>,
        P<Parameter                 , P<void                         , void>>,
        P<EnvironmentWidget         , P<MBSimEnvironmentWidget       , UnknownWidget<EnvironmentWidget>>>,
        P<GeneralizedForceLawWidget , P<BilateralConstraintWidget    , UnknownWidget<GeneralizedForceLawWidget>>>,
        P<FrictionForceLawWidget    , P<PlanarCoulombFrictionWidget  , UnknownWidget<FrictionForceLawWidget>>>,
        P<GeneralizedImpactLawWidget, P<BilateralImpactWidget        , UnknownWidget<GeneralizedImpactLawWidget>>>,
        P<FrictionImpactLawWidget   , P<PlanarCoulombImpactWidget    , UnknownWidget<FrictionImpactLawWidget>>>,
        P<TyreModelWidget           , P<MagicFormulaSharpWidget      , UnknownWidget<TyreModelWidget>>>,
        P<FunctionWidget            , P<void                         , UnknownWidget<FunctionWidget>>>,
        P<GravityFunctionWidget     , P<void                         , UnknownWidget<GravityFunctionWidget>>>,
        P<OMBVRigidBodyWidget       , P<void                         , UnknownWidget<OMBVRigidBodyWidget>>>,
        P<OMBVObjectWidget          , P<void                         , UnknownWidget<OMBVObjectWidget>>>
      >;

      // All errors during class registration are catched and added to a global error message using addErrorMsg.
      // These errors can then get via getAndClearErrorMsg.
      // This is needed since its not possible to even print error messages to e.g. cerr during class registration
      // since this may happen pre-main (where cerr is not initialized yet).
      void addErrorMsg(const std::string &msg);
      std::string getAndClearErrorMsg();

    private:
      ObjectFactory() = default; // !!!!! this code is called pre-main: take care !!!!!
      ~ObjectFactory() = default;

      // A map from a XML FQN  element name to the Funcs hold by this XML element name.
      // This map is filled by the registerClass function
      mutable std::map<MBXMLUtils::FQN, std::unique_ptr<Funcs>> mapElementNameToValue;

      // convert MapContainerToDefaultAndUnknown to a boost::fusion::map AllTypesForContainer.
      // - the key of AllTypesForContainer is the same as the key of MapContainerToDefaultAndUnknown
      // - the value of AllTypesForContainer is std::vector<const Funcs*> (which references to the functions stored in mapElementNameToValue)
      using AllTypesForContainer = boost::mpl::fold< // create a new type by folding
        MapContainerToDefaultAndUnknown, // the list to traverse to build the new type
        boost::fusion::map<>, // the new type to create (empty, will be filled by the fold)
        boost::mpl::push_back< // for each traverse add a new entry ...
          boost::mpl::_1, // ... to this type (the boost::fusion::map)
          boost::fusion::pair<boost::mpl::first<boost::mpl::_2>, std::vector<const Funcs*>> // add this type (_2 is the current value of MapContainerToDefaultAndUnknown
        >
      >::type;
      // A map storing for each container type a list of all registered types.
      mutable AllTypesForContainer allTypesForContainer;

      // the errors used by addErrorMsg and getAndClearErrorMsg.
      std::string errorMsg;
  };

  // Helper class, used by the macro MBSIMGUI_REGOBJECTFACTORY(T) to register T by the object factory at load time.
  template<class T>
  class RegObjectFactory {
    public:
      RegObjectFactory() noexcept {
        try {
          // !!!!! this code is called pre-main: take care !!!!!
          ObjectFactory::getInstance().registerClass<T>();
        }
        // catch all errors and add to addErrorMsg since even e.g. cerr cannot be used.
        catch(std::exception &ex) {
          ObjectFactory::getInstance().addErrorMsg(ex.what());
        }
        catch(...) {
          ObjectFactory::getInstance().addErrorMsg("Unknown error");
        }
      }
  };

  //! Register the class T in the object factory.
  //! This macro must be called at global scope to execute the registration at load time.
  #define MBSIMGUI_REGOBJECTFACTORY(T) \
    static MBSimGUI::RegObjectFactory<T> BOOST_PP_CAT(regObjectFactoryVar_, __LINE__);

  template<class T>
  void ObjectFactory::registerClass() {
    // !!!!! this code is called pre-main: take care !!!!!

    // add to mapElementNameToValue and get a reference to the added functions
    auto [it, created]=mapElementNameToValue.insert({T::getXMLTypeStatic(), std::unique_ptr<Funcs>(new Funcs{
      [](Element *e, QWidget *pw) {
        // If T has a ctor(Element *, QWidget *) create the class using this ctor
        if constexpr (std::is_constructible_v<T, Element*, QWidget*>)
          return new T(e, pw);
        // Else use the default ctor
        else if constexpr (std::is_constructible_v<T>)
          return new T;
        else
          static_assert(!std::is_same_v<T, void>, "Cannot register class T since it has no proper ctor.");
      },
      &T::getTypeStatic,
      &T::getXMLTypeStatic,
      &T::getTypePathStatic,
      typeid(T),
    })});
    if(!created)
      throw std::runtime_error("The object factory already contains a XML type {"+
                               T::getXMLTypeStatic().first+"}"+T::getXMLTypeStatic().second+".");
    auto &func = it->second;
    // add to each allTypesForContainer<T> if the object if of type T
    boost::fusion::for_each(allTypesForContainer, [&func](auto &x) {
      if constexpr (std::is_base_of_v<typename std::remove_reference_t<decltype(x)>::first_type, T>) {
        // keep the list sorted
        auto it=std::find_if(x.second.begin(), x.second.end(), [](auto &v){ return T::getTypeStatic() < v->getType(); });
        x.second.insert(it, func.get());
      }
    });
  }

}

#endif
