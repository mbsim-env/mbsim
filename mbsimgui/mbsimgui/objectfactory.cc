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

#include <config.h>
#include "objectfactory.h"
#include "object.h"
#include "parameter.h"
#include "solver.h"
#include "contour.h"
#include "group.h"
#include "constraint.h"
#include "observer.h"
#include "frame.h"
#include "link_.h"
#include "environment_widgets.h"
#include "kinetics_widgets.h"
#include "function_widget.h"
#include "function_widget.h"
#include "function_widgets.h"

using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSimGUI {

  static_assert(boost::mpl::size<ObjectFactory::MapContainerToDefaultAndUnknown>::value== // size of MapContainerToDefaultAndUnknown
    boost::mpl::size<
      boost::mpl::fold< // create a new type by folding
        ObjectFactory::MapContainerToDefaultAndUnknown, // the list to traverse to build the new type
        boost::mpl::map<>, // the new type to create (empty, will be filled by the fold)
        boost::mpl::insert< // for each traverse insert a new entry (this removes duplicate keys) ...
          boost::mpl::_1, // ... to this type (the boost::mpl::map)
          boost::mpl::_2 // add this type
        >
      >::type
    >::value, // size of MapContainerToDefaultAndUnknown with duplicates removed
    "ObjectFactory::MapContainerToDefaultAndUnknown contains none unique keys.");

  ObjectFactory& ObjectFactory::getInstance() {
    // !!!!! this code is called pre-main: take care !!!!!
    static ObjectFactory instance;
    return instance;
  }

  template<class Container>
  Container* ObjectFactory::create(xercesc::DOMElement *element, Element *e, QWidget *pw) const {
    auto it=mapElementNameToValue.find(MBXMLUtils::E(element)->getTagName());
    if(it!=mapElementNameToValue.end()) {
      ObjectFactoryBase *ele=it->second->ctor(e, pw);
      Container *cont=dynamic_cast<Container*>(ele);
      if(!cont) {
        delete ele;
        throw runtime_error(string("This XML element is not of type ")+boost::core::demangle(typeid(Container).name())+".");
      }
      return cont;
    }
    static_assert(boost::mpl::has_key<MapContainerToDefaultAndUnknown, Container>::value,
      "ObjectFactroy::create called for Container but MapContainerToDefaultAndUnknown has no such key. "
      "Please add Container to MapContainerToDefaultAndUnknown.");
    if constexpr (!is_same_v<typename boost::mpl::at<MapContainerToDefaultAndUnknown, Container>::type::second, void>)
      return new typename boost::mpl::at<MapContainerToDefaultAndUnknown, Container>::type::second;
    throw runtime_error(string("No class found for XML element {")+
            MBXMLUtils::E(element)->getTagName().first+"}"+MBXMLUtils::E(element)->getTagName().second+
            " and no \"unknown\" class exists for the type "+boost::core::demangle(typeid(Container).name()));
  }

  // explizit instantiation
  #define X(Type) \
    template Type* ObjectFactory::create<Type>(xercesc::DOMElement *element, Element *e, QWidget *pw) const;
  MBSIMGUI_TREE_CONTAINERS
  #undef X

  template<class Container>
  const vector<const ObjectFactory::Funcs*>& ObjectFactory::getAllTypesForContainer() const {
    static_assert(boost::mpl::has_key<MapContainerToDefaultAndUnknown, Container>::value,
      "ObjectFactory::getAllTypesForContainer called with Container not part of type MapContainerToDefaultAndUnknown. "
      "Add Container key to type MapContainerToDefaultAndUnknown.");
    return boost::fusion::at_key<Container>(allTypesForContainer);
  }

  // explizit instantiation
  #define X(Type) \
    template const vector<const ObjectFactory::Funcs*>& ObjectFactory::getAllTypesForContainer<Type>() const;
  MBSIMGUI_ALL_CONTAINERS
  #undef X

  void ObjectFactory::addErrorMsg(const string &msg) {
    errorMsg+=msg+"\n";
  }

  string ObjectFactory::getAndClearErrorMsg() {
    auto ret=errorMsg;
    errorMsg.clear();
    return ret;
  }

}
