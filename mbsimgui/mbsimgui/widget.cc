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

#include <widget.h>
#include "kinetics_widgets.h"
#include "unknown_widget.h"
#include "environment_widgets.h"
#include "function_widget.h"
#include "function_widgets.h"
#include "ombv_widgets.h"

namespace MBSimGUI {

  template<class Container>
  WidgetFactoryFor<Container>::WidgetFactoryFor(Element *e_, QWidget *pw_) :e(e_), pw(pw_) {}

  template<class Container>
  QString WidgetFactoryFor<Container>::getName(int i) const {
    return ObjectFactory::getInstance().getAllTypesForContainer<Container>()[i]->getType();
  }

  template<class Container>
  MBXMLUtils::FQN WidgetFactoryFor<Container>::getXMLName(int i) const {
    return ObjectFactory::getInstance().getAllTypesForContainer<Container>()[i]->getXMLType();
  }

  template<class Container>
  int WidgetFactoryFor<Container>::getDefaultIndex() const {
    static int defaultIndex=-1;
    if(defaultIndex==-1) {
      defaultIndex=0;
      for(const auto *func : ObjectFactory::getInstance().getAllTypesForContainer<Container>()) {
        if(func->typeInfo==
           typeid(typename boost::mpl::at<ObjectFactory::MapContainerToDefaultAndUnknown, Container>::type::first))
          break;
        defaultIndex++;
      }
    }
    return defaultIndex;
  }

  template<class Container>
  int WidgetFactoryFor<Container>::getFallbackIndex() const {
    static int fallBackIndex=-1;
    if(fallBackIndex==-1) {
      fallBackIndex=0;
      for(const auto *func : ObjectFactory::getInstance().getAllTypesForContainer<Container>()) {
        if(func->typeInfo==
           typeid(typename boost::mpl::at<ObjectFactory::MapContainerToDefaultAndUnknown, Container>::type::second))
          break;
        fallBackIndex++;
      }
    }
    return fallBackIndex;
  }

  template<class Container>
  Widget* WidgetFactoryFor<Container>::createWidget(int i) {
    return static_cast<Container*>(ObjectFactory::getInstance().getAllTypesForContainer<Container>()[i]->ctor(e, pw));
  }

  template<class Container>
  int WidgetFactoryFor<Container>::getSize() const {
    return ObjectFactory::getInstance().getAllTypesForContainer<Container>().size();
  }

  // explizit instantiation
  #define X(Type) \
    template class WidgetFactoryFor<Type>;
  MBSIMGUI_WIDGET_CONTAINERS
  #undef X

}
