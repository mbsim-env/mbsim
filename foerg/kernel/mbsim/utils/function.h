/* Copyright (C) 2004-2009 MBSim Development Team
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
 * Contact: martin.o.foerg@googlemail.com
 */

#ifndef FUNCTION_H_
#define FUNCTION_H_

#include <mbxmlutilstinyxml/tinyxml.h>
#include <mbsim/element.h>
#include <mbsim/utils/utils.h>
#include <mbsim/utils/eps.h>
#include <fmatvec/fmatvec.h>
#include <fmatvec/function.h>

namespace MBSim {

  template<typename Sig>
    class Function;

  template<typename Ret, typename Arg>
    class Function<Ret(Arg)> : public fmatvec::Function<Ret(Arg)> {
      public:
        virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
        virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { 
          MBXMLUtils::TiXmlElement *ele0=new MBXMLUtils::TiXmlElement(MBSIMNS+getType());
          parent->LinkEndChild(ele0);
          return ele0;
        }
        virtual std::string getType() const { return "Function1"; }
    };

  template<typename Ret, typename Arg1, typename Arg2>
    class Function<Ret(Arg1, Arg2)> : public fmatvec::Function<Ret(Arg1, Arg2)> {
      public:
        virtual void initializeUsingXML(MBXMLUtils::TiXmlElement *element) {}
        virtual MBXMLUtils::TiXmlElement* writeXMLFile(MBXMLUtils::TiXmlNode *parent) { 
          MBXMLUtils::TiXmlElement *ele0=new MBXMLUtils::TiXmlElement(MBSIMNS+getType());
          parent->LinkEndChild(ele0);
          return ele0;
        }
        virtual std::string getType() const { return "Function2"; }
    };

}

#endif /* FUNCTION_H_ */

