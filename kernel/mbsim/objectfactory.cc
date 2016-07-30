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

#include <config.h>
#include <mbsim/objectfactory.h>
#include <boost/algorithm/string.hpp>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;

namespace MBSim {

void DOMEvalExceptionStack::add(const string &type, const std::shared_ptr<DOMEvalException> &ex) {
  exVec.push_back(make_pair(type, ex));
}

namespace {
  size_t calculateMaxDepth(const vector<pair<string, std::shared_ptr<DOMEvalException> > > &exVec, size_t depth=0) {
    size_t maxDepth=depth;
    for(vector<pair<string, std::shared_ptr<DOMEvalException> > >::const_iterator it=exVec.begin(); it!=exVec.end(); ++it) {
      std::shared_ptr<DOMEvalExceptionStack> stack=std::dynamic_pointer_cast<DOMEvalExceptionStack>(it->second);
      if(stack) {
        size_t d=calculateMaxDepth(stack->getExceptionVector(), depth+1);
        if(d>maxDepth)
          maxDepth=d;
      }
    }
    return maxDepth;
  }

  string possibleType(int nr, int size, const string &type) {
    stringstream str;
    str<<nr<<(nr==1?"st":nr==2?"nd":nr==3?"rd":"th")<<" ("<<type<<") of "<<size<<" possible objects:";
    return str.str();
  }
}

vector<pair<string, std::shared_ptr<DOMEvalException> > > &DOMEvalExceptionStack::getExceptionVector() {
  return exVec;
}

const char* DOMEvalExceptionStack::what() const throw() {
  stringstream str;
  generateWhat(str, string(2*calculateMaxDepth(exVec), ' '));
  whatStr=str.str();
  if(whatStr.length()>0)
    whatStr.resize(whatStr.length()-1); // remote the trailing line feed
  return whatStr.c_str();
}

void DOMEvalExceptionStack::generateWhat(std::stringstream &str, const std::string &indent) const {
  int nr=1;
  for(vector<pair<string, std::shared_ptr<DOMEvalException> > >::const_iterator it=exVec.begin(); it!=exVec.end(); ++it, ++nr) {
    std::shared_ptr<DOMEvalExceptionStack> stack=std::dynamic_pointer_cast<DOMEvalExceptionStack>(it->second);
    if(stack) {
      stack->generateWhat(str, indent.substr(0, indent.length()-2));
      str<<indent<<"+++ Created by "<<possibleType(nr, exVec.size(), it->first)<<endl;
      DOMEvalException::locationStack2Stream(indent, stack->getLocationStack(), "", str);
    }
  }
  nr=1;
  bool printNotCastableObjects=getenv("MBSIM_PRINT_NOT_CASTABLE")?true:false;
  int notPrintedWrongTypeErrors=0;
  for(vector<pair<string, std::shared_ptr<DOMEvalException> > >::const_iterator it=exVec.begin(); it!=exVec.end(); ++it, ++nr) {
    std::shared_ptr<DOMEvalExceptionStack> stack=std::dynamic_pointer_cast<DOMEvalExceptionStack>(it->second);
    if(!stack) {
      std::shared_ptr<DOMEvalExceptionWrongType> wrongType=std::dynamic_pointer_cast<DOMEvalExceptionWrongType>(it->second);
      if(!wrongType || printNotCastableObjects)
        str<<indent<<"*** Error from "<<possibleType(nr, exVec.size(), it->first)<<endl;
      if(wrongType) {
        if(printNotCastableObjects)
          str<<indent<<"Not castable to type "<<it->second->getMessage()<<endl;
        else
          notPrintedWrongTypeErrors++;
      }
      else
        str<<it->second->getMessage()<<endl;
      if(!wrongType || printNotCastableObjects)
        DOMEvalException::locationStack2Stream(indent, it->second->getLocationStack(), "", str);
    }
  }
  if(notPrintedWrongTypeErrors>0)
    str<<indent<<"*** Not shown are "<<notPrintedWrongTypeErrors<<" of "<<exVec.size()<<" objects which are not castable."
       <<" Set envvar MBSIM_PRINT_NOT_CASTABLE to show these."<<endl;
}

ObjectFactory& ObjectFactory::instance() {
  static ObjectFactory of;
  return of;
}

void ObjectFactory::registerXMLName(const FQN &name, AllocateFkt alloc, DeallocateFkt dealloc) {
  AllocDeallocVector &allocDealloc=instance().registeredType.insert(make_pair(name, AllocDeallocVector())).first->second;
  if(find(allocDealloc.begin(), allocDealloc.end(), make_pair(alloc, dealloc))==allocDealloc.end())
    allocDealloc.push_back(make_pair(alloc, dealloc));
}

void ObjectFactory::deregisterXMLName(const FQN &name, AllocateFkt alloc) {
  NameMapIt nameIt=instance().registeredType.find(name);
  if(nameIt==instance().registeredType.end())
    return;
  AllocDeallocVectorIt allocDeallocIt=nameIt->second.begin();
  while(allocDeallocIt!=nameIt->second.end() && allocDeallocIt->first!=alloc)
    ++allocDeallocIt;
  if(allocDeallocIt!=nameIt->second.end())
    nameIt->second.erase(allocDeallocIt);
  if(nameIt->second.empty())
    instance().registeredType.erase(nameIt);
}

}
