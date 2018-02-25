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
#include <boost/algorithm/string/replace.hpp>

using namespace std;
using namespace fmatvec;
using namespace MBXMLUtils;

namespace MBSim {

void DOMEvalExceptionStack::add(const string &type, const std::shared_ptr<DOMEvalException> &ex) {
  exVec.emplace_back(type, ex);
}

namespace {
  size_t calculateMaxDepth(const vector<pair<string, std::shared_ptr<DOMEvalException> > > &exVec, size_t depth=0) {
    size_t maxDepth=depth;
    for(const auto & it : exVec) {
      std::shared_ptr<DOMEvalExceptionStack> stack=std::dynamic_pointer_cast<DOMEvalExceptionStack>(it.second);
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
    if(nr==1)
      str<<"Created by object of type "<<type;
    else
      str<<"Created by object of type "<<type<<" being the "<<
           nr<<(nr==2?"nd":nr==3?"rd":"th")<<" of "<<size<<" possible object types.";
    return str.str();
  }
}

vector<pair<string, std::shared_ptr<DOMEvalException> > > &DOMEvalExceptionStack::getExceptionVector() {
  return exVec;
}

const char* DOMEvalExceptionStack::what() const noexcept {
  stringstream str;
  generateWhat(str, string(2*calculateMaxDepth(exVec), ' '));
  whatStr=str.str();
  if(whatStr.length()>0)
    whatStr.resize(whatStr.length()-1); // remote the trailing line feed
  return whatStr.c_str();
}

void DOMEvalExceptionStack::generateWhat(std::stringstream &str, const std::string &indent) const {
  int nr=1;
  for(auto it=exVec.begin(); it!=exVec.end(); ++it, ++nr) {
    std::shared_ptr<DOMEvalExceptionStack> stack=std::dynamic_pointer_cast<DOMEvalExceptionStack>(it->second);
    if(stack) {
      stack->generateWhat(str, indent.substr(0, indent.length()-2));
      stack->setMessage(possibleType(nr, exVec.size(), it->first));
      stack->setSubsequentError(true);
      str<<indent<<boost::replace_all_copy(string(stack->DOMEvalException::what()), "\n", "\n"+indent)<<endl;
    }
  }
  nr=1;
  bool printNotCastableObjects=getenv("MBSIM_PRINT_NOT_CASTABLE")?true:false;
  int notPrintedWrongTypeErrors=0;
  for(auto it=exVec.begin(); it!=exVec.end(); ++it, ++nr) {
    std::shared_ptr<DOMEvalExceptionStack> stack=std::dynamic_pointer_cast<DOMEvalExceptionStack>(it->second);
    if(!stack) {
      std::shared_ptr<DOMEvalExceptionWrongType> wrongType=std::dynamic_pointer_cast<DOMEvalExceptionWrongType>(it->second);
      string errorMsg;
      if(wrongType) {
        if(printNotCastableObjects)
          errorMsg="Not castable to type "+it->second->getMessage();
        else
          notPrintedWrongTypeErrors++;
      }
      else
        errorMsg=it->second->getMessage();
      if(!wrongType || printNotCastableObjects) {
        it->second->setMessage(errorMsg+"\n"+
                               indent+possibleType(nr, exVec.size(), it->first));
        str<<indent<<boost::replace_all_copy(string(it->second->what()), "\n", "\n"+indent)<<endl;
      }
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

void registerClass_internal(const FQN &name, const AllocateBase *alloc, const DeallocateBase *dealloc) {
  ObjectFactory::AllocDeallocVector &allocDealloc=ObjectFactory::instance().registeredType.insert(make_pair(name, ObjectFactory::AllocDeallocVector())).first->second;
  if(find_if(allocDealloc.begin(), allocDealloc.end(), [&alloc](const ObjectFactory::AllocDeallocPair &x){
    return *x.first == *alloc;
  })!=allocDealloc.end())
    throw MBSimError("Internal error: Redundant registration of a class in the XML object factory.");
  allocDealloc.emplace_back(alloc, dealloc);
}

void deregisterClass_internal(const FQN &name, const AllocateBase *alloc) {
  auto nameIt=ObjectFactory::instance().registeredType.find(name);
  if(nameIt==ObjectFactory::instance().registeredType.end())
    return;
  auto allocDeallocIt=nameIt->second.begin();
  while(allocDeallocIt!=nameIt->second.end() && !(*allocDeallocIt->first == *alloc))
    ++allocDeallocIt;
  if(allocDeallocIt!=nameIt->second.end()) {
    delete allocDeallocIt->first;
    delete allocDeallocIt->second;
    nameIt->second.erase(allocDeallocIt);
  }
  if(nameIt->second.empty())
    ObjectFactory::instance().registeredType.erase(nameIt);
  delete alloc;
}

std::string fixXMLLocalName(std::string name) {
  name=name.substr(0, name.find('<'));
  size_t c=name.rfind(':');
  if(c!=std::string::npos)
    return name.substr(c+1);
  return name;
}

}
