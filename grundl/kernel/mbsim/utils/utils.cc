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

#include "config.h"
#include "mbsim/utils/utils.h"
#include "mbsim/utils/eps.h"
#if defined HAVE_LIBUNWIND_H && defined HAVE_LIBUNWIND_X86_64
#  include <libunwind.h>
#endif
#if defined HAVE_CXXABI_H
#  include <cxxabi.h>
#endif
#include "mbxmlutilstinyxml/tinynamespace.h"

using namespace std;
using namespace fmatvec;

std::string numtostr(int i){
  std::ostringstream oss;
  oss << i;
  return oss.str(); 
}

std::string numtostr(double d) {
  std::ostringstream oss;
  oss << d;
  return oss.str(); 
}

double degtorad(double alpha) {return alpha/180.*M_PI; }
double radtodeg(double phi) {return phi/M_PI*180.; }
fmatvec::Vec degtorad(fmatvec::Vec alpha) {return alpha/180.*M_PI; }
fmatvec::Vec radtodeg(fmatvec::Vec phi) {return phi/M_PI*180.; }

double sign(double x) {
  if(x>0)
    return 1.0;
  else if(x<0)
    return -1.0;
  else 
    return 0;
}

int min(int i, int j) {
  return i<j?i:j;
}  

Vec tildetovec(const SqrMat &A) {
  Vec x(3,NONINIT);
  x(0) = A(2,1);
  x(1) = A(0,2);
  x(2) = A(1,0);
  return x;
}

double ArcTan(double x, double y) {
  double phi;
  phi = atan2(y, x);

  if (phi < -MBSim::macheps())
    phi += 2 * M_PI;
  return phi;
}


set<vector<string> > Deprecated::allMessages;
bool Deprecated::atExitRegistred=false;

void Deprecated::registerMessage(const std::string &message, TiXmlElement *e) {
  if(!atExitRegistred) {
    atexit(&Deprecated::printAllMessages);
    atExitRegistred=true;
  }

  vector<string> stack;
  stack.push_back(message);
  if(e) {
    vector<string> out=TiXml_location_vec(e, "at ", "");
    stack.insert(stack.end(), out.begin(), out.end());
  }
  else {
#if defined HAVE_LIBUNWIND_H && defined HAVE_LIBUNWIND_X86_64
    unw_context_t context;
    unw_getcontext(&context);
    unw_cursor_t cp;
    unw_init_local(&cp, &context);
    unw_step(&cp);
    unw_word_t offp;
    char name[102400];
    char *demangledName=NULL;
    int status=1;
    int nr=0;
    do {
      unw_get_proc_name(&cp, name, 102400, &offp);
#if defined HAVE_CXXABI_H
      demangledName=abi::__cxa_demangle(name, NULL, NULL, &status);
#endif
      if(status==0)
        stack.push_back((nr==0?"at ":"by ")+string(demangledName));
      else
        stack.push_back((nr==0?"at ":"by ")+string(name));
      nr++;
    }
    while(unw_step(&cp)>0 && string(name)!="main");
    free(demangledName);
#else
    stack.push_back("(no stack trace available)");
#endif
  }
  allMessages.insert(stack);
}

void Deprecated::printAllMessages() {
  cerr<<endl;
  cerr<<"WARNING: "<<allMessages.size()<<" deprecated features were called during simulation:"<<endl;
  set<vector<string> >::const_iterator it;
  int nr=0;
  for(it=allMessages.begin(); it!=allMessages.end(); it++) {
    nr++;
    cerr<<"* "<<"("<<nr<<"/"<<allMessages.size()<<") "<<(*it)[0]<<endl;
    vector<string>::const_iterator it2=it->begin();
    it2++;
    for(; it2!=it->end(); it2++)
      cerr<<"  "<<*it2<<endl;
  }
}
