/* Copyright (C) 2004-2014 MBSim Development Team
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
 * Contact: thorsten.schindler@mytum.de
 */

#include <config.h>
#ifdef _WIN32
#  include <windows.h>
#else
#  include <unistd.h>
#  include <dlfcn.h>
#endif
#include <iostream>
#include "fmi_utils.h"
#include <xercesc/dom/DOMDocument.hpp>
#include <mbxmlutilshelper/dom.h>
#include <mbsimxml/mbsimflatxml.h>
#include "mbsim/objectfactory.h"
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/integrators/integrator.h>
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/algorithm/string.hpp>

namespace fmi {

#ifndef _WIN32
// stores the current directory at the time this shared object was loaded
static char currentDirOnLoad[2048];

// initialize the currentDirOnLoad variable when loading the shared object (this is achived by the GCC constructor attribute)
__attribute__((constructor))
static void initCurrentDirOnLoad() {
  char* unused __attribute__((unused));
  unused = getcwd(currentDirOnLoad, 2048);
}
#endif

boost::filesystem::path getSharedLibDir() {
  // Get the file containing this function
#ifdef _WIN32
  wchar_t moduleName[2048];
  GetModuleFileNameW(GetModuleHandle("mbsim.dll"), moduleName, sizeof(moduleName));
  boost::filesystem::path dllPath(moduleName);
#else
  Dl_info info;
#ifdef __GNUC__
__extension__
#endif
  dladdr(reinterpret_cast<void*>(&getSharedLibDir), &info);
  boost::filesystem::path dllPath;
  if(info.dli_fname[0]=='/') // use an absolute path as it
     dllPath=info.dli_fname;
  else // prefix a relative path with the current path at the time this shared object was loaded.
       // This is required since dladdr returns the string which was used by dlopen to open the shared object
       // which may be a relative path which has to be interpreted relative to the current directory at the time the shared object was loaded.
     dllPath=boost::filesystem::path(currentDirOnLoad)/info.dli_fname;
#endif
  dllPath.remove_filename();
  return boost::filesystem::canonical(dllPath);
}

void initDssWithXml(std::string xmlpath, MBSim::DynamicSystemSolver **system, MBSimIntegrator::Integrator **integrator){

  using namespace MBXMLUtils;
  using namespace xercesc;
  using namespace MBSim;
  using namespace boost;

  // setup message streams
  static PrefixedStringBuf infoBuf("Info:    ", std::cout);
  static PrefixedStringBuf warnBuf("Warning: ", std::cerr);
  fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info, boost::make_shared<std::ostream>(&infoBuf));
  fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn, boost::make_shared<std::ostream>(&warnBuf));

  // load MBSim XML document
  boost::shared_ptr<DOMParser> parser=DOMParser::create(false);
  boost::shared_ptr<DOMDocument> doc=parser->parse(xmlpath.c_str());
  if(doc==false)
    throw MBSimError(std::string("ERROR! Unable to load file: ")+xmlpath);
  DOMElement *e=doc->getDocumentElement();

  // create object for root element and check correct type
  *system=ObjectFactory::createAndInit<DynamicSystemSolver>(e->getFirstElementChild());


//   (*system)->setInformationOutput(false);
//  (*system)->setPlotFeatureRecursive(plotRecursive,disabled);
}

void initIntegratorWithXml(std::string xmlpath, MBSimIntegrator::Integrator **integrator){
  using namespace MBXMLUtils;
  using namespace xercesc;
  using namespace MBSim;
  // load MBSim XML document
  boost::shared_ptr<DOMParser> parser=DOMParser::create(false);
  boost::shared_ptr<DOMDocument> doc=parser->parse(xmlpath.c_str());
  if(doc==false)
    throw MBSimError(std::string("ERROR! Unable to load file: ")+xmlpath);
  DOMElement *e=doc->getDocumentElement();

  // create object for root element and check correct type
  *integrator=ObjectFactory::createAndInit<MBSimIntegrator::Integrator>(e->getFirstElementChild()->getNextElementSibling());
}


const std::string getCurrentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%dT%XZ", &tstruct);

    return buf;
}

} // end namespace fmi


// Put the above code to the main FMU library MODELNAME.so.
// Call getSharedLibDir() to get the absolute path where the MODELNAME.so/.dll is stored
// (call getSharedLibDir().string() to get this path as a std::string value)
// Hence in a FMU which consists of the following structure
//
// modelDescription.xml
// bianries/linux64/MODELNAME.so
// resources/MBS.mbsim.flat.xml
//
// call
// getSharedLibDir().parent_path().parent_path()/"resources"/"MBS.mbsim.flat.xml (filename of type boost::filesystem::path)"
// or
// (getSharedLibDir().parent_path().parent_path()/"resources"/"MBS.mbsim.flat.xml).string()" (filename of type std::string)
// to get the MBSim flat XML file to load.
//
// As already discussed last time:
// Link MODELNAME.so with option -Wl,-rpath=$ORIGIN
// and put all depdendent MBSim libs also to binaries/linux64
// OR (for testing)
// export LD_LIBRARY_PATH=/path/to/mbsim/local/lib



