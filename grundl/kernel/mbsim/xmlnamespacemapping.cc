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

#include <mbsim/xmlnamespacemapping.h>
#include <mbsim/element.h>
#include <mbsim/integrators/integrator.h>
#include <mbxmlutilstinyxml/tinynamespace.h>
#include <set>

using namespace std;

namespace MBSim {

XMLNamespaceMapping::M_NSPRE XMLNamespaceMapping::getNamespacePrefixMapping() {
  // generate the namespace-prefix mapping considering the priority
  M_NSPRE nsprefix;
  set<string> prefix;
  for(MM_PRINSPRE::reverse_iterator i=instance().priorityNSPrefix.rbegin(); i!=instance().priorityNSPrefix.rend(); i++) {
    // insert only if the prefix does not already exist
    if(prefix.find(i->second.second)!=prefix.end())
      continue;
    // insert only if the namespace does not already exist
    pair<M_NSPRE::iterator, bool> ret=nsprefix.insert(i->second);
    if(ret.second)
      prefix.insert(i->second.second);
  }

  return nsprefix;
}

void XMLNamespaceMapping::registerXMLNamespaceMapping(double priority, const string &ns, const string &prefix) {
  instance().priorityNSPrefix.insert(make_pair(priority, make_pair(ns, prefix)));
}

XMLNamespaceMapping &XMLNamespaceMapping::instance() {
  static XMLNamespaceMapping nsmap;
  return nsmap;
}

RegisterXMLNamespaceMapping::RegisterXMLNamespaceMapping(double priority, const string &ns, const string &prefix) {
  XMLNamespaceMapping::registerXMLNamespaceMapping(priority, ns, prefix);
}



// Register the MBSim kernel XML namespace mappings

// mbsim namespace (very high priority for the default namespace)
MBSIM_REGISTER_XMLNAMESPACEMAPPING(100, MBSIMNS_, "")
MBSIM_REGISTER_XMLNAMESPACEMAPPING( 90, MBSIMNS_, "mbsim")
// mbsim-integrator namespace (very high priority for the default namespace)
MBSIM_REGISTER_XMLNAMESPACEMAPPING( 90, MBSIMINTNS_, "")
MBSIM_REGISTER_XMLNAMESPACEMAPPING( 80, MBSIMINTNS_, "mbsimint")
// XInclude
MBSIM_REGISTER_XMLNAMESPACEMAPPING(  2, XINCLUDENS_, "xi")
MBSIM_REGISTER_XMLNAMESPACEMAPPING(  1, XINCLUDENS_, "xinclude")
// MBXMLUtils
MBSIM_REGISTER_XMLNAMESPACEMAPPING( 50, "http://openmbv.berlios.de/MBXMLUtils/physicalvariable", "pv")
MBSIM_REGISTER_XMLNAMESPACEMAPPING( 50, "http://openmbv.berlios.de/MBXMLUtils/parameter",        "param")
#ifdef HAVE_OPENMBVCPPINTERFACE
// OpenMBV
MBSIM_REGISTER_XMLNAMESPACEMAPPING( 80, OPENMBVNS_, "")
MBSIM_REGISTER_XMLNAMESPACEMAPPING( 70, OPENMBVNS_, "ombv")
MBSIM_REGISTER_XMLNAMESPACEMAPPING( 60, OPENMBVNS_, "openmbv")
#endif

}
