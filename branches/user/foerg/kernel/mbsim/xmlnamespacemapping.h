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

#ifndef _MBSIM_XMLNAMESPACEMAPPING_H_
#define _MBSIM_XMLNAMESPACEMAPPING_H_

#include <string>
#include <map>

namespace MBSim {

/** XML namespace <-> XML namespace prefix mapping */
class XMLNamespaceMapping {
  public:

    typedef std::map<std::string, std::string> M_NSPRE;

    /** Return the XML prefix <-> XML namespace mapping */
    static M_NSPRE getNamespacePrefixMapping();

    /** Do not use this function directly use MBSIM_REGISTER_XMLNAMESPACEMAPPING instead */
    static void registerXMLNamespaceMapping(double priority, const std::string &ns, const std::string &prefix);

  private:

    typedef std::pair<std::string, std::string> P_NSPRE;
    typedef std::multimap<double, P_NSPRE> MM_PRINSPRE;
    typedef std::pair<double, P_NSPRE> P_PRINSPRE;

    XMLNamespaceMapping() {}

    static XMLNamespaceMapping &instance();

    MM_PRINSPRE priorityNSPrefix;
};

/** Register a new priority based mapping.
 * Do not use this class directly, use MBSIM_REGISTER_XMLNAMESPACEMAPPING. */
class RegisterXMLNamespaceMapping {
  public:

    RegisterXMLNamespaceMapping(double priority, const std::string &ns, const std::string &prefix);

};

#define MBSIM_XMLNAMESPACEMAPPING_CONCAT1(X, Y) X##Y
#define MBSIM_XMLNAMESPACEMAPPING_CONCAT(X, Y) MBSIM_XMLNAMESPACEMAPPING_CONCAT1(X, Y)
#define MBSIM_XMLNAMESPACEMAPPING_APPENDLINE(X) MBSIM_XMLNAMESPACEMAPPING_CONCAT(X, __LINE__)

/** Register a priority based mapping for XML ns to XML prefix */
#define MBSIM_REGISTER_XMLNAMESPACEMAPPING(priority, ns, prefix) \
  static MBSim::RegisterXMLNamespaceMapping \
    MBSIM_XMLNAMESPACEMAPPING_APPENDLINE(registerXMLNamespaceMappingDummyVariable)(priority, ns, prefix);

}

#endif
