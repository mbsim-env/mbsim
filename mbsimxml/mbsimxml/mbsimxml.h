#ifndef _MBSIMXML_MBSIMXML_H_
#define _MBSIMXML_MBSIMXML_H_

#include <boost/filesystem.hpp>
#include <mbxmlutilshelper/dom.h>

namespace MBSim {

  std::shared_ptr<xercesc::DOMDocument> getMBSimXMLCatalog(const std::set<boost::filesystem::path> &searchDirs={}, bool printPluginSearch = false); //MISSING remove ={} if mbsimfmi supports searchDirs

}

#endif
