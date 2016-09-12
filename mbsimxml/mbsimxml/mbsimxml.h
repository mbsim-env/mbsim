#ifndef _MBSIMXML_MBSIMXML_H_
#define _MBSIMXML_MBSIMXML_H_

#include <boost/filesystem.hpp>
#include <mbxmlutilshelper/dom.h>

namespace MBSim {

  std::set<boost::filesystem::path> getMBSimXMLSchemas();

}

#endif
