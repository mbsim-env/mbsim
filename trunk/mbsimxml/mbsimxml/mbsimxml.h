#ifndef _MBSIMXML_MBSIMXML_H_
#define _MBSIMXML_MBSIMXML_H_

#include <boost/filesystem.hpp>

namespace MBSim {

  void generateMBSimXMLSchema(const boost::filesystem::path &mbsimxml_xsd, const boost::filesystem::path &MBXMLUTILSSCHEMA);

}

#endif
