#include "config.h"
#include <mbsimxml.h>
#include <vector>
#include <mbxmlutilshelper/dom.h>
#include <mbxmlutilshelper/getinstallpath.h>
#include <xercesc/dom/DOMDocument.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>

namespace bfs=boost::filesystem;
using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

set<bfs::path> getMBSimXMLSchemas() {
  bfs::path MBXMLUTILSSCHEMA=getInstallPath()/"share"/"mbxmlutils"/"schema";
  set<bfs::path> schemas {
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSimXML"/"mbsimproject.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSim"/"mbsim.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSimIntegrator"/"mbsimintegrator.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSimAnalyser"/"mbsimanalyser.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBXMLUtils"/"physicalvariable.xsd"
  };


  // create parser for plugin.xml
  static const NamespaceURI MBSIMPLUGIN("http://www.mbsim-env.de/MBSimPlugin");
  std::shared_ptr<DOMParser> parser;
  parser=DOMParser::create({MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSimPlugin"/"plugin.xsd"});

  // read plugin schemas
  for(auto it=bfs::directory_iterator(getInstallPath()/"share"/"mbsimxml"/"plugins"); it!=bfs::directory_iterator(); it++) {
    string path=it->path().string();
    if(path.length()<=string(".plugin.xml").length() || path.substr(path.length()-string(".plugin.xml").length())!=".plugin.xml")
      continue;
    std::shared_ptr<xercesc::DOMDocument> doc=parser->parse(*it);
    for(xercesc::DOMElement *e=E(doc->getDocumentElement())->getFirstElementChildNamed(MBSIMPLUGIN%"schemas")->getFirstElementChild();
        e!=NULL; e=e->getNextElementSibling()) {
      bfs::path xsdFile;
      if(E(e)->getTagName()==MBSIMPLUGIN%"File") {
        string location=E(e)->getAttribute("location");
        boost::algorithm::replace_all(location, "@MBSIMSCHEMADIR@", MBXMLUTILSSCHEMA.string());
        xsdFile=location;
      }
      schemas.insert(xsdFile);
    }
  }

  return schemas;
}

}
