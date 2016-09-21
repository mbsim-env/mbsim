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
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_OpenMBV"/"openmbv.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSim"/"mbsim.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSimIntegrator"/"mbsimintegrator.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSimAnalyser"/"mbsimanalyser.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBXMLUtils"/"physicalvariable.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBXMLUtils_CasADi"/"casadi.xsd"
  };

  // create parser for mbsimmodule.xml files
  static const NamespaceURI MBSIMMODULE("http://www.mbsim-env.de/MBSimModule");
  std::shared_ptr<DOMParser> parser;
  parser=DOMParser::create({MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSimModule"/"mbsimmodule.xsd"});

  // read MBSim module schemas
  for(auto it=bfs::directory_iterator(getInstallPath()/"share"/"mbsimmodules"); it!=bfs::directory_iterator(); it++) {
    string path=it->path().string();
    if(path.length()<=string(".mbsimmodule.xml").length() || path.substr(path.length()-string(".mbsimmodule.xml").length())!=".mbsimmodule.xml")
      continue;
    std::shared_ptr<xercesc::DOMDocument> doc=parser->parse(*it);
    for(xercesc::DOMElement *e=E(doc->getDocumentElement())->getFirstElementChildNamed(MBSIMMODULE%"schemas")->getFirstElementChild();
        e!=NULL; e=e->getNextElementSibling()) {
      bfs::path xsdFile;
      if(E(e)->getTagName()==MBSIMMODULE%"File") {
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
