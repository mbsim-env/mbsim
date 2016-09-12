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

void generateMBSimXMLSchema(const bfs::path &mbsimxml_xsd, const bfs::path &MBXMLUTILSSCHEMA) {
  vector<pair<string, bfs::path> > schema; // pair<namespace, schemaLocation>

  static const NamespaceURI MBSIMPLUGIN("http://www.mbsim-env.de/MBSimPlugin");
  std::shared_ptr<DOMParser> parser;
  parser=DOMParser::create({getInstallPath()/"share"/"mbxmlutils"/"schema"/"http___www_mbsim-env_de_MBSimPlugin"/"plugin.xsd"});

  // read plugin schemas
  for(auto it=bfs::directory_iterator(getInstallPath()/"share"/"mbsimxml"/"plugins"); it!=bfs::directory_iterator(); it++) {
    string path=it->path().string();
    if(path.length()<=string(".plugin.xml").length() || path.substr(path.length()-string(".plugin.xml").length())!=".plugin.xml")
      continue;
    std::shared_ptr<xercesc::DOMDocument> doc=parser->parse(*it);
    for(xercesc::DOMElement *e=E(E(doc->getDocumentElement())->getFirstElementChildNamed(MBSIMPLUGIN%"schemas"))->
        getFirstElementChildNamed(MBSIMPLUGIN%"Schema");
        e!=NULL; e=e->getNextElementSibling()) {
      bfs::path xsdFile;
      xercesc::DOMElement *c=e->getFirstElementChild();
      if(E(c)->getTagName()==MBSIMPLUGIN%"file") {
        string location=E(c)->getAttribute("location");
        boost::algorithm::replace_all(location, "@MBSIMSCHEMADIR@", MBXMLUTILSSCHEMA.string());
        xsdFile=location;
      }
      schema.push_back(make_pair(E(e)->getAttribute("namespace"), xsdFile));
    }
  }

  // write MBSimXML schema
  bfs::ofstream file(mbsimxml_xsd);
  file<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>"<<endl;
  file<<"<xs:schema targetNamespace=\"http://www.mbsim-env.de/MBSimXML\""<<endl;
  file<<"  elementFormDefault=\"qualified\""<<endl;
  file<<"  attributeFormDefault=\"unqualified\""<<endl;
  file<<"  xmlns=\"http://www.mbsim-env.de/MBSimXML\""<<endl;
  file<<"  xmlns:xs=\"http://www.w3.org/2001/XMLSchema\">"<<endl;
  file<<endl;
  // include the schema for MBSimProject (this imports the MBSim and MBSimIntegrator schemas)
  file<<"  <xs:include schemaLocation=\""<<MBXMLUTILSSCHEMA.generic_string()<<"/http___www_mbsim-env_de_MBSimXML/mbsimproject.xsd\"/>"<<endl;
  file<<endl;
  // import all schemas from mbsim modules (plugins)
  for(auto it=schema.begin(); it!=schema.end(); it++) {
    file<<"  <xs:import namespace=\""<<it->first<<"\""<<endl;
    file<<"             schemaLocation=\""<<it->second.string()<<"\"/>"<<endl;
  }
  file<<"</xs:schema>"<<endl;
}

}
