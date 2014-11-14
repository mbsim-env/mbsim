#include <mbsimxml.h>
#include <vector>
#include <mbxmlutilshelper/dom.h>
#include <mbxmlutilshelper/getinstallpath.h>
#include <xercesc/dom/DOMDocument.hpp>
#include <boost/filesystem/fstream.hpp>

namespace bfs=boost::filesystem;
using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;

namespace MBSim {

void generateMBSimXMLSchema(const bfs::path &mbsimxml_xsd, const bfs::path &MBXMLUTILSSCHEMA) {
  vector<pair<string, string> > schema; // pair<namespace, schemaLocation>

  static const NamespaceURI MBSIMPLUGIN("http://mbsim.berlios.de/MBSimPlugin");
  static boost::shared_ptr<DOMParser> parser;
  if(!parser) {
    parser=DOMParser::create(true);
    parser->loadGrammar(getInstallPath()/"share"/"mbxmlutils"/"schema"/"http___mbsim_berlios_de_MBSimPlugin"/"plugin.xsd");
  }

  // read plugin schemas
  string ns, loc;
  for(bfs::directory_iterator it=bfs::directory_iterator(getInstallPath()/"share"/"mbsimxml"/"plugins"); it!=bfs::directory_iterator(); it++) {
    if(it->path().string().substr(it->path().string().length()-string(".plugin.xml").length())!=".plugin.xml") continue;
    boost::shared_ptr<xercesc::DOMDocument> doc=parser->parse(*it);
    for(xercesc::DOMElement *e=E(E(doc->getDocumentElement())->getFirstElementChildNamed(MBSIMPLUGIN%"schemas"))->
        getFirstElementChildNamed(MBSIMPLUGIN%"Schema");
        e!=NULL; e=e->getNextElementSibling())
      schema.push_back(make_pair(E(e)->getAttribute("namespace"), E(e)->getAttribute("schemaLocation")));
  }

  // write MBSimXML schema
  bfs::ofstream file(mbsimxml_xsd);
  file<<"<?xml version=\"1.0\" encoding=\"UTF-8\"?>"<<endl;
  file<<"<xs:schema targetNamespace=\"http://mbsim.berlios.de/MBSimXML\""<<endl;
  file<<"  elementFormDefault=\"qualified\""<<endl;
  file<<"  attributeFormDefault=\"unqualified\""<<endl;
  file<<"  xmlns=\"http://mbsim.berlios.de/MBSimXML\""<<endl;
  file<<"  xmlns:xs=\"http://www.w3.org/2001/XMLSchema\">"<<endl;
  file<<endl;
  // include the schema for MBSimProject (this imports the MBSim and MBSimIntegrator schemas)
  file<<"  <xs:include schemaLocation=\""<<MBXMLUTILSSCHEMA.generic_string()<<"/http___mbsim_berlios_de_MBSimXML/mbsimproject.xsd\"/>"<<endl;
  file<<endl;
  // import all schemas from mbsim modules (plugins)
  for(vector<pair<string, string> >::iterator it=schema.begin(); it!=schema.end(); it++) {
    file<<"  <xs:import namespace=\""<<it->first<<"\""<<endl;
    file<<"             schemaLocation=\""<<MBXMLUTILSSCHEMA.generic_string()<<"/"<<it->second<<"\"/>"<<endl;
  }
  file<<"</xs:schema>"<<endl;
}

}
