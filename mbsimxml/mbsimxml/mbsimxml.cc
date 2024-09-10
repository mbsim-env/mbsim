#include "config.h"
#include <mbxmlutils/pycppwrapper.h>
#include <mbsimxml.h>
#include <vector>
#include <mbxmlutilshelper/dom.h>
#include <mbxmlutilshelper/thislinelocation.h>
#include <xercesc/dom/DOMDocument.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/algorithm/string.hpp>

namespace bfs=boost::filesystem;
using namespace std;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace PythonCpp;

namespace {

ThisLineLocation loc;

boost::filesystem::path installPath() {
  return boost::filesystem::canonical(loc()).parent_path().parent_path();
}

void initPython() {
  static bool isInitialized=false;
  if(isInitialized)
    return;

#ifdef _WIN32
    string binLib("bin");
#else
    string binLib("lib");
#endif

  // init python
  initializePython(installPath()/"bin"/"mbsimxml", PYTHON_VERSION, {
    // prepand the installation/../mbsim-env-python-site-packages dir to the python path (Python pip of mbsim-env is configured to install user defined python packages there)
    installPath().parent_path()/"mbsim-env-python-site-packages",
  }, {
    // append the installation/bin dir to the python path (SWIG generated python modules (e.g. OpenMBV.py) are located there)
    installPath()/"bin",
  }, {
    installPath(),
    boost::filesystem::path(PYTHON_PREFIX),
  }, {
    // append to PATH (on Windows using os.add_dll_directory)
    installPath().parent_path()/"mbsim-env-python-site-packages"/binLib,
  });

  isInitialized=true;
}

}

namespace MBSim {

shared_ptr<DOMDocument> getMBSimXMLCatalog(const set<bfs::path> &searchDirs) {
  bfs::path MBXMLUTILSSCHEMA=installPath()/"share"/"mbxmlutils"/"schema";
  set<bfs::path> schemas {
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSimXML"/"mbsimproject.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_OpenMBV"/"openmbv.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSim"/"mbsim.xsd",
    MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBXMLUtils"/"mbxmlutils.xsd",
  };

  // create parser for mbsimmodule.xml files
  static const NamespaceURI MBSIMMODULE("http://www.mbsim-env.de/MBSimModule");
  std::shared_ptr<DOMParser> parser;
  parser=DOMParser::create({MBXMLUTILSSCHEMA/"http___www_mbsim-env_de_MBSimModule"/"mbsimmoduleCatalog.xml"});

  set<bfs::path> allSearchDirs=searchDirs;
  allSearchDirs.insert(installPath()/"share"/"mbsimmodules");
  allSearchDirs.insert(bfs::current_path());
  // add directories from configuration file
#ifdef _WIN32
  boost::filesystem::path modulePathConfigFile(
    boost::filesystem::path(getenv("APPDATA")?getenv("APPDATA"):"")/"mbsim-env"/"mbsimxml.modulepath");
#else
  boost::filesystem::path modulePathConfigFile(
    boost::filesystem::path(getenv("HOME")?getenv("HOME"):"")/".config"/"mbsim-env"/"mbsimxml.modulepath");
#endif
  if(boost::filesystem::exists(modulePathConfigFile)) {
    boost::filesystem::ifstream modulePathConfig(modulePathConfigFile);
    for(string line; getline(modulePathConfig, line);)
      allSearchDirs.insert(line);
  }

  // read MBSim module schemas
  enum Stage { SearchPath, Loading }; // we load in two stages: first just add all search path then do the real load
  for(auto stage: {SearchPath, Loading})
    for(auto &dir: allSearchDirs) {
      if(stage==SearchPath)
        fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<"Searching for MBSimXML plugins in directory: "<<dir<<endl;
      for(auto it=bfs::directory_iterator(dir); it!=bfs::directory_iterator(); it++) {
        string path=it->path().string();
        if(path.length()<=string(".mbsimmodule.xml").length()) continue;
        if(path.substr(path.length()-string(".mbsimmodule.xml").length())!=".mbsimmodule.xml") continue;
        if(stage==SearchPath)
          fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<" - load XSD for "<<it->path().filename().string()<<endl;
        std::shared_ptr<DOMDocument> doc=parser->parse(*it, nullptr, false);
        if(E(doc->getDocumentElement())->getTagName()!=MBSIMMODULE%"MBSimModule")
          throw runtime_error("The root element of a MBSim module XML file must be {"+MBSIMMODULE.getNamespaceURI()+"}MBSimModule");
        for(xercesc::DOMElement *e=E(doc->getDocumentElement())->getFirstElementChildNamed(MBSIMMODULE%"schemas")->getFirstElementChild();
            e!=nullptr; e=e->getNextElementSibling()) {
          if(stage==Loading && E(e)->getTagName()==MBSIMMODULE%"File") {
            string location=E(e)->getAttribute("location");
            if(location.substr(0, 17)=="@MBSIMSCHEMADIR@/")
              schemas.insert(MBXMLUTILSSCHEMA/location.substr(17));
            else
              schemas.insert(E(e)->convertPath(location));
          }
          if(E(e)->getTagName()==MBSIMMODULE%"PythonGenerated") {
            string moduleName=E(e)->getAttribute("moduleName");
            initPython();
            boost::filesystem::path location=E(e)->convertPath(E(e)->getAttribute("location"));
            if(stage==SearchPath) {
              // add python path
              PyO pyPath(CALLPYB(PySys_GetObject, const_cast<char*>("path")));
              PyO pyBinPath(CALLPY(PyUnicode_FromString, location.string()));
              CALLPY(PyList_Append, pyPath, pyBinPath);
            }
            if(stage==Loading) {
              // load python module
              PyO pyModule(CALLPY(PyImport_ImportModule, moduleName));
              // get python function and call it
              PyO pyGenerateXMLSchemaFile(CALLPY(PyObject_GetAttrString, pyModule, "generateXMLSchemaFile"));
              PyO pySchema(CALLPY(PyObject_CallObject, pyGenerateXMLSchemaFile, nullptr));
              // write to file
              PyO pyET(CALLPY(PyImport_ImportModule, "xml.etree.cElementTree"));
              PyO pyET_(CALLPY(PyObject_GetAttrString, pyET, "ElementTree"));
              PyO pyTree(CALLPY(PyObject_CallObject, pyET_, Py_BuildValue_("(O)", pySchema.get())));
              PyO pyWrite(CALLPY(PyObject_GetAttrString, pyTree, "write"));
              bfs::path xsdFile=bfs::current_path()/(".mbsimmodule.python."+moduleName+".xsd");
              CALLPY(PyObject_CallObject, pyWrite, Py_BuildValue_("(ssO)", xsdFile.string().c_str(), "UTF-8", Py_True));
              schemas.insert(xsdFile);
            }
          }
        }
      }
    }

  auto xmlCatalogDoc=DOMParser::create()->createDocument();
  auto catalogRoot=D(xmlCatalogDoc)->createElement(XMLCATALOG%"catalog");
  xmlCatalogDoc->appendChild(catalogRoot);
  shared_ptr<DOMParser> nonValParser=DOMParser::create();
  for(auto &schema: schemas) {
    shared_ptr<DOMDocument> doc=nonValParser->parse(schema);//MISSING use sax parser since we just need to parse one attribute of the root element
    string ns=E(doc->getDocumentElement())->getAttribute("targetNamespace");
    auto uri=D(xmlCatalogDoc)->createElement(XMLCATALOG%"uri");
    catalogRoot->appendChild(uri);
    E(uri)->setAttribute("name", ns);
    E(uri)->setAttribute("uri", schema.string());
  }

  return xmlCatalogDoc;
}

}
