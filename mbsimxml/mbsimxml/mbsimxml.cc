#include "config.h"
#if MBSIMXML_COND_PYTHON
  #include <mbxmlutils/py2py3cppwrapper.h>
#endif
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
#if MBSIMXML_COND_PYTHON
  using namespace PythonCpp;
#endif

namespace MBSim {

set<bfs::path> getMBSimXMLSchemas(const set<bfs::path> &searchDirs) {
#if MBSIMXML_COND_PYTHON
  initializePython((getInstallPath()/"bin"/"mbsimxml").string());
#endif

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

  set<bfs::path> allSearchDirs=searchDirs;
  allSearchDirs.insert(getInstallPath()/"share"/"mbsimmodules");
  allSearchDirs.insert(bfs::current_path());

  // read MBSim module schemas
  for(auto &dir: {getInstallPath()/"share"/"mbsimmodules", boost::filesystem::current_path()})
    for(auto it=bfs::directory_iterator(dir); it!=bfs::directory_iterator(); it++) {
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
        if(E(e)->getTagName()==MBSIMMODULE%"PythonGenerated") {
          string moduleName=E(e)->getAttribute("moduleName");
#if MBSIMXML_COND_PYTHON
          bfs::path location=E(e)->getAttribute("location");
          if(!it->path().is_absolute())
            location=it->path().parent_path()/location;

          // add python path
          PyO pyPath=CALLPYB(PySys_GetObject, const_cast<char*>("path"));
          PyO pyBinPath=CALLPY(PyUnicode_FromString, (getInstallPath()/"bin").string());
          CALLPY(PyList_Append, pyPath, pyBinPath);
          for(auto &dir: allSearchDirs) {
            PyO pyCurPath=CALLPY(PyUnicode_FromString, dir.string());
            CALLPY(PyList_Append, pyPath, pyCurPath);
          }
          // load python module
          PyO pyModule=CALLPY(PyImport_ImportModule, moduleName);
          // get python function and call it
          PyO pyGenerateXMLSchemaFile=CALLPY(PyObject_GetAttrString, pyModule, "generateXMLSchemaFile");
          PyO pySchema=CALLPY(PyObject_CallObject, pyGenerateXMLSchemaFile, nullptr);
          // write to file
          PyO pyET=CALLPY(PyImport_ImportModule, "xml.etree.cElementTree");
          PyO pyET_=CALLPY(PyObject_GetAttrString, pyET, "ElementTree");
          PyO pyTree=CALLPY(PyObject_CallObject, pyET_, Py_BuildValue("(O)", pySchema.get()));
          PyO pyWrite=CALLPY(PyObject_GetAttrString, pyTree, "write");
          xsdFile=bfs::current_path()/(".mbsimmodule.python."+moduleName+".xsd");
          CALLPY(PyObject_CallObject, pyWrite, Py_BuildValue("(ssO)", xsdFile.string().c_str(), "UTF-8", Py_True));
#else
          fmatvec::Atom::msgStatic(fmatvec::Atom::Warn)<<
            "Python MBSim module found in "+path+" '"+moduleName+"'\n"<<
            "but MBSim is not build with Python support. Skipping this module.\n";
          continue;
#endif
        }
        schemas.insert(xsdFile);
      }
    }

  return schemas;
}

}
