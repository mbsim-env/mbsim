#include "../config.h"
#include <fmiinstance.h>
#include <mbsim/dynamic_system_solver.h>

#include <mbsimxml/mbsimflatxml.h>
#include <mbsim/objectfactory.h>

using namespace std;
using namespace MBSim;
using namespace boost::filesystem;

namespace MBSimFMI {

  void FMIInstance::addModelParametersAndCreateDSS(vector<std::shared_ptr<Variable> > &varSim) {
    // get the model file
    path mbsimflatxmlfile=path(MBXMLUtils::getFMUSharedLibPath()).parent_path().parent_path().parent_path()/"model"/"Model.mbsimprj.flat.xml";

    // load all MBSim modules
    msg(Debug)<<"Load MBSim modules."<<endl;
    MBSimXML::loadModules();
  
    // load MBSim project XML document
    msg(Debug)<<"Read MBSim flat XML model file."<<endl;
    std::shared_ptr<xercesc::DOMDocument> doc=parser->parse(mbsimflatxmlfile);
  
    // create object for DynamicSystemSolver
    msg(Debug)<<"Create DynamicSystemSolver."<<endl;
    dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(
      MBXMLUtils::E(doc->getDocumentElement())->getFirstElementChildNamed(MBSIM%"DynamicSystemSolver")));
  }

}
