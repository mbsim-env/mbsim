#ifdef _WIN32
#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#endif
#include "config.h"
#include <clocale>
#include <cfenv>
#include <cassert>
#include <mbxmlutils/eval.h>
#include <mbsimxml/mbsimflatxml.h>
#include <boost/dll.hpp>
#include <mbxmlutilshelper/shared_library.h>
#include <mbxmlutilshelper/windows_signal_conversion.h>
#include <mbsim/objectfactory.h>
#include <mbsim/dynamic_system_solver.h>
#include <mbsim/integrators/integrator.h>
#include <mbsimxml/mbsimxml.h>
#include <openmbvcppinterface/objectfactory.h>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include "zip.h"
#include "../../mbsimxml/mbsimxml/set_current_path.h"
#include <../general/fmi_variables.h>
#include <../general/xmlpp_utils.h>
#include <../general/mbsimsrc_fmi.h>

using namespace std;
using namespace boost::filesystem;
using namespace MBSim;
using namespace MBXMLUtils;
using namespace xercesc;
using namespace MBSimFMI;

namespace {
  // some platform dependent values
#ifdef _WIN32
  string USERENVVAR("USERNAME");
  path LIBDIR="bin";
#else
  string USERENVVAR("USER");
  path LIBDIR="lib";
#endif

  void copyShLibToFMU(const std::shared_ptr<DOMParser> &parser,
                      CreateZip &fmuFile, const path &dst, const path &depdstdir, const path &src);
}

int main(int argc, char *argv[]) {
#ifdef _WIN32
  SetConsoleCP(CP_UTF8);
  SetConsoleOutputCP(CP_UTF8);
#endif
  MBXMLUtils::handleFPE();
  setlocale(LC_ALL, "C");
  MBXMLUtils::convertWMCLOSEtoSIGTERM();

  try {
    // check for errors during ObjectFactory
    string errorMsg(OpenMBV::ObjectFactory::getAndClearErrorMsg());
    if(!errorMsg.empty()) {
      cerr<<"The following errors occured during the pre-main code of the OpenMBVC++Interface object factory:"<<endl;
      cerr<<errorMsg;
      cerr<<"Exiting now."<<endl;
      return 1;
    }

    // check for errors during ObjectFactory
    string errorMsg2(ObjectFactory::getAndClearErrorMsg());
    if(!errorMsg2.empty()) {
      cerr<<"The following errors occured during the pre-main code of the MBSim object factory:"<<endl;
      cerr<<errorMsg2;
      cerr<<"Exiting now."<<endl;
      return 1;
    }

    boost::filesystem::path installPath(boost::dll::program_location().parent_path().parent_path());

    // convert args to c++
    list<string> args;
    for(int i=1; i<argc; i++)
      args.emplace_back(argv[i]);

    // help
    if(argc<2 || find(args.begin(), args.end(), "-h")!=args.end() || find(args.begin(), args.end(), "--help")!=args.end()) {
      cout<<"Usage: "<<argv[0]<<" [-h|--help] [--nocompress] [--cosim] [--param <name> [--param <name> ...]]"<<endl;
      cout<<"  [-C <dir/file>|--CC] <MBSim Project XML File>|<MBSim FMI shared library>"<<endl;
      cout<<endl;
      cout<<"Create Functional Mock-Up Unit (FMI/FMU 1.0)."<<endl;
      cout<<"Creates mbsim.fmu in the current directory."<<endl;
      cout<<endl;
      cout<<"<MBSim Project XML File>    Create a FMU from XML project file"<<endl;
      cout<<"                            Must be the last argument!"<<endl;
      cout<<"<MBSim FMI shared library>  Create a FMU from shared library"<<endl;
      cout<<"                            Must be the last argument!"<<endl;
      cout<<"--param <name>              <name> defines a parameter name of"<<endl;
      cout<<"                            the root Embed element of <MBSim Project XML File> which should be"<<endl;
      cout<<"                            provided as an FMU parameter. Note that only parameters which do not depend"<<endl;
      cout<<"                            on any other parameters are allowed. If no --param option is given all are exported."<<endl;
      cout<<"                            Only for XML project files."<<endl;
      cout<<"--noparam                   Ignore all parameters defined the DynamicSystemSolver Embed element."<<endl;
      cout<<"                            Only for XML project files."<<endl;
      cout<<"--nocompress                Zip FMU without compression."<<endl;
      cout<<"--cosim                     Generate a FMI for Cosimulation FMU instead of a FMI for Model-Exchange FMU."<<endl;
      cout<<"-C <dir/file>               Change current to dir to <dir>/dir of <file> first."<<endl;
      cout<<"                            All arguments are still relative to the original current dir."<<endl;
      cout<<"--CC                        Change current dir to dir of <mbsimprjfile> first."<<endl;
      cout<<"                            All arguments are still relative to the original current dir."<<endl;
      return 0;
    }

    // current directory and adapt paths
    path inputFilename=args.back();
    boost::filesystem::path newCurrentPath;
    if(auto i=std::find(args.begin(), args.end(), "--CC"); i!=args.end()) {
      newCurrentPath=inputFilename.parent_path();
      args.erase(i);
    }
    if(auto i=std::find(args.begin(), args.end(), "-C"); i!=args.end()) {
      auto i2=i; i2++;
      if(boost::filesystem::is_directory(*i2))
        newCurrentPath=*i2;
      else
        newCurrentPath=boost::filesystem::path(*i2).parent_path();
      args.erase(i);
      args.erase(i2);
    }
    SetCurrentPath currentPath(newCurrentPath);
    inputFilename=currentPath.adaptPath(inputFilename);

    // get parameters
    set<string> useParam;
    decltype(args)::iterator i;
    while((i=std::find(args.begin(), args.end(), "--param")) != args.end()) {
      auto i2=i; i2++;
      useParam.insert(*i2);
      args.erase(i);
      args.erase(i2);
    }
    bool compress=true;
    if(auto i=std::find(args.begin(), args.end(), "--nocompress"); i!=args.end()) {
      compress=false;
      args.erase(i);
    }
    bool noParam=false;
    if(auto i=std::find(args.begin(), args.end(), "--noparam"); i!=args.end()) {
      noParam=true;
      args.erase(i);
    }
    bool cosim=false;
    if(auto i=std::find(args.begin(), args.end(), "--cosim"); i!=args.end()) {
      cosim=true;
      args.erase(i);
    }

    // get model file
    if(args.size()!=1) {
      cout<<"Wrong command line."<<endl;
      return 1;
    }

    // xml or shared library
    bool xmlFile=false;
    if(boost::iequals(inputFilename.extension().string(), ".mbsx"))
      xmlFile=true;

    // create FMU zip file
    CreateZip fmuFile("mbsim.fmu", compress);

    // load all MBSim modules
    cout<<"Load MBSim modules."<<endl;
    set<path> moduleLibs=MBSimXML::loadModules();
    // check for errors during ObjectFactory
    string errorMsg3(ObjectFactory::getAndClearErrorMsg());
    if(!errorMsg3.empty()) {
      cerr<<"The following errors occured during the loading of MBSim modules object factory:"<<endl;
      cerr<<errorMsg3;
      cerr<<"Exiting now."<<endl;
      return 1;
    }

    // create parser (none validating parser for use in copyShLibToFMU)
    std::shared_ptr<DOMParser> parserNoneVali=DOMParser::create();

    // note: the order of these variable definitions is importend for proper deallocation
    std::shared_ptr<DynamicSystemSolver> dss;
    std::shared_ptr<Integrator> integrator;

    vector<path> dependencies;

    PredefinedParameterStruct predefinedParameterStruct;
    cout<<"Create predefined parameters."<<endl;
    vector<std::shared_ptr<Variable> > var;
    addPredefinedParameters(cosim, var, predefinedParameterStruct, true);

    vector<std::shared_ptr<Variable> > xmlParam;

    string evalName="octave"; // default evaluator
    std::shared_ptr<Eval> eval;

    // Create dss from XML file
    if(xmlFile) {
      cout<<"Create MBSimXML XML schema including all modules."<<endl;
      auto xmlCatalog=getMBSimXMLCatalog();

      // preprocess XML file
      cout<<"Preprocess XML project file."<<endl;
      Preprocess preprocess(inputFilename, xmlCatalog->getDocumentElement(), true);
      auto modelDoc = preprocess.processAndGetDocument();
      auto modelEle = modelDoc->getDocumentElement();
      dependencies = preprocess.getDependencies();
      eval = preprocess.getEvaluator();
      evalName = eval->getName();
      auto param = preprocess.getParam();

      // convert the parameter list from the mbxmlutils preprocessor to a Variable vector
      convertParamSetToVariable(param, xmlParam, eval);
      // remove all variables which are not in useParam
      vector<std::shared_ptr<Variable> > xmlParam2;
      for(auto & it : xmlParam) {
        string name=it->getName();
        size_t pos=name.find('[');
        if(pos!=string::npos)
          name=name.substr(0, pos);
        if(!noParam && (useParam.size()==0 || useParam.find(name)!=useParam.end())) {
          fmatvec::Atom::msgStatic(fmatvec::Atom::Info)<<"Using MBSimProject parameter '"<<name<<"'."<<endl;
          useParam.erase(name);
          xmlParam2.push_back(it);
        }
      }
      for(auto &p : useParam)
        fmatvec::Atom::msgStatic(fmatvec::Atom::Warn) << "No parameter '" << p << "' found in MBSimProject. Cannot add this parameter to the FMU." <<endl;
      xmlParam=xmlParam2;
      cout<<"Create model parameters."<<endl;
      var.insert(var.end(), xmlParam.begin(), xmlParam.end());

      if(xmlParam.empty()) {
        cout<<"Copy preprocessed XML model file to FMU."<<endl;
        // no parameters -> save preprocessed model to FMU
        string ppModelStr;
        // serialize to string: this automatically normalized modelEle
        DOMParser::serializeToString(modelEle, ppModelStr);
        fmuFile.add(path("resources")/"model"/"Model.flat.mbsx", ppModelStr);
      }
      else {
        cout<<"Copy original XML model file to FMU."<<endl;
        // parameters existing -> save original XML file to FMU including all dependencies
        // Note: We copy the XML model file including all dependencies to resources/model. However the full absolute path
        // (excluding the leading '/' or 'c:\') of each file is added as subdir of resources/model. This will
        // enable XML models with file references to all parent directories. Hence only absolute file references
        // are not allowed!

        // path to XML project file relative to resources/model
        fmuFile.add(path("resources")/"model"/"XMLProjectFile.txt", absolute(inputFilename).relative_path().string());
        // copy XML project file
        fmuFile.add(path("resources")/"model"/absolute(inputFilename).relative_path(), inputFilename);
        // copy dependencies
        cout<<"Copy dependent files of the original XML model file to FMU."<<endl;
        for(auto & dependency : dependencies)
          if(!is_directory(dependency)) {
            if(dependency.is_absolute())
              throw runtime_error("A XML model file with parameters may only reference files by a relative path.\n"
                                  "However the model references the absolute file '"+dependency.string()+"'.\n"+
                                  "Use the --noparam options OR rework the model to not contain any absolute file path.");
            cout<<"."<<flush;
            fmuFile.add(path("resources")/"model"/current_path().relative_path()/dependency, dependency);
          }
        cout<<endl;
      }

      // create object for DynamicSystemSolver
      // Note: The document modelEle must be normalized (done above, see above)
      cout<<"Build up the model."<<endl;
      DOMElement *dssEle=E(modelEle)->getFirstElementChildNamed(MBSIM%"DynamicSystemSolver");
      dss.reset(ObjectFactory::createAndInit<DynamicSystemSolver>(dssEle));

      // create object for Integrator (just to get the start/end time for DefaultExperiment)
      integrator.reset(ObjectFactory::createAndInit<Integrator>(dssEle->getNextElementSibling()));
    }
    // Create dss from shared library
    else {
      // save binary model to FMU
      cout<<"Copy the shared library model file and dependencies to FMU."<<endl;
      copyShLibToFMU(parserNoneVali, fmuFile, path("resources")/"model"/("libmbsimfmi_model"+SHEXT), path("resources")/"model",
                     inputFilename);
      cout<<endl;

      // load the shared library and call mbsimSrcFMI function to get the dss
      cout<<"Build up the model (by just getting it from the shared library)."<<endl;
      DynamicSystemSolver *dssPtr;
      Integrator *integratorPtr;
      SharedLibrary::getSymbol<mbsimSrcFMIPtr>(canonical(inputFilename).string(), "mbsimSrcFMI")(dssPtr, integratorPtr);
      dss.reset(dssPtr);

      if(!cosim)
        // we do not have a integrator for src models (but this is only used for convinence settings)
        integrator.reset();
      else {
        if(!integratorPtr)
          throw runtime_error("A cosim FMU is generated but not Integrator is provided.");
        integrator.reset(integratorPtr);
      }
    }

    // disable all plotting (we wont any output here)
    dss->setPlotFeatureRecursive(plotRecursive, false);
    dss->setPlotFeatureRecursive(openMBV, false);
    // initialize dss
    cout<<"Initialize the model."<<endl;
    dss->initialize();

    if(cosim) {
      try {
        integrator->setSystem(dss.get());
        integrator->preIntegrate();
      }
      catch(MBSimError &ex) {
        auto &integratorRef=*integrator;
        throw runtime_error("The used integrator "+boost::core::demangle(typeid(integratorRef).name())+
          " failed to initialize the co-simulation interface. Error message was:\n"+
          ex.what()+"\n"+
          "The model may be wrong or this integrator cannot be used for cosim FMUs.");
      }
    }

    // build list of value references
    cout<<"Create model input/output variables."<<endl;
    addModelInputOutputs(var, dss.get());

    // create DOM of modelDescription.xml
    std::shared_ptr<DOMDocument> modelDescDoc(parserNoneVali->createDocument());

    // root element fmiModelDescription and its attributes
    cout<<"Create the modelDescription.xml file."<<endl;
    DOMElement *modelDesc=D(modelDescDoc)->createElement("fmiModelDescription");
    modelDescDoc->appendChild(modelDesc);
    E(modelDesc)->setAttribute("author", getenv(USERENVVAR.c_str()));
    E(modelDesc)->setAttribute("description", "FMI export of a MBSim-XML model");
    E(modelDesc)->setAttribute("fmiVersion", "1.0");
    E(modelDesc)->setAttribute("generationDateAndTime",
      boost::posix_time::to_iso_extended_string(boost::posix_time::second_clock::local_time())+"Z");
    E(modelDesc)->setAttribute("generationTool", string("MBSimFMI Version ")+VERSION);
    E(modelDesc)->setAttribute("version", "1.0");
    E(modelDesc)->setAttribute("guid", "mbsimfmi_guid");
    E(modelDesc)->setAttribute("modelIdentifier", "mbsim");
    path desc=inputFilename.filename();
    desc.replace_extension();
    desc.replace_extension();
    E(modelDesc)->setAttribute("modelName", desc.string());
    E(modelDesc)->setAttribute("numberOfContinuousStates", cosim?0:dss->getzSize());
    E(modelDesc)->setAttribute("numberOfEventIndicators", cosim?0:dss->getsvSize());
    E(modelDesc)->setAttribute("variableNamingConvention", "structured");

      // Type definition
      // get a unique list of all enumeration types
      set<Variable::EnumList> enumType;
      for(auto & vr : var)
        if(vr->getEnumerationList())
          enumType.insert(vr->getEnumerationList());
      // write all enumeration type to xml file
      DOMElement *typeDef=D(modelDescDoc)->createElement("TypeDefinitions");
      modelDesc->appendChild(typeDef);
        for(const auto & it : enumType) {
          DOMElement *type=D(modelDescDoc)->createElement("Type");
          typeDef->appendChild(type);
          E(type)->setAttribute("name", "EnumType_"+boost::lexical_cast<string>(it));
            DOMElement *enumEle=D(modelDescDoc)->createElement("EnumerationType");
            type->appendChild(enumEle);
            E(enumEle)->setAttribute("min", "1");
            E(enumEle)->setAttribute("max", it->size());
            for(size_t id=0; id<it->size(); ++id) {
              DOMElement *item=D(modelDescDoc)->createElement("Item");
              enumEle->appendChild(item);
              E(item)->setAttribute("name", (*it)[id].second);
              E(item)->setAttribute("description", (*it)[id].second);
            }
        }

      // DefaultExperiment element and its attributes (only if a integrator object exists)
      if(integrator) {
        DOMElement *defaultExp=D(modelDescDoc)->createElement("DefaultExperiment");
        modelDesc->appendChild(defaultExp);
        E(defaultExp)->setAttribute("startTime", integrator->getStartTime());
        E(defaultExp)->setAttribute("stopTime", integrator->getEndTime());
        E(defaultExp)->setAttribute("tolerance", 1e-5);
      }

      // ModelVariables element
      DOMElement *modelVars=D(modelDescDoc)->createElement("ModelVariables");
      modelDesc->appendChild(modelVars);

        // loop over all FMI variables
        for(size_t vr=0; vr<var.size(); ++vr) {
          // create ScalarVariable element
          DOMElement *scalarVar=D(modelDescDoc)->createElement("ScalarVariable");
          modelVars->appendChild(scalarVar);

            // create datatype element
            string datatypeEleName;
            switch(var[vr]->getDatatypeChar()) {
              case 'r': datatypeEleName="Real"; break;
              case 'i': datatypeEleName= var[vr]->getEnumerationList() ? "Enumeration" : "Integer"; break;
              case 'b': datatypeEleName="Boolean"; break;
              case 's': datatypeEleName="String"; break;
            }
            DOMElement *varType=D(modelDescDoc)->createElement(datatypeEleName);
            scalarVar->appendChild(varType);

            // handle enumeration
            if(var[vr]->getEnumerationList()) {
              E(varType)->setAttribute("declaredType", "EnumType_"+boost::lexical_cast<string>(var[vr]->getEnumerationList()));
              E(varType)->setAttribute("min", "1");
              E(varType)->setAttribute("max", var[vr]->getEnumerationList()->size());
            }

          // attributes on ScalarVariable element
          E(scalarVar)->setAttribute("name", var[vr]->getName());
          E(scalarVar)->setAttribute("description", var[vr]->getDescription());
          E(scalarVar)->setAttribute("valueReference", vr);
          switch(var[vr]->getType()) {
            case Parameter:
              E(scalarVar)->setAttribute("causality", "internal");
              E(scalarVar)->setAttribute("variability", "parameter");
              E(varType)->setAttribute("start", var[vr]->getValueAsString());
              E(varType)->setAttribute("fixed", "true");
              break;
            case Input:
              E(scalarVar)->setAttribute("causality", "input");
              E(scalarVar)->setAttribute("variability", "continuous");
              E(varType)->setAttribute("start", var[vr]->getValueAsString());
              E(varType)->setAttribute("fixed", "true");
              break;
            case Output:
              E(scalarVar)->setAttribute("causality", "output");
              E(scalarVar)->setAttribute("variability", "continuous");
              break;
          }
        }

      if(cosim) {
        // Implementation element
        DOMElement *implementation=D(modelDescDoc)->createElement("Implementation");
        modelDesc->appendChild(implementation);
          DOMElement *cosimStandalone=D(modelDescDoc)->createElement("CoSimulation_StandAlone");
          implementation->appendChild(cosimStandalone);
            DOMElement *capabilities=D(modelDescDoc)->createElement("Capabilities");
            cosimStandalone->appendChild(capabilities);
            E(capabilities)->setAttribute("canHandleVariableCommunicationStepSize", true);
            E(capabilities)->setAttribute("canInterpolateInputs", false);
            E(capabilities)->setAttribute("canNotUseMemoryManagementFunctions", true);
      }

    // add modelDescription.xml file to FMU
    cout<<"Copy the modelDescription.xml file to FMU."<<endl;
    string modelDescriptionStr;
    DOMParser::serializeToString(modelDescDoc.get(), modelDescriptionStr);
    fmuFile.add("modelDescription.xml", modelDescriptionStr);

    // add main FMU binary (and other required files) to FMU file
    if(xmlFile) {
      if(xmlParam.empty()) {
        // xml with no parameters -> save libmbsimxml_fmi.so to FMU
        cout<<"Copy MBSim FMI library for preprocessed XML models and dependencies to FMU."<<endl;
        copyShLibToFMU(parserNoneVali, fmuFile, path("resources")/"local"/LIBDIR/("libmbsimXXX_fmi"+SHEXT), path("resources")/"local"/LIBDIR,
                       installPath/LIBDIR/("libmbsimxml_fmi"+SHEXT));
        cout<<endl;
      }
      else {
        // xml with parameters -> save libmbsimppxml_fmi.so to FMU
        cout<<"Copy MBSim FMI library for (normal) XML models and dependencies to FMU."<<endl;
        copyShLibToFMU(parserNoneVali, fmuFile, path("resources")/"local"/LIBDIR/("libmbsimXXX_fmi"+SHEXT), path("resources")/"local"/LIBDIR,
                       installPath/LIBDIR/("libmbsimppxml_fmi"+SHEXT));
        cout<<endl;

        cout<<"Copy XML schema files to FMU."<<endl;
        path schemaDir=installPath/"share"/"mbxmlutils"/"schema";
        size_t depth=distance(schemaDir.begin(), schemaDir.end());
        for(recursive_directory_iterator srcIt=recursive_directory_iterator(schemaDir);
            srcIt!=recursive_directory_iterator(); ++srcIt) {
          if(is_directory(*srcIt)) // skip directories
            continue;
          path::iterator dstIt=srcIt->path().begin();
          for(int i=0; i<depth; ++i) ++dstIt;
          path dst;
          for(; dstIt!=srcIt->path().end(); ++dstIt)
            dst/=*dstIt;
          cout<<"."<<flush;
          fmuFile.add(path("resources")/"local"/"share"/"mbxmlutils"/"schema"/dst, srcIt->path());
        }
        cout<<endl;

        cout<<"Copy MBXMLUtils measurement.xml file to FMU."<<endl;
        fmuFile.add(path("resources")/"local"/"share"/"mbxmlutils"/"xml"/"measurement.xml",
          installPath/"share"/"mbxmlutils"/"xml"/"measurement.xml");

        map<path, pair<path, bool> > &files=eval->requiredFiles();
        cout<<"Copy files required by the evaluator and dependencies to FMU."<<endl;
        string evalFile="libmbxmlutils-eval-global-"+evalName+SHEXT;
        if(!exists(installPath/LIBDIR/evalFile))
          evalFile="libmbxmlutils-eval-"+evalName+SHEXT;
        copyShLibToFMU(parserNoneVali, fmuFile, path("resources")/"local"/LIBDIR/evalFile,
                       path("resources")/"local"/LIBDIR, installPath/LIBDIR/evalFile);
        for(auto & file : files) {
          cout<<"."<<flush;
          if(!file.second.second) {
            if(file.first.filename()=="PKG_ADD") {
              // a special hack for octave: fix PKG_ADD file before adding to the archive
              // see also the fixes in the Dockerfile's in mbsim-env/build
              boost::filesystem::ifstream pkgAdd(file.first);
              string content;
              for(string line; getline(pkgAdd, line);) {
                if(line.find("gnuplot")!=string::npos) continue;
                content+=line+"\n";
              }
              fmuFile.add(path("resources")/"local"/file.second.first/file.first.filename(), content);
            }
            else
              fmuFile.add(path("resources")/"local"/file.second.first/file.first.filename(), file.first);
          }
          else
            copyShLibToFMU(parserNoneVali, fmuFile, path("resources")/"local"/file.second.first/file.first.filename(),
                           path("resources")/"local"/LIBDIR, file.first);
        }
        cout<<endl;
      }

      cout<<"Copy MBSim module files to FMU."<<endl;
      for(directory_iterator srcIt=directory_iterator(installPath/"share"/"mbsimmodules");
        srcIt!=directory_iterator(); ++srcIt) {
        cout<<"."<<flush;
        fmuFile.add(path("resources")/"local"/"share"/"mbsimmodules"/srcIt->path().filename(), srcIt->path());
      }
      cout<<endl;
      for(const auto & moduleLib : moduleLibs) {
        cout<<"Copy MBSim module "<<moduleLib.filename()<<" and dependencies to FMU."<<endl;
        copyShLibToFMU(parserNoneVali, fmuFile, path("resources")/"local"/LIBDIR/moduleLib.filename(), path("resources")/"local"/LIBDIR,
                       moduleLib);
        cout<<endl;
      }
    }
    else {
      // source model (always without parameters) -> save libmbsimppxml_fmi.so to FMU
      cout<<"Copy MBSim FMI library for source code models and dependencies to FMU."<<endl;
      copyShLibToFMU(parserNoneVali, fmuFile, path("resources")/"local"/LIBDIR/("libmbsimXXX_fmi"+SHEXT), path("resources")/"local"/LIBDIR,
                     installPath/LIBDIR/("libmbsimsrc_fmi"+SHEXT));
      cout<<endl;
    }

    cout<<"Copy MBSim FMI wrapper library and dependencies to FMU."<<endl;
    string fmuLibName(cosim?"mbsim_cosim":"mbsim_me");
    copyShLibToFMU(parserNoneVali, fmuFile, path("binaries")/FMIOS/("mbsim"+SHEXT), path("binaries")/FMIOS,
                   installPath/"lib"/(fmuLibName+SHEXT));
    cout<<endl;

    fmuFile.close();

    return 0;
  }
  catch(const exception &ex) {
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<ex.what()<<endl;
    return 1;
  }
  catch(...) {
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<"Unknwon exception."<<endl;
    return 1;
  }
}

namespace {

  path addDebugExtension(const path &p) {
    path debug=p;
    return debug.replace_extension(debug.extension().string()+".debug");
  }

  void copyShLibToFMU(const std::shared_ptr<DOMParser> &parser,
                      CreateZip &fmuFile, const path &dst, const path &depdstdir, const path &src) {
    static boost::filesystem::path installPath(boost::dll::program_location().parent_path().parent_path());

    // copy src to FMU
    cout<<"."<<flush;
    fmuFile.add(dst, src);
    if(exists(addDebugExtension(src)))
      fmuFile.add(addDebugExtension(dst), addDebugExtension(src));

    // check if *.deplibs file exits
    path depFile=src.parent_path()/(src.filename().string()+".deplibs");
    if(!exists(depFile)) {
      depFile=installPath/"share"/"deplibs"/(src.filename().string()+".deplibs");
      if(!exists(depFile)) {
        fmatvec::Atom::msgStatic(fmatvec::Atom::Warn)<<"No *.deplibs file found for library '"<<src<<"'."<<endl<<"Some dependent libraries may be missing in the FMU."<<endl;
        return;
      }
    }

    // read *.deplibs file
    std::shared_ptr<DOMDocument> depDoc=parser->parse(depFile);
    for(DOMElement *e=depDoc->getDocumentElement()->getFirstElementChild(); e!=nullptr; e=e->getNextElementSibling()) {
      string file=X()%E(e)->getFirstTextChild()->getData();

      // check for file in reldir and copy it to FMU
      path reldir=E(e)->getAttribute("reldir");
      if(exists(installPath/reldir/file)) {
        cout<<"."<<flush;
        fmuFile.add(depdstdir/file, installPath/reldir/file);
        if(exists(addDebugExtension(installPath/reldir/file)))
          fmuFile.add(addDebugExtension(depdstdir/file), addDebugExtension(installPath/reldir/file));
        continue;
      }

      // check for file in orgdir and copy it to FMU
      path orgdir=E(e)->getAttribute("orgdir");
      if(exists(orgdir/file)) {
        cout<<"."<<flush;
        fmuFile.add(depdstdir/file, orgdir/file);
        if(exists(addDebugExtension(orgdir/file)))
          fmuFile.add(addDebugExtension(depdstdir/file), addDebugExtension(orgdir/file));
        continue;
      }

      // not found
      fmatvec::Atom::msgStatic(fmatvec::Atom::Warn)<<"Dependent library '"<<file<<"' not found."<<endl<<"This dependent libraries will be missing in the FMU."<<endl;
    }
  }

}
