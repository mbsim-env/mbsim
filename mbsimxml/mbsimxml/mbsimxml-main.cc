#include "config.h"
#include <iostream>
#include <boost/regex.hpp>
#include <list>
#include <vector>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <mbxmlutilshelper/getinstallpath.h>
#include <mbxmlutilshelper/last_write_time.h>
#include <mbsim/element.h>
#include <mbsim/integrators/integrator.h>
#include <cstdio>
#if !defined _WIN32
#  include <spawn.h>
#  include <sys/types.h>
#  include <sys/wait.h>
#else
#  include <process.h>
#  include <windows.h>
#endif
#include <mbxmlutilshelper/dom.h>
#include <xercesc/dom/DOMDocument.hpp>
#include "mbsimxml.h"

using namespace std;
using namespace MBXMLUtils;
namespace bfs=boost::filesystem;

int runProgram(const vector<string> &arg) {
#if !defined _WIN32
  // convert arg to c style
  auto **argv=new char*[arg.size()+1];
  for(size_t i=0; i<arg.size(); i++)
    argv[i]=const_cast<char*>(arg[i].c_str());
  argv[arg.size()]=nullptr;

  pid_t child;
  int spawnRet;
  // Use posix_spawn from a older glibc implementation to allow binary distributions to run on older systems
  #if defined(__GNUC__) // GNU compiler
    #if !defined(__LP64__) // 32 bit
      __asm__(".symver posix_spawn, posix_spawn@GLIBC_2.2");
    #else // 64 bit
      __asm__(".symver posix_spawn, posix_spawn@GLIBC_2.2.5");
    #endif
  #endif
  spawnRet=posix_spawn(&child, argv[0], nullptr, nullptr, argv, environ);
  delete[]argv;
  if(spawnRet!=0)
    throw runtime_error("Unable to spawn process.");
  int status;
  waitpid(child, &status, 0);
  if(WIFEXITED(status))
    return WEXITSTATUS(status);
  else
    throw runtime_error("Spawn process terminated abnormally.");
#else
  // convert arg to c style and quote with " (this is required by Windows even so earch arg is passed seperately)
  auto **argv=new char*[arg.size()+1];
  list<string> quotedArgs;
  argv[0]=const_cast<char*>(arg[0].c_str());
  for(size_t i=1; i<arg.size(); i++) {
    quotedArgs.emplace_back("\""+arg[i]+"\"");
    argv[i]=const_cast<char*>(quotedArgs.back().c_str());
  }
  argv[arg.size()]=nullptr;

  int ret;
  ret=_spawnv(_P_WAIT, argv[0], argv);
  delete[]argv;
  if(ret==-1)
    throw runtime_error("Unable to spawn process.");
  return ret;
#endif
}

// return true if filanamea is newer then filenameb (modification time) (OS-independent)
bool newer(const string &filenamea, const string &filenameb) {
  if(!bfs::exists(filenamea.c_str()) || !bfs::exists(filenameb.c_str()))
    return false;

  return boost::myfilesystem::last_write_time(filenamea.c_str())>
         boost::myfilesystem::last_write_time(filenameb.c_str());
}


// return filename without path but with extension (OS-independent)
string basename(const string &filename) {
  bfs::path p(filename.c_str());
  return p.filename().generic_string();
}

// create filename if it does not exist or touch it if if exists (OS-independent)
void createOrTouch(const string &filename) {
  ofstream f(filename.c_str()); // Note: ofstream use precise file timestamp
}

//! A string buffer which prefixes every line.
class PrefixedStringBuf : public std::stringbuf {
  public:
    //! Prefix each line with prefix_  and postfix with postfix_ and print to str_.
    PrefixedStringBuf(std::string prefix_, std::string postfix_, std::ostream &outstr_) :
      std::stringbuf(std::ios_base::out), prefix(std::move(prefix_)), postfix(std::move(postfix_)), outstr(outstr_) {}
  protected:
    int sync() override {
      outstr<<prefix<<str()<<postfix<<flush;
      str("");
      return 0;
    }
    std::string prefix;
    std::string postfix;
    std::ostream &outstr;
};

int main(int argc, char *argv[]) {
  try {

    // convert args to c++
    list<string> args;
    list<string>::iterator i, i2;
    for(int i=1; i<argc; i++)
      args.emplace_back(argv[i]);

    bool ONLYLISTSCHEMAS=std::find(args.begin(), args.end(), "--onlyListSchemas")!=args.end();
  
    // help
    if(args.size()<1 ||
       std::find(args.begin(), args.end(), "-h")!=args.end() ||
       std::find(args.begin(), args.end(), "--help")!=args.end() ||
       std::find(args.begin(), args.end(), "-?")!=args.end()) {
      cout<<"Usage: mbsimxml [--onlypreprocess|--donotintegrate|--stopafterfirststep|"<<endl
          <<"                 --autoreload|--onlyListSchemas]"<<endl
          <<"                [--modulesPath <dir> [--modulePath <dir> ...]]"<<endl
          <<"                [--stdout <msg> [--stdout <msg> ...]] [--stderr <msg> [--stderr <msg> ...]]"<<endl
          <<"                <mbsimprjfile>"<<endl
          <<""<<endl
          <<"Copyright (C) 2004-2009 MBSim Development Team"<<endl
          <<"This is free software; see the source for copying conditions. There is NO"<<endl
          <<"warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE."<<endl
          <<""<<endl
          <<"Licensed under the GNU Lesser General Public License (LGPL)"<<endl
          <<""<<endl
          <<"--onlypreprocess     Stop after the preprocessing stage"<<endl
          <<"--donotintegrate     Stop after the initialization stage, do not integrate"<<endl
          <<"--stopafterfirststep Stop after outputting the first step (usually at t=0)"<<endl
          <<"                     This generates a HDF5 output file with only one time serie"<<endl
          <<"--autoreload         Same as --stopafterfirststep but rerun mbsimxml each time"<<endl
          <<"                     a input file is newer than the output file"<<endl
          <<"--onlyListSchemas    List all XML schema files including MBSim modules"<<endl
          <<"--modulePath <dir>   Add <dir> to MBSim module serach path. The central MBSim installation"<<endl
          <<"                     module dir and the current dir is always included."<<endl
          <<"--stdout <msg>       Print on stdout messages of type <msg>."<<endl
          <<"                     <msg> may be info~<pre>~<post>, warn~<pre>~<post>, debug~<pre>~<post>"<<endl
          <<"                     error~<pre>~<post>~ or depr~<pre>~<post>~."<<endl
          <<"                     Each message is prefixed/postfixed with <pre>/<post>."<<endl
          <<"                     --stdout may be specified multiple times."<<endl
          <<"                     If --stdout and --stderr is not specified --stdout 'info~Info: ~'"<<endl
          <<"                     --stderr 'warn~Warn: ~' --stderr 'error~~' --stderr 'depr~Depr:~'"<<endl
          <<"                     --stderr 'status~~\\r' is used."<<endl
          <<"--stderr <msg>       Analog to --stdout but prints to stderr."<<endl
          <<"<mbsimprjfile>       Use <mbsimprjfile> as mbsim xml project file"<<endl;
      return 0;
    }

    // defaults for --stdout and --stderr
    if(find(args.begin(), args.end(), "--stdout")==args.end() &&
       find(args.begin(), args.end(), "--stderr")==args.end()) {
      args.push_back("--stdout"); args.push_back("info~Info: ~");
      args.push_back("--stderr"); args.push_back("warn~Warn: ~");
      args.push_back("--stderr"); args.push_back("error~~");
      args.push_back("--stderr"); args.push_back("depr~Depr: ~");
      args.push_back("--stdout"); args.push_back("status~~\r");
    }

    // disable all streams
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Info      , std::make_shared<bool>(false));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Warn      , std::make_shared<bool>(false));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Debug     , std::make_shared<bool>(false));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Error     , std::make_shared<bool>(false));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Deprecated, std::make_shared<bool>(false));
    fmatvec::Atom::setCurrentMessageStream(fmatvec::Atom::Status    , std::make_shared<bool>(false));

    // get --stdout --stderr args to pass to other processes
    vector<string> streamArgs;
    for(auto it=args.begin(); it!=args.end(); ++it) {
      if(*it!="--stdout" && *it!="--stderr") continue;

      streamArgs.emplace_back(*it);
      streamArgs.emplace_back(*next(it));
    }

    // handle --stdout and --stderr args
    list<string>::iterator it;
    while((it=find_if(args.begin(), args.end(), [](const string &x){ return x=="--stdout" || x=="--stderr"; }))!=args.end()) {
      if(*it!="--stdout" && *it!="--stderr") continue;

      ostream &ostr=*it=="--stdout"?cout:cerr;
      auto itn=next(it);
      if(itn==args.end()) {
        cerr<<"Invalid argument"<<endl;
        return 1;
      }
      fmatvec::Atom::MsgType msgType;
      if     (itn->substr(0, 5)=="info~"  ) msgType=fmatvec::Atom::Info;
      else if(itn->substr(0, 5)=="warn~"  ) msgType=fmatvec::Atom::Warn;
      else if(itn->substr(0, 6)=="debug~" ) msgType=fmatvec::Atom::Debug;
      else if(itn->substr(0, 6)=="error~" ) msgType=fmatvec::Atom::Error;
      else if(itn->substr(0, 5)=="depr~"  ) msgType=fmatvec::Atom::Deprecated;
      else if(itn->substr(0, 7)=="status~") msgType=fmatvec::Atom::Status;
      else throw runtime_error("Unknown message stream.");
      static boost::regex re(".*~(.*)~(.*)", boost::regex::extended);
      boost::smatch m;
      if(!boost::regex_match(*itn, m, re)) {
        cerr<<"Invalid argument"<<endl;
        return 1;
      }
      static list<PrefixedStringBuf> buf;
      buf.emplace_back(m.str(1), m.str(2), ostr);
      fmatvec::Atom::setCurrentMessageStream(msgType, std::make_shared<bool>(true),
        std::make_shared<ostream>(&buf.back()));

      args.erase(itn);
      args.erase(it);
    }
  
    // get path of this executable
    string EXEEXT;
#ifdef _WIN32 // Windows
    EXEEXT=".exe";
#else // Linux
    EXEEXT="";
#endif
  
    bfs::path MBXMLUTILSBIN=getInstallPath()/"bin";
    bfs::path MBXMLUTILSSCHEMA=getInstallPath()/"share"/"mbxmlutils"/"schema";
  
    // parse parameters

    // generate mbsimxml.xsd
    set<bfs::path> searchDirs;
    while((i=find(args.begin(), args.end(), "--modulePath"))!=args.end()) {
      i2=i; i2++;
      searchDirs.insert(*i2);
      args.erase(i);
      args.erase(i2);
    }
    set<bfs::path> schemas=MBSim::getMBSimXMLSchemas(searchDirs);
    if(ONLYLISTSCHEMAS) {
      for(auto &schema: schemas)
        cout<<schema.string()<<endl;
      return 0;
    }
  
    bool ONLYPP=false;
    if((i=std::find(args.begin(), args.end(), "--onlypreprocess"))!=args.end()) {
      ONLYPP=true;
      args.erase(i);
    }
  
    string NOINT;
    if((i=std::find(args.begin(), args.end(), "--donotintegrate"))!=args.end()) {
      NOINT="--donotintegrate";
      args.erase(i);
    }
  
    string ONLY1OUT;
    if((i=std::find(args.begin(), args.end(), "--stopafterfirststep"))!=args.end()) {
      ONLY1OUT="--stopafterfirststep";
      args.erase(i);
    }
  
    int AUTORELOADTIME=0;
    if((i=std::find(args.begin(), args.end(), "--autoreload"))!=args.end()) {
      i2=i; i2++;
      char *error;
      ONLY1OUT="--stopafterfirststep";
      AUTORELOADTIME=strtol(i2->c_str(), &error, 10);
      // AUTORELOAD is set delayed since we currently do not know the MBSim file name (see later)
      if(AUTORELOADTIME<0) AUTORELOADTIME=250;
      args.erase(i);
      if(error && strlen(error)==0)
        args.erase(i2);
      else
        AUTORELOADTIME=250;
    }
  
    string MBSIMPRJ=*args.begin();
    args.erase(args.begin());
    string PPMBSIMPRJ=".pp."+basename(MBSIMPRJ);
    string DEPMBSIMPRJ=".dep."+basename(MBSIMPRJ);
    string ERRFILE=".err."+basename(MBSIMPRJ);
  
    vector<string> AUTORELOAD;
    if(AUTORELOADTIME>0) { // AUTORELOAD is now set (see above)
      AUTORELOAD.emplace_back("--dependencies");
      AUTORELOAD.push_back(DEPMBSIMPRJ);
    }
  
    // execute
    int ret; // comamnd return value
    bool runAgain=true; // always run the first time
    while(runAgain) {
      unlink(PPMBSIMPRJ.c_str());
      unlink(ERRFILE.c_str());
  
      // run preprocessor
      // validate the project file with mbsimxml.xsd
      vector<string> command;
      command.push_back((MBXMLUTILSBIN/(string("mbxmlutilspp")+EXEEXT)).string());
      command.insert(command.end(), streamArgs.begin(), streamArgs.end());
      command.insert(command.end(), AUTORELOAD.begin(), AUTORELOAD.end());
      for(auto &schema: schemas)
        command.push_back(schema.string());
      command.push_back(MBSIMPRJ);
      ret=runProgram(command);
  
      if(!ONLYPP && ret==0) {
        vector<string> command;
        command.push_back((MBXMLUTILSBIN/(string("mbsimflatxml")+EXEEXT)).string());
        if(NOINT!="") command.push_back(NOINT);
        if(ONLY1OUT!="") command.push_back(ONLY1OUT);
        command.insert(command.end(), streamArgs.begin(), streamArgs.end());
        command.push_back(PPMBSIMPRJ);
        ret=runProgram(command);
      }
  
      if(ret!=0) createOrTouch(ERRFILE);
  
      runAgain=false; // only run ones except --autoreload is given
      if(AUTORELOADTIME>0) {
        // get dependent files
        vector<string> depfiles;
        ifstream deps(DEPMBSIMPRJ.c_str());
        while(true) {
          string depfile;
          getline(deps, depfile);
          if(deps.fail()) break;
          depfiles.push_back(depfile);
        }
        deps.close();
        // check for dependent files being newer then the output file
        while(!runAgain) {
#ifndef _WIN32
          usleep(AUTORELOADTIME*1000);
#else
          Sleep(AUTORELOADTIME);
#endif
          for(const auto & depfile : depfiles) {
            if(newer(depfile, PPMBSIMPRJ) || newer(depfile, ERRFILE)) {
              runAgain=true;
              break;
            }
          }
        }
      }
    }
  
    return ret;
  }
  catch(const exception &e) {
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<e.what()<<endl;
    return 1;
  }
  catch(...) {
    fmatvec::Atom::msgStatic(fmatvec::Atom::Error)<<"Unknown exception"<<endl;
    return 1;
  }
  return 0;
}
