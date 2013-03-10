#include "config.h"
#include <iostream>
#include <list>
#include <vector>
#include <algorithm>
#include <string.h>
#include <fstream>
#include "env.h"
#include <boost/filesystem.hpp>
#ifdef MBSIMXML_MINGW // Windows
#  include <windows.h>
#  include <process.h>
#else
#  include <unistd.h>
#  include <spawn.h>
#  include <sys/wait.h>
#endif

/* This is a varaint of the boost::filesystem::last_write_time functions.
 * It only differs in the argument/return value being here a boost::posix_time::ptime instead of a time_t.
 * This enables file timestamps on microsecond level. */
#include <boost/filesystem.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/stat.h>
#ifdef _WIN32
#include <windows.h>
#endif
namespace boost {
  namespace myfilesystem {
    boost::posix_time::ptime last_write_time(const boost::filesystem::path &p) {
#ifndef _WIN32
      struct stat st;
      if(stat(p.generic_string().c_str(), &st)!=0)
        throw boost::filesystem::filesystem_error("system stat call failed", p, boost::system::error_code());
      boost::posix_time::ptime time;
      time=boost::posix_time::from_time_t(st.st_mtime);
      time+=boost::posix_time::microsec(st.st_mtim.tv_nsec/1000);
      return time;
#else
      HANDLE f=CreateFile(p.generic_string().c_str(), GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
      if(f==INVALID_HANDLE_VALUE)
        throw boost::filesystem::filesystem_error("CreateFile failed", p, boost::system::error_code());
      FILETIME create, lastAccess, lastWrite;
      if(GetFileTime(f, &create, &lastAccess, &lastWrite)==0) {
        CloseHandle(f);
        throw boost::filesystem::filesystem_error("GetFileTime failed", p, boost::system::error_code());
      }
      CloseHandle(f);
      uint64_t microSecSince1601=((((uint64_t)(lastWrite.dwHighDateTime))<<32)+lastWrite.dwLowDateTime)/10;
      uint64_t hoursSince1601=microSecSince1601/1000000/60/60;
      return boost::posix_time::ptime(boost::gregorian::date(1601,boost::gregorian::Jan,1),
                                      boost::posix_time::hours(hoursSince1601)+
                                      boost::posix_time::microseconds(microSecSince1601-hoursSince1601*60*60*1000000));
#endif
    }
    void last_write_time(const boost::filesystem::path &p, const boost::posix_time::ptime &time) {
#ifndef _WIN32
      struct timeval times[2];
      boost::posix_time::time_period sinceEpoch(boost::posix_time::ptime(boost::gregorian::date(1970, boost::gregorian::Jan, 1)), time);
      times[0].tv_sec =sinceEpoch.length().total_seconds();
      times[0].tv_usec=sinceEpoch.length().total_microseconds()-1000000*times[0].tv_sec;
      times[1].tv_sec =times[0].tv_sec;
      times[1].tv_usec=times[0].tv_usec;
      if(utimes(p.generic_string().c_str(), times)!=0)
        throw boost::filesystem::filesystem_error("system utimes call failed", p, boost::system::error_code());
#else
      HANDLE f=CreateFile(p.generic_string().c_str(), FILE_WRITE_ATTRIBUTES, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
      if(f==INVALID_HANDLE_VALUE)
        throw boost::filesystem::filesystem_error("CreateFile failed", p, boost::system::error_code());
      boost::posix_time::time_period since1601(boost::posix_time::ptime(boost::gregorian::date(1601, boost::gregorian::Jan, 1)), time);
      boost::posix_time::time_duration dt=since1601.length();
      uint64_t winTime=((uint64_t)(dt.hours()))*60*60*10000000;
      dt-=boost::posix_time::hours(dt.hours());
      winTime+=dt.total_microseconds()*10;
      FILETIME changeTime;
      changeTime.dwHighDateTime=(winTime>>32);
      changeTime.dwLowDateTime=(winTime & ((((uint64_t)1)<<32)-1));
      if(SetFileTime(f, NULL, &changeTime, &changeTime)==0) {
        CloseHandle(f);
        throw boost::filesystem::filesystem_error("SetFileTime failed", p, boost::system::error_code());
      }
      CloseHandle(f);
#endif
    }
  }
}

using namespace std;
namespace bfs=boost::filesystem;

// run the program in arg[0] with the options arg[1], arg[2], ...
// asyncronously (wait for arg[0] to finish) and
// return -1 or error or the exit code of arg[0].
int runProgramSyncronous(const vector<string> &arg) {
  char **argv=new char*[arg.size()+1];
  for(size_t i=0; i<arg.size(); i++)
    argv[i]=const_cast<char*>(arg[i].c_str());
  argv[arg.size()]=NULL;

#if !defined MBSIMXML_MINGW
  pid_t child;
  int ret;
  ret=posix_spawn(&child, argv[0], NULL, NULL, argv, environ);
  delete[]argv;
  if(ret!=0)
    return -1;

  int status;
  waitpid(child, &status, 0);

  if(WIFEXITED(status))
    return WEXITSTATUS(status);
  else
    return -1;
#else
  int ret;
  ret=_spawnv(_P_WAIT, argv[0], argv);
  delete[]argv;
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
  return (--p.end())->generic_string();
}

// create filename if it does not exist or touch it if if exists (OS-independent)
void createOrTouch(const string &filename) {
  ofstream f(filename.c_str()); // Note: ofstream use precise file timestamp
}

string getInstallPath() {
  // get path of this executable
  static char exePath[4096]="";
  if(strcmp(exePath, "")!=0) return string(exePath)+"/..";

#ifdef _WIN32 // Windows
  GetModuleFileName(NULL, exePath, sizeof(exePath));
  for(size_t i=0; i<strlen(exePath); i++) if(exePath[i]=='\\') exePath[i]='/'; // convert '\' to '/'
  *strrchr(exePath, '/')=0; // remove the program name
#else // Linux
#ifdef DEVELOPER_HACK_EXEPATH
  // use hardcoded exePath
  strcpy(exePath, DEVELOPER_HACK_EXEPATH);
#else
  int exePathLength=readlink("/proc/self/exe", exePath, sizeof(exePath)); // get abs path to this executable
  exePath[exePathLength]=0; // null terminate
  *strrchr(exePath, '/')=0; // remove the program name
#endif
#endif

  return string(exePath)+"/..";
}

int main(int argc, char *argv[]) {
  // convert args to c++
  list<string> arg;
  list<string>::iterator i, i2;
  for(int i=1; i<argc; i++)
    arg.push_back(argv[i]);

  // help
  if(arg.size()<=1 ||
     (i=std::find(arg.begin(), arg.end(), "-h"))!=arg.end() ||
     (i=std::find(arg.begin(), arg.end(), "--help"))!=arg.end() ||
     (i=std::find(arg.begin(), arg.end(), "-?"))!=arg.end()) {
    cout<<"Usage: mbsimxml [--onlypreprocess|--donotintegrate|--stopafterfirststep|"<<endl
        <<"                 --autoreload]"<<endl
        <<"                [--mpath <dir> [--mpath <dir>]]"<<endl
        <<"                [--mbsimparam <mbsimparameterfile>] <mbsimfile>"<<endl
        <<"                [--intparam <integratorparameterfile>] <integratorfile>"<<endl
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
        <<"--mpath              Add <dir> to the octave search path for m-files"<<endl
        <<"--mbsimparam <file>  Use <file> as parameter file for mbsim xml file"<<endl
        <<"<mbsimfile>          Use <mbsimfile> as mbsim xml file"<<endl
        <<"--intparam <file>    Use <file> as parameter file for mbsim integrator xml file"<<endl
        <<"<integratorfile>     Use <integratorfile> as mbsim integrator xml file"<<endl;
    return 0;
  }

  // get path of this executable
  string EXEEXT;
#ifdef MBSIMXML_MINGW // Windows
  EXEEXT=".exe";
#else // Linux
  EXEEXT="";
#endif

  // check for environment variables (none default installation)
  string MBXMLUTILSBIN;
  string MBXMLUTILSSCHEMA;
  string MBSIMXMLBIN;
  char *env;
  MBXMLUTILSBIN=MBXMLUTILSBIN_DEFAULT; // default: from build configuration
  if(!bfs::exists((MBXMLUTILSBIN+"/mbxmlutilspp"+EXEEXT).c_str())) MBXMLUTILSBIN=getInstallPath()+"/bin"; // use rel path if build configuration dose not work
  if(!bfs::exists((MBXMLUTILSBIN+"/mbxmlutilspp"+EXEEXT).c_str())) MBXMLUTILSBIN=getInstallPath()+"/bin"; // use rel path if build configuration dose not work
  if((env=getenv("MBXMLUTILSBINDIR"))) MBXMLUTILSBIN=env; // overwrite with envvar if exist
  MBXMLUTILSSCHEMA=MBXMLUTILSSCHEMA_DEFAULT; // default: from build configuration
  if(!bfs::exists((MBXMLUTILSSCHEMA+"/http___mbsim_berlios_de_MBSimXML/mbsimxml.xsd").c_str())) MBXMLUTILSSCHEMA=getInstallPath()+"/share/mbxmlutils/schema"; // use rel path if build configuration dose not work
  if((env=getenv("MBXMLUTILSSCHEMADIR"))) MBXMLUTILSSCHEMA=env; // overwrite with envvar if exist
  MBSIMXMLBIN=MBSIMXMLBIN_DEFAULT; // default: from build configuration
  if(!bfs::exists((MBSIMXMLBIN+"/mbsimflatxml"+EXEEXT).c_str())) MBSIMXMLBIN=getInstallPath()+"/bin"; // use rel path if build configuration dose not work
  if((env=getenv("MBSIMXMLBINDIR"))) MBSIMXMLBIN=env; // overwrite with envvar if exist

  // parse parameters

  // mpath
  vector<string> MPATH;
  if((i=std::find(arg.begin(), arg.end(), "--mpath"))!=arg.end()) {
    i2=i; i2++;
    MPATH.push_back(*i);
    MPATH.push_back(*i2);
    arg.erase(i); arg.erase(i2);
  }

  bool ONLYPP=false;
  if((i=std::find(arg.begin(), arg.end(), "--onlypreprocess"))!=arg.end()) {
    ONLYPP=true;
    arg.erase(i);
  }

  string NOINT;
  if((i=std::find(arg.begin(), arg.end(), "--donotintegrate"))!=arg.end()) {
    NOINT="--donotintegrate";
    arg.erase(i);
  }

  string ONLY1OUT;
  if((i=std::find(arg.begin(), arg.end(), "--stopafterfirststep"))!=arg.end()) {
    ONLY1OUT="--stopafterfirststep";
    arg.erase(i);
  }

  int AUTORELOADTIME=0;
  if((i=std::find(arg.begin(), arg.end(), "--autoreload"))!=arg.end()) {
    i2=i; i2++;
    char *error;
    ONLY1OUT="--stopafterfirststep";
    AUTORELOADTIME=strtol(i2->c_str(), &error, 10);
    // AUTORELOAD is set delayed since we currently do not know the MBSim file name (see later)
    if(AUTORELOADTIME<0) AUTORELOADTIME=250;
    arg.erase(i);
    if(error && strlen(error)==0)
      arg.erase(i2);
    else
      AUTORELOADTIME=250;
  }

  string PARAM="none";
  if((i=std::find(arg.begin(), arg.end(), "--mbsimparam"))!=arg.end()) {
    i2=i; i2++;
    PARAM=*i2;
    arg.erase(i); arg.erase(i2);
  }

  string MBSIM=*arg.begin();
  arg.erase(arg.begin());
  string PPMBSIM=".pp."+basename(MBSIM);
  string DEPMBSIM=".dep."+basename(MBSIM);
  string ERRFILE=".err."+basename(MBSIM);

  vector<string> AUTORELOAD;
  if(AUTORELOADTIME>0) { // AUTORELOAD is now set (see above)
    AUTORELOAD.push_back("--dependencies");
    AUTORELOAD.push_back(DEPMBSIM);
  }

  string PARAMINT="none";
  if((i=std::find(arg.begin(), arg.end(), "--intparam"))!=arg.end()) {
    i2=i; i2++;
    PARAMINT=*i2;
    arg.erase(i); arg.erase(i2);
  }

  string MBSIMINT=*arg.begin();
  arg.erase(arg.begin());
  string PPMBSIMINT=".pp."+basename(MBSIMINT);

  // execute
  int ret; // comamnd return value
  bool runAgain=true; // always run the first time
  while(runAgain) {
    unlink(PPMBSIM.c_str());
    unlink(PPMBSIMINT.c_str());
    unlink(ERRFILE.c_str());

    // run preprocessor
    vector<string> command;
    command.push_back(MBXMLUTILSBIN+"/mbxmlutilspp");
    command.insert(command.end(), AUTORELOAD.begin(), AUTORELOAD.end());
    command.insert(command.end(), MPATH.begin(), MPATH.end());
    command.push_back(PARAM);
    command.push_back(MBSIM);
    command.push_back(MBXMLUTILSSCHEMA+"/http___mbsim_berlios_de_MBSimXML/mbsimxml.xsd");
    command.push_back(PARAMINT);
    command.push_back(MBSIMINT);
    command.push_back(MBXMLUTILSSCHEMA+"/http___mbsim_berlios_de_MBSim/mbsimintegrator.xsd");
    ret=runProgramSyncronous(command);

    if(!ONLYPP && ret==0) {
      vector<string> command;
      command.push_back(MBSIMXMLBIN+"/mbsimflatxml");
      if(NOINT!="") command.push_back(NOINT);
      if(ONLY1OUT!="") command.push_back(ONLY1OUT);
      command.push_back(PPMBSIM);
      command.push_back(PPMBSIMINT);
      ret=runProgramSyncronous(command);
    }

    if(ret!=0) createOrTouch(ERRFILE);

    runAgain=false; // only run ones except --autoreload is given
    if(AUTORELOADTIME>0) {
      // get dependent files
      vector<string> depfiles;
      ifstream deps(DEPMBSIM.c_str());
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
        for(size_t i=0; i<depfiles.size(); i++) {
          if(newer(depfiles[i], PPMBSIM) || newer(depfiles[i], PPMBSIMINT) || newer(depfiles[i], ERRFILE)) {
            runAgain=true;
            break;
          }
        }
      }
    }
  }

  return ret;
}
