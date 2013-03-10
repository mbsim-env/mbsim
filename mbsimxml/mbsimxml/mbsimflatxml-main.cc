#include "config.h"
#include <cstring>
#include "mbsimflatxml.h"
#include "mbsim/mbsim_event.h"
#include "mbsim/dynamic_system_solver.h"
#include <boost/filesystem.hpp>

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
using namespace MBSim;

int main(int argc, char *argv[]) {
  try {

    Integrator *integrator;
    DynamicSystemSolver *dss;
  
    bool doNotIntegrate=false;
    if(argc>=2 && strcmp(argv[1],"--donotintegrate")==0)
      doNotIntegrate=true;
  
    MBSimXML::preInitDynamicSystemSolver(argc, argv, dss);
  
    MBSimXML::initDynamicSystemSolver(argc, argv, dss);
  
    MBSimXML::initIntegrator(argc, argv, integrator);
  
    if(doNotIntegrate==false) {
      MBSimXML::main(integrator, dss);
    }
    // Remove the following block if --lastframe works in OpenMBV.
    // If this is removed openmbv should be opened with the --lastframe option.
    // Currently we use this block if --stopafterfirststep is given to reload the XML/H5 file in OpenMBV again
    // after the first step has been written since this is not possible by the file locking mechanism in OpenMBVCppInterface.
    if(strcmp(argv[1],"--stopafterfirststep")==0) {
      // touch the OpenMBV files
      boost::myfilesystem::last_write_time((dss->getName()+".ombv.xml").c_str(), boost::posix_time::microsec_clock::universal_time());
      boost::myfilesystem::last_write_time((dss->getName()+".ombv.h5" ).c_str(), boost::posix_time::microsec_clock::universal_time());
    }
  
    MBSimXML::postMain(argc, argv, integrator, dss);
  }
  catch (MBSimError error) {
    error.printExceptionMessage();
    return 1;
  }

  return 0;
}
