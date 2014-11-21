#ifndef _MBSIMFMI_UTILS_H_
#define _MBSIMFMI_UTILS_H_

#include <boost/filesystem.hpp>
#include <sstream>

extern "C" {
  #include <3rdparty/fmiModelFunctions.h>
}

namespace MBSimFMI {

  //! get the directory of this shared library
  boost::filesystem::path getSharedLibDir();

  //! stream buffer used for FMI logging
  class LoggerBuffer : public std::stringbuf {
    public:
      LoggerBuffer(fmiCallbackLogger logger_, fmiComponent c_, const std::string &instanceName_, fmiStatus status_, const std::string &category_) :
          std::stringbuf(std::ios_base::out),
          logger(logger_),
          c(c_),
          instanceName(instanceName_),
          status(status_),
          category(category_) {
      }
    protected:
      fmiCallbackLogger logger;
      fmiComponent c;
      std::string instanceName;
      fmiStatus status;
      std::string category;
  
      int sync(); // overwrite the sync function from stringbuf
  };

}

#endif
