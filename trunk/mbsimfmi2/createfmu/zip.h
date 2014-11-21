#ifndef _MBSIMFMI_ZIP_H_
#define _MBSIMFMI_ZIP_H_

#include <string>
#include <boost/filesystem.hpp>

struct archive;
struct archive_entry;

namespace MBSimFMI {

class CreateZip {
  public:
    CreateZip(const boost::filesystem::path &zipFile_);
    ~CreateZip();
    void close();
    void add(const boost::filesystem::path &filenameInZip, const boost::filesystem::path &srcFilename);
    void add(const boost::filesystem::path &filenameInZip, const std::string &textContent);
  protected:
    bool closed;
    boost::filesystem::path zipFile;
    archive *a;
    archive_entry *entry;
};

}

#endif
