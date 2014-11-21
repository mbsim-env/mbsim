#ifndef _MBSIMFMI_ZIP_H_
#define _MBSIMFMI_ZIP_H_

#include <string>
#include <boost/filesystem.hpp>
#include <fmatvec/atom.h>

struct archive;
struct archive_entry;

namespace MBSimFMI {

//! Create a zip file
class CreateZip : virtual public fmatvec::Atom {
  public:
    //! Create zip file name zipFile_
    CreateZip(const boost::filesystem::path &zipFile_);
    //! Destroy zipfile. Implicitly close the file if not already done by close().
    ~CreateZip();
    //! Close the zip file. Write all content to disk.
    void close();
    //! Add the content of srcFilename to zip file path filenameInZip.
    void add(const boost::filesystem::path &filenameInZip, const boost::filesystem::path &srcFilename);
    //! Add the string textContent to zip file path filenameInZip.
    void add(const boost::filesystem::path &filenameInZip, const std::string &textContent);

  protected:
    bool closed; // already closed flag
    boost::filesystem::path zipFile; // zip file name
    archive *a; // zip archive handle
    archive_entry *entry; // zip entry handle
};

}

#endif
