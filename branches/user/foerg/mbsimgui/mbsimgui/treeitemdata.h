#ifndef _TREEITEMDATA__H_
#define _TREEITEMDATA__H_

#include <string>

class PropertyDialog;

class TreeItemData {
  public:
    virtual ~TreeItemData() {}
    virtual const std::string& getName() const = 0;
    virtual std::string getType() const = 0;
    virtual void setName(const std::string &data) = 0;
    virtual void setType(const std::string &data) {}
    virtual PropertyDialog* createPropertyDialog() {return 0;}
    virtual bool isRemovable() {return true;}
};

#endif
