/*
 * This file is NOT part of TinyXML. Is was added by Markus
 * Friedrich to attach minimal namespace awareness to TinyXML.
 *
 * This File is Public Domain. Do what ever you want with it.
*/

#include "tinynamespace.h"
#include "tinystr.h"
#include <sstream>
#include <algorithm>

using namespace std;

const TiXmlElement* TiXml_GetElementWithXmlBase(TiXmlElement *e, int i) {
  if(e==NULL) return NULL;
  if(e->ToElement() && e->ToElement()->Attribute("xml:base") && i==0)
    return e->ToElement();
  else if(e->ToElement() && e->ToElement()->Attribute("xml:base") && i>0 && e->Parent())
    return TiXml_GetElementWithXmlBase(e->Parent()->ToElement(),i-1);
  else if(e->Parent())
    return TiXml_GetElementWithXmlBase(e->Parent()->ToElement(),i);
  else
    return 0;
}

void TiXml_PostLoadFile(TiXmlDocument *doc) {
  if(doc->FirstChildElement()->Attribute("xml:base")==0)
    doc->FirstChildElement()->SetAttribute("xml:base", doc->Value());
}


string TiXml_itoa(int i) {
  ostringstream str;
  str<<i;
  return str.str();
}

void TiXml_addLineNrAsProcessingInstruction(TiXmlElement *e) {
  TiXmlUnknown line;
  line.SetValue("?LineNr "+TiXml_itoa(e->Row())+"?");
  if(e->FirstChild())
    e->InsertBeforeChild(e->FirstChild(), line);
  else
    e->InsertEndChild(line);

  TiXmlElement *c=e->FirstChildElement();
  while(c) {
    TiXml_addLineNrAsProcessingInstruction(c);
    c=c->NextSiblingElement();
  }
}

void TiXml_setLineNrFromProcessingInstruction(TiXmlElement *e) {
  TiXmlUnknown *u;
  if(e->FirstChild()) {
    for(u=e->FirstChild()->ToUnknown(); u && u->ValueStr().substr(0,8)!="?LineNr "; u=u->NextSibling()->ToUnknown());
    if(u) {
      string line=u->ValueStr().substr(8);
      line=line.substr(0,line.length()-1);
      e->SetRow(atoi(line.c_str()));
      e->RemoveChild(u);
    }
  }

  TiXmlElement *c=e->FirstChildElement();
  while(c) {
    TiXml_setLineNrFromProcessingInstruction(c);
    c=c->NextSiblingElement();
  }
}

void TiXml_deletePIandComm(TiXmlElement *e) {
}

void TiXml_location(TiXmlElement *e, const string &pre, const string &post) {
  cout<<pre<<TiXml_GetElementWithXmlBase(e,0)->Attribute("xml:base")<<":"<<e->Row()<<post<<endl;
  const TiXmlElement *p;
  for(int i=1; (p=TiXml_GetElementWithXmlBase(e,i))!=0; i++) {
    const TiXmlNode *c=TiXml_GetElementWithXmlBase(e,i-1)->FirstChild();
    const TiXmlUnknown *u;
    for(u=c->ToUnknown(); u && u->ValueStr().substr(0,23)!="?OriginalElementLineNr "; u=u->NextSibling()->ToUnknown());
    string line=u->ValueStr().substr(23);
    for(u=c->ToUnknown(); u && u->ValueStr().substr(0,14)!="?EmbedCountNr "; u=u->NextSibling()->ToUnknown());
    string count;
    if(u) {
      count=u->ValueStr().substr(14);
      count=count.substr(0,count.length()-1);
      count=":[count="+count+"]";
    }
    cout<<"  included by: "<<p->Attribute("xml:base")<<":"<<line.substr(0,line.length()-1)<<count<<endl;
  }
}

string tinyNamespaceCompStr;
bool comp(pair<string,string> p) {
  if(p.second==tinyNamespaceCompStr) return true; else return false;
}
void incorporateNamespace(TiXmlElement* e, map<string,string> &nsprefix, map<string,string> prefixns, ostream *dependencies) {
  // overwrite existing namespace prefixes with new ones
  // save a list of ALL ns->prefix mappings in nsprefix (this map can be used in unIncorporateNamespace)
  TiXmlAttribute* a=e->FirstAttribute();
  while(a!=NULL) {
    TiXmlAttribute* aNext=a->Next();
    string ns=a->ValueStr();
    string prefix="{NOTAPREFIX}";
    if(strncmp(a->Name(),"xmlns:",6)==0) // none default prefix
      prefix=a->Name()+6;
    else if(strcmp(a->Name(),"xmlns")==0) // default prefix
      prefix="";
    if(prefix=="{NOTAPREFIX}") { a=aNext; continue; }
    prefixns[prefix]=ns;
    if(nsprefix.find(ns)==nsprefix.end()) {
      int i=0;
      tinyNamespaceCompStr=prefix;
      while(find_if(nsprefix.begin(), nsprefix.end(), comp)!=nsprefix.end()) {
        stringstream istr; istr<<++i;
        tinyNamespaceCompStr=prefix+"_"+istr.str();
      }
      nsprefix[ns]=tinyNamespaceCompStr;
    }
    e->RemoveAttribute(a->Name());
    a=aNext;
  }

  // set element name to '{<namespace>}<localname>'
  for(map<string,string>::iterator i=prefixns.begin(); i!=prefixns.end(); i++) {
    // none default prefix
    if(e->ValueStr().compare(0,(*i).first.length()+1,(*i).first+":")==0) {
      e->SetValue("{"+(*i).second+"}"+e->ValueStr().substr((*i).first.length()+1));
      break;
    }
    // default prefix
    if(e->ValueStr().find(":")==string::npos) {
      e->SetValue("{"+(*i).second+"}"+e->ValueStr());
      break;
    }
  }

  if(e->ValueStr()==XINCLUDENS"include") {
    string newFile=fixPath(e->GetDocument()->ValueStr(), e->Attribute("href"));
    if(dependencies!=NULL)
      (*dependencies)<<newFile<<endl;
    // for a xi:include element include the href file in the tree
    TiXmlDocument docInclude;
    docInclude.LoadFile(newFile);
    map<string, string> dummy;
    incorporateNamespace(docInclude.FirstChildElement(), nsprefix, dummy, dependencies);
    docInclude.FirstChildElement()->SetAttribute("xml:base", newFile);

    // include a processing instruction with the line number of the original element
    TiXmlUnknown xincLine;
    xincLine.SetValue("?OriginalElementLineNr "+TiXml_itoa(e->Row())+"?");
    docInclude.FirstChildElement()->InsertBeforeChild(docInclude.FirstChildElement()->FirstChild(), xincLine);

    e->Parent()->InsertAfterChild(e,*docInclude.FirstChildElement());
    e->Parent()->RemoveChild(e);
  }
  else {
    // apply recusively for all child elements
    TiXmlElement* c=e->FirstChildElement();
    while(c!=0) {
      TiXmlElement* cNext=c->NextSiblingElement();
      incorporateNamespace(c, nsprefix, prefixns, dependencies);
      c=cNext;
    }
  }
}

void unIncorporateNamespace(TiXmlElement *e, map<string,string>& nsprefix, bool firstCall) {
  // extract namespace form element name
  string ns=e->ValueStr().substr(1,e->ValueStr().find('}')-1);

  // if the namespace of the element is not found in the map add a unique dummy prefix
  if(nsprefix.find(ns)==nsprefix.end()) {
    string prefix="ns";
    int i=1;
    tinyNamespaceCompStr=prefix+"_1";
    while(find_if(nsprefix.begin(), nsprefix.end(), comp)!=nsprefix.end()) {
      stringstream istr; istr<<++i;
      tinyNamespaceCompStr=prefix+"_"+istr.str();
    }
    nsprefix[ns]=tinyNamespaceCompStr;
  }

  // set element name to '<nsprefix>:<localname>'
  e->SetValue(nsprefix[ns]+":"+e->ValueStr().substr(ns.length()+2));
  // delete the leading ':' in case of the default namespace prefix
  if(e->ValueStr().substr(0,1)==":")
    e->SetValue(e->ValueStr().substr(1));

  // apply recusively for all child elements
  TiXmlElement* c=e->FirstChildElement();
  while(c!=0) {
    unIncorporateNamespace(c, nsprefix, false);
    c=c->NextSiblingElement();
  }

  // add namespace prefix attribute to the root element: 'xmlns:...=...'
  if(firstCall)
    for(map<string,string>::iterator i=nsprefix.begin(); i!=nsprefix.end(); i++) {
      if((*i).second!="")
        e->SetAttribute("xmlns:"+(*i).second, (*i).first);
      else
        e->SetAttribute("xmlns", (*i).first);
    }
}

string fixPath(string oldFile, string newFile) {
  // fix relative path name of file to be included (will hopefully work also on windows)
  // if new file has a relative path
  if(!( newFile[0]=='/' ||
       (((newFile[0]>='a' && newFile[0]<='z') || (newFile[0]>='A' && newFile[0]<='Z')) && newFile[1]==':') ||
       (newFile[0]=='\\' && newFile[1]=='\\'))) {
    // find last slash or backslash of old file
    int i=oldFile.find_last_of('/');
    int i2=oldFile.find_last_of('\\');
    i=(i>i2)?i:i2;
    // if old file has a path, prefix new file with this path
    if(i>=0) newFile=oldFile.substr(0,i)+"/"+newFile;
  }
  return newFile;
}
