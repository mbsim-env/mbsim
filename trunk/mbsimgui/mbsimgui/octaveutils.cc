#include "octaveutils.h"
#include "mbsimguitinyxml/tinyxml-src/tinyxml.h"
#include "mbsimguitinyxml/tinyxml-src/tinynamespace.h"
#include <unistd.h>
#ifdef HAVE_UNORDERED_MAP
#  include <unordered_map>
#else
#  include <map>
#  define unordered_map map
#endif
#ifdef HAVE_UNORDERED_SET
#  include <unordered_set>
#else
#  include <set>
#  define unordered_set set
#endif

using namespace std;

string SCHEMADIR;

string nslocation;

int machinePrec;

int orgstderr;
streambuf *orgcerr;
int disableCount=0;
// disable output off stderr (including stack)
void disable_stderr() {
  if(disableCount==0) {
    orgcerr=std::cerr.rdbuf(0);
    orgstderr=dup(fileno(stderr));
#ifdef _WIN32
    if(freopen("nul", "w", stderr)==0) throw(1);
#else
    if(freopen("/dev/null", "w", stderr)==0) throw(1);
#endif
  }
  disableCount++;
}
// enable output off stderr (including stack)
void enable_stderr() {
  disableCount--;
  if(disableCount==0) {
    std::cerr.rdbuf(orgcerr);
    dup2(orgstderr, fileno(stderr));
    close(orgstderr);
  }
}

// map of the current parameters
map<string, octave_value> currentParam;
// stack of parameters
stack<map<string, octave_value> > paramStack;
vector<int> currentParamHash;

// add a parameter to the parameter list (used by octavePushParam and octavePopParams)
void octaveAddParam(const string &paramName, const octave_value& value) {
  // add paramter to parameter list if a parameter of the same name dose not exist in the list
  currentParam[paramName]=value;
  if(paramStack.size()>=currentParamHash.size()) {
    currentParamHash.resize(paramStack.size()+1);
    currentParamHash[paramStack.size()]=0;
  }
  currentParamHash[paramStack.size()]++;
}

// push all parameters from list to a parameter stack
void octavePushParams() {
  paramStack.push(currentParam);
}

// pop all parameters from list from the parameter stack
void octavePopParams() {
  // restore previous parameter list
  currentParam=paramStack.top();
  paramStack.pop();
}

#define PATHLENGTH 10240
// evaluate a single statement or a statement list and save the result in the variable 'ret'
void octaveEvalRet(string str, TiXmlElement *e) {
  // delete leading new lines in str
  for(unsigned int j=0; j<str.length() && (str[j]==' ' || str[j]=='\n' || str[j]=='\t'); j++)
    str[j]=' ';
  // delete trailing new lines in str
  for(unsigned int j=str.length()-1; j>=0 && (str[j]==' ' || str[j]=='\n' || str[j]=='\t'); j--)
    str[j]=' ';

  // a cache
  if(paramStack.size()>=currentParamHash.size()) {
    currentParamHash.resize(paramStack.size()+1);
    currentParamHash[paramStack.size()]=0;
  }
  stringstream s; s<<paramStack.size()<<";"<<currentParamHash[paramStack.size()]<<";"; string id=s.str();
  static unordered_map<string, octave_value> cache;
  pair<unordered_map<string, octave_value>::iterator, bool> ins=cache.insert(pair<string, octave_value>(id+str, octave_value()));
  if(!ins.second) {
    symbol_table::varref("ret")=ins.first->second;
    return;
  }

  // clear octave
  symbol_table::clear_variables();
  // restore current parameters
  for(map<string, octave_value>::iterator i=currentParam.begin(); i!=currentParam.end(); i++)
    symbol_table::varref(i->first)=i->second;

  char savedPath[PATHLENGTH];
  if(e) { // set working dir to path of current file, so that octave works with correct relative paths
    if(getcwd(savedPath, PATHLENGTH)==0) throw(1);
    if(chdir(fixPath(TiXml_GetElementWithXmlBase(e,0)->Attribute("xml:base"),".").c_str())!=0) throw(1);
  }

  int dummy;
  disable_stderr();
  eval_string("ret="+str,true,dummy); // eval as single statement, and save in 'ret'
  enable_stderr();
  if(error_state!=0) { // if error, maybe it is a statement list
    error_state=0;
    eval_string(str,true,dummy,0); // eval as statement list
    if(error_state!=0) { // if error => wrong code => throw error
      error_state=0;
      if(e) if(chdir(savedPath)!=0) throw(1);
      throw string("Error in octave expression: "+str);
    }
    if(!symbol_table::is_variable("ret")) {
      cout<<"ERRRO\n";
      error_state=0;
      if(e) if(chdir(savedPath)!=0) throw(1);
      throw string("'ret' not defined in octave statement list: "+str);
    }
  }
  if(e) if(chdir(savedPath)!=0) throw(1);

  ins.first->second=symbol_table::varval("ret"); // add to cache
}

void checkType(const octave_value& val, ValueType expectedType) {
  ValueType type;
  // get type of val
  if(val.is_scalar_type() && val.is_real_type() && (val.is_string()!=1))
    type=ScalarType;
  else if(val.is_matrix_type() && val.is_real_type() && (val.is_string()!=1)) {
    Matrix m=val.matrix_value();
    type=m.cols()==1?VectorType:MatrixType;
  }
  else if(val.is_string())
    type=StringType;
  else // throw on unknown type
    throw(string("Unknown type: none of scalar, vector, matrix or string"));
  // check for correct type
  if(expectedType!=ArbitraryType) {
    if(type==ScalarType && expectedType==StringType)
      throw string("Got scalar value, while a string is expected");
    if(type==VectorType && (expectedType==StringType || expectedType==ScalarType)) 
      throw string("Got column-vector value, while a ")+(expectedType==StringType?"string":"scalar")+" is expected";
    if(type==MatrixType && (expectedType==StringType || expectedType==ScalarType || expectedType==VectorType))
      throw string("Got matrix value, while a ")+(expectedType==StringType?"string":(expectedType==ScalarType?"scalar":"column-vector"))+" is expected";
    if(type==StringType && (expectedType==MatrixType || expectedType==ScalarType || expectedType==VectorType))
      throw string("Got string value, while a ")+(expectedType==MatrixType?"matrix":(expectedType==ScalarType?"scalar":"column-vector"))+" is expected";
  }
}

// return the value of 'ret'
string octaveGetRet(ValueType expectedType) {
  octave_value o=symbol_table::varval("ret"); // get 'ret'

  ostringstream ret;
  ret.precision(machinePrec);
  if(o.is_scalar_type() && o.is_real_type() && (o.is_string()!=1)) {
    ret<<o.double_value();
  }
  else if(o.is_matrix_type() && o.is_real_type() && (o.is_string()!=1)) {
    Matrix m=o.matrix_value();
    ret<<"[";
    for(int i=0; i<m.rows(); i++) {
      for(int j=0; j<m.cols(); j++)
        ret<<m(j*m.rows()+i)<<(j<m.cols()-1?",":"");
      ret<<(i<m.rows()-1?" ; ":"]");
    }
  }
  else if(o.is_string()) {
    ret<<"\""<<o.string_value()<<"\"";
  }
  else { // if not scalar, matrix or string => error
    throw(string("Unknown type: none of scalar, vector, matrix or string"));
  }
  checkType(o, expectedType);

  return ret.str();
}

int fillParam(TiXmlElement *e) {
  // generate a vector of parameters
  vector<Param> param;
  for(TiXmlElement *ee=e->FirstChildElement(); ee!=0; ee=ee->NextSiblingElement())
    param.push_back(Param(ee->Attribute("name"), ee->GetText(), ee));
  fillParam(param);
}
// fill octave with parameters
int fillParam(vector<Param> param) {
  // generate a vector of parameters

  // outer loop to resolve recursive parameters
  for(vector<Param>::iterator j=param.begin(); j!=param.end(); j++)
    // evaluate parameter
    for(vector<Param>::iterator i=j; i!=param.end(); i++) {
      disable_stderr();
      int err=0;
      octave_value ret;
      try { 
        octaveEvalRet(i->equ, i->ele);
        ret=symbol_table::varval("ret");
       // checkType(ret, i->ele->ValueStr()=="{http://openmbv.berlios.de/MBXMLUtils/parameter}scalarParameter"?ScalarType:
       //                i->ele->ValueStr()=="{http://openmbv.berlios.de/MBXMLUtils/parameter}vectorParameter"?VectorType:
       //                i->ele->ValueStr()=="{http://openmbv.berlios.de/MBXMLUtils/parameter}matrixParameter"?MatrixType:
       //                i->ele->ValueStr()=="{http://openmbv.berlios.de/MBXMLUtils/parameter}stringParameter"?StringType:ArbitraryType);
      }
      catch(...) { err=1; }
      enable_stderr();
      if(err==0) { // if no error
        octaveAddParam(i->name, ret); // add param to list
        vector<Param>::iterator isave=i-1; // delete param from vector
        bool restorej=j==i;
        param.erase(i);
        i=isave;
        if(restorej) j=isave;
      }
    }
  if(param.size()>0) { // if parameters are left => error
    cout<<"Error in one of the following parameters or infinit loop in this parameters:\n";
    for(size_t i=0; i<param.size(); i++) {
      try {
        octaveEvalRet(param[i].equ, param[i].ele); // output octave error
        octave_value ret=symbol_table::varval("ret");
        checkType(ret, param[i].ele->ValueStr()=="{http://openmbv.berlios.de/MBXMLUtils/parameter}scalarParameter"?ScalarType:
                       param[i].ele->ValueStr()=="{http://openmbv.berlios.de/MBXMLUtils/parameter}vectorParameter"?VectorType:
                       param[i].ele->ValueStr()=="{http://openmbv.berlios.de/MBXMLUtils/parameter}matrixParameter"?MatrixType:
                       param[i].ele->ValueStr()=="{http://openmbv.berlios.de/MBXMLUtils/parameter}stringParameter"?StringType:ArbitraryType);
      }
      catch(string str) { cout<<str<<endl; }
      TiXml_location(param[i].ele, "", ": "+param[i].name+": "+param[i].equ); // output location of element
    }
    return 1;
  }

  return 0;
}

//int embed(TiXmlElement *&e, map<string,string> &nsprefix, map<string,string> &units, ostream &dependencies) {
//  try {
//    if(e->ValueStr()==MBXMLUTILSPVNS"embed") {
//      octavePushParams();
//      // check if only href OR child element (This is not checked by the schema)
//      TiXmlElement *l=0, *dummy;
//      for(dummy=e->FirstChildElement(); dummy!=0; l=dummy, dummy=dummy->NextSiblingElement());
//      if((e->Attribute("href") && l && l->ValueStr()!=MBXMLUTILSPVNS"localParameter") ||
//         (e->Attribute("href")==0 && (l==0 || l->ValueStr()==MBXMLUTILSPVNS"localParameter"))) {
//        TiXml_location(e, "", ": Only the href attribute OR a child element (expect pv:localParameter) is allowed in embed!");
//        return 1;
//      }
//      // check if attribute count AND counterName or none of both
//      if((e->Attribute("count")==0 && e->Attribute("counterName")!=0) ||
//         (e->Attribute("count")!=0 && e->Attribute("counterName")==0)) {
//        TiXml_location(e, "", ": Only both, the count and counterName attribute must be given or none of both!");
//        return 1;
//      }
//  
//      // get file name if href attribute exist
//      string file="";
//      if(e->Attribute("href")) {
//        file=fixPath(TiXml_GetElementWithXmlBase(e,0)->Attribute("xml:base"), e->Attribute("href"));
//        dependencies<<file<<endl;
//      }
//  
//      // get onlyif attribute if exist
//      string onlyif="1";
//      if(e->Attribute("onlyif"))
//        onlyif=e->Attribute("onlyif");
//  
//      // evaluate count using parameters
//      int count=1;
//      if(e->Attribute("count")) {
//        octaveEvalRet(e->Attribute("count"), e);
//        octave_value v=symbol_table::varval("ret");
//        checkType(v, ScalarType);
//        count=int(round(v.double_value()));
//      }
//  
//      // couter name
//      string counterName="MBXMLUtilsDummyCounterName";
//      if(e->Attribute("counterName"))
//        counterName=e->Attribute("counterName");
//  
//      TiXmlDocument *enewdoc=NULL;
//      TiXmlElement *enew;
//      // validate/load if file is given
//      if(file!="") {
//        if(validate(nslocation, file)!=0) {
//          TiXml_location(e, "  included by: ", "");
//          return 1;
//        }
//        cout<<"Read "<<file<<endl;
//        TiXmlDocument *enewdoc=new TiXmlDocument;
//        enewdoc->LoadFile(file.c_str()); TiXml_PostLoadFile(enewdoc);
//        enew=enewdoc->FirstChildElement();
//        map<string,string> dummy;
//        incorporateNamespace(enew, nsprefix, dummy, &dependencies);
//        // convert embeded file to octave notation
//        cout<<"Process xml[Matrix|Vector] elements in "<<file<<endl;
//        if(toOctave(enew)!=0) {
//          TiXml_location(e, "  included by: ", "");
//          return 1;
//        }
//      }
//      else { // or take the child element (as a clone, because the embed element is deleted)
//        if(e->FirstChildElement()->ValueStr()==MBXMLUTILSPVNS"localParameter")
//          enew=(TiXmlElement*)e->FirstChildElement()->NextSiblingElement()->Clone();
//        else
//          enew=(TiXmlElement*)e->FirstChildElement()->Clone();
//        enew->SetAttribute("xml:base", TiXml_GetElementWithXmlBase(e,0)->Attribute("xml:base")); // add a xml:base attribute
//      }
//  
//      // include a processing instruction with the line number of the original element
//      TiXmlUnknown embedLine;
//      embedLine.SetValue("?OriginalElementLineNr "+TiXml_itoa(e->Row())+"?");
//      enew->InsertBeforeChild(enew->FirstChild(), embedLine);
//  
//  
//      // generate local paramter for embed
//      if(e->FirstChildElement() && e->FirstChildElement()->ValueStr()==MBXMLUTILSPVNS"localParameter") {
//        // check if only href OR p:parameter child element (This is not checked by the schema)
//        if((e->FirstChildElement()->Attribute("href") && e->FirstChildElement()->FirstChildElement()) ||
//           (!e->FirstChildElement()->Attribute("href") && !e->FirstChildElement()->FirstChildElement())) {
//          TiXml_location(e->FirstChildElement(), "", ": Only the href attribute OR the child element p:parameter) is allowed here!");
//          return 1;
//        }
//        cout<<"Generate local octave parameter string for "<<(file==""?"<inline element>":file)<<endl;
//        if(e->FirstChildElement()->FirstChildElement()) // inline parameter
//          fillParam(e->FirstChildElement()->FirstChildElement());
//        else { // parameter from href attribute
//          string paramFile=fixPath(TiXml_GetElementWithXmlBase(e,0)->Attribute("xml:base"), e->FirstChildElement()->Attribute("href"));
//          // add local parameter file to dependencies
//          dependencies<<paramFile<<endl;
//          // validate local parameter file
//          if(validate(SCHEMADIR+"/http___openmbv_berlios_de_MBXMLUtils/parameter.xsd", paramFile)!=0) {
//            TiXml_location(e->FirstChildElement(), "  included by: ", "");
//            return 1;
//          }
//          // read local parameter file
//          cout<<"Read local parameter file "<<paramFile<<endl;
//          TiXmlDocument *localparamxmldoc=new TiXmlDocument;
//          localparamxmldoc->LoadFile(paramFile.c_str()); TiXml_PostLoadFile(localparamxmldoc);
//          TiXmlElement *localparamxmlroot=localparamxmldoc->FirstChildElement();
//          map<string,string> dummy,dummy2;
//          incorporateNamespace(localparamxmlroot,dummy,dummy2,&dependencies);
//          // generate local parameters
//          fillParam(localparamxmlroot);
//          delete localparamxmldoc;
//        }
//      }
//  
//      // delete embed element and insert count time the new element
//      for(int i=1; i<=count; i++) {
//        // embed only if 'onlyif' attribute is true
//        
//        octave_value o((double)i);
//        octaveAddParam(counterName, o);
//        octaveEvalRet(onlyif, e);
//        octave_value v=symbol_table::varval("ret");
//        checkType(v, ScalarType);
//        if(round(v.double_value())==1) {
//          cout<<"Embed "<<(file==""?"<inline element>":file)<<" ("<<i<<"/"<<count<<")"<<endl;
//          if(i==1) e=(TiXmlElement*)(e->Parent()->ReplaceChild(e, *enew));
//          else e=(TiXmlElement*)(e->Parent()->InsertAfterChild(e, *enew));
//  
//          // include a processing instruction with the count number
//          TiXmlUnknown countNr;
//          countNr.SetValue("?EmbedCountNr "+TiXml_itoa(i)+"?");
//          e->InsertAfterChild(e->FirstChild(), countNr);
//      
//          // apply embed to new element
//          if(embed(e, nsprefix, units, dependencies)!=0) return 1;
//        }
//        else
//          cout<<"Skip embeding "<<(file==""?"<inline element>":file)<<" ("<<i<<"/"<<count<<"); onlyif attribute is false"<<endl;
//      }
//      if(enewdoc)
//        delete enewdoc;
//      else
//        delete enew;
//      octavePopParams();
//      return 0;
//    }
//    else {
//      // THIS IS A WORKAROUND! Actually not all Text-Elements should be converted but only the Text-Elements
//      // of XML elementx of a type devived from pv:scalar, pv:vector, pv:matrix and pv:string. But for that a
//      // schema aware processor is needed!
//      if(e->GetText()) {
//        // eval text node
//        octaveEvalRet(e->GetText(), e);
//        // convert unit
//        if(e->Attribute("unit") || e->Attribute("convertUnit")) {
//          map<string, octave_value> savedCurrentParam;
//          savedCurrentParam=currentParam; // save parameters
//          currentParam.clear(); // clear parameters
//          octaveAddParam("value", symbol_table::varval("ret")); // add 'value=ret', since unit-conversion used 'value'
//          if(e->Attribute("unit")) { // convert with predefined unit
//            octaveEvalRet(units[e->Attribute("unit")]);
//            e->RemoveAttribute("unit");
//          }
//          if(e->Attribute("convertUnit")) { // convert with user defined unit
//            octaveEvalRet(e->Attribute("convertUnit"));
//            e->RemoveAttribute("convertUnit");
//          }
//          currentParam=savedCurrentParam; // restore parameter
//        }
//        // wrtie eval to xml
//        e->FirstChild()->SetValue(octaveGetRet());
//        e->FirstChild()->ToText()->SetCDATA(false);
//      }
//    
//      // THIS IS A WORKAROUND! Actually not all 'name' and 'ref*' attributes should be converted but only the
//      // XML attributes of a type devived from pv:fullOctaveString and pv:partialOctaveString. But for that a
//      // schema aware processor is needed!
//      for(TiXmlAttribute *a=e->FirstAttribute(); a!=0; a=a->Next())
//        if(a->Name()==string("name") || string(a->Name()).substr(0,3)=="ref") {
//          string s=a->ValueStr();
//          int i;
//          while((i=s.find('{'))>=0) {
//            int j=s.find('}');
//            octaveEvalRet(s.substr(i+1,j-i-1), e);
//            s=s.substr(0,i)+octaveGetRet(ScalarType)+s.substr(j+1);
//          }
//          a->SetValue(s);
//        }
//    }
//  
//    TiXmlElement *c=e->FirstChildElement();
//    while(c) {
//      if(embed(c, nsprefix, units, dependencies)!=0) return 1;
//      c=c->NextSiblingElement();
//    }
//  }
//  catch(string str) {
//    TiXml_location(e, "", ": "+str);
//    return 1;
//  }
// 
//  return 0;
//}

void initializeOctave() {
   string OCTAVEDIR="/home/foerg/install/share/mbxmlutils/octave"; // default: from build configuration

    // initialize octave
    char **octave_argv=(char**)malloc(2*sizeof(char*));
    octave_argv[0]=(char*)malloc(6*sizeof(char*)); strcpy(octave_argv[0], "dummy");
    octave_argv[1]=(char*)malloc(3*sizeof(char*)); strcpy(octave_argv[1], "-q");
    octave_main(2, octave_argv, 1);
    int dummy;
    eval_string("warning('error','Octave:divide-by-zero');",true,dummy,0); // statement list
    eval_string("addpath('"+OCTAVEDIR+"');",true,dummy,0); // statement list
  
    // calcaulate machine precision
    double machineEps;
    for(machineEps=1.0; (1.0+machineEps)>1.0; machineEps*=0.5);
    machineEps*=2.0;
    machinePrec=(int)(-log(machineEps)/log(10))+1;
}


