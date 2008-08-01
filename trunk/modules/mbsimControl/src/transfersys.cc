/* Copyright (C) 2006 Mathias Bachmayer
 *
 * Institute of Applied Mechanics
 * Technical University of Munich

 *
 * Contact:
 *   bachmayer@amm.mw.tum.de
 *
 */ 
#include <config.h>
#include "transfersys.h"

#ifdef MBSIMXML
#include <xercesc/dom/DOMNode.hpp>
#include <xercesc/dom/DOMElement.hpp>
#include <xmlutils.h>
using namespace XERCES_CPP_NAMESPACE;
#endif  

TransferSys::TransferSys(const string& name) : SPSys(name) {
    ABCDdefined=false;
    R1=0.002;
    R2=1;
    c=1;
    modus=" ";
    Single_Input=true;
}


void TransferSys::updatedx(double t, double dt){
    xd=(A*x+B*(this->*Uin)(t))*dt;
}
void TransferSys::updatexd(double t){
    xd=A*x+B*(this->*Uin)(t);
    }

void TransferSys::updateStage1(double t){
    y=(this->*OutForm)(t);
    //TFOutputSignal->setSignal(y);
    }
//OUTCD SYS DEFINITIONEN ***************************************************
Vec TransferSys::OutC(double t){
    return C*x;
    }
Vec TransferSys::OutD(double t){
    return D*(this->*Uin)(t);
    }

Vec TransferSys::OutCD(double t){
    return C*x+D*(this->*Uin)(t);
    }

Vec TransferSys::OutCDD(double t){
    if (t>Tt){
    Dd=((this->*Uin)(t)-dz)/(t-Tt);
    dz=(this->*Uin)(t);
    Tt=t;
    }
    return C*x+D*(this->*Uin)(t)+DFactor*Dd;
    }


//OUTCDFORMEN ENDE-----------------------------------------------------------------------

// SETABCDAREA ANFANG
void TransferSys::showABCD(){
    cout<<"State space modell of Object \""<<name<<"\":"<<endl;
    cout<<"A Matrix:"<<A<<endl;
    cout<<"B Matrix:"<<B<<endl;
    cout<<"C Matrix:"<<C<<endl;
    cout<<"D Matrix:"<<D<<endl;
    cout<<"D Factor:"<<DFactor<<endl;  
    cout<<"Weightvectors:"<<endl;
    cout<<"ktF"<<kTF<<endl;
    cout<<"kPos"<<kPos<<endl;
    cout<<"kVin"<<kVin<<endl;
    cout<<"Number of systeminputs:"<<NumberofInputs<<endl;
    cout<<"Elements stored in signalinputvector:"<<SigInputs.size()<<endl;
    cout<<"Elements stored in positioninputvector:"<<PositionInputs.size()<<endl;
    cout<<"Elements stored in velocityinputvector:"<<VInputs.size()<<endl;
    }

void TransferSys::setABCD(Mat A_,Mat B_,Mat C_,Mat D_){
modus="ABCD definition mode (setABCD)";
    if (ABCDdefined) 
    {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setABCD redefiniert!"<<endl;
    cout<<"Redefinition happens in Object: "<<name<<endl;
    throw 45;
    }

    ABCDdefined=true;

    A=A_;
    B=B_;
    C=C_;
    D=D_;

    xSize=A.rows();
    x=Vec(xSize,INIT,0);
    if (A.rows()!=B.rows()||C.rows()!=D.rows()||B.cols()!=D.cols()||A.cols()!=C.cols())
    { 
    showABCD();
    cout<<"ERROR: In Object with name \""<<name<<"\" is a conflict in Matrix dimensions!!!!! "<<endl;
    cout<<"Please provide Matrices with Dimensions for a standard StateSpace Model!"<<endl;
    throw 4;
    }
    if (A.rows()!=A.cols())
    {
    cout<<"ERROR: A Matrix in Object \""<<name<<"\" must be a square Matrix!"<<endl;
    throw 5; 
    }

    OutForm=&TransferSys::OutCD;
}

void TransferSys::setPID(double P_, double I_, double D_){
modus+="PID Controller Mode (setPID)";

    if (D_!=0){OutForm=&TransferSys::OutCDD;} else {if (I_!=0) {OutForm=&TransferSys::OutCD;} else {OutForm=&TransferSys::OutD;}}
 
    if (ABCDdefined) {
    cout<<"WARNING!!!! State space matrices ABCD are going to be redefined by setPID!"<<endl;
    cout<<"Redefinition in Object: "<<name<<endl;
    throw 34;
    }

    ABCDdefined=true;

    Mat MA(1,1);
    Mat MB(1,1);
    Mat MC(1,1);
    Mat MD(1,1);
    MA(0,0)=0;
    MB(0,0)=1;
    MC(0,0)=I_;
    MD(0,0)=P_;
    A=MA;
    B=MB;
    C=MC;
    D=MD;
    xSize=A.rows();
    x=Vec(xSize,INIT,0); 
    Vec VD(1);
    VD(0)=D_;
    
    DFactor=VD;
    VD(0)=0;
    Dd=VD; //Init der Hilfsvariablen zur
    dz=VD; //numerischen Differentiation
    Tt=0;
}

void TransferSys::setPIDKausal(double P_, double I_, double D_){
modus+="PID Regler Modus (setPIDKausal)";

 //   OutForm=&TransferSys::OutCD;
 if (D_!=0){OutForm=&TransferSys::OutCD;} else {if (I_!=0) {OutForm=&TransferSys::OutCD;} else {OutForm=&TransferSys::OutD;}}
 

    if (ABCDdefined) {
    cout<<"ERROR!!!! Zustandsraummatrizen ABCD werden von setPID redefiniert!"<<endl;
    cout<<"Redefinition in Objekt: "<<name<<endl;
    throw 34;
    }

    ABCDdefined=true;
if (D_==0)
{
    Mat MA(1,1);
    Mat MB(1,1);
    Mat MC(1,1);
    Mat MD(1,1);
    MA(0,0)=0;
    MB(0,0)=1;
    MC(0,0)=I_;
    MD(0,0)=P_;
    A=MA;
    B=MB;
    C=MC;
    D=MD;
}
else
{  
    
    Mat MA(2,2);
    Mat MB(2,1);
    Mat MC(1,2);
    Mat MD(1,1);
    MA(1,1)=-1/(R1*c);
    MB(0,0)=1;
    MB(1,0)= 1/(R1*c);
    MC(0,0)=I_;
    MC(0,1)=-D_*R2*c/(R1*c);
    MD(0,0)=P_+D_*R2*c/(R1*c);
    A=MA;
    B=MB;
    C=MC;
    D=MD;
    
}   
    xSize=A.rows();
    x=Vec(xSize,INIT,0); 
    //Vec VD(1);
    //VD(0)=D_;
    
    //DFactor=VD;
    //VD(0)=0;
    //Dd=VD; //Init der Hilfsvariablen zur
    //dz=VD; //numerischen Differentiation
    //Tt=0;
    }



void TransferSys::setBandwidth(double Hz_fg)
{
double omegag;
if (Hz_fg>0){
  omegag=2*3.141592654*Hz_fg;
  R1=1/(omegag*c);
  R2=sqrt(R1*R1+1/(c*c));
 }
 else
 { 
  cout<<"Error! As internal Derive Bandwith is a positive nun zero frequency required!"<<endl;
  cout<<"Non valid Argument of Method setBandwidth: "<<Hz_fg<<endl;
  cout<<"Message Source: Object "<<name<<endl;
  throw 55;
 }
}

void TransferSys::setDECoeffs(Vec Numerator, Vec Denominator)
{modus+="Differential equation's coeffizients defined mode(setDECoeffs)";
    if (ABCDdefined) {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setDECoeffs!"<<endl;cout<<"Redefinition in object: "<<name<<endl;
    throw 34;
     }
    ABCDdefined=true;
 
    Vec Rest;
    //Normierung
     double Normkonstante;
	Normkonstante=PolyNormierung(Denominator);  
	 Numerator=Numerator/Normkonstante;
    //Normierungsende
 
    if (Numerator.rows()>Denominator.rows()) 
    { 
	Vec Zwerg(Numerator.rows()-1);
	cout<<"Zählergrad größer Nennergrad! =>System hat differenzierenden Charakter!"<<endl;
	if (Numerator.rows()>Denominator.rows()+1) {cout<<name<<" : System requires twice time derivatives, but maximal one time derivative is supportet in this version"<<endl; throw 55;}

     Rest=PolynomDivision(Numerator,Denominator,Zwerg);
     OutForm=&TransferSys::OutCDD;

        Vec Dint(1); Dint(0)=Zwerg(1); //DFactor 

        Mat durch(1,1); durch(0,0)=Zwerg(0); //Durchgriff

        Mat Cint(1,Denominator.rows()-1); Cint.row(0)(0,Rest.rows()-1)=trans(Rest(0,Rest.rows()-1)); //Abgriff

        Mat Mziel(Cint.cols(),Cint.cols());
	    if (Cint.cols()>1){
	    DiagMat DAM(Mziel.cols()-1,INIT,1); 
	    Mziel(Index(0,Mziel.rows()-2),Index(1,Mziel.cols()-1))<<DAM; //RNForm die erste...
	    Mziel.row(Mziel.rows()-1) = -trans(Denominator(0,Denominator.rows()-2));//Systemmatrix die zweite!
	    } else 	
	    {Mziel(0,0)=-Denominator(0);} //Systemmatrix falls maximal 1 Zustand!
	Mat Bziel(Cint.cols(),1); Bziel(Cint.cols()-1,0)=1;//Eingriffsmatrix
        A=Mziel;
	B=Bziel;
        C=Cint;
	D=durch;
	DFactor=Dint;
	Vec V(1);
	V(0)=0;
	Dd=V;
	dz=V;
	Tt=0;
    }
    else 
    {
	Vec Zwerg(Numerator.rows());
    if (Numerator.rows()==Denominator.rows())
       { 
       //cout<<"Zählergrad ist gleich Nennergrad!=> System hat Durchgriff"<<endl;
       Rest=PolynomDivision(Numerator,Denominator,Zwerg);

       OutForm=&TransferSys::OutCD;
           
        Mat durch(1,1); durch(0,0)=Zwerg(0); //Durchgriff
	Mat Cint;
	    if (Denominator.rows()>1)
	    {Mat Ci(1,Denominator.rows()-1);Cint=Ci;} 
	    else 
	    {Mat Ci(1,1);Cint=Ci;}
	if (Rest.rows()>0)
	{Cint.row(0)(0,Rest.rows()-1)=trans(Rest(0,Rest.rows()-1));} //Abgriff
        
        Mat Mziel(Cint.cols(),Cint.cols());
        if (Cint.cols()>1)
	 {DiagMat DAM(Mziel.cols()-1,INIT,1);
	  Mziel(Index(0,Mziel.rows()-2),Index(1,Mziel.cols()-1))<<DAM;
          Mziel.row(Mziel.rows()-1) = -trans(Denominator(0,Denominator.rows()-2));//Systemmatrix
         } 
	else 
	    {Mziel(0,0)=-Denominator(0);} //Systemmatrix
	
	Mat Bziel(Cint.cols(),1);Bziel(Cint.cols()-1,0)=1;//Eingriff
       
	A=Mziel;
	B=Bziel;
        C=Cint;
	D=durch;
       }
       else
       {
       //cout<<"Zählergrad kleiner als Nennergrad! => System hat keinen Durchgriff!"<<endl;
       Rest=PolynomDivision(Numerator,Denominator,Zwerg);
       //cout<<Rest<<Numerator<<Nenner<<Zwerg<<endl; 
       OutForm=&TransferSys::OutC;


        Mat Cint(1,Denominator.rows()-1); Cint.row(0)(0,Rest.rows()-1)=trans(Rest(0,Rest.rows()-1)); //Abgriff
        Mat Mziel(Cint.cols(),Cint.cols());
	    if (Cint.cols()>1){
	        DiagMat DAM(Mziel.cols()-1,INIT,1);
	        Mziel(Index(0,Mziel.rows()-2),Index(1,Mziel.rows()-1))<<DAM;
	        Mziel.row(Mziel.rows()-1) = -trans(Denominator(0,Denominator.rows()-2));//Systemmatrix
		} 
	    else 
	      {Mziel(0,0)=-Denominator(0);} //Systemmatrix
        Mat Bziel(Cint.cols(),1); Bziel(Cint.cols()-1,0)=1;
        A=Mziel;
	B=Bziel;
        C=Cint;
       }
    }

xSize=A.rows();
x=Vec(xSize,INIT,0);
}

void TransferSys::setPolesnZerosnGain(Vec Poles,Vec Zeros,double Gain)
{ modus+="Poles and zeros defined mode (setPolesnZeros)";
    if (ABCDdefined) {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined setPolesnZeros!"<<endl;cout<<"Redefinition in object: "<<name<<endl;
    throw 34;
     }
    if ((Poles.rows()>0)){
	
    Vec Denominator;
    Vec Numerator;
    Denominator=Pole2Koeff(Poles);
    if (Zeros.rows()>0) {Numerator=Pole2Koeff(-Zeros);} else {Vec Z(1); Z(0)=1; Numerator=Z;}
        Numerator=Gain*Numerator;
   // cout<<"Uebertragungsfunktion des Systems"<<name<<endl;
   // cout<<"Zaehlerpolynom :";PolynomVis(Numerator);cout<<endl;
   // cout<<"Nennerpolynom :";PolynomVis(Denominator);cout<<endl;
    setDECoeffs(Numerator,Denominator);
    }
    else
    cout<<"Error Tilde Tilde in Object "<<name<<": At least one Pole is required for using this function!"<<endl;
}

void TransferSys::setIntegrator(double OutputGain){
 modus+="Integrator mode";
    if (ABCDdefined) {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setIntegrator!"<<endl;cout<<"Redefinition in Objekt: "<<name<<endl;
    throw 34;
     }
    ABCDdefined=true;
    xSize=1;
    x=Vec(xSize,INIT,0);
 Mat A_(1,1),B_(1,1),C_(1,1);
 A_(0,0)=0;
 B_(0,0)=1;
 C_(0,0)=OutputGain;
 A=A_;
 B=B_;
 C=C_;   
 OutForm=&TransferSys::OutC;
};

void TransferSys::setI2(double OutputGain){
modus+="Double integrator mode";
     if (ABCDdefined) {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setI2!"<<endl;cout<<"Redefinition in Object: "<<name<<endl;
    throw 34;
     }
Vec Pole(2,INIT,0);
Vec Zeros;
  setPolesnZerosnGain(Pole,Zeros,OutputGain);
}

void TransferSys::setPT1(double P,double T){
modus+="PT1 Modus";
      if (ABCDdefined) {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setPT1!"<<endl;cout<<"Redefinition in Object: "<<name<<endl;
    throw 34;
     }

Vec Pole(1,INIT,-1/T);
Vec Zeros;
  setPolesnZerosnGain(Pole,Zeros,P/T);
    
}

void TransferSys::setGain(double P){
modus+="Gain Mode";
      if (ABCDdefined) {
    cout<<"ERROR!!!! State space matrices ABCD are going to be redefined by setGain!"<<endl;cout<<"Redefinition in Object: "<<name<<endl;
    throw 34;
     }

setPID(P,0,0);
  
}

//Plot Defs Anfang************************************************************
void TransferSys::plot(double t, double dt){
// double eingang;
// eingang=(this->*Uin)(t)(0);
  Element::plot(t,dt);
 // plotfile<<" "<< (*TFOutputSignal)(t)(0)<<" "<<eingang;
plotfile<<" "<<y(0)<<" "<<(this->*Uin)(t)(0);

 }

void TransferSys::initPlotFiles() {

  Element::initPlotFiles();
  plotfile <<"# "<< plotNr++ << ": Sigout	" << endl;
  plotfile <<"# "<< plotNr++ << ": Sigin        " << endl;
  plotfile <<"#  Object "<<name<<" is in "<<modus<<endl;
}
//Plot Defs Ende--------------------------------------------------------------------
#ifdef MBSIMXML
DOMNode *TransferSys::DOMInput(DOMNode *n,MultiBodySystem* parentmbs){
  n=SPSys::DOMInput(n,parentmbs);
  *logf << "Building TransferSys: "<< name << endl; 
  XMLInputUtils xml; 
  if(xml.hasName(n,"http://www.amm.mw.tu-muenchen.de/mbsim::PID")) {
    DOMNode* npid=n->getFirstChild();
    double P_=xml.getScalar(npid);
    *logf << "P: " << P_ << endl;
    npid=npid->getNextSibling();
    double I_=xml.getScalar(npid);
    *logf << "I: " << I_ << endl;
    npid=npid->getNextSibling();
    double D_=xml.getScalar(npid);
    *logf << "D: " << D_ << endl;
    setPID(P_,I_,D_);
  }
  else if(xml.hasName(n,"http://www.amm.mw.tu-muenchen.de/mbsim::PT1"))  {
    DOMNode* npt1=n->getFirstChild();
    double P_=xml.getScalar(npt1);
    *logf << "P: " << P_ << endl;
    npt1=npt1->getNextSibling();
    double T_=xml.getScalar(npt1);
    *logf << "T: " << T_ << endl;
    setPT1(P_,T_);
  }
  n=n->getNextSibling();
  return n;
}
#endif
