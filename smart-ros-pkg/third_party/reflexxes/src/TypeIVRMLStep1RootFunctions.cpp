



































#include <TypeIVRMLStep1RootFunctions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1Decisions.h>
#include <math.h>
#ifdef WIN32
#pragma\
 warning( disable : 4100 )	// to suppress the C4100 compiler warning (unreferenced formal parameter)
#endif


void diBqY::FW1Ej(const double&oczAG,const double&yZ3xv,const double&RMuwv,const
 double&MYjyK,const double&U3tDV,const double&gstgF,double*rJ4o9,double*hw4eR){
if(pVKSx(yZ3xv,oczAG,RMuwv,MYjyK,U3tDV)){
*rJ4o9=(KNYa5(yZ3xv)+2.0*U3tDV*(MYjyK-RMuwv))/(2.0*oczAG*U3tDV);}else{*rJ4o9=0.0
;}
*hw4eR=(KNYa5(yZ3xv)-2.0*KNYa5(oczAG)+2.0*U3tDV*(gstgF-RMuwv))/(2.0*oczAG*U3tDV)
;if(*hw4eR<0.0){*hw4eR=0.0;}if(*rJ4o9<0.0){*rJ4o9=0.0;}PHbIW(rJ4o9,hw4eR);}

double diBqY::eHpM4(const double&JWb5x,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double qDi3C=0.0,qw9PQ=0.0,GfDUt=0.0,h7eWG=0.0,lbFpW=0.0,
ZwYHN=0.0,VOIFm=0.0,WtmID=0.0,OQh_D=0.0;qw9PQ=yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;
WtmID=U3tDV*RMuwv;OQh_D=oczAG*oczAG;VOIFm=OQh_D*OQh_D;ZwYHN=U3tDV*U3tDV;qDi3C=
JWb5x*JWb5x;h7eWG=RMuwv*RMuwv;lbFpW=MYjyK*MYjyK;return(
0.41666666666666666666666666666666666667e-1*(0.3e1*GfDUt+0.8e1*qw9PQ*yZ3xv*oczAG
-0.24e2*yZ3xv*oczAG*WtmID-0.6e1*qw9PQ*(0.7e1*OQh_D+0.4e1*oczAG*U3tDV*JWb5x+0.2e1
*WtmID)+0.48e2*VOIFm+0.72e2*OQh_D*oczAG*U3tDV*JWb5x+0.24e2*oczAG*ZwYHN*(Ixvmb-
0.1e1*NS_M5+0.2e1*JWb5x*RMuwv)+0.12e2*OQh_D*U3tDV*(0.2e1*U3tDV*qDi3C+0.7e1*RMuwv
+MYjyK)+0.12e2*ZwYHN*(h7eWG-0.1e1*lbFpW))/oczAG/ZwYHN);}

double diBqY::wEEQc(const double&JWb5x,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double GV1Fh=0.0,qw9PQ=0.0;qw9PQ=yZ3xv*yZ3xv;GV1Fh=oczAG*
oczAG;return((-0.1e1*qw9PQ+0.3e1*GV1Fh+0.2e1*oczAG*U3tDV*JWb5x+0.2e1*U3tDV*RMuwv
)/U3tDV);}

double diBqY::LhdXy(const double&JWb5x,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double qw9PQ=0.0,EFQ3W=0.0;qw9PQ=yZ3xv*yZ3xv;EFQ3W=oczAG*
oczAG;return(-0.5e0*(qw9PQ+0.2e1*yZ3xv*oczAG-0.8e1*EFQ3W-0.4e1*oczAG*U3tDV*JWb5x
-0.2e1*U3tDV*RMuwv+0.2e1*U3tDV*MYjyK)/oczAG/U3tDV);}

void diBqY::MWTWT(const double&oczAG,const double&yZ3xv,const double&RMuwv,const
 double&MYjyK,const double&U3tDV,const double&gstgF,double*LCa4Y,double*Bczkr){
double xpBG4=0.0;xpBG4=RMuwv+(KNYa5(oczAG)-0.5*KNYa5(yZ3xv))/U3tDV;
if(xpBG4>MYjyK){
*Bczkr=-SUYGr(U3tDV*(xpBG4-MYjyK));}else{*Bczkr=0.0;}
*LCa4Y=-SUYGr(U3tDV*(gstgF-MYjyK));if(*LCa4Y<-oczAG){*LCa4Y=-oczAG;}if(*Bczkr>
0.0){*Bczkr=0.0;}PHbIW(LCa4Y,Bczkr);return;}

double diBqY::WP2Bw(const double&ZlrOe,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double qw9PQ=0.0,WtmID=0.0,OQh_D=0.0,XEDmG=0.0,GfDUt=0.0,
KrVnZ=0.0,ZOgWU=0.0,JRmPU=0.0,JoxHe=0.0;qw9PQ=yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;
WtmID=U3tDV*RMuwv;OQh_D=oczAG*oczAG;JRmPU=ZlrOe*ZlrOe;JoxHe=JRmPU*JRmPU;XEDmG=
U3tDV*U3tDV;KrVnZ=RMuwv*RMuwv;ZOgWU=MYjyK*MYjyK;return(
0.41666666666666666666666666666667e-1*(-0.3e1*GfDUt+0.8e1*qw9PQ*yZ3xv*oczAG-
0.24e2*yZ3xv*oczAG*WtmID-0.6e1*qw9PQ*(OQh_D-0.2e1*WtmID)+0.12e2*JoxHe+0.24e2*
JRmPU*U3tDV*MYjyK-0.24e2*oczAG*(JRmPU*ZlrOe+XEDmG*(-0.1e1*Ixvmb+NS_M5)+0.2e1*
ZlrOe*U3tDV*MYjyK)+0.12e2*XEDmG*(-0.1e1*KrVnZ+ZOgWU)+0.12e2*OQh_D*(JRmPU+U3tDV*(
RMuwv+MYjyK)))/oczAG/XEDmG);}

double diBqY::O_tX1(const double&ZlrOe,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double qw9PQ=0.0,EFQ3W=0.0,fCqXA=0.0;qw9PQ=oczAG*oczAG;EFQ3W
=ZlrOe*ZlrOe;fCqXA=U3tDV*U3tDV;return((0.1e1*qw9PQ*ZlrOe-0.3e1*oczAG*EFQ3W+0.2e1
*EFQ3W*ZlrOe-0.2e1*oczAG*U3tDV*MYjyK+0.2e1*ZlrOe*U3tDV*MYjyK)/oczAG/fCqXA);}

double diBqY::hxVF1(const double&ZlrOe,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double EFQ3W=0.0,WtmID=0.0,qw9PQ=0.0;qw9PQ=yZ3xv*yZ3xv;EFQ3W
=oczAG*oczAG;WtmID=ZlrOe*ZlrOe;return(0.5e0*(qw9PQ-0.2e1*yZ3xv*oczAG+0.2e1*EFQ3W
-0.4e1*oczAG*ZlrOe+0.2e1*WtmID-0.2e1*U3tDV*RMuwv+0.2e1*U3tDV*MYjyK)/oczAG/U3tDV)
;}

void diBqY::tlbcQ(const double&oczAG,const double&yZ3xv,const double&RMuwv,const
 double&MYjyK,const double&U3tDV,const double&gstgF,double*vc3Y1,double*JL22f){
double xb3Z_=0.0;xb3Z_=RMuwv+(KNYa5(yZ3xv)-2.0*KNYa5(oczAG))/(2.0*U3tDV);if(
xb3Z_>MYjyK){*vc3Y1=yZ3xv;}else{
*vc3Y1=SUYGr(0.5*(KNYa5(yZ3xv)+2.0*KNYa5(oczAG)+2.0*U3tDV*(MYjyK-RMuwv)));}
*JL22f=SUYGr(0.5*(KNYa5(yZ3xv)+2.0*U3tDV*(gstgF-RMuwv)));if(*JL22f>oczAG){*JL22f
=oczAG;}if(*vc3Y1<0.0){*vc3Y1=0.0;}PHbIW(vc3Y1,JL22f);return;}

double diBqY::_WRKy(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double A7maA=0.0,qw9PQ=0.0,GfDUt=0.0,qDi3C=0.0,orIqt=0.0,
OQh_D=0.0,m778l=0.0,U_Zir=0.0,WtmID=0.0;qw9PQ=yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;
WtmID=U3tDV*RMuwv;OQh_D=oczAG*oczAG;m778l=lV4Xh*lV4Xh;U_Zir=m778l*m778l;orIqt=
U3tDV*U3tDV;qDi3C=RMuwv*RMuwv;A7maA=MYjyK*MYjyK;return(
0.41666666666666666666666666666667e-1*(0.3e1*GfDUt+0.8e1*qw9PQ*yZ3xv*oczAG-
0.24e2*yZ3xv*oczAG*WtmID-0.6e1*qw9PQ*(OQh_D+0.4e1*oczAG*lV4Xh+0.2e1*m778l+0.2e1*
WtmID)+0.12e2*U_Zir+0.24e2*m778l*U3tDV*RMuwv+0.24e2*oczAG*(m778l*lV4Xh+orIqt*(
Ixvmb-0.1e1*NS_M5)+0.2e1*lV4Xh*U3tDV*RMuwv)+0.12e2*orIqt*(qDi3C-0.1e1*A7maA)+
0.12e2*OQh_D*(m778l+U3tDV*(RMuwv+MYjyK)))/oczAG/orIqt);}

double diBqY::ni1W7(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double qw9PQ=0.0,XEDmG=0.0,ArZnN=0.0,Ge1KL=0.0;qw9PQ=yZ3xv*
yZ3xv;Ge1KL=oczAG*oczAG;ArZnN=lV4Xh*lV4Xh;XEDmG=U3tDV*U3tDV;return((qw9PQ*(-
0.1e1*oczAG-0.1e1*lV4Xh)+0.1e1*Ge1KL*lV4Xh+0.3e1*oczAG*ArZnN+0.2e1*ArZnN*lV4Xh+
0.2e1*oczAG*U3tDV*RMuwv+0.2e1*lV4Xh*U3tDV*RMuwv)/oczAG/XEDmG);}

double diBqY::vhFWf(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double qw9PQ=0.0,EFQ3W=0.0,WtmID=0.0;qw9PQ=yZ3xv*yZ3xv;EFQ3W
=oczAG*oczAG;WtmID=lV4Xh*lV4Xh;return(-0.5e0*(qw9PQ+0.2e1*yZ3xv*oczAG-0.2e1*
EFQ3W-0.4e1*oczAG*lV4Xh-0.2e1*WtmID-0.2e1*U3tDV*RMuwv+0.2e1*U3tDV*MYjyK)/oczAG/
U3tDV);}

void diBqY::AkYML(const double&oczAG,const double&yZ3xv,const double&RMuwv,const
 double&MYjyK,const double&U3tDV,const double&gstgF,double*vc3Y1,double*JL22f){
double Gljzl=0.0,riX5T=0.0,Ps2R8=0.0;Gljzl=RMuwv+0.5*KNYa5(yZ3xv)/U3tDV;if(Gljzl
>MYjyK){*vc3Y1=yZ3xv;}else{
*vc3Y1=SUYGr(0.5*(KNYa5(yZ3xv)+2.0*U3tDV*(MYjyK-RMuwv)));}
riX5T=SUYGr(0.5*(KNYa5(U3tDV)*(KNYa5(yZ3xv)+2.0*KNYa5(oczAG))+D_rgb(U3tDV)*(
MYjyK-RMuwv)))/U3tDV;if(riX5T>oczAG){riX5T=oczAG;}
Ps2R8=SUYGr(0.5*(KNYa5(yZ3xv)+2.0*U3tDV*(gstgF-RMuwv)));if(Ps2R8>oczAG){Ps2R8=
oczAG;}if(Ps2R8<riX5T){*JL22f=Ps2R8;}else{*JL22f=riX5T;}if(*vc3Y1<0.0){*vc3Y1=
0.0;}PHbIW(vc3Y1,JL22f);return;}

double diBqY::nNN9F(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double JRmPU=0.0,qw9PQ=0.0,ArZnN=0.0,A7maA=0.0;qw9PQ=yZ3xv*
yZ3xv;ArZnN=lV4Xh*lV4Xh;JRmPU=SUYGr(-0.2e1*qw9PQ+0.4e1*ArZnN+0.4e1*U3tDV*RMuwv-
0.4e1*U3tDV*MYjyK);A7maA=U3tDV*U3tDV;return(
0.8333333333333333333333333333333333333333333333333e-1*(0.4e1*qw9PQ*yZ3xv-0.12e2
*yZ3xv*U3tDV*RMuwv-0.3e1*qw9PQ*(0.4e1*lV4Xh+JRmPU)+0.12e2*ArZnN*lV4Xh+0.24e2*
lV4Xh*U3tDV*RMuwv+0.6e1*U3tDV*(0.2e1*U3tDV*(Ixvmb-0.1e1*NS_M5)+(RMuwv+MYjyK)*
JRmPU)+0.6e1*ArZnN*JRmPU)/A7maA);}

double diBqY::v2SZG(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double ZwYHN=0.0,qw9PQ=0.0,EFQ3W=0.0,ArZnN=0.0,JWb5x=0.0,
JoxHe=0.0;qw9PQ=lV4Xh*lV4Xh;EFQ3W=lV4Xh*U3tDV;ArZnN=yZ3xv*yZ3xv;JWb5x=U3tDV*
RMuwv;JoxHe=SUYGr(-0.2e1*ArZnN+0.4e1*qw9PQ+0.4e1*JWb5x-0.4e1*U3tDV*MYjyK);ZwYHN=
U3tDV*U3tDV;return((0.6e1*qw9PQ*lV4Xh+0.6e1*EFQ3W*RMuwv-0.2e1*EFQ3W*MYjyK+0.3e1*
qw9PQ*JoxHe+0.2e1*JWb5x*JoxHe+ArZnN*(-0.3e1*lV4Xh-0.1e1*JoxHe))/ZwYHN/JoxHe);}

double diBqY::dnOyS(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double JWb5x=0.0,GV1Fh=0.0,bRoo6=0.0;GV1Fh=yZ3xv*yZ3xv;bRoo6
=lV4Xh*lV4Xh;JWb5x=SUYGr(-0.2e1*GV1Fh+0.4e1*bRoo6+0.4e1*U3tDV*RMuwv-0.4e1*U3tDV*
MYjyK);return((-0.1e1*yZ3xv+0.2e1*lV4Xh+JWb5x)/U3tDV);}

void diBqY::Fgf1e(const double&oczAG,const double&yZ3xv,const double&RMuwv,const
 double&MYjyK,const double&U3tDV,const double&gstgF,double*vc3Y1,double*JL22f){
double ImKba=0.0;*JL22f=yZ3xv;ImKba=RMuwv+(0.5*KNYa5(yZ3xv)+KNYa5(oczAG))/U3tDV;
if(ImKba<MYjyK){*vc3Y1=0.0;}else{
*vc3Y1=SUYGr(0.5*(KNYa5(yZ3xv)+2.0*KNYa5(oczAG)+2.0*U3tDV*(RMuwv-MYjyK)));}if(*
vc3Y1<0.0){*vc3Y1=0.0;}if(*JL22f<0.0){*JL22f=0.0;}PHbIW(vc3Y1,JL22f);return;}

double diBqY::WkhoZ(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double orIqt=0.0,qw9PQ=0.0,GfDUt=0.0,qDi3C=0.0,A7maA=0.0,
OQh_D=0.0,U_Zir=0.0,m778l=0.0,WtmID=0.0;qw9PQ=yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;
WtmID=U3tDV*RMuwv;OQh_D=oczAG*oczAG;m778l=lV4Xh*lV4Xh;U_Zir=m778l*m778l;orIqt=
U3tDV*U3tDV;qDi3C=RMuwv*RMuwv;A7maA=MYjyK*MYjyK;return(
0.416666666666666666666666666666666667e-1*(-0.3e1*GfDUt+0.8e1*qw9PQ*yZ3xv*oczAG+
0.24e2*yZ3xv*oczAG*WtmID+0.6e1*qw9PQ*(OQh_D-0.4e1*oczAG*lV4Xh+0.2e1*m778l-0.2e1*
WtmID)-0.12e2*U_Zir+0.24e2*m778l*U3tDV*RMuwv+0.24e2*oczAG*(m778l*lV4Xh+orIqt*(
Ixvmb-0.1e1*NS_M5)-0.2e1*lV4Xh*U3tDV*RMuwv)-0.12e2*orIqt*(qDi3C-0.1e1*A7maA)-
0.12e2*OQh_D*(m778l-0.1e1*U3tDV*(RMuwv+MYjyK)))/oczAG/orIqt);}

double diBqY::P7bor(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double qw9PQ=0.0,EFQ3W=0.0,ArZnN=0.0,XEDmG=0.0;qw9PQ=oczAG*
oczAG;EFQ3W=lV4Xh*lV4Xh;ArZnN=yZ3xv*yZ3xv;XEDmG=U3tDV*U3tDV;return((-0.1e1*qw9PQ
*lV4Xh+0.3e1*oczAG*EFQ3W-0.2e1*EFQ3W*lV4Xh+ArZnN*(-0.1e1*oczAG+0.1e1*lV4Xh)-
0.2e1*oczAG*U3tDV*RMuwv+0.2e1*lV4Xh*U3tDV*RMuwv)/oczAG/XEDmG);}

double diBqY::SpAka(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double qw9PQ=0.0,m778l=0.0,GV1Fh=0.0,llIMw=0.0;qw9PQ=yZ3xv*
yZ3xv;GV1Fh=oczAG*oczAG;llIMw=lV4Xh*lV4Xh;m778l=U3tDV*U3tDV;return((0.1e1*qw9PQ-
0.1e1*GV1Fh+0.6e1*oczAG*lV4Xh-0.6e1*llIMw+0.2e1*U3tDV*RMuwv)/oczAG/m778l);}

double diBqY::IVmDp(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double bRoo6=0.0,ArZnN=0.0,qw9PQ=0.0;qw9PQ=yZ3xv*yZ3xv;bRoo6
=oczAG*oczAG;ArZnN=lV4Xh*lV4Xh;return(0.5e0*(-0.1e1*qw9PQ+0.2e1*yZ3xv*oczAG+
0.2e1*bRoo6-0.4e1*oczAG*lV4Xh+0.2e1*ArZnN-0.2e1*U3tDV*RMuwv+0.2e1*U3tDV*MYjyK)/
oczAG/U3tDV);}

void diBqY::_aosC(const double&oczAG,const double&yZ3xv,const double&RMuwv,const
 double&MYjyK,const double&U3tDV,const double&gstgF,double*vc3Y1,double*JL22f){
double xpBG4=0.0;xpBG4=RMuwv+(KNYa5(oczAG)-0.5*KNYa5(yZ3xv))/U3tDV;if(xpBG4>
MYjyK){*JL22f=yZ3xv;}else{
*JL22f=SUYGr(0.5*(KNYa5(yZ3xv)+2.0*KNYa5(oczAG)+2.0*U3tDV*(RMuwv-MYjyK)));}*
vc3Y1=0.0;PHbIW(vc3Y1,JL22f);return;}

double diBqY::tIQ8E(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double A7maA=0.0,qw9PQ=0.0,ArZnN=0.0,JRmPU=0.0;qw9PQ=yZ3xv*
yZ3xv;ArZnN=lV4Xh*lV4Xh;JRmPU=SUYGr(-0.2e1*qw9PQ+0.4e1*ArZnN-0.4e1*U3tDV*RMuwv+
0.4e1*U3tDV*MYjyK);A7maA=U3tDV*U3tDV;return(
0.8333333333333333333333333333333333333333e-1*(0.4e1*qw9PQ*yZ3xv+0.12e2*yZ3xv*
U3tDV*RMuwv+0.3e1*qw9PQ*(-0.4e1*lV4Xh+JRmPU)+0.12e2*ArZnN*lV4Xh-0.24e2*lV4Xh*
U3tDV*RMuwv-0.6e1*ArZnN*JRmPU+0.6e1*U3tDV*(0.2e1*U3tDV*(Ixvmb-0.1e1*NS_M5)+(
RMuwv+MYjyK)*JRmPU))/A7maA);}

double diBqY::pqZE_(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double ZwYHN=0.0,qw9PQ=0.0,ArZnN=0.0,JoxHe=0.0,EFQ3W=0.0,
JWb5x=0.0;qw9PQ=lV4Xh*lV4Xh;EFQ3W=lV4Xh*U3tDV;ArZnN=yZ3xv*yZ3xv;JWb5x=U3tDV*
RMuwv;JoxHe=SUYGr(-0.2e1*ArZnN+0.4e1*qw9PQ-0.4e1*JWb5x+0.4e1*U3tDV*MYjyK);ZwYHN=
U3tDV*U3tDV;return((-0.6e1*qw9PQ*lV4Xh+0.6e1*EFQ3W*RMuwv-0.2e1*EFQ3W*MYjyK+0.3e1
*qw9PQ*JoxHe-0.2e1*JWb5x*JoxHe+ArZnN*(0.3e1*lV4Xh-0.1e1*JoxHe))/ZwYHN/JoxHe);}

double diBqY::vueAo(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double qw9PQ=0.0,avZHM=0.0,JoxHe=0.0,qfrNo=0.0,GfDUt=0.0,
U_Zir=0.0,cs2CS=0.0,WTa2M=0.0,c1UjC=0.0,EFQ3W=0.0,MA5ni=0.0,bRoo6=0.0;qw9PQ=
yZ3xv*yZ3xv;GfDUt=qw9PQ*qw9PQ;EFQ3W=lV4Xh*lV4Xh;bRoo6=EFQ3W*EFQ3W;avZHM=U3tDV*
RMuwv;JoxHe=U3tDV*MYjyK;qfrNo=-0.2e1*qw9PQ+0.4e1*EFQ3W-0.4e1*avZHM+0.4e1*JoxHe;
U_Zir=SUYGr(qfrNo);cs2CS=U3tDV*U3tDV;WTa2M=RMuwv*RMuwv;c1UjC=MYjyK*MYjyK;MA5ni=
SUYGr(qfrNo);return((-0.6e1*GfDUt-0.48e2*bRoo6+EFQ3W*U3tDV*(0.72e2*RMuwv-0.72e2*
MYjyK)+0.24e2*EFQ3W*lV4Xh*U_Zir+lV4Xh*U3tDV*(-0.24e2*RMuwv+0.24e2*MYjyK)*U_Zir+
cs2CS*(-0.24e2*WTa2M+0.32e2*RMuwv*MYjyK-0.8e1*c1UjC)+qw9PQ*(0.36e2*EFQ3W-0.24e2*
avZHM+0.16e2*JoxHe-0.12e2*lV4Xh*U_Zir))/cs2CS/MA5ni/qfrNo);}

double diBqY::VuRcE(const double&lV4Xh,const double&yZ3xv,const double&RMuwv,
const double&MYjyK,const double&Ixvmb,const double&NS_M5,const double&U3tDV,
const double&oczAG){double OQh_D=0.0,GfDUt=0.0,EFQ3W=0.0;GfDUt=yZ3xv*yZ3xv;EFQ3W
=lV4Xh*lV4Xh;OQh_D=SUYGr(-0.2e1*GfDUt+0.4e1*EFQ3W-0.4e1*U3tDV*RMuwv+0.4e1*U3tDV*
MYjyK);return((yZ3xv-0.2e1*lV4Xh+OQh_D)/U3tDV);}
