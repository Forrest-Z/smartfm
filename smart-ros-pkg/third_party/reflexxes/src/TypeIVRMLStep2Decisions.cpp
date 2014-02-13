
































#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLStep2Decisions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLABK.h>
#include <TypeIVRMLStep2RootFunctions.h>
#include <math.h>








bool diBqY::UhFqM(const double&_Qa5G){return(FlP2m(_Qa5G));}


bool diBqY::rJATa(const double&_Qa5G,const double&E6Xnb){
return(W5C39(_Qa5G,E6Xnb));}


bool diBqY::vkGJM(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){
return(dS4fr(_Qa5G,g93mR,g85jV,sguyX));}


bool diBqY::xhmNB(const double&g93mR,const double&g85jV){
return(HkXG5(g93mR,g85jV));}


bool diBqY::Py5pe(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(k9CDo(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::cM9j6(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){return(XXwMs(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::sP7Xe(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){

return(bySSx(_Qa5G,g93mR,g85jV,sguyX));}


bool diBqY::KVYN5(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(NpVpV(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::cENOS(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(UW66G(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::jU0dH(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&g85jV,const double&sguyX){
return(iiDer(_Qa5G,g93mR,hmD2W,g85jV,sguyX));}


bool diBqY::va_fp(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
return(XXwMs(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::qDFXQ(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&g85jV,const double&iQ2lh,const double&SynchronizationTime){
double GzZSa=0.0
,Z4ILN=0.0
,XVPX2=0.0
,ASKNP=0.0
,c_QBZ=0.0
,BKRPw=0.0
,AG0zS=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
GzZSa=(E6Xnb-qPN_6)/sguyX;c_QBZ=qPN_6*GzZSa+0.5*sguyX*KNYa5(GzZSa);
XVPX2=E6Xnb/sguyX;AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
BKRPw=hmD2W-gL6qW-c_QBZ-AG0zS;Z4ILN=BKRPw/E6Xnb;
ASKNP=SynchronizationTime-iQ2lh-GzZSa-Z4ILN-XVPX2;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*Z4ILN+0.5*qPN_6*KNYa5(Z4ILN);
gL6qW+=BKRPw;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;
gL6qW=hmD2W;qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
if(hmD2W>0.0){return(Fb1Vh<=AUze_+g85jV*WTpjf*(1.0-(g85jV-hmD2W)/g85jV));}else{
return(Fb1Vh<=AUze_);}}


bool diBqY::OMxqE(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){

double GzZSa=0.0
,Z4ILN=0.0
,XVPX2=0.0
,MBb3Z=0.0
,NckOJ=0.0
,c_QBZ=0.0
,QSroW=0.0
,AG0zS=0.0
,T6g70=0.0
,CdoNz=0.0;

GzZSa=(E6Xnb-_Qa5G)/sguyX;
XVPX2=MBb3Z=NckOJ=E6Xnb/sguyX;c_QBZ=_Qa5G*GzZSa+0.5*sguyX*KNYa5(GzZSa);AG0zS=0.5
*KNYa5(E6Xnb)/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-g93mR-c_QBZ-AG0zS-T6g70-CdoNz;Z4ILN=QSroW/E6Xnb;return((iQ2lh+GzZSa+
Z4ILN+XVPX2+MBb3Z+NckOJ)>=SynchronizationTime);}


bool diBqY::R7UcQ(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){

double GzZSa=0.0
,Z4ILN=0.0
,XVPX2=0.0
,ASKNP=0.0
,MBb3Z=0.0
,NckOJ=0.0
,c_QBZ=0.0
,BKRPw=0.0
,AG0zS=0.0
,T6g70=0.0
,CdoNz=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
GzZSa=(E6Xnb-qPN_6)/sguyX;c_QBZ=qPN_6*GzZSa+0.5*sguyX*KNYa5(GzZSa);
XVPX2=MBb3Z=NckOJ=E6Xnb/sguyX;AG0zS=0.5*KNYa5(E6Xnb)/sguyX;T6g70=CdoNz=-0.5*
KNYa5(E6Xnb)/sguyX;
BKRPw=hmD2W-gL6qW-c_QBZ-AG0zS-T6g70-CdoNz;Z4ILN=BKRPw/E6Xnb;
ASKNP=SynchronizationTime-iQ2lh-GzZSa-Z4ILN-XVPX2-MBb3Z-NckOJ;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*Z4ILN+0.5*qPN_6*KNYa5(Z4ILN);
gL6qW+=BKRPw;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;
gL6qW+=AG0zS;qPN_6=(0x2d9+4554-0x14a3);
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::wemY7(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
return(b_6Ox(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::HdiZh(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){
double CnAKY=0.0
,GzZSa=0.0
,Z4ILN=0.0
,XVPX2=0.0
,c_QBZ=0.0
,BKRPw=0.0
,AG0zS=0.0
,gL6qW=g93mR;

CnAKY=_Qa5G/sguyX;gL6qW+=_Qa5G*CnAKY-0.5*sguyX*KNYa5(CnAKY);

GzZSa=XVPX2=E6Xnb/sguyX;c_QBZ=AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
BKRPw=hmD2W-gL6qW-c_QBZ-AG0zS;Z4ILN=BKRPw/E6Xnb;return((iQ2lh+CnAKY+GzZSa+Z4ILN+
XVPX2)>=SynchronizationTime);}


bool diBqY::gK0Le(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){double k01PI=0.0,A7maA=0.0,qw9PQ=
0.0,ezhxj=0.0,JWb5x=0.0,dMwdk=0.0
,JoxHe=0.0,uK79H=0.0,U_Zir=0.0,EFQ3W=0.0,Ge1KL=0.0,llIMw=0.0,HlaZx=0.0;qw9PQ=(
SynchronizationTime-iQ2lh)*(SynchronizationTime-iQ2lh);EFQ3W=_Qa5G*_Qa5G;Ge1KL=
sguyX*sguyX;llIMw=0.1e1/Ge1KL;JWb5x=E6Xnb*E6Xnb;JoxHe=0.1e1/sguyX;U_Zir=_Qa5G*
E6Xnb;A7maA=Ge1KL*sguyX;k01PI=SUYGr(EFQ3W*Ge1KL-0.2e1*U_Zir*Ge1KL+0.4e1*JWb5x*
Ge1KL-0.2e1*E6Xnb*(SynchronizationTime-iQ2lh)*A7maA-0.2e1*A7maA*g93mR+0.2e1*
A7maA*hmD2W);ezhxj=SUYGr(0.2e1);uK79H=0.1e1/A7maA;HlaZx=k01PI*llIMw;dMwdk=E6Xnb*
qw9PQ/0.2e1-EFQ3W*_Qa5G*llIMw/0.6e1+EFQ3W*E6Xnb*llIMw-0.3e1/0.2e1*_Qa5G*JWb5x*
llIMw-EFQ3W*(SynchronizationTime-iQ2lh)*JoxHe/0.2e1+U_Zir*(SynchronizationTime-
iQ2lh)*JoxHe-0.3e1/0.2e1*JWb5x*(SynchronizationTime-iQ2lh)*JoxHe+Z45C3+(
SynchronizationTime-iQ2lh)*g93mR-E6Xnb*g93mR*JoxHe+E6Xnb*hmD2W*JoxHe-EFQ3W*k01PI
/ezhxj*uK79H/0.2e1+ezhxj*_Qa5G*E6Xnb*k01PI*uK79H/0.2e1+ezhxj*E6Xnb*(
SynchronizationTime-iQ2lh)*k01PI*llIMw/0.2e1+ezhxj*g93mR*HlaZx/0.2e1-ezhxj*hmD2W
*HlaZx/0.2e1;return(dMwdk<=AUze_);}


bool diBqY::GN20j(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX,const double&iQ2lh,const double&SynchronizationTime){double gL6qW=
g93mR,qPN_6=_Qa5G,VFLYs=0.0
,eUZpd=0.0;

eUZpd=qPN_6/sguyX;
VFLYs=SynchronizationTime-iQ2lh-eUZpd;
gL6qW+=qPN_6*VFLYs;qPN_6=qPN_6;
gL6qW+=qPN_6*eUZpd-0.5*sguyX*KNYa5(eUZpd);return(gL6qW>hmD2W);}


bool diBqY::u4kxD(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){double jaJi5=0.0
,GscyC=0.0
,iZ7FW=0.0
,CnAKY=0.0
,bZn23=0.0
,ebVM3=0.0
,xP9LI=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
jaJi5=(sguyX*(KNYa5(qPN_6)/sguyX+2.0*gL6qW-2.0*hmD2W))/(2.0*qPN_6+2.0*sguyX*
iQ2lh-2.0*sguyX*SynchronizationTime);
GscyC=(qPN_6-jaJi5)/sguyX;bZn23=qPN_6*GscyC-0.5*sguyX*KNYa5(GscyC);
CnAKY=jaJi5/sguyX;xP9LI=0.5*KNYa5(jaJi5)/sguyX;
iZ7FW=SynchronizationTime-iQ2lh-GscyC-CnAKY;ebVM3=jaJi5*iZ7FW;
Fb1Vh+=gL6qW*GscyC+0.5*qPN_6*KNYa5(GscyC)-sguyX*D_rgb(GscyC)/6.0;gL6qW+=bZn23;
qPN_6=jaJi5;
Fb1Vh+=gL6qW*iZ7FW+0.5*qPN_6*KNYa5(iZ7FW);gL6qW+=ebVM3;qPN_6=jaJi5;
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)-sguyX*D_rgb(CnAKY)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::zm2sr(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){double jaJi5=0.0
,GzZSa=0.0
,GscyC=0.0
,iZ7FW=0.0
,CnAKY=0.0
,c_QBZ=0.0
,bZn23=0.0
,ebVM3=0.0
,xP9LI=0.0
,Jqlam=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;Jqlam=(2.0*qPN_6-4.0*E6Xnb-2.0*
sguyX*iQ2lh+2.0*sguyX*SynchronizationTime);if(Jqlam==0.0){Jqlam=DeB6Y;}
jaJi5=(sguyX*((1.0*KNYa5(qPN_6))/sguyX-(2.0*KNYa5(E6Xnb))/sguyX-2.0*gL6qW+2.0*
hmD2W))/Jqlam;if(jaJi5>E6Xnb){jaJi5=E6Xnb;}if(jaJi5<0.0){jaJi5=0.0;}
GzZSa=(E6Xnb-qPN_6)/sguyX;c_QBZ=qPN_6*GzZSa+0.5*sguyX*KNYa5(GzZSa);
GscyC=(E6Xnb-jaJi5)/sguyX;bZn23=E6Xnb*GscyC-0.5*sguyX*KNYa5(GscyC);
CnAKY=jaJi5/sguyX;xP9LI=0.5*KNYa5(jaJi5)/sguyX;
iZ7FW=SynchronizationTime-iQ2lh-GzZSa-GscyC-CnAKY;ebVM3=iZ7FW*jaJi5;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*GscyC+0.5*qPN_6*KNYa5(GscyC)-sguyX*D_rgb(GscyC)/6.0;gL6qW+=bZn23;
qPN_6=jaJi5;
Fb1Vh+=gL6qW*iZ7FW+0.5*qPN_6*KNYa5(iZ7FW);gL6qW+=ebVM3;qPN_6=jaJi5;
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)-sguyX*D_rgb(CnAKY)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::crPkq(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){
double HaMHK=0.0
,GzZSa=0.0
,CnAKY=0.0
,JvrEy=0.0
,c_QBZ=0.0
,xP9LI=0.0;

GzZSa=(E6Xnb-_Qa5G)/sguyX;c_QBZ=_Qa5G*GzZSa+0.5*sguyX*KNYa5(GzZSa);
CnAKY=E6Xnb/sguyX;xP9LI=0.5*KNYa5(E6Xnb)/sguyX;
HaMHK=SynchronizationTime-iQ2lh-GzZSa-CnAKY;JvrEy=_Qa5G*HaMHK;return((g93mR+
JvrEy+c_QBZ+xP9LI)>hmD2W);}


bool diBqY::tXRmH(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double NzBTh=0.0;

NzBTh=(0.08333333333333333333333333333333333*D_rgb(_Qa5G)+KNYa5(_Qa5G)*(-0.5*
E6Xnb-0.25*sguyX*iQ2lh+0.25*sguyX*SynchronizationTime)+_Qa5G*(0.5*KNYa5(E6Xnb)+
0.5*sguyX*g93mR-0.5*sguyX*hmD2W)+sguyX*(KNYa5(E6Xnb)*(0.5*iQ2lh-0.5*
SynchronizationTime)+E6Xnb*(-1.0*g93mR+1.0*hmD2W)+sguyX*(-0.5*iQ2lh*g93mR+0.5*
SynchronizationTime*g93mR-0.5*iQ2lh*hmD2W+0.5*SynchronizationTime*hmD2W)))/KNYa5
(sguyX);return((Z45C3+NzBTh)<=AUze_);}


bool diBqY::kwgCa(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){double jaJi5=0.0
,frEOw=0.0
,iZ7FW=0.0
,CnAKY=0.0
,y_psI=0.0
,ebVM3=0.0
,xP9LI=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
jaJi5=0.5*qPN_6-0.5*sguyX*iQ2lh+0.5*sguyX*SynchronizationTime-0.25*sguyX*SUYGr((
-4.0*KNYa5(qPN_6))/KNYa5(sguyX)-(8.0*qPN_6*iQ2lh)/sguyX+4.0*KNYa5(iQ2lh)+(8.0*
qPN_6*SynchronizationTime)/sguyX-8.0*iQ2lh*SynchronizationTime+4.0*KNYa5(
SynchronizationTime)+(16.0*gL6qW)/sguyX-(16.0*hmD2W)/sguyX);
frEOw=(jaJi5-qPN_6)/sguyX;y_psI=qPN_6*frEOw+0.5*sguyX*KNYa5(frEOw);
CnAKY=jaJi5/sguyX;xP9LI=0.5*KNYa5(jaJi5)/sguyX;
iZ7FW=SynchronizationTime-iQ2lh-frEOw-CnAKY;ebVM3=jaJi5*iZ7FW;
Fb1Vh+=gL6qW*frEOw+0.5*qPN_6*KNYa5(frEOw)+sguyX*D_rgb(frEOw)/6.0;gL6qW+=y_psI;
qPN_6=jaJi5;
Fb1Vh+=gL6qW*iZ7FW+0.5*qPN_6*KNYa5(iZ7FW);gL6qW+=ebVM3;qPN_6=jaJi5;
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)-sguyX*D_rgb(CnAKY)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::CcqKD(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){return(crPkq(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX,iQ2lh,
SynchronizationTime));}


bool diBqY::KmXwx(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){double iZ7FW=0.0
,Ntxm9=0.0
,zqaUH=0.0
,ebVM3=0.0
,YhsYM=0.0
,h4z3k=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
mcZn2=1.0*qPN_6+0.70710678118654752440084436210485*SUYGr(1.0*KNYa5(qPN_6)+2.0*
qPN_6*sguyX*iQ2lh-2.0*qPN_6*sguyX*SynchronizationTime-2.0*sguyX*gL6qW+2.0*sguyX*
hmD2W);
Ntxm9=(mcZn2-qPN_6)/sguyX;YhsYM=qPN_6*Ntxm9+0.5*sguyX*KNYa5(Ntxm9);
zqaUH=mcZn2/sguyX;h4z3k=0.5*KNYa5(mcZn2)/sguyX;
iZ7FW=SynchronizationTime-iQ2lh-Ntxm9-zqaUH;ebVM3=iZ7FW*qPN_6;
Fb1Vh+=gL6qW*iZ7FW+0.5*qPN_6*KNYa5(iZ7FW);gL6qW+=ebVM3;qPN_6=qPN_6;
Fb1Vh+=gL6qW*Ntxm9+0.5*qPN_6*KNYa5(Ntxm9)+sguyX*D_rgb(Ntxm9)/6.0;gL6qW+=YhsYM;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*zqaUH+0.5*qPN_6*KNYa5(zqaUH)-sguyX*D_rgb(zqaUH)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::gPCJs(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){double iZ7FW=0.0
,hHY1C=0.0
,OWR_N=0.0
,YtA02=0.0
,ebVM3=0.0
,b60LR=0.0
,Rpl3H=0.0
,i8b0z=0.0
,Jqlam=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;Jqlam=(2.0*qPN_6-2.0*E6Xnb);if(
Jqlam==0.0){Jqlam=Lxcdx;}
OWR_N=(KNYa5(qPN_6)/sguyX-(4.0*qPN_6*E6Xnb)/sguyX+(2.0*KNYa5(E6Xnb))/sguyX-2.0*
qPN_6*iQ2lh+2.0*qPN_6*SynchronizationTime+2.0*gL6qW-2.0*hmD2W)/Jqlam;Rpl3H=OWR_N
*E6Xnb;
hHY1C=(E6Xnb-qPN_6)/sguyX;b60LR=qPN_6*hHY1C+0.5*sguyX*KNYa5(hHY1C);
YtA02=E6Xnb/sguyX;i8b0z=0.5*KNYa5(E6Xnb)/sguyX;
iZ7FW=SynchronizationTime-iQ2lh-hHY1C-OWR_N-YtA02;ebVM3=iZ7FW*qPN_6;
Fb1Vh+=gL6qW*iZ7FW+0.5*qPN_6*KNYa5(iZ7FW);gL6qW+=ebVM3;qPN_6=qPN_6;
Fb1Vh+=gL6qW*hHY1C+0.5*qPN_6*KNYa5(hHY1C)+sguyX*D_rgb(hHY1C)/6.0;gL6qW+=b60LR;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*OWR_N+0.5*qPN_6*KNYa5(OWR_N);gL6qW+=Rpl3H;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*YtA02+0.5*qPN_6*KNYa5(YtA02)-sguyX*D_rgb(YtA02)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::uYalO(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){double jaJi5=0.0
,frEOw=0.0
,iZ7FW=0.0
,EKaPB=0.0
,XWGlh=0.0
,y_psI=0.0
,ebVM3=0.0
,Zux5W=0.0
,T1rRn=0.0
,Jqlam=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;Jqlam=(2.0*qPN_6-4.0*E6Xnb-2.0*
sguyX*iQ2lh+2.0*sguyX*SynchronizationTime);if(Jqlam==0.0){Jqlam=DeB6Y;}
jaJi5=E6Xnb+(1.0*KNYa5(qPN_6))/Jqlam-(2.0*qPN_6*E6Xnb)/Jqlam+(2.0*pow(E6Xnb,
(0x306+732-0x5e0)))/Jqlam+(2.0*E6Xnb*sguyX*iQ2lh)/Jqlam-(2.0*E6Xnb*sguyX*
SynchronizationTime)/Jqlam-(2.0*sguyX*gL6qW)/Jqlam+(2.0*sguyX*hmD2W)/Jqlam;
frEOw=(jaJi5-qPN_6)/sguyX;y_psI=qPN_6*frEOw+0.5*sguyX*KNYa5(frEOw);
EKaPB=(E6Xnb-jaJi5)/sguyX;Zux5W=jaJi5*EKaPB+0.5*sguyX*KNYa5(EKaPB);
XWGlh=E6Xnb/sguyX;T1rRn=0.5*KNYa5(E6Xnb)/sguyX;
iZ7FW=SynchronizationTime-iQ2lh-frEOw-EKaPB-XWGlh;ebVM3=iZ7FW*jaJi5;
Fb1Vh+=gL6qW*frEOw+0.5*qPN_6*KNYa5(frEOw)+sguyX*D_rgb(frEOw)/6.0;gL6qW+=y_psI;
qPN_6=jaJi5;
Fb1Vh+=gL6qW*iZ7FW+0.5*qPN_6*KNYa5(iZ7FW);gL6qW+=ebVM3;qPN_6=jaJi5;
Fb1Vh+=gL6qW*EKaPB+0.5*qPN_6*KNYa5(EKaPB)+sguyX*D_rgb(EKaPB)/6.0;gL6qW+=Zux5W;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XWGlh+0.5*qPN_6*KNYa5(XWGlh)-sguyX*D_rgb(XWGlh)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::wlfXo(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){double CnAKY=0.0
,ASKNP=0.0
,hHY1C=0.0
,OWR_N=0.0
,YtA02=0.0
,xP9LI=0.0
,b60LR=0.0
,Rpl3H=0.0
,i8b0z=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
CnAKY=qPN_6/sguyX;xP9LI=qPN_6*CnAKY-0.5*KNYa5(CnAKY)*sguyX;
hHY1C=YtA02=E6Xnb/sguyX;b60LR=i8b0z=0.5*KNYa5(E6Xnb)/sguyX;
Rpl3H=hmD2W-gL6qW-xP9LI-b60LR-i8b0z;OWR_N=Rpl3H/E6Xnb;
ASKNP=SynchronizationTime-iQ2lh-CnAKY-hHY1C-OWR_N-YtA02;
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)-sguyX*D_rgb(CnAKY)/6.0;gL6qW+=xP9LI;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*hHY1C+0.5*qPN_6*KNYa5(hHY1C)+sguyX*D_rgb(hHY1C)/6.0;gL6qW+=b60LR;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*OWR_N+0.5*qPN_6*KNYa5(OWR_N);
gL6qW+=Rpl3H;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*YtA02+0.5*qPN_6*KNYa5(YtA02)-sguyX*D_rgb(YtA02)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::N28On(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=(E6Xnb-qPN_6)/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;


gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;


gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW<=hmD2W);}


bool diBqY::uSm3a(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){
double GzZSa=0.0
,XVPX2=0.0
,Er19i=0.0
,HIiti=0.0
,y5lm5=0.0
,c_QBZ=0.0
,AG0zS=0.0
,ryOK6=0.0
,Qo14A=0.0
,ci_fh=0.0;

GzZSa=(E6Xnb-_Qa5G)/sguyX;c_QBZ=_Qa5G*GzZSa+0.5*sguyX*KNYa5(GzZSa);
XVPX2=Er19i=y5lm5=E6Xnb/sguyX;AG0zS=ryOK6=ci_fh=0.5*KNYa5(E6Xnb)/sguyX;
Qo14A=hmD2W-g93mR-c_QBZ-AG0zS-ryOK6-ci_fh;HIiti=Qo14A/E6Xnb;return((iQ2lh+GzZSa+
XVPX2+Er19i+HIiti+y5lm5)>=SynchronizationTime);}


bool diBqY::p90k5(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
return(gK0Le(_Qa5G,E6Xnb,g93mR,hmD2W,Z45C3,AUze_,sguyX,iQ2lh,SynchronizationTime
));}


bool diBqY::QgomO(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double NIPNK=0.0
,XiHdh=0.0
,Er19i=0.0
,HIiti=0.0
,y5lm5=0.0
,xNBx6=0.0
,wB7yF=0.0
,ryOK6=0.0
,Qo14A=0.0
,ci_fh=0.0
,mcZn2=0.0
,qS318=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;qS318=SynchronizationTime-iQ2lh;
mcZn2=E6Xnb-SUYGr(KNYa5(qPN_6)*KNYa5(sguyX)-2.0*qPN_6*E6Xnb*KNYa5(sguyX)+4.0*
KNYa5(E6Xnb)*KNYa5(sguyX)-2.0*E6Xnb*qS318*D_rgb(sguyX)-2.0*D_rgb(sguyX)*gL6qW+
2.0*D_rgb(sguyX)*hmD2W)/(SUYGr(2.0)*sguyX);
NIPNK=(mcZn2-qPN_6)/sguyX;xNBx6=qPN_6*NIPNK+0.5*sguyX*KNYa5(NIPNK);
XiHdh=mcZn2/sguyX;wB7yF=0.5*KNYa5(mcZn2)/sguyX;
Er19i=y5lm5=E6Xnb/sguyX;ryOK6=ci_fh=0.5*KNYa5(E6Xnb)/sguyX;
HIiti=(qPN_6-2.0*E6Xnb-2.0*mcZn2+sguyX*qS318)/sguyX;Qo14A=HIiti*E6Xnb;
Fb1Vh+=gL6qW*NIPNK+0.5*qPN_6*KNYa5(NIPNK)+sguyX*D_rgb(NIPNK)/6.0;gL6qW+=xNBx6;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*XiHdh+0.5*qPN_6*KNYa5(XiHdh)-sguyX*D_rgb(XiHdh)/6.0;gL6qW+=wB7yF;
qPN_6=0.0;
Fb1Vh+=gL6qW*Er19i+0.5*qPN_6*KNYa5(Er19i)+sguyX*D_rgb(Er19i)/6.0;gL6qW+=ryOK6;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*HIiti+0.5*qPN_6*KNYa5(HIiti);
gL6qW+=Qo14A;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*y5lm5+0.5*qPN_6*KNYa5(y5lm5)-sguyX*D_rgb(y5lm5)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::Y8ugT(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double GzZSa=0.0
,XVPX2=0.0
,ASKNP=0.0
,Er19i=0.0
,HIiti=0.0
,y5lm5=0.0
,c_QBZ=0.0
,AG0zS=0.0
,ryOK6=0.0
,Qo14A=0.0
,ci_fh=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
GzZSa=(E6Xnb-qPN_6)/sguyX;c_QBZ=qPN_6*GzZSa+0.5*sguyX*KNYa5(GzZSa);
XVPX2=Er19i=y5lm5=E6Xnb/sguyX;AG0zS=ryOK6=ci_fh=0.5*KNYa5(E6Xnb)/sguyX;
Qo14A=hmD2W-gL6qW-c_QBZ-AG0zS-ryOK6-ci_fh;HIiti=Qo14A/E6Xnb;
ASKNP=SynchronizationTime-iQ2lh-GzZSa-XVPX2-Er19i-HIiti-y5lm5;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;gL6qW+=AG0zS;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*Er19i+0.5*qPN_6*KNYa5(Er19i)+sguyX*D_rgb(Er19i)/6.0;gL6qW+=ryOK6;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*HIiti+0.5*qPN_6*KNYa5(HIiti);
gL6qW+=Qo14A;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*y5lm5+0.5*qPN_6*KNYa5(y5lm5)-sguyX*D_rgb(y5lm5)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::yOinu(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double Er19i=0.0
,HIiti=0.0
,y5lm5=0.0
,ASKNP=0.0
,GzZSa=0.0
,XVPX2=0.0
,ryOK6=0.0
,Qo14A=0.0
,ci_fh=0.0
,c_QBZ=0.0
,AG0zS=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
Er19i=(E6Xnb-qPN_6)/sguyX;ryOK6=qPN_6*Er19i+0.5*sguyX*KNYa5(Er19i);
y5lm5=GzZSa=XVPX2=E6Xnb/sguyX;ci_fh=c_QBZ=AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
Qo14A=hmD2W-gL6qW-ryOK6-ci_fh-c_QBZ-AG0zS;HIiti=Qo14A/E6Xnb;
ASKNP=SynchronizationTime-iQ2lh-Er19i-HIiti-y5lm5-GzZSa-XVPX2;
Fb1Vh+=gL6qW*Er19i+0.5*qPN_6*KNYa5(Er19i)+sguyX*D_rgb(Er19i)/6.0;gL6qW+=ryOK6;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*HIiti+0.5*qPN_6*KNYa5(HIiti);
gL6qW+=Qo14A;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*y5lm5+0.5*qPN_6*KNYa5(y5lm5)-sguyX*D_rgb(y5lm5)/6.0;
gL6qW+=ci_fh;qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::Ak5ow(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){
double GzZSa=0.0
,XVPX2=0.0
,NIPNK=0.0
,XiHdh=0.0
,c_QBZ=0.0
,AG0zS=0.0
,mcZn2=0.0;

GzZSa=(E6Xnb-_Qa5G)/sguyX;c_QBZ=_Qa5G*GzZSa+0.5*sguyX*KNYa5(GzZSa);
XVPX2=E6Xnb/sguyX;AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
mcZn2=SUYGr(sguyX*(hmD2W-g93mR-c_QBZ-AG0zS));
NIPNK=XiHdh=mcZn2/sguyX;return((iQ2lh+GzZSa+XVPX2+NIPNK+XiHdh)>=
SynchronizationTime);}


bool diBqY::CZBQ_(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){
double GzZSa=0.0
,XVPX2=0.0
,NIPNK=0.0
,XiHdh=0.0
,c_QBZ=0.0
,AG0zS=0.0
,mcZn2=0.0;

GzZSa=XVPX2=E6Xnb/sguyX;c_QBZ=AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
mcZn2=SUYGr((KNYa5(_Qa5G)+2.0*sguyX*(hmD2W-g93mR-c_QBZ-AG0zS))/2.0);
NIPNK=(mcZn2-_Qa5G)/sguyX;
XiHdh=mcZn2/sguyX;return((iQ2lh+NIPNK+XiHdh+GzZSa+XVPX2)>=SynchronizationTime);}


bool diBqY::dM0Ji(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double NIPNK=0.0
,XiHdh=0.0
,ASKNP=0.0
,GzZSa=0.0
,XVPX2=0.0
,xNBx6=0.0
,wB7yF=0.0
,c_QBZ=0.0
,AG0zS=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
GzZSa=XVPX2=E6Xnb/sguyX;c_QBZ=AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*(hmD2W-gL6qW-c_QBZ-AG0zS))/2.0);
NIPNK=(mcZn2-qPN_6)/sguyX;xNBx6=qPN_6*NIPNK+0.5*sguyX*KNYa5(NIPNK);
XiHdh=mcZn2/sguyX;wB7yF=0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-NIPNK-XiHdh-GzZSa-XVPX2;
Fb1Vh+=gL6qW*NIPNK+0.5*qPN_6*KNYa5(NIPNK)+sguyX*D_rgb(NIPNK)/6.0;gL6qW+=xNBx6;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*XiHdh+0.5*qPN_6*KNYa5(XiHdh)-sguyX*D_rgb(XiHdh)/6.0;gL6qW+=wB7yF;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::FqFh3(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){double qJR5J=0.0,WxeUn=0.0,SU5ic=0.0,yMJaQ=0.0,qw9PQ
=0.0,JWb5x=0.0,m778l=0.0,EFQ3W=0.0,WtmID=0.0,yZrpM=0.0
,c8VVD=0.0
,pExj7=0.0
,tpyUy=0.0
,mcZn2=0.0;

mcZn2=(SUYGr(sguyX)*SUYGr(KNYa5(_Qa5G)/sguyX-2.0*g93mR+2.0*hmD2W))/2.0;
yZrpM=(mcZn2-_Qa5G)/sguyX;
c8VVD=pExj7=tpyUy=mcZn2/sguyX;;if((iQ2lh+yZrpM+c8VVD+pExj7+tpyUy)<
SynchronizationTime){return(false);}qJR5J=0.25*_Qa5G+0.25*(SynchronizationTime-
iQ2lh)*sguyX-0.25*SUYGr(3.0*KNYa5(_Qa5G)-2.0*_Qa5G*(SynchronizationTime-iQ2lh)*
sguyX+sguyX*(8.0*(hmD2W-g93mR)-KNYa5(SynchronizationTime-iQ2lh)*sguyX));WxeUn=
0.25*_Qa5G+0.25*(SynchronizationTime-iQ2lh)*sguyX+0.25*SUYGr(3.0*KNYa5(_Qa5G)-
2.0*_Qa5G*(SynchronizationTime-iQ2lh)*sguyX+sguyX*(8.0*(hmD2W-g93mR)-KNYa5(
SynchronizationTime-iQ2lh)*sguyX));if(qJR5J<_Qa5G){qJR5J=_Qa5G;}if(WxeUn<_Qa5G){
WxeUn=_Qa5G;}qw9PQ=_Qa5G*_Qa5G;EFQ3W=WxeUn+qJR5J;WtmID=WxeUn*WxeUn;JWb5x=qJR5J*
qJR5J;m778l=sguyX*sguyX;yMJaQ=Z45C3+(qw9PQ*_Qa5G/0.3e1-qw9PQ*EFQ3W-_Qa5G*sguyX*
g93mR+WtmID*WxeUn+0.2e1*WtmID*qJR5J+JWb5x*qJR5J+0.2e1*EFQ3W*sguyX*g93mR)/m778l;
if(yMJaQ>AUze_){return(true);}SU5ic=Z45C3+(qw9PQ*_Qa5G/0.3e1-qw9PQ*EFQ3W-_Qa5G*
sguyX*g93mR+JWb5x*qJR5J+0.2e1*JWb5x*WxeUn+WtmID*WxeUn+0.2e1*EFQ3W*sguyX*g93mR)/
m778l;if(SU5ic<AUze_){return(true);}return(false);}


bool diBqY::pFTdk(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double GzZSa=0.0
,XVPX2=0.0
,ASKNP=0.0
,ISB4o=0.0
,Ruz9E=0.0
,c_QBZ=0.0
,AG0zS=0.0
,OFocS=0.0
,rADbh=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
GzZSa=(E6Xnb-qPN_6)/sguyX;c_QBZ=qPN_6*GzZSa+0.5*sguyX*KNYa5(GzZSa);
XVPX2=E6Xnb/sguyX;AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
mcZn2=SUYGr(sguyX*(hmD2W-gL6qW-c_QBZ-AG0zS));
ISB4o=Ruz9E=mcZn2/sguyX;OFocS=rADbh=0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-GzZSa-XVPX2-ISB4o-Ruz9E;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;gL6qW+=AG0zS;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*ISB4o+0.5*qPN_6*KNYa5(ISB4o)+sguyX*D_rgb(ISB4o)/6.0;gL6qW+=OFocS;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*Ruz9E+0.5*qPN_6*KNYa5(Ruz9E)-sguyX*D_rgb(Ruz9E)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::jBuI7(const double&E6Xnb,const double&g93mR,const double&hmD2W,const
 double&sguyX,const double&iQ2lh,const double&SynchronizationTime){
double Ntxm9=0.0
,zqaUH=0.0
,hHY1C=0.0
,OWR_N=0.0
,YtA02=0.0
,YhsYM=0.0
,h4z3k=0.0
,b60LR=0.0
,QSroW=0.0
,i8b0z=0.0;

Ntxm9=zqaUH=hHY1C=YtA02=E6Xnb/sguyX;YhsYM=h4z3k=-0.5*KNYa5(E6Xnb)/sguyX;b60LR=
i8b0z=0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-g93mR-YhsYM-h4z3k-b60LR-i8b0z;OWR_N=QSroW/E6Xnb;return((iQ2lh+Ntxm9+
zqaUH+hHY1C+OWR_N+YtA02)>=SynchronizationTime);}


bool diBqY::DQvyi(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double MBb3Z=0.0
,NckOJ=0.0
,ASKNP=0.0
,GzZSa=0.0
,Z4ILN=0.0
,XVPX2=0.0
,T6g70=0.0
,CdoNz=0.0
,c_QBZ=0.0
,BKRPw=0.0
,AG0zS=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
MBb3Z=NckOJ=GzZSa=XVPX2=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;c_QBZ=
AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
BKRPw=hmD2W-gL6qW-T6g70-CdoNz-c_QBZ-AG0zS;Z4ILN=BKRPw/E6Xnb;
ASKNP=SynchronizationTime-iQ2lh-MBb3Z-NckOJ-GzZSa-Z4ILN-XVPX2;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;gL6qW+=CdoNz;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*Z4ILN+0.5*qPN_6*KNYa5(Z4ILN);
gL6qW+=BKRPw;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::X1Z8S(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX,const double&iQ2lh,const double&SynchronizationTime){double CnAKY=
0.0
,ISB4o=0.0
,Ruz9E=0.0
,xP9LI=0.0
,mcZn2=0.0;

CnAKY=_Qa5G/sguyX;xP9LI=0.5*KNYa5(_Qa5G)/sguyX;
mcZn2=SUYGr(sguyX*(hmD2W-g93mR-xP9LI));
ISB4o=Ruz9E=mcZn2/sguyX;return((iQ2lh+CnAKY+ISB4o+Ruz9E)>=SynchronizationTime);}


bool diBqY::viC08(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){return(Ak5ow(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX,iQ2lh,
SynchronizationTime));}


bool diBqY::CtgpQ(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){return(pFTdk(_Qa5G,E6Xnb,g93mR,
hmD2W,Z45C3,AUze_,sguyX,iQ2lh,SynchronizationTime));}


bool diBqY::dnj2T(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){double CnAKY=0.0
,ASKNP=0.0
,Ntxm9=0.0
,zqaUH=0.0
,xP9LI=0.0
,YhsYM=0.0
,h4z3k=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
CnAKY=qPN_6/sguyX;xP9LI=qPN_6*CnAKY-0.5*KNYa5(CnAKY)*sguyX;
mcZn2=SUYGr(sguyX*(hmD2W-gL6qW-xP9LI));
Ntxm9=zqaUH=mcZn2/sguyX;YhsYM=h4z3k=0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-CnAKY-Ntxm9-zqaUH;
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)-sguyX*D_rgb(CnAKY)/6.0;gL6qW+=xP9LI;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*Ntxm9+0.5*qPN_6*KNYa5(Ntxm9)+sguyX*D_rgb(Ntxm9)/6.0;gL6qW+=YhsYM;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*zqaUH+0.5*qPN_6*KNYa5(zqaUH)-sguyX*D_rgb(zqaUH)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::HsH3A(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){return(Ak5ow(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX,iQ2lh,
SynchronizationTime));}


bool diBqY::iNgQd(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX){return(IrAqR(_Qa5G,g93mR,hmD2W,sguyX));}


bool diBqY::ML6NP(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){return(pFTdk(_Qa5G,E6Xnb,g93mR,
hmD2W,Z45C3,AUze_,sguyX,iQ2lh,SynchronizationTime));}


bool diBqY::rqzRD(const double&E6Xnb,const double&g93mR,const double&hmD2W,const
 double&sguyX,const double&iQ2lh,const double&SynchronizationTime){
double GzZSa=0.0
,XVPX2=0.0
,xk0GU=0.0
,fijfm=0.0
,c_QBZ=0.0
,AG0zS=0.0
,mcZn2=0.0
,gL6qW=g93mR,aaprD=hmD2W;
GzZSa=XVPX2=E6Xnb/sguyX;c_QBZ=AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
gL6qW=-gL6qW;aaprD=-aaprD;c_QBZ=-c_QBZ;AG0zS=-AG0zS;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW-c_QBZ-AG0zS));
gL6qW=-gL6qW;aaprD=-aaprD;c_QBZ=-c_QBZ;AG0zS=-AG0zS;mcZn2=-mcZn2;
xk0GU=fijfm=(-mcZn2)/sguyX;return((iQ2lh+GzZSa+XVPX2+xk0GU+fijfm)>=
SynchronizationTime);}


bool diBqY::dJusL(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double NIPNK=0.0
,XiHdh=0.0
,ASKNP=0.0
,GzZSa=0.0
,XVPX2=0.0
,xNBx6=0.0
,wB7yF=0.0
,c_QBZ=0.0
,AG0zS=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;
GzZSa=XVPX2=E6Xnb/sguyX;c_QBZ=AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
aaprD=-aaprD;gL6qW=-gL6qW;c_QBZ=-c_QBZ;AG0zS=-AG0zS;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW-c_QBZ-AG0zS));
mcZn2=-mcZn2;aaprD=-aaprD;gL6qW=-gL6qW;c_QBZ=-c_QBZ;AG0zS=-AG0zS;
NIPNK=XiHdh=(-mcZn2)/sguyX;xNBx6=wB7yF=-0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-NIPNK-XiHdh-GzZSa-XVPX2;
Fb1Vh+=gL6qW*NIPNK+0.5*qPN_6*KNYa5(NIPNK)-sguyX*D_rgb(NIPNK)/6.0;gL6qW+=xNBx6;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*XiHdh+0.5*qPN_6*KNYa5(XiHdh)+sguyX*D_rgb(XiHdh)/6.0;gL6qW+=wB7yF;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::Kg0C9(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&g85jV,const 
double&iQ2lh,const double&SynchronizationTime){
double ISB4o=0.0
,Ruz9E=0.0
,ASKNP=0.0
,OFocS=0.0
,rADbh=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*(hmD2W-gL6qW))/2.0);
ISB4o=(mcZn2-qPN_6)/sguyX;OFocS=qPN_6*ISB4o+0.5*sguyX*KNYa5(ISB4o);
Ruz9E=mcZn2/sguyX;rADbh=0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-ISB4o-Ruz9E;
Fb1Vh+=gL6qW*ISB4o+0.5*qPN_6*KNYa5(ISB4o)+sguyX*D_rgb(ISB4o)/6.0;gL6qW+=OFocS;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*Ruz9E+0.5*qPN_6*KNYa5(Ruz9E)-sguyX*D_rgb(Ruz9E)/6.0;gL6qW=hmD2W;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=hmD2W;qPN_6=0.0;if(hmD2W>0.0){return(Fb1Vh<=AUze_+g85jV*WTpjf*(1.0-(g85jV-
hmD2W)/g85jV));}else{return(Fb1Vh<=AUze_);}}


bool diBqY::SzDHL(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){
double GzZSa=0.0
,XVPX2=0.0
,cpEY4=0.0
,YQYKj=0.0
,mcZn2=0.0
,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;
GzZSa=(E6Xnb-qPN_6)/sguyX;
gL6qW+=qPN_6*GzZSa+0.5*sguyX*KNYa5(GzZSa);qPN_6=E6Xnb;
XVPX2=E6Xnb/sguyX;
gL6qW+=qPN_6*XVPX2-0.5*sguyX*KNYa5(XVPX2);qPN_6=0.0;

aaprD=-aaprD;gL6qW=-gL6qW;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW));
aaprD=-aaprD;gL6qW=-gL6qW;mcZn2=-mcZn2;
cpEY4=YQYKj=(-mcZn2)/sguyX;return((iQ2lh+GzZSa+XVPX2+cpEY4+YQYKj)>=
SynchronizationTime);}


bool diBqY::HrBoU(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double GzZSa=0.0
,XVPX2=0.0
,ASKNP=0.0
,cpEY4=0.0
,YQYKj=0.0
,c_QBZ=0.0
,AG0zS=0.0
,SWM9y=0.0
,bjuLh=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;
GzZSa=(E6Xnb-qPN_6)/sguyX;c_QBZ=qPN_6*GzZSa+0.5*sguyX*KNYa5(GzZSa);
XVPX2=E6Xnb/sguyX;AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
aaprD=-aaprD;gL6qW=-gL6qW;c_QBZ=-c_QBZ;AG0zS=-AG0zS;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW-c_QBZ-AG0zS));
mcZn2=-mcZn2;aaprD=-aaprD;gL6qW=-gL6qW;c_QBZ=-c_QBZ;AG0zS=-AG0zS;
cpEY4=YQYKj=(-mcZn2)/sguyX;SWM9y=bjuLh=-0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-GzZSa-XVPX2-cpEY4-YQYKj;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;
gL6qW+=AG0zS;qPN_6=(0xbec+953-0xfa5);
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*cpEY4+0.5*qPN_6*KNYa5(cpEY4)-sguyX*D_rgb(cpEY4)/6.0;gL6qW+=SWM9y;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*YQYKj+0.5*qPN_6*KNYa5(YQYKj)+sguyX*D_rgb(YQYKj)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::ZNV0H(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX,const double&iQ2lh,const double&SynchronizationTime){return(X1Z8S(
_Qa5G,g93mR,hmD2W,sguyX,iQ2lh,SynchronizationTime));}


bool diBqY::dswaO(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX){return(IrAqR(_Qa5G,g93mR,hmD2W,sguyX));}


bool diBqY::U1aEP(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX,const double&iQ2lh,const double&SynchronizationTime){return(GN20j(
_Qa5G,g93mR,hmD2W,sguyX,iQ2lh,SynchronizationTime));}


bool diBqY::qYLF3(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){return(u4kxD(_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
iQ2lh,SynchronizationTime));}


bool diBqY::Twslo(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){return(kwgCa(_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
iQ2lh,SynchronizationTime));}


bool diBqY::WQvar(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){return(KmXwx(_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
iQ2lh,SynchronizationTime));}


bool diBqY::Vvsmm(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){
return(FqFh3(_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,iQ2lh,SynchronizationTime));}


bool diBqY::X6Pnz(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){return(dnj2T(_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
iQ2lh,SynchronizationTime));}


bool diBqY::IrAqR(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX){
double Time=0.0
,BCZPg=0.0
,mcZn2=0.0
,gL6qW=g93mR,qPN_6=_Qa5G;BCZPg=qPN_6;
Time=qPN_6/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;
mcZn2=SUYGr(sguyX*(hmD2W-gL6qW));return(BCZPg>=mcZn2);}


bool diBqY::cKRk0(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
return(v19ag(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::SpHZS(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&g85jV,const 
double&iQ2lh,const double&SynchronizationTime){
double CnAKY=0.0
,cpEY4=0.0
,YQYKj=0.0
,ASKNP=0.0
,xP9LI=0.0
,SWM9y=0.0
,bjuLh=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;
CnAKY=qPN_6/sguyX;xP9LI=0.5*KNYa5(qPN_6)/sguyX;
aaprD=-aaprD;gL6qW=-gL6qW;xP9LI=-xP9LI;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW-xP9LI));
mcZn2=-mcZn2;aaprD=-aaprD;gL6qW=-gL6qW;xP9LI=-xP9LI;
cpEY4=YQYKj=(-mcZn2)/sguyX;SWM9y=bjuLh=-0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-CnAKY-cpEY4-YQYKj;
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)-sguyX*D_rgb(CnAKY)/6.0;gL6qW+=xP9LI;
qPN_6=0.0;
Fb1Vh+=gL6qW*cpEY4+0.5*qPN_6*KNYa5(cpEY4)-sguyX*D_rgb(cpEY4)/6.0;gL6qW+=SWM9y;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*YQYKj+0.5*qPN_6*KNYa5(YQYKj)+sguyX*D_rgb(YQYKj)/6.0;gL6qW=aaprD;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=aaprD;qPN_6=0.0;if(hmD2W<0.0){return(Fb1Vh>=AUze_-g85jV*WTpjf*(1.0-(g85jV+
hmD2W)/g85jV));}else{return(Fb1Vh>=AUze_);}}


bool diBqY::reH2S(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){
double MBb3Z=0.0
,NckOJ=0.0
,Ntxm9=0.0
,zqaUH=0.0
,T6g70=0.0
,CdoNz=0.0
,mcZn2=0.0
,gL6qW=g93mR,qPN_6=_Qa5G;
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
gL6qW+=T6g70;qPN_6=-E6Xnb;
gL6qW+=CdoNz;qPN_6=0.0;
mcZn2=SUYGr(sguyX*(hmD2W-gL6qW));
Ntxm9=zqaUH=mcZn2/sguyX;return((iQ2lh+MBb3Z+NckOJ+Ntxm9+zqaUH)>=
SynchronizationTime);}


bool diBqY::kjG_Z(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double MBb3Z=0.0
,NckOJ=0.0
,ASKNP=0.0
,ISB4o=0.0
,Ruz9E=0.0
,T6g70=0.0
,CdoNz=0.0
,OFocS=0.0
,rADbh=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
mcZn2=SUYGr(sguyX*(hmD2W-gL6qW-T6g70-CdoNz));
ISB4o=Ruz9E=mcZn2/sguyX;OFocS=rADbh=0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-MBb3Z-NckOJ-ISB4o-Ruz9E;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;gL6qW+=CdoNz;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*ISB4o+0.5*qPN_6*KNYa5(ISB4o)+sguyX*D_rgb(ISB4o)/6.0;gL6qW+=OFocS;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*Ruz9E+0.5*qPN_6*KNYa5(Ruz9E)-sguyX*D_rgb(Ruz9E)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::RPTai(const double&E6Xnb,const double&g93mR,const double&hmD2W,const
 double&sguyX,const double&iQ2lh,const double&SynchronizationTime){
double MBb3Z=0.0
,P1gs6=0.0
,NckOJ=0.0
,GzZSa=0.0
,XVPX2=0.0
,T6g70=0.0
,UM2M2=0.0
,CdoNz=0.0
,c_QBZ=0.0
,AG0zS=0.0;

MBb3Z=NckOJ=GzZSa=XVPX2=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;c_QBZ=
AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
UM2M2=hmD2W-g93mR-T6g70-CdoNz-c_QBZ-AG0zS;P1gs6=UM2M2/(-E6Xnb);return((iQ2lh+
MBb3Z+P1gs6+NckOJ+GzZSa+XVPX2)>=SynchronizationTime);}


bool diBqY::A4XxK(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double MBb3Z=0.0
,P1gs6=0.0
,NckOJ=0.0
,ASKNP=0.0
,GzZSa=0.0
,XVPX2=0.0
,T6g70=0.0
,UM2M2=0.0
,CdoNz=0.0
,c_QBZ=0.0
,AG0zS=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
MBb3Z=NckOJ=GzZSa=XVPX2=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;c_QBZ=
AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
UM2M2=hmD2W-gL6qW-T6g70-CdoNz-c_QBZ-AG0zS;P1gs6=UM2M2/(-E6Xnb);
ASKNP=SynchronizationTime-iQ2lh-MBb3Z-P1gs6-NckOJ-GzZSa-XVPX2;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*P1gs6+0.5*qPN_6*KNYa5(P1gs6);
gL6qW+=UM2M2;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;
gL6qW+=CdoNz;qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::KxYW3(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){double CnAKY=0.0
,ASKNP=0.0
,Ntxm9=0.0
,zqaUH=0.0
,xP9LI=0.0
,YhsYM=0.0
,h4z3k=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;
CnAKY=qPN_6/sguyX;xP9LI=qPN_6*CnAKY-0.5*KNYa5(CnAKY)*sguyX;
aaprD=-aaprD;gL6qW=-gL6qW;xP9LI=-xP9LI;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW-xP9LI));
aaprD=-aaprD;gL6qW=-gL6qW;xP9LI=-xP9LI;mcZn2=-mcZn2;
Ntxm9=zqaUH=(-mcZn2)/sguyX;YhsYM=h4z3k=-0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-CnAKY-Ntxm9-zqaUH;
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)-sguyX*D_rgb(CnAKY)/6.0;gL6qW+=xP9LI;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*Ntxm9+0.5*qPN_6*KNYa5(Ntxm9)-sguyX*D_rgb(Ntxm9)/6.0;gL6qW+=YhsYM;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*zqaUH+0.5*qPN_6*KNYa5(zqaUH)+sguyX*D_rgb(zqaUH)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::hj6Lo(const double&g93mR,const double&hmD2W,const double&Z45C3,const
 double&AUze_,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){return(FqFh3(0.0,-g93mR,-hmD2W,-Z45C3,-AUze_,sguyX,iQ2lh,
SynchronizationTime));}


bool diBqY::Uhhzv(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){return(zFqoc(_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
iQ2lh,SynchronizationTime));}


bool diBqY::B3YwL(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
return(pVKSx(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::MvKFA(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){
double NIPNK=0.0
,XiHdh=0.0
,MBb3Z=0.0
,NckOJ=0.0
,T6g70=0.0
,CdoNz=0.0
,mcZn2=0.0;

MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
mcZn2=SUYGr((KNYa5(_Qa5G)+2.0*sguyX*(hmD2W-g93mR-T6g70-CdoNz))/2.0);
NIPNK=(mcZn2-_Qa5G)/sguyX;
XiHdh=mcZn2/sguyX;return((iQ2lh+NIPNK+XiHdh+MBb3Z+NckOJ)>=SynchronizationTime);}


bool diBqY::ArK6j(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double NIPNK=0.0
,XiHdh=0.0
,ASKNP=0.0
,MBb3Z=0.0
,NckOJ=0.0
,xNBx6=0.0
,wB7yF=0.0
,T6g70=0.0
,CdoNz=0.0
,deRRY=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*(hmD2W-gL6qW-T6g70-CdoNz))/2.0);
NIPNK=(mcZn2-qPN_6)/sguyX;xNBx6=qPN_6*NIPNK+0.5*sguyX*KNYa5(NIPNK);
XiHdh=mcZn2/sguyX;wB7yF=0.5*KNYa5(mcZn2)/sguyX;
deRRY=gL6qW+xNBx6+wB7yF;
ASKNP=SynchronizationTime-iQ2lh-NIPNK-XiHdh-MBb3Z-NckOJ;
Fb1Vh+=gL6qW*NIPNK+0.5*qPN_6*KNYa5(NIPNK)+sguyX*D_rgb(NIPNK)/6.0;gL6qW+=xNBx6;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*XiHdh+0.5*qPN_6*KNYa5(XiHdh)-sguyX*D_rgb(XiHdh)/6.0;gL6qW+=wB7yF;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::b4HOI(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){
double GzZSa=0.0
,XVPX2=0.0
,MBb3Z=0.0
,P1gs6=0.0
,NckOJ=0.0
,T6g70=0.0
,UM2M2=0.0
,CdoNz=0.0
,gL6qW=g93mR,qPN_6=_Qa5G;
GzZSa=(E6Xnb-qPN_6)/sguyX;gL6qW+=qPN_6*GzZSa+0.5*sguyX*KNYa5(GzZSa);qPN_6=E6Xnb;
XVPX2=E6Xnb/sguyX;gL6qW+=qPN_6*XVPX2-0.5*sguyX*KNYa5(XVPX2);qPN_6=0.0;
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
UM2M2=hmD2W-gL6qW-T6g70-CdoNz;P1gs6=UM2M2/(-E6Xnb);return((iQ2lh+GzZSa+XVPX2+
MBb3Z+P1gs6+NckOJ)>=SynchronizationTime);}


bool diBqY::i5puw(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double GzZSa=0.0
,XVPX2=0.0
,ASKNP=0.0
,QTEIk=0.0
,hxFqw=0.0
,FQl_r=0.0
,c_QBZ=0.0
,AG0zS=0.0
,ZyUNF=0.0
,c7IBe=0.0
,btMol=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
GzZSa=(E6Xnb-qPN_6)/sguyX;c_QBZ=qPN_6*GzZSa+0.5*sguyX*KNYa5(GzZSa);
XVPX2=E6Xnb/sguyX;AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
QTEIk=FQl_r=E6Xnb/sguyX;ZyUNF=btMol=-0.5*KNYa5(E6Xnb)/sguyX;
c7IBe=hmD2W-gL6qW-c_QBZ-AG0zS-ZyUNF-btMol;hxFqw=c7IBe/(-E6Xnb);
ASKNP=SynchronizationTime-iQ2lh-GzZSa-XVPX2-QTEIk-hxFqw-FQl_r;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;gL6qW+=AG0zS;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*QTEIk+0.5*qPN_6*KNYa5(QTEIk)-sguyX*D_rgb(QTEIk)/6.0;gL6qW+=ZyUNF;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*hxFqw+0.5*qPN_6*KNYa5(hxFqw);
gL6qW+=c7IBe;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*FQl_r+0.5*qPN_6*KNYa5(FQl_r)+sguyX*D_rgb(FQl_r)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::eni4Q(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&g85jV,const double&iQ2lh,const double&SynchronizationTime){
double CnAKY=0.0
,MBb3Z=0.0
,P1gs6=0.0
,NckOJ=0.0
,ASKNP=0.0
,xP9LI=0.0
,T6g70=0.0
,UM2M2=0.0
,CdoNz=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
CnAKY=qPN_6/sguyX;xP9LI=0.5*KNYa5(qPN_6)/sguyX;
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
UM2M2=hmD2W-gL6qW-xP9LI-T6g70-CdoNz;P1gs6=UM2M2/(-E6Xnb);
ASKNP=SynchronizationTime-iQ2lh-CnAKY-MBb3Z-P1gs6-NckOJ;
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)-sguyX*D_rgb(CnAKY)/6.0;gL6qW+=xP9LI;
qPN_6=0.0;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*P1gs6+0.5*qPN_6*KNYa5(P1gs6);
gL6qW+=UM2M2;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;
gL6qW=hmD2W;qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=hmD2W;qPN_6=0.0;if(hmD2W<0.0){return(Fb1Vh>=AUze_-g85jV*WTpjf*(1.0-(g85jV+
hmD2W)/g85jV));}else{return(Fb1Vh>=AUze_);}}


bool diBqY::sVbun(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){double CnAKY=0.0
,ASKNP=0.0
,hHY1C=0.0
,OWR_N=0.0
,YtA02=0.0
,xP9LI=0.0
,b60LR=0.0
,Rpl3H=0.0
,i8b0z=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
CnAKY=qPN_6/sguyX;xP9LI=qPN_6*CnAKY-0.5*KNYa5(CnAKY)*sguyX;
hHY1C=YtA02=E6Xnb/sguyX;b60LR=i8b0z=-0.5*KNYa5(E6Xnb)/sguyX;
Rpl3H=hmD2W-gL6qW-xP9LI-b60LR-i8b0z;OWR_N=Rpl3H/(-E6Xnb);
ASKNP=SynchronizationTime-iQ2lh-CnAKY-hHY1C-OWR_N-YtA02;
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)-sguyX*D_rgb(CnAKY)/6.0;gL6qW+=xP9LI;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*hHY1C+0.5*qPN_6*KNYa5(hHY1C)-sguyX*D_rgb(hHY1C)/6.0;gL6qW+=b60LR;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*OWR_N+0.5*qPN_6*KNYa5(OWR_N);
gL6qW+=Rpl3H;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*YtA02+0.5*qPN_6*KNYa5(YtA02)+sguyX*D_rgb(YtA02)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::Z2pMH(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
double qPN_6=_Qa5G,Time=0.0,gL6qW=g93mR;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;


gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=0.0;


gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;


gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW>=hmD2W);}


bool diBqY::_hipW(const double&E6Xnb,const double&g93mR,const double&hmD2W,const
 double&sguyX,const double&iQ2lh,const double&SynchronizationTime){
double MBb3Z=0.0
,NckOJ=0.0
,QTEIk=0.0
,hxFqw=0.0
,FQl_r=0.0
,T6g70=0.0
,CdoNz=0.0
,ZyUNF=0.0
,c7IBe=0.0
,btMol=0.0;

MBb3Z=NckOJ=QTEIk=FQl_r=E6Xnb/sguyX;T6g70=CdoNz=ZyUNF=btMol=-0.5*KNYa5(E6Xnb)/
sguyX;
c7IBe=hmD2W-g93mR-T6g70-CdoNz-ZyUNF-btMol;hxFqw=c7IBe/(-E6Xnb);return((iQ2lh+
MBb3Z+NckOJ+QTEIk+hxFqw+FQl_r)>=SynchronizationTime);}


bool diBqY::SDK9Y(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double MBb3Z=0.0
,P1gs6=0.0
,NckOJ=0.0
,NIPNK=0.0
,XiHdh=0.0
,T6g70=0.0
,UM2M2=0.0
,CdoNz=0.0
,xNBx6=0.0
,wB7yF=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
mcZn2=0.5*(-2.0*E6Xnb+2.8284271247461903*SUYGr(1.0*KNYa5(E6Xnb)+0.5*E6Xnb*sguyX*
iQ2lh-0.5*E6Xnb*sguyX*SynchronizationTime+0.5*sguyX*gL6qW-0.5*sguyX*hmD2W));
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
P1gs6=(-4.0*E6Xnb)/sguyX-1.0*iQ2lh+SynchronizationTime+(2.8284271247461903*SUYGr
(1.0*KNYa5(E6Xnb)+0.5*E6Xnb*sguyX*iQ2lh-0.5*E6Xnb*sguyX*SynchronizationTime+0.5*
sguyX*gL6qW-0.5*sguyX*hmD2W))/sguyX;UM2M2=(-E6Xnb)*P1gs6;
NIPNK=XiHdh=(-mcZn2)/sguyX;xNBx6=wB7yF=-0.5*KNYa5(mcZn2)/sguyX;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*P1gs6+0.5*qPN_6*KNYa5(P1gs6);
gL6qW+=UM2M2;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;
gL6qW+=CdoNz;qPN_6=0.0;
Fb1Vh+=gL6qW*NIPNK+0.5*qPN_6*KNYa5(NIPNK)-sguyX*D_rgb(NIPNK)/6.0;gL6qW+=xNBx6;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*XiHdh+0.5*qPN_6*KNYa5(XiHdh)+sguyX*D_rgb(XiHdh)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::S9z58(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double NIPNK=0.0
,XiHdh=0.0
,MBb3Z=0.0
,P1gs6=0.0
,NckOJ=0.0
,xNBx6=0.0
,wB7yF=0.0
,T6g70=0.0
,UM2M2=0.0
,CdoNz=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
mcZn2=0.5*(-2.0*E6Xnb+2.8284271247461903*SUYGr(1.0*KNYa5(E6Xnb)+0.5*E6Xnb*sguyX*
iQ2lh-0.5*E6Xnb*sguyX*SynchronizationTime+0.5*sguyX*gL6qW-0.5*sguyX*hmD2W));
NIPNK=XiHdh=(-mcZn2)/sguyX;xNBx6=wB7yF=-0.5*KNYa5(mcZn2)/sguyX;
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
P1gs6=(-4.0*E6Xnb)/sguyX-1.0*iQ2lh+SynchronizationTime+(2.8284271247461903*SUYGr
(1.0*KNYa5(E6Xnb)+0.5*E6Xnb*sguyX*iQ2lh-0.5*E6Xnb*sguyX*SynchronizationTime+0.5*
sguyX*gL6qW-0.5*sguyX*hmD2W))/sguyX;UM2M2=(-E6Xnb)*P1gs6;
Fb1Vh+=gL6qW*NIPNK+0.5*qPN_6*KNYa5(NIPNK)-sguyX*D_rgb(NIPNK)/6.0;gL6qW+=xNBx6;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*XiHdh+0.5*qPN_6*KNYa5(XiHdh)+sguyX*D_rgb(XiHdh)/6.0;gL6qW+=wB7yF;
qPN_6=0.0;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*P1gs6+0.5*qPN_6*KNYa5(P1gs6);
gL6qW+=UM2M2;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::zFqoc(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX,const double&iQ2lh,const 
double&SynchronizationTime){double jaJi5=0.0
,frEOw=0.0
,iZ7FW=0.0
,GscyC=0.0
,y_psI=0.0
,ebVM3=0.0
,bZn23=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
jaJi5=0.5*qPN_6+0.5*sguyX*iQ2lh-0.5*sguyX*SynchronizationTime+0.25*sguyX*SUYGr((
-4.*KNYa5(qPN_6))/KNYa5(sguyX)+(8.0*qPN_6*iQ2lh)/sguyX+4.0*KNYa5(iQ2lh)-(8.0*
qPN_6*SynchronizationTime)/sguyX-8.0*iQ2lh*SynchronizationTime+4.0*KNYa5(
SynchronizationTime)-(16.0*gL6qW)/sguyX+(16.0*hmD2W)/sguyX);
frEOw=(qPN_6-jaJi5)/sguyX;y_psI=qPN_6*frEOw-0.5*sguyX*KNYa5(frEOw);
GscyC=(-jaJi5)/sguyX;bZn23=-0.5*KNYa5(jaJi5)/sguyX;
iZ7FW=SynchronizationTime-iQ2lh-frEOw-GscyC;ebVM3=iZ7FW*jaJi5;
Fb1Vh+=gL6qW*frEOw+0.5*qPN_6*KNYa5(frEOw)-sguyX*D_rgb(frEOw)/6.0;gL6qW+=y_psI;
qPN_6=jaJi5;
Fb1Vh+=gL6qW*iZ7FW+0.5*qPN_6*KNYa5(iZ7FW);gL6qW+=ebVM3;qPN_6=jaJi5;
Fb1Vh+=gL6qW*GscyC+0.5*qPN_6*KNYa5(GscyC)+sguyX*D_rgb(GscyC)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::I7QAT(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){double jaJi5=0.0
,MBb3Z=0.0
,GscyC=0.0
,iZ7FW=0.0
,CnAKY=0.0
,T6g70=0.0
,bZn23=0.0
,ebVM3=0.0
,_lOHs=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
jaJi5=(sguyX*(gL6qW-hmD2W)-KNYa5(E6Xnb))/(2.0*E6Xnb-sguyX*(SynchronizationTime-
iQ2lh));
MBb3Z=(E6Xnb+qPN_6)/sguyX;T6g70=qPN_6*MBb3Z-0.5*sguyX*KNYa5(MBb3Z);
GscyC=(jaJi5+E6Xnb)/sguyX;bZn23=-E6Xnb*GscyC+0.5*sguyX*KNYa5(GscyC);
CnAKY=(-jaJi5)/sguyX;_lOHs=-0.5*KNYa5(jaJi5)/sguyX;
iZ7FW=SynchronizationTime-iQ2lh-MBb3Z-GscyC-CnAKY;ebVM3=jaJi5*iZ7FW;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb,
Fb1Vh+=gL6qW*GscyC+0.5*qPN_6*KNYa5(GscyC)+sguyX*D_rgb(GscyC)/6.0;gL6qW+=bZn23;
qPN_6=jaJi5;
Fb1Vh+=gL6qW*iZ7FW+0.5*qPN_6*KNYa5(iZ7FW);gL6qW+=ebVM3;qPN_6=jaJi5;
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)+sguyX*D_rgb(CnAKY)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::BOevi(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){double jaJi5=0.0
,frEOw=0.0
,iZ7FW=0.0
,MBb3Z=0.0
,CnAKY=0.0
,y_psI=0.0
,ebVM3=0.0
,T6g70=0.0
,_lOHs=0.0
,qS318=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;qS318=SynchronizationTime-iQ2lh;
jaJi5=(sguyX*(gL6qW-hmD2W)-KNYa5(E6Xnb))/(2.0*E6Xnb-qS318*sguyX);
frEOw=(qPN_6-jaJi5)/sguyX;y_psI=qPN_6*frEOw-0.5*sguyX*KNYa5(frEOw);
MBb3Z=(E6Xnb+jaJi5)/sguyX;T6g70=jaJi5*MBb3Z-0.5*MBb3Z*(E6Xnb+jaJi5);
CnAKY=E6Xnb/sguyX;_lOHs=-0.5*KNYa5(E6Xnb)/sguyX;
iZ7FW=SynchronizationTime-iQ2lh-frEOw-MBb3Z-CnAKY;ebVM3=jaJi5*iZ7FW;
Fb1Vh+=gL6qW*frEOw+0.5*qPN_6*KNYa5(frEOw)-sguyX*D_rgb(frEOw)/6.0;gL6qW+=y_psI;
qPN_6=jaJi5;
Fb1Vh+=gL6qW*iZ7FW+0.5*qPN_6*KNYa5(iZ7FW);gL6qW+=ebVM3;qPN_6=jaJi5;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb,
Fb1Vh+=gL6qW*CnAKY+0.5*qPN_6*KNYa5(CnAKY)+sguyX*D_rgb(CnAKY)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::WfbVM(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double MBb3Z=0.0
,NckOJ=0.0
,ASKNP=0.0
,QTEIk=0.0
,hxFqw=0.0
,FQl_r=0.0
,T6g70=0.0
,CdoNz=0.0
,ZyUNF=0.0
,c7IBe=0.0
,btMol=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
MBb3Z=NckOJ=QTEIk=FQl_r=E6Xnb/sguyX;T6g70=CdoNz=ZyUNF=btMol=-0.5*KNYa5(E6Xnb)/
sguyX;
c7IBe=hmD2W-gL6qW-T6g70-CdoNz-ZyUNF-btMol;hxFqw=c7IBe/(-E6Xnb);
ASKNP=SynchronizationTime-iQ2lh-MBb3Z-NckOJ-QTEIk-hxFqw-FQl_r;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;gL6qW+=CdoNz;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*QTEIk+0.5*qPN_6*KNYa5(QTEIk)-sguyX*D_rgb(QTEIk)/6.0;gL6qW+=ZyUNF;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*hxFqw+0.5*qPN_6*KNYa5(hxFqw);
gL6qW+=c7IBe;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*FQl_r+0.5*qPN_6*KNYa5(FQl_r)+sguyX*D_rgb(FQl_r)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::Swe2Y(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double QTEIk=0.0
,hxFqw=0.0
,FQl_r=0.0
,ASKNP=0.0
,MBb3Z=0.0
,NckOJ=0.0
,ZyUNF=0.0
,c7IBe=0.0
,btMol=0.0
,T6g70=0.0
,CdoNz=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
QTEIk=FQl_r=MBb3Z=NckOJ=E6Xnb/sguyX;ZyUNF=btMol=T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/
sguyX;
c7IBe=hmD2W-gL6qW-ZyUNF-btMol-T6g70-CdoNz;hxFqw=c7IBe/(-E6Xnb);
ASKNP=SynchronizationTime-iQ2lh-QTEIk-hxFqw-FQl_r-MBb3Z-NckOJ;
Fb1Vh+=gL6qW*QTEIk+0.5*qPN_6*KNYa5(QTEIk)-sguyX*D_rgb(QTEIk)/6.0;gL6qW+=ZyUNF;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*hxFqw+0.5*qPN_6*KNYa5(hxFqw);
gL6qW+=c7IBe;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*FQl_r+0.5*qPN_6*KNYa5(FQl_r)+sguyX*D_rgb(FQl_r)/6.0;
gL6qW+=btMol;qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::KF5aQ(const double&E6Xnb,const double&g93mR,const double&hmD2W,const
 double&sguyX,const double&iQ2lh,const double&SynchronizationTime){
double MBb3Z=0.0
,NckOJ=0.0
,NIPNK=0.0
,XiHdh=0.0
,T6g70=0.0
,CdoNz=0.0
,mcZn2=0.0
,aaprD=hmD2W,gL6qW=g93mR;
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
aaprD=-aaprD;gL6qW=-gL6qW;T6g70=-T6g70;CdoNz=-CdoNz;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW-T6g70-CdoNz));
aaprD=-aaprD;gL6qW=-gL6qW;T6g70=-T6g70;CdoNz=-CdoNz;mcZn2=-mcZn2;
NIPNK=XiHdh=(-mcZn2)/sguyX;return((iQ2lh+MBb3Z+NckOJ+NIPNK+XiHdh)>=
SynchronizationTime);}


bool diBqY::nXl0h(const double&E6Xnb,const double&g93mR,const double&hmD2W,const
 double&sguyX,const double&iQ2lh,const double&SynchronizationTime){
double NIPNK=0.0
,XiHdh=0.0
,MBb3Z=0.0
,NckOJ=0.0
,T6g70=0.0
,CdoNz=0.0
,mcZn2=0.0,aaprD=hmD2W,gL6qW=g93mR;
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
aaprD=-aaprD;gL6qW=-gL6qW;T6g70=-T6g70;CdoNz=-CdoNz;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW-T6g70-CdoNz));
aaprD=-aaprD;gL6qW=-gL6qW;T6g70=-T6g70;CdoNz=-CdoNz;mcZn2=-mcZn2;
NIPNK=XiHdh=(-mcZn2)/sguyX;return((iQ2lh+NIPNK+XiHdh+MBb3Z+NckOJ)>=
SynchronizationTime);}


bool diBqY::vnASA(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
return(SDK9Y(_Qa5G,E6Xnb,g93mR,hmD2W,Z45C3,AUze_,sguyX,iQ2lh,SynchronizationTime
));}


bool diBqY::hyL5c(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double NIPNK=0.0
,XiHdh=0.0
,ASKNP=0.0
,MBb3Z=0.0
,NckOJ=0.0
,xNBx6=0.0
,wB7yF=0.0
,T6g70=0.0
,CdoNz=0.0
,mcZn2=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
aaprD=-aaprD;gL6qW=-gL6qW;T6g70=-T6g70;CdoNz=-CdoNz;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW-T6g70-CdoNz));
aaprD=-aaprD;gL6qW=-gL6qW;T6g70=-T6g70;CdoNz=-CdoNz;
NIPNK=XiHdh=mcZn2/sguyX;xNBx6=wB7yF=-0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-NIPNK-XiHdh-MBb3Z-NckOJ;
Fb1Vh+=gL6qW*NIPNK+0.5*qPN_6*KNYa5(NIPNK)-sguyX*D_rgb(NIPNK)/6.0;gL6qW+=xNBx6;
qPN_6=-mcZn2;
Fb1Vh+=gL6qW*XiHdh+0.5*qPN_6*KNYa5(XiHdh)+sguyX*D_rgb(XiHdh)/6.0;gL6qW+=wB7yF;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::F7vDL(const double&g93mR,const double&hmD2W,const double&Z45C3,const
 double&AUze_,const double&sguyX,const double&iQ2lh,const double&
SynchronizationTime){
return(hj6Lo(g93mR,hmD2W,Z45C3,AUze_,sguyX,iQ2lh,SynchronizationTime));}


bool diBqY::I3AxS(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
double&iQ2lh,const double&SynchronizationTime){
double MBb3Z=0.0
,NckOJ=0.0
,ASKNP=0.0
,NIPNK=0.0
,XiHdh=0.0
,T6g70=0.0
,CdoNz=0.0
,xNBx6=0.0
,wB7yF=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;
MBb3Z=NckOJ=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
aaprD=-aaprD;gL6qW=-gL6qW;T6g70=-T6g70;CdoNz=-CdoNz;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW-T6g70-CdoNz));
aaprD=-aaprD;gL6qW=-gL6qW;T6g70=-T6g70;CdoNz=-CdoNz;mcZn2=-mcZn2;
NIPNK=XiHdh=(-mcZn2)/sguyX;xNBx6=wB7yF=-0.5*KNYa5(mcZn2)/sguyX;
ASKNP=SynchronizationTime-iQ2lh-MBb3Z-NckOJ-NIPNK-XiHdh;
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;gL6qW+=CdoNz;
qPN_6=0.0;
Fb1Vh+=gL6qW*ASKNP;
gL6qW=gL6qW;qPN_6=0.0;
Fb1Vh+=gL6qW*NIPNK+0.5*qPN_6*KNYa5(NIPNK)-sguyX*D_rgb(NIPNK)/6.0;gL6qW+=xNBx6;
qPN_6=mcZn2;
Fb1Vh+=gL6qW*XiHdh+0.5*qPN_6*KNYa5(XiHdh)+sguyX*D_rgb(XiHdh)/6.0;return(Fb1Vh>=
AUze_);}
