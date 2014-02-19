
































#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLABK.h>
#include <TypeIVRMLStep1RootFunctions.h>
#include <math.h>
#include <stdlib.h>








bool diBqY::FlP2m(const double&_Qa5G){return(_Qa5G>=0.0);}


bool diBqY::W5C39(const double&_Qa5G,const double&E6Xnb){return(_Qa5G<=E6Xnb);}


bool diBqY::dS4fr(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){double gL6qW=g93mR,Time=0.0;Time=_Qa5G/sguyX;gL6qW+=_Qa5G*Time-
0.5*sguyX*KNYa5(Time);return(gL6qW<=g85jV);}


bool diBqY::HkXG5(const double&g93mR,const double&g85jV){return(g93mR>=-g85jV);}


bool diBqY::k9CDo(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

double gL6qW=g93mR,Time=0.0;Time=(E6Xnb-_Qa5G)/sguyX;
gL6qW+=_Qa5G*Time+0.5*sguyX*KNYa5(Time);return(gL6qW>-g85jV);}


bool diBqY::MBiIH(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){return(XXwMs(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::bySSx(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){

double gL6qW=g93mR,qPN_6=_Qa5G,Time=0.0,mcZn2=0.0;

mcZn2=SUYGr(KNYa5(qPN_6)-2.0*sguyX*(g85jV+gL6qW));
gL6qW=-g85jV;qPN_6=mcZn2;

Time=qPN_6/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW<=g85jV);}


bool diBqY::NpVpV(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

double gL6qW=g93mR,qPN_6=_Qa5G,Time=0.0;
gL6qW=-g85jV;qPN_6=E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW>g85jV);}


bool diBqY::UW66G(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

double gL6qW=g93mR,qPN_6=_Qa5G,Time=0.0;
Time=(E6Xnb-qPN_6)/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=E6Xnb*Time-0.5*sguyX*KNYa5(Time);return(gL6qW>g85jV);}


bool diBqY::iiDer(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&g85jV,const double&sguyX){
double gL6qW=g93mR,qPN_6=_Qa5G,Time=0.0;
Time=qPN_6/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);if(gL6qW>g85jV){gL6qW=
g85jV;}return(gL6qW<=hmD2W);}


bool diBqY::XXwMs(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
double gL6qW=g93mR,qPN_6=_Qa5G,Time=0.0;
Time=(E6Xnb-qPN_6)/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW<=hmD2W);}


bool diBqY::GKTD_(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
yrgzo=(E6Xnb-qPN_6)/sguyX;
eUZpd=E6Xnb/sguyX;I2uSa=qPN_6*yrgzo+0.5*sguyX*KNYa5(yrgzo);Wkp0J=0.5*KNYa5(E6Xnb
)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::JMglI(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&sguyX){

double gL6qW=g93mR,qPN_6=_Qa5G,Time=0.0;
gL6qW=g85jV;qPN_6=0.0;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;


gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW<=hmD2W);}


bool diBqY::EuH1E(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){

double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;

yrgzo=(E6Xnb-qPN_6)/sguyX;
eUZpd=E6Xnb/sguyX;I2uSa=qPN_6*yrgzo+0.5*sguyX*KNYa5(yrgzo);Wkp0J=0.5*KNYa5(E6Xnb
)/sguyX;
QSroW=g85jV-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
gL6qW=g85jV;qPN_6=0.0;

gL6qW=-gL6qW;aaprD=-aaprD;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW));
gL6qW=-gL6qW;aaprD=-aaprD;mcZn2=-mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/
6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::qrbZo(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){

double GzZSa=0.0
,VFLYs=0.0
,XVPX2=0.0
,MBb3Z=0.0
,NckOJ=0.0
,c_QBZ=0.0
,QSroW=0.0
,AG0zS=0.0
,T6g70=0.0
,CdoNz=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
GzZSa=(E6Xnb-qPN_6)/sguyX;
XVPX2=MBb3Z=NckOJ=E6Xnb/sguyX;c_QBZ=qPN_6*GzZSa+0.5*sguyX*KNYa5(GzZSa);AG0zS=0.5
*KNYa5(E6Xnb)/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-c_QBZ-AG0zS-T6g70-CdoNz;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;
gL6qW+=AG0zS;qPN_6=0.0;
Fb1Vh+=gL6qW*MBb3Z-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::itUTt(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){

double yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;

yrgzo=(E6Xnb-qPN_6)/sguyX;
eUZpd=E6Xnb/sguyX;I2uSa=qPN_6*yrgzo+0.5*sguyX*KNYa5(yrgzo);Wkp0J=0.5*KNYa5(E6Xnb
)/sguyX;
QSroW=g85jV-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
gL6qW=g85jV;qPN_6=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/(-E6Xnb);
Fb1Vh+=gL6qW*yrgzo-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::b_6Ox(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=qPN_6/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;


gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW<=hmD2W);}


bool diBqY::FC1BN(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
Time=qPN_6/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::bJE0B(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW>=-g85jV);}


bool diBqY::QCr9U(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Ntxm9=0.0
,zqaUH=0.0
,hHY1C=0.0
,VFLYs=0.0
,YtA02=0.0
,YhsYM=0.0
,h4z3k=0.0
,b60LR=0.0
,QSroW=0.0
,i8b0z=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
Ntxm9=zqaUH=hHY1C=YtA02=E6Xnb/sguyX;YhsYM=h4z3k=-0.5*KNYa5(E6Xnb)/sguyX;b60LR=
i8b0z=0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-YhsYM-h4z3k-b60LR-i8b0z;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*Ntxm9-sguyX*D_rgb(Ntxm9)/6.0;gL6qW+=YhsYM;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*zqaUH+0.5*qPN_6*KNYa5(zqaUH)+sguyX*D_rgb(zqaUH)/6.0;
gL6qW+=h4z3k;qPN_6=0.0;
Fb1Vh+=gL6qW*hHY1C+sguyX*D_rgb(hHY1C)/6.0;gL6qW+=b60LR;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*YtA02+0.5*qPN_6*KNYa5(YtA02)-sguyX*D_rgb(YtA02)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::iiKT8(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){
double MBb3Z=0.0
,P1gs6=0.0
,NckOJ=0.0
,GzZSa=0.0
,Z4ILN=0.0
,XVPX2=0.0
,T6g70=0.0
,UM2M2=0.0
,CdoNz=0.0
,c_QBZ=0.0
,BKRPw=0.0
,AG0zS=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
MBb3Z=NckOJ=GzZSa=XVPX2=E6Xnb/sguyX;T6g70=CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;c_QBZ=
AG0zS=0.5*KNYa5(E6Xnb)/sguyX;
UM2M2=-g85jV-gL6qW-T6g70-CdoNz;P1gs6=UM2M2/(-E6Xnb);
BKRPw=hmD2W-(-g85jV)-c_QBZ-AG0zS;Z4ILN=BKRPw/E6Xnb;

Fb1Vh+=gL6qW*MBb3Z-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*P1gs6+0.5*qPN_6*KNYa5(P1gs6);
gL6qW+=UM2M2;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;
gL6qW=-g85jV;qPN_6=0.0;

Fb1Vh+=gL6qW*GzZSa+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*Z4ILN+0.5*qPN_6*KNYa5(Z4ILN);
gL6qW+=BKRPw;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::d01Xk(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){
double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;

gL6qW=-gL6qW;
mcZn2=SUYGr(sguyX*(g85jV-gL6qW));
gL6qW=-gL6qW;mcZn2=-mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time-sguyX*D_rgb(Time)/6.0;gL6qW+=-0.5*sguyX*
KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;gL6qW=-g85jV;qPN_6
=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::W0NNw(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,mcZn2=0.0;
Time=qPN_6/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;

mcZn2=SUYGr(sguyX*(hmD2W-gL6qW));
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::Mey4c(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(n6URU(&WkhoZ,&P7bor,&Fgf1e,_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,
Z45C3,AUze_,sguyX));}


bool diBqY::kU_fE(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){
return(bJE0B(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::yDoCF(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,eUZpd=0.0
,I2uSa=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;
yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=0.5*KNYa5(E6Xnb)/sguyX;
gL6qW=-gL6qW;aaprD=-aaprD;I2uSa=-I2uSa;Wkp0J=-Wkp0J;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW-I2uSa-Wkp0J));
gL6qW=-gL6qW;aaprD=-aaprD;I2uSa=-I2uSa;Wkp0J=-Wkp0J;mcZn2=-mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/
6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;gL6qW+=qPN_6*Time+
0.5*sguyX*KNYa5(Time);qPN_6=0.0;
Fb1Vh+=gL6qW*yrgzo+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::qBJRY(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
gL6qW=-g85jV;qPN_6=0.0;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;


gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW>=hmD2W);}


bool diBqY::P42Y9(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){

double Time=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,mcZn2=0.0;

gL6qW=-gL6qW;
mcZn2=SUYGr(sguyX*(g85jV-gL6qW));
gL6qW=-gL6qW;mcZn2=-mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/
6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;gL6qW=-g85jV;qPN_6
=0.0;

mcZn2=SUYGr(sguyX*(hmD2W-gL6qW));
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::u1x2w(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
return(yDoCF(_Qa5G,E6Xnb,g93mR,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::nqosm(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,mcZn2=0.0;
mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*(hmD2W-gL6qW))/2.0);
Time=(mcZn2-qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::r7FrR(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){


double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=(E6Xnb-qPN_6)/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW<=g85jV);}


bool diBqY::cb5iY(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){

double Time=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W,mcZn2=0.0;
Time=(E6Xnb-qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;
Time=E6Xnb/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;

aaprD=-aaprD;gL6qW=-gL6qW;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW));
aaprD=-aaprD;gL6qW=-gL6qW;mcZn2=-mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/
6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::BXz3s(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){

double Time=0.0,mcZn2=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;

mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*(g85jV-gL6qW))/2.0);
Time=(mcZn2-qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW=g85jV;qPN_6=0.0;

aaprD=-aaprD;gL6qW=-gL6qW;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW));
aaprD=-aaprD;gL6qW=-gL6qW;mcZn2=-mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/
6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/
6.0;return(Fb1Vh<=AUze_);}


bool diBqY::LFzJL(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){return(W0NNw(_Qa5G,g93mR,
hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::v19ag(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=qPN_6/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;


gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW<=hmD2W);}


bool diBqY::NpywH(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W,mcZn2=0.0;
Time=qPN_6/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;

gL6qW=-gL6qW;aaprD=-aaprD;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW));
mcZn2=-mcZn2;gL6qW=-gL6qW;aaprD=-aaprD;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/
6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;return(Fb1Vh<=
AUze_);}


bool diBqY::pVKSx(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=(E6Xnb-qPN_6)/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;


gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;


gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW<=hmD2W);}


bool diBqY::wEMsY(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){
return(r7FrR(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::aMPmZ(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,eUZpd=0.0
,I2uSa=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*(hmD2W-gL6qW-I2uSa-Wkp0J))/2.0);
Time=(mcZn2-qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;
Fb1Vh+=gL6qW*yrgzo-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::C8FaE(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
Time=(E6Xnb-qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;
Time=E6Xnb/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;
yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/(-E6Xnb);

Fb1Vh+=gL6qW*yrgzo-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::WCknA(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
gL6qW=g85jV;qPN_6=0.0;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW<=hmD2W);}


bool diBqY::tuhux(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
return(aMPmZ(_Qa5G,E6Xnb,g93mR,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::gMHCB(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){

double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;

mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*(g85jV-gL6qW))/(0x15a3+1167-0x1a30));
Time=(mcZn2-qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW=g85jV;qPN_6=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/(-E6Xnb);
Fb1Vh+=gL6qW*yrgzo-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::kFJN5(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){
return(bJE0B(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::HQGXz(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,eUZpd=0.0
,I2uSa=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
Fb1Vh+=gL6qW*yrgzo-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
gL6qW+=Wkp0J;qPN_6=0.0;
mcZn2=SUYGr(sguyX*(hmD2W-gL6qW));
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+sguyX*D_rgb(Time)/6.0;gL6qW+=0.5*sguyX*KNYa5(
Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::Sxb7R(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
gL6qW=-g85jV;qPN_6=0.0;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;


gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW>=hmD2W);}


bool diBqY::vumc9(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){
double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=-g85jV-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/(-E6Xnb);
Fb1Vh+=gL6qW*yrgzo-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
gL6qW=-g85jV;qPN_6=0.0;

mcZn2=SUYGr(sguyX*(hmD2W-gL6qW));
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+sguyX*D_rgb(Time)/6.0;gL6qW+=0.5*sguyX*KNYa5(
Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::Xxly5(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double MBb3Z=0.0
,P1gs6=0.0
,NckOJ=0.0
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
Fb1Vh+=gL6qW*MBb3Z-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*P1gs6+0.5*qPN_6*KNYa5(P1gs6);
gL6qW+=UM2M2;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;
gL6qW+=CdoNz;qPN_6=0.0;
Fb1Vh+=gL6qW*GzZSa+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::BkWQF(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
Time=qPN_6/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/(-E6Xnb);
Fb1Vh+=gL6qW*yrgzo-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh<=AUze_);}


bool diBqY::OanKO(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){
return(r7FrR(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}







bool diBqY::v27dc(const double&_Qa5G){return(FlP2m(_Qa5G));}


bool diBqY::cSAKs(const double&_Qa5G,const double&E6Xnb){
return(W5C39(_Qa5G,E6Xnb));}


bool diBqY::G3uRc(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){
return(dS4fr(_Qa5G,g93mR,g85jV,sguyX));}


bool diBqY::F1ksV(const double&g93mR,const double&g85jV){
return(HkXG5(g93mR,g85jV));}


bool diBqY::OMNF9(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(k9CDo(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::CLbMG(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){return(XXwMs(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::KI7Wo(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){

return(bySSx(_Qa5G,g93mR,g85jV,sguyX));}


bool diBqY::NRpBq(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(NpVpV(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::jXpTt(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(UW66G(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::KuHU1(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=qPN_6/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return((i5gUI(gL6qW)==
i5gUI(hmD2W)));}


bool diBqY::JHVC_(const double&_Qa5G,const double&g93mR,const double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=qPN_6/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW<
(0xf70+2882-0x1ab2));}


bool diBqY::KqcAy(const double&_Qa5G){
return(FlP2m(_Qa5G));}


bool diBqY::cvP8V(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&g85jV,const double&sguyX){
return(iiDer(_Qa5G,g93mR,hmD2W,g85jV,sguyX));}


bool diBqY::AI3js(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){

return(XXwMs(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::Lfptr(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){

double yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
yrgzo=(E6Xnb-qPN_6)/sguyX;
eUZpd=E6Xnb/sguyX;I2uSa=qPN_6*yrgzo+0.5*sguyX*KNYa5(yrgzo);Wkp0J=0.5*KNYa5(E6Xnb
)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh<=AUze_+O_Jby+a9b7d*fabs(Z45C3-AUze_));}


bool diBqY::DFR1Z(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){

double Time=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,mcZn2=0.0;
mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*(hmD2W-gL6qW))/2.0);
Time=(mcZn2-qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
return(Fb1Vh<=AUze_+O_Jby+a9b7d*fabs(Z45C3-AUze_));}


bool diBqY::v7km6(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){

double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=(E6Xnb+qPN_6)/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW<=hmD2W);}


bool diBqY::uEHKl(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){

double Time=0.0,mcZn2=0.0,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;
qPN_6=-qPN_6;gL6qW=-gL6qW;aaprD=-aaprD;
mcZn2=SUYGr((KNYa5(qPN_6)+(aaprD-gL6qW)*2.0*sguyX)/2.0);
qPN_6=-qPN_6;gL6qW=-gL6qW;aaprD=-aaprD;mcZn2=-mcZn2;
Time=(qPN_6-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/
6.0;return(Fb1Vh>=AUze_+O_Jby+a9b7d*fabs(Z45C3-AUze_));}


bool diBqY::D722S(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){

double yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
yrgzo=(E6Xnb+qPN_6)/sguyX;I2uSa=qPN_6*yrgzo-0.5*sguyX*KNYa5(yrgzo);
eUZpd=E6Xnb/sguyX;Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/(-E6Xnb);
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh>=AUze_+O_Jby+a9b7d*fabs(Z45C3-AUze_));}


bool diBqY::MwJGz(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=(-qPN_6)/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW>hmD2W);
}







bool diBqY::kELK9(const double&_Qa5G){return(FlP2m(_Qa5G));}


bool diBqY::J4hnf(const double&_Qa5G,const double&E6Xnb){
return(W5C39(_Qa5G,E6Xnb));}


bool diBqY::B2SR3(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){
return(dS4fr(_Qa5G,g93mR,g85jV,sguyX));}


bool diBqY::EkfQK(const double&g93mR,const double&g85jV){
return(HkXG5(g93mR,g85jV));}


bool diBqY::RYtZY(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(k9CDo(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::lLDLn(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){return(XXwMs(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::ha93u(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){

return(bySSx(_Qa5G,g93mR,g85jV,sguyX));}


bool diBqY::DV5xt(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(NpVpV(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::dLV3B(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(UW66G(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::bHIVJ(const double&_Qa5G,const double&g93mR,const double&sguyX){
return(JHVC_(_Qa5G,g93mR,sguyX));}


bool diBqY::NuTgm(const double&_Qa5G){
return(FlP2m(_Qa5G));}


bool diBqY::TVphj(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&g85jV,const double&sguyX){
return(iiDer(_Qa5G,g93mR,hmD2W,g85jV,sguyX));}


bool diBqY::ABNtV(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
return(b_6Ox(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::OS6qO(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
return(FC1BN(_Qa5G,E6Xnb,g93mR,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::xJQ_3(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&sguyX){
double Time=0.0,gL6qW=g93mR,qPN_6=_Qa5G;
Time=(E6Xnb+qPN_6)/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=0.0;return(gL6qW
>=(0x11af+3506-0x1f61));}


bool diBqY::wfj20(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
return(QCr9U(_Qa5G,E6Xnb,g93mR,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::KDxXd(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(n6URU(&_WRKy,&ni1W7,&tlbcQ,-_Qa5G,E6Xnb,-g93mR,g85jV,-hmD2W
,-Z45C3,-AUze_,sguyX));}


bool diBqY::dMvfh(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
yrgzo=(E6Xnb+qPN_6)/sguyX;I2uSa=qPN_6*yrgzo-0.5*sguyX*KNYa5(yrgzo);
eUZpd=E6Xnb/sguyX;Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=(0x42b+8126-0x23e9)-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/(-E6Xnb);
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
gL6qW=0.0;qPN_6=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-(0x1358+2933-0x1ecd)-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::RTv7w(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(n6URU(&eHpM4,&wEEQc,&FW1Ej,-_Qa5G,E6Xnb,-g93mR,g85jV,-hmD2W
,-Z45C3,-AUze_,sguyX));}


bool diBqY::KOi7e(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
qPN_6=-qPN_6;gL6qW=-gL6qW;
mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*((0x539+328-0x681)-gL6qW))/2.0);
qPN_6=-qPN_6;gL6qW=-gL6qW;mcZn2=-mcZn2;
Time=(-mcZn2+qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/
6.0;gL6qW=0.0;qPN_6=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::UPA1D(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(KDxXd(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::fx4dT(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){
return(W0NNw(_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::JMEcZ(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&sguyX){
return(xJQ_3(_Qa5G,E6Xnb,g93mR,sguyX));}


bool diBqY::fJ4Eo(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
return(QCr9U(_Qa5G,E6Xnb,g93mR,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::WpD6G(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
return(yDoCF(_Qa5G,E6Xnb,g93mR,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::tdH3Q(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(A68S7(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::itEEY(const double&E6Xnb,const double&hmD2W,const double&sguyX){
double GzZSa=0.0
,XVPX2=0.0
,c_QBZ=0.0
,AG0zS=0.0
,x2IqQ=0.0;

GzZSa=XVPX2=E6Xnb/sguyX;c_QBZ=AG0zS=0.5*KNYa5(E6Xnb)/sguyX;x2IqQ=hmD2W-c_QBZ-
AG0zS;return(x2IqQ>(0x1a18+567-0x1c4f));}


bool diBqY::hhiih(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){return(
KOi7e(_Qa5G,E6Xnb,g93mR,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::psyRE(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(A68S7(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::I5MSf(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(KDxXd(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::AgQlv(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;

qPN_6=-qPN_6;gL6qW=-gL6qW;
mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*((0x9e8+3501-0x1795)-gL6qW))/2.0);
qPN_6=-qPN_6;gL6qW=-gL6qW;mcZn2=-mcZn2;
Time=(-mcZn2+qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/
6.0;gL6qW=0.0;qPN_6=0.0;

mcZn2=SUYGr((hmD2W-(0x6c5+4136-0x16ed))*sguyX);
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::A68S7(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(n6URU(&nNN9F,&v2SZG,&AkYML,-_Qa5G,E6Xnb,-g93mR,g85jV,-hmD2W
,-Z45C3,-AUze_,sguyX));}


bool diBqY::qs90k(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(n6URU(&tIQ8E,&pqZE_,&_aosC,_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,
Z45C3,AUze_,sguyX));}


bool diBqY::K3p0Y(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){

double gL6qW=g93mR,qPN_6=_Qa5G,Time=0.0;
Time=(E6Xnb+qPN_6)/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW>hmD2W);}


bool diBqY::Ba8IA(const double&E6Xnb,const double&hmD2W,const double&sguyX){

double c_QBZ=0.0
,AG0zS=0.0;

c_QBZ=AG0zS=0.5*KNYa5(E6Xnb)/sguyX;return((hmD2W-c_QBZ-AG0zS)>=
(0x22fd+902-0x2683));}


bool diBqY::x9Zgn(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){

double MBb3Z=0.0
,P1gs6=0.0
,NckOJ=0.0
,GzZSa=0.0
,XVPX2=0.0
,T6g70=0.0
,UM2M2=0.0
,CdoNz=0.0
,c_QBZ=0.0
,AG0zS=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
MBb3Z=(E6Xnb+qPN_6)/sguyX;T6g70=qPN_6*MBb3Z-0.5*sguyX*KNYa5(MBb3Z);
NckOJ=GzZSa=XVPX2=E6Xnb/sguyX;CdoNz=-0.5*KNYa5(E6Xnb)/sguyX;c_QBZ=AG0zS=0.5*
KNYa5(E6Xnb)/sguyX;
UM2M2=hmD2W-gL6qW-T6g70-CdoNz-c_QBZ-AG0zS;P1gs6=UM2M2/(-E6Xnb);
Fb1Vh+=gL6qW*MBb3Z+0.5*qPN_6*KNYa5(MBb3Z)-sguyX*D_rgb(MBb3Z)/6.0;gL6qW+=T6g70;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*P1gs6+0.5*qPN_6*KNYa5(P1gs6);
gL6qW+=UM2M2;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*NckOJ+0.5*qPN_6*KNYa5(NckOJ)+sguyX*D_rgb(NckOJ)/6.0;
gL6qW+=CdoNz;qPN_6=0.0;
Fb1Vh+=gL6qW*GzZSa+0.5*qPN_6*KNYa5(GzZSa)+sguyX*D_rgb(GzZSa)/6.0;gL6qW+=c_QBZ;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*XVPX2+0.5*qPN_6*KNYa5(XVPX2)-sguyX*D_rgb(XVPX2)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::l9t2A(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(w10As(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::VN7tH(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;

yrgzo=(E6Xnb+qPN_6)/sguyX;I2uSa=qPN_6*yrgzo-0.5*sguyX*KNYa5(yrgzo);
eUZpd=E6Xnb/sguyX;Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=(0x146f+1091-0x18b2)-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/(-E6Xnb);
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
gL6qW=0.0;qPN_6=0.0;

mcZn2=SUYGr((hmD2W-(0x6f6+3196-0x1372))*sguyX);
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::w10As(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(n6URU(&WP2Bw,&O_tX1,&MWTWT,-_Qa5G,E6Xnb,-g93mR,g85jV,-hmD2W
,-Z45C3,-AUze_,sguyX));}


bool diBqY::vhEYc(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&sguyX){return(xJQ_3(_Qa5G,E6Xnb,g93mR,sguyX));}


bool diBqY::p5v2y(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){

double Time=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
Time=(E6Xnb+qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;
Time=E6Xnb/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=0.0;

mcZn2=SUYGr((hmD2W-gL6qW)*sguyX);
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::GruKb(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(A68S7(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::Fmqc5(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX){
return(MwJGz(_Qa5G,g93mR,hmD2W,sguyX));}


bool diBqY::CnP0q(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
double Time=0.0
,gL6qW=g93mR,qPN_6=_Qa5G;
Time=-qPN_6/sguyX;gL6qW+=0.5*qPN_6*Time;qPN_6=0.0;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;


gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW>hmD2W);}


bool diBqY::RkfAv(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
Time=-qPN_6/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=-QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::U5UeM(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,mcZn2=0.0;
Time=-qPN_6/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=0.0;

mcZn2=SUYGr(sguyX*(gL6qW-hmD2W));
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;return(Fb1Vh>=
AUze_);}


bool diBqY::mtrbZ(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(n6URU(&tIQ8E,&pqZE_,&_aosC,-_Qa5G,E6Xnb,-g93mR,g85jV,-hmD2W
,-Z45C3,-AUze_,sguyX));}


bool diBqY::rpBOH(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
double Time=0.0
,gL6qW=g93mR,qPN_6=_Qa5G;
Time=-qPN_6/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=0.0;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;


gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW>hmD2W);}


bool diBqY::xtJ6k(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
double Time=0.0
,gL6qW=g93mR,qPN_6=_Qa5G;
Time=(E6Xnb+qPN_6)/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=0.0;


gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;


gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW>hmD2W);}


bool diBqY::aOLsR(const double&E6Xnb,const double&hmD2W,const double&sguyX){


double c_QBZ=0.0;
c_QBZ=KNYa5(E6Xnb)/sguyX;return((hmD2W-c_QBZ)>(0x596+2902-0x10ec));}


bool diBqY::liWft(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,eUZpd=0.0
,I2uSa=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=0.5*KNYa5(E6Xnb)/sguyX;
mcZn2=SUYGr((KNYa5(qPN_6)+2.0*KNYa5(E6Xnb)+2.0*sguyX*(gL6qW-hmD2W))/2.0);
Time=(qPN_6+mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-mcZn2;
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=0.0;
Fb1Vh+=gL6qW*yrgzo+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=+E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::CaKTg(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(A68S7(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::Zty4X(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&sguyX){return(xJQ_3(_Qa5G,E6Xnb,g93mR,sguyX));}


bool diBqY::PQg1A(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
Time=(E6Xnb+qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;
Time=E6Xnb/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=0.0;
yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh>=AUze_);}


bool diBqY::ILzvk(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(KDxXd(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}







bool diBqY::M4RBQ(const double&_Qa5G){return(FlP2m(_Qa5G));}


bool diBqY::gpW03(const double&_Qa5G,const double&E6Xnb){
return(W5C39(_Qa5G,E6Xnb));}


bool diBqY::QwoBP(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){
return(dS4fr(_Qa5G,g93mR,g85jV,sguyX));}


bool diBqY::FzDX_(const double&g93mR,const double&g85jV){
return(HkXG5(g93mR,g85jV));}


bool diBqY::QxCTg(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(k9CDo(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::npy1a(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){return(XXwMs(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::n1jSJ(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){

return(bySSx(_Qa5G,g93mR,g85jV,sguyX));}


bool diBqY::aTBHx(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(NpVpV(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::ZJVcc(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(UW66G(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::LuWav(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&sguyX){
double gL6qW=g93mR+0.5*KNYa5(_Qa5G)/sguyX;return((gL6qW<0.0)&&(hmD2W>0.0));}


bool diBqY::Onldg(const int p6eP1){return((p6eP1==diBqY::tmFJN)||(p6eP1==diBqY::
bC_iO)||(p6eP1==diBqY::YPfXT)||(p6eP1==diBqY::IflJS)||(p6eP1==diBqY::OyKu3)||(
p6eP1==diBqY::BEy1V));}


bool diBqY::lnU6U(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){return(XXwMs(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::Wd_zo(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){return(b_6Ox(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::Bp1jS(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
Time=qPN_6/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh+O_Jby+a9b7d*fabs(Z45C3-AUze_)<=AUze_);}




bool diBqY::zs6Zz(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX,double*ZCUBa,double*SEtWr){bool gQP2R=false;double fDo_w=0.0,Y6GNL=
0.0,OwiV9=0.0,Q5mXg=0.0,e7NjR=JCQ6Q,UAbXk=JCQ6Q;if(!(AAcif(BEy1V,_Qa5G,E6Xnb,
g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX))){if((ZCUBa!=NULL)&&(SEtWr!=NULL)){*ZCUBa=
JCQ6Q;*SEtWr=JCQ6Q;}return(false);}e7NjR=n9iwH(&WkhoZ,&P7bor,&Fgf1e,&IVmDp,Z45C3
,AUze_,g93mR,g85jV,hmD2W,_Qa5G,E6Xnb,sguyX,true,MUsk2,0.0,&gQP2R,&fDo_w,BEy1V);
if(!gQP2R){UAbXk=n9iwH(&WkhoZ,&P7bor,&Fgf1e,&IVmDp,Z45C3,AUze_,g93mR,g85jV,hmD2W
,_Qa5G,E6Xnb,sguyX,false,MUsk2,0.0,&gQP2R,&Y6GNL,BEy1V);}if(!gQP2R){OwiV9=WkhoZ(
0.5*(fDo_w+Y6GNL),_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);Q5mXg=WkhoZ(0.5*
fDo_w,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if((OwiV9>=O_Jby+a9b7d*fabs(
Z45C3-AUze_))||(Q5mXg>=O_Jby+a9b7d*fabs(Z45C3-AUze_))){if((ZCUBa!=NULL)&&(SEtWr
!=NULL)){*ZCUBa=e7NjR;if(WkhoZ(0.5*Y6GNL,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
E6Xnb)<0.0){*SEtWr=UAbXk;}else{*SEtWr=JCQ6Q;}}return(true);}else{if((ZCUBa!=NULL
)&&(SEtWr!=NULL)){*ZCUBa=JCQ6Q;*SEtWr=JCQ6Q;}return(false);}}else{if((ZCUBa!=
NULL)&&(SEtWr!=NULL)){*ZCUBa=JCQ6Q;*SEtWr=JCQ6Q;}return(false);}}


bool diBqY::bdqe9(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){
double Time=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,mcZn2=0.0;
Time=qPN_6/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=0.0;

mcZn2=SUYGr(sguyX*(hmD2W-gL6qW));
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;gL6qW=hmD2W;qPN_6=
0.0;return(Fb1Vh+O_Jby+a9b7d*fabs(Z45C3-AUze_)<=AUze_);}




bool diBqY::QGHcW(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX,double*ZCUBa,double*SEtWr){bool gQP2R=false,Vfs7k=false;double 
fDo_w=0.0,Y6GNL=0.0,OwiV9=0.0,Q5mXg=0.0,yKN4X=0.0,u7ROa=0.0,edCBt=0.0,jWXjz=0.0,
XTgJr=0.0,e7NjR=JCQ6Q,UAbXk=JCQ6Q;if(!(AAcif(BEy1V,_Qa5G,E6Xnb,g93mR,g85jV,hmD2W
,Z45C3,AUze_,sguyX))){if((ZCUBa!=NULL)&&(SEtWr!=NULL)){*ZCUBa=JCQ6Q;*SEtWr=JCQ6Q
;}return(false);}e7NjR=n9iwH(&WkhoZ,&P7bor,&Fgf1e,&IVmDp,Z45C3,AUze_,g93mR,g85jV
,hmD2W,_Qa5G,E6Xnb,sguyX,true,MUsk2,0.0,&gQP2R,&fDo_w,BEy1V);if(gQP2R){if((ZCUBa
!=NULL)&&(SEtWr!=NULL)){*ZCUBa=JCQ6Q;*SEtWr=JCQ6Q;}return(false);}
Vfs7k=XKbpN(_Qa5G,E6Xnb,g93mR,hmD2W,Z45C3,AUze_,sguyX,&XTgJr);if(Vfs7k){

n9iwH(&tIQ8E,&pqZE_,&_aosC,&VuRcE,Z45C3,AUze_,g93mR,g85jV,hmD2W,_Qa5G,E6Xnb,
sguyX,true,MUsk2,0.0,&gQP2R,&Y6GNL,OyKu3);u7ROa=0.5*(fDo_w+Y6GNL);if(u7ROa>XTgJr
){OwiV9=WkhoZ(u7ROa,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);}else{OwiV9=tIQ8E
(u7ROa,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);}if(OwiV9<0.0){if((ZCUBa!=NULL
)&&(SEtWr!=NULL)){*ZCUBa=JCQ6Q;*SEtWr=JCQ6Q;}return(false);}}
if(Vfs7k){UAbXk=n9iwH(&WkhoZ,&P7bor,&Fgf1e,&IVmDp,Z45C3,AUze_,g93mR,g85jV,hmD2W,
_Qa5G,E6Xnb,sguyX,false,MUsk2,0.0,&gQP2R,&Y6GNL,BEy1V);}else{UAbXk=n9iwH(&tIQ8E,
&pqZE_,&_aosC,&VuRcE,Z45C3,AUze_,g93mR,g85jV,hmD2W,_Qa5G,E6Xnb,sguyX,true,MUsk2,
0.0,&gQP2R,&Y6GNL,OyKu3);}if(!gQP2R){u7ROa=0.5*(fDo_w+Y6GNL);edCBt=0.5*fDo_w;if(
u7ROa>XTgJr){OwiV9=WkhoZ(u7ROa,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);}else{
OwiV9=tIQ8E(u7ROa,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);}if(edCBt>XTgJr){
Q5mXg=WkhoZ(edCBt,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);}else{Q5mXg=tIQ8E(
edCBt,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);}if((OwiV9>=O_Jby+a9b7d*fabs(
Z45C3-AUze_))||(Q5mXg>=O_Jby+a9b7d*fabs(Z45C3-AUze_))){if((ZCUBa!=NULL)&&(SEtWr
!=NULL)){*ZCUBa=e7NjR;jWXjz=0.5*Y6GNL;if(jWXjz>XTgJr){yKN4X=WkhoZ(jWXjz,_Qa5G,
g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);}else{yKN4X=tIQ8E(jWXjz,_Qa5G,g93mR,hmD2W,
Z45C3,AUze_,sguyX,E6Xnb);}if(yKN4X<0.0){*SEtWr=UAbXk;}else{*SEtWr=JCQ6Q;}}return
(true);}else{if((ZCUBa!=NULL)&&(SEtWr!=NULL)){*ZCUBa=JCQ6Q;*SEtWr=JCQ6Q;}return(
false);}}else{if((ZCUBa!=NULL)&&(SEtWr!=NULL)){*ZCUBa=JCQ6Q;*SEtWr=JCQ6Q;}return
(false);}}


bool diBqY::qq5iI(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(qs90k(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::q65de(const double&_Qa5G,const double&g93mR,const double&hmD2W,const
 double&Z45C3,const double&AUze_,const double&sguyX){return(bdqe9(_Qa5G,g93mR,
hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::XKbpN(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,double*
dhSEu){double Du4TV=Z45C3,KnFkY=g93mR,Time=0.0;*dhSEu=SUYGr(0.5*(KNYa5(_Qa5G)+
2.0*KNYa5(E6Xnb)+2.0*sguyX*(g93mR-hmD2W)));Time=(_Qa5G-(*dhSEu))/sguyX;if(Time<
0.0){Time=0.0;}Du4TV+=KnFkY*Time+0.5*_Qa5G*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
KnFkY+=_Qa5G*Time-0.5*sguyX*KNYa5(Time);Time=(E6Xnb-(*dhSEu))/sguyX;if(Time<0.0)
{Time=0.0;}Du4TV+=KnFkY*Time+0.5*(*dhSEu)*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;
KnFkY+=(*dhSEu)*Time+0.5*sguyX*KNYa5(Time);Time=E6Xnb/sguyX;Du4TV+=KnFkY*Time+
0.5*E6Xnb*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;return(Du4TV<=AUze_);}




bool diBqY::gS8__(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX,double*ZCUBa,double*SEtWr){bool gQP2R=false;double fDo_w=0.0,Y6GNL=
0.0,OwiV9=0.0,Q5mXg=0.0,e7NjR=JCQ6Q,UAbXk=JCQ6Q;if(!(AAcif(OyKu3,_Qa5G,E6Xnb,
g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX))){if((ZCUBa!=NULL)&&(SEtWr!=NULL)){*ZCUBa=
JCQ6Q;*SEtWr=JCQ6Q;}return(false);}e7NjR=n9iwH(&tIQ8E,&pqZE_,&_aosC,&VuRcE,Z45C3
,AUze_,g93mR,g85jV,hmD2W,_Qa5G,E6Xnb,sguyX,true,MUsk2,0.0,&gQP2R,&fDo_w,OyKu3);
if(!gQP2R){UAbXk=n9iwH(&tIQ8E,&pqZE_,&_aosC,&VuRcE,Z45C3,AUze_,g93mR,g85jV,hmD2W
,_Qa5G,E6Xnb,sguyX,false,MUsk2,0.0,&gQP2R,&Y6GNL,OyKu3);}if(!gQP2R){OwiV9=tIQ8E(
0.5*(fDo_w+Y6GNL),_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);Q5mXg=tIQ8E(0.5*
fDo_w,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,E6Xnb);if((OwiV9>=O_Jby+a9b7d*fabs(
Z45C3-AUze_))||(Q5mXg>=O_Jby+a9b7d*fabs(Z45C3-AUze_))){if((ZCUBa!=NULL)&&(SEtWr
!=NULL)){*ZCUBa=e7NjR;if(tIQ8E(0.5*Y6GNL,_Qa5G,g93mR,hmD2W,Z45C3,AUze_,sguyX,
E6Xnb)<0.0){*SEtWr=UAbXk;}else{*SEtWr=JCQ6Q;}}return(true);}else{if((ZCUBa!=NULL
)&&(SEtWr!=NULL)){*ZCUBa=JCQ6Q;*SEtWr=JCQ6Q;}return(false);}}else{if((ZCUBa!=
NULL)&&(SEtWr!=NULL)){*ZCUBa=JCQ6Q;*SEtWr=JCQ6Q;}return(false);}}







bool diBqY::vwVqL(const double&_Qa5G){return(FlP2m(_Qa5G));}


bool diBqY::l7omV(const double&_Qa5G,const double&E6Xnb){
return(W5C39(_Qa5G,E6Xnb));}


bool diBqY::QNJM_(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){
return(dS4fr(_Qa5G,g93mR,g85jV,sguyX));}


bool diBqY::FtH2C(const double&g93mR,const double&g85jV){
return(HkXG5(g93mR,g85jV));}


bool diBqY::mE5Hh(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(k9CDo(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::uZESL(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){return(XXwMs(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::NdjPI(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&sguyX){

return(bySSx(_Qa5G,g93mR,g85jV,sguyX));}


bool diBqY::eRkwV(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(NpVpV(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::Ua0IY(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){

return(UW66G(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::_lqQb(const double&hmD2W){return(hmD2W<=0.0);}


bool diBqY::L4uIC(const double&_Qa5G){

return(FlP2m(_Qa5G));}


bool diBqY::V66ya(const int&zaG8H,const int&cTO1U){return((zaG8H==diBqY::BEy1V)
||(zaG8H==diBqY::OyKu3)||(cTO1U==diBqY::BEy1V)||(cTO1U==diBqY::OyKu3));}


bool diBqY::UBpeN(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX){
return(r7FrR(_Qa5G,E6Xnb,g93mR,g85jV,sguyX));}


bool diBqY::oL4md(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&sguyX){return(JMglI(_Qa5G,E6Xnb,
g93mR,g85jV,hmD2W,sguyX));}


bool diBqY::a0Alv(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){
return(EuH1E(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::AUj_t(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(n6URU(&WP2Bw,&O_tX1,&MWTWT,_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,
Z45C3,AUze_,sguyX));}


bool diBqY::JidWA(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){
return(itUTt(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::HqfuH(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(n6URU(&eHpM4,&wEEQc,&FW1Ej,_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,
Z45C3,AUze_,sguyX));}


bool diBqY::gChmN(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&hmD2W,const double&sguyX){
return(pVKSx(_Qa5G,E6Xnb,g93mR,hmD2W,sguyX));}


bool diBqY::yLL7T(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){return(n6URU(&_WRKy,&ni1W7,&tlbcQ,_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,
Z45C3,AUze_,sguyX));}


bool diBqY::ip9ju(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&sguyX){
return(WCknA(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,sguyX));}


bool diBqY::awseK(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX){
return(BXz3s(_Qa5G,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}


bool diBqY::wdvzU(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX){
return(gMHCB(_Qa5G,E6Xnb,g93mR,g85jV,hmD2W,Z45C3,AUze_,sguyX));}







bool diBqY::NmEnT(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&sguyX,const bool&a0l41){

double Time=0.0
,gL6qW=g93mR,qPN_6=_Qa5G;
gL6qW=g85jV;qPN_6=0.0;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;


gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW+XXEod*((a0l41)?(1.0):(-1.0)
)+oxhrD*((a0l41)?(1.0):(-1.0))*fabs(gL6qW)<=hmD2W);}


bool diBqY::ZkBn3(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX,const bool&a0l41){

double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W;

yrgzo=(E6Xnb-qPN_6)/sguyX;
eUZpd=E6Xnb/sguyX;I2uSa=qPN_6*yrgzo+0.5*sguyX*KNYa5(yrgzo);Wkp0J=0.5*KNYa5(E6Xnb
)/sguyX;
QSroW=g85jV-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
gL6qW=g85jV;qPN_6=0.0;

gL6qW=-gL6qW;aaprD=-aaprD;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW));
gL6qW=-gL6qW;aaprD=-aaprD;mcZn2=-mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/
6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;


Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/6.0;return(Fb1Vh+XXEod
*((a0l41)?(1.0):(-1.0))+oxhrD*((a0l41)?(1.0):(-1.0))*fabs(Fb1Vh-AUze_)<=AUze_);}


bool diBqY::AD6q3(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX,const bool&a0l41){

double yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;

yrgzo=(E6Xnb-qPN_6)/sguyX;
eUZpd=E6Xnb/sguyX;I2uSa=qPN_6*yrgzo+0.5*sguyX*KNYa5(yrgzo);Wkp0J=0.5*KNYa5(E6Xnb
)/sguyX;
QSroW=g85jV-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/E6Xnb;
Fb1Vh+=gL6qW*yrgzo+0.5*qPN_6*KNYa5(yrgzo)+sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;
qPN_6=E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)-sguyX*D_rgb(eUZpd)/6.0;
gL6qW=g85jV;qPN_6=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/(-E6Xnb);
Fb1Vh+=gL6qW*yrgzo-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh+XXEod*((a0l41)?(1.0):(-1.0))+oxhrD*((a0l41)?(1.0):(-1.0))*fabs(
Fb1Vh-AUze_)<=AUze_);}


bool diBqY::HUFN1(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&sguyX,const bool&a0l41){


double gL6qW=g93mR,qPN_6=_Qa5G,Time=0.0;
Time=(E6Xnb-qPN_6)/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);return(gL6qW+XXEod*((
a0l41)?(1.0):(-1.0))+oxhrD*((a0l41)?(1.0):(-1.0))*fabs(gL6qW)<=g85jV);}


bool diBqY::GKvMJ(const double&_Qa5G,const double&g93mR,const double&g85jV,const
 double&hmD2W,const double&Z45C3,const double&AUze_,const double&sguyX,const 
bool&a0l41){

double Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G,aaprD=hmD2W,Time=0.0,mcZn2=0.0;

mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*(g85jV-gL6qW))/2.0);
Time=(mcZn2-qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW=g85jV;qPN_6=0.0;

aaprD=-aaprD;gL6qW=-gL6qW;
mcZn2=SUYGr(sguyX*(aaprD-gL6qW));
aaprD=-aaprD;gL6qW=-gL6qW;mcZn2=-mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/
6.0;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=(-mcZn2)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(Time)/
6.0;return(Fb1Vh+XXEod*((a0l41)?(1.0):(-1.0))+oxhrD*((a0l41)?(1.0):(-1.0))*fabs(
Fb1Vh-AUze_)<=AUze_);}


bool diBqY::M3TyU(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&sguyX,const bool&a0l41){
double gL6qW=g93mR,qPN_6=_Qa5G,Time=0.0;
gL6qW=g85jV;qPN_6=0.0;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time-0.5*sguyX*KNYa5(Time);qPN_6=-E6Xnb;
Time=E6Xnb/sguyX;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);return(gL6qW+XXEod*((
a0l41)?(1.0):(-1.0))+oxhrD*((a0l41)?(1.0):(-1.0))*fabs(gL6qW)<=hmD2W);}


bool diBqY::IuCbq(const double&_Qa5G,const double&E6Xnb,const double&g93mR,const
 double&g85jV,const double&hmD2W,const double&Z45C3,const double&AUze_,const 
double&sguyX,const bool&a0l41){

double Time=0.0
,yrgzo=0.0
,VFLYs=0.0
,eUZpd=0.0
,I2uSa=0.0
,QSroW=0.0
,Wkp0J=0.0
,mcZn2=0.0
,Fb1Vh=Z45C3,gL6qW=g93mR,qPN_6=_Qa5G;
mcZn2=SUYGr((KNYa5(qPN_6)+2.0*sguyX*(g85jV-gL6qW))/(0xe74+397-0xfff));
Time=(mcZn2-qPN_6)/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)+sguyX*D_rgb(
Time)/6.0;gL6qW+=qPN_6*Time+0.5*sguyX*KNYa5(Time);qPN_6=mcZn2;
Time=mcZn2/sguyX;Fb1Vh+=gL6qW*Time+0.5*qPN_6*KNYa5(Time)-sguyX*D_rgb(Time)/6.0;
gL6qW=g85jV;qPN_6=0.0;

yrgzo=eUZpd=E6Xnb/sguyX;I2uSa=Wkp0J=-0.5*KNYa5(E6Xnb)/sguyX;
QSroW=hmD2W-gL6qW-I2uSa-Wkp0J;VFLYs=QSroW/(-E6Xnb);
Fb1Vh+=gL6qW*yrgzo-sguyX*D_rgb(yrgzo)/6.0;gL6qW+=I2uSa;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*VFLYs+0.5*qPN_6*KNYa5(VFLYs);
gL6qW+=QSroW;qPN_6=-E6Xnb;
Fb1Vh+=gL6qW*eUZpd+0.5*qPN_6*KNYa5(eUZpd)+sguyX*D_rgb(eUZpd)/6.0;
return(Fb1Vh+XXEod*((a0l41)?(1.0):(-1.0))+oxhrD*((a0l41)?(1.0):(-1.0))*fabs(
Fb1Vh-AUze_)<=AUze_);}
