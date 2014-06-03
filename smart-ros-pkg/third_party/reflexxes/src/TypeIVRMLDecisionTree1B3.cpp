
































#include <TypeIVRMLDecisionTree1B3.h>
#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLStep1RootFunctions.h>
#include <TypeIVRMLStep1Profiles.h>
#include <RMLPositionFlags.h>


bool diBqY::z8RFA(const double&Z45C3,const double&g93mR,const double&_Qa5G,const
 double&sguyX,const double&E6Xnb,const double&g85jV,const double&AUze_,const 
double&hmD2W,const int&Pn4_H,const int&hS63U,fT_oe*MqVoX,double*jxnfV,double*
Ldd82){bool gQP2R=false;double MzfY6=0.0,BkjIW=Z45C3,APBny=g93mR,N2UxV=_Qa5G,
YOMpW=AUze_,sabLl=hmD2W,EEKof=0.0,ZCUBa=JCQ6Q,SEtWr=JCQ6Q;*Ldd82=JCQ6Q;
if(M4RBQ(N2UxV)){goto E_xXQ;}
else{
tvfxt(&BkjIW,&APBny,&N2UxV,&YOMpW,&sabLl);goto E_xXQ;}E_xXQ:
if(gpW03(N2UxV,E6Xnb)){goto WY7Va;}
else{
x96T7(&MzfY6,&BkjIW,&APBny,&N2UxV,E6Xnb,sguyX);goto WY7Va;}WY7Va:
if(QwoBP(N2UxV,APBny,g85jV,sguyX)){goto oGun2;}
else{
AFNhq(&MzfY6,&BkjIW,&APBny,&N2UxV,sguyX);
tvfxt(&BkjIW,&APBny,&N2UxV,&YOMpW,&sabLl);goto nVEbT;}oGun2:
if(FzDX_(APBny,g85jV)){goto gAdE9;}
else{goto nVEbT;}nVEbT:if(hS63U==RMLPositionFlags::GET_INTO_BOUNDARIES_FAST){
if(QxCTg(N2UxV,E6Xnb,APBny,g85jV,sguyX)){goto oNDBP;}
else{goto gvnpt;}}else
{
if(npy1a(N2UxV,E6Xnb,APBny,-g85jV,sguyX)){yb9FF(&MzfY6,&BkjIW,&APBny,&N2UxV,
g85jV,E6Xnb,sguyX);}
else{Fg7ou(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);}goto gAdE9;}oNDBP:
if(n1jSJ(N2UxV,APBny,g85jV,sguyX)){
SOwi2(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);goto gAdE9;}
else{
trCCb(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);goto gAdE9;}gvnpt:
if(aTBHx(N2UxV,E6Xnb,APBny,g85jV,sguyX)){goto P74JJ;}
else{
vRLUs(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,E6Xnb,sguyX);goto gAdE9;}P74JJ:
if(ZJVcc(N2UxV,E6Xnb,APBny,g85jV,sguyX)){
trCCb(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);goto gAdE9;}
else{
eWaLB(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,E6Xnb,sguyX);goto gAdE9;}gAdE9:EEKof=
fabs(N2UxV/sguyX);if((WFRE7((APBny+0.5*N2UxV*EEKof),sabLl,pgyq8))&&(WFRE7((BkjIW
+APBny*EEKof+0.5*N2UxV*KNYa5(EEKof)-wModI(N2UxV)*D_rgb(EEKof)*sguyX/6.0),YOMpW,
pgyq8))){MzfY6+=EEKof;goto Z05Oy;}
if(LuWav(N2UxV,APBny,sabLl,sguyX)){goto _tNlq;}else{MzfY6=JCQ6Q;goto Z05Oy;}
_tNlq:
if(Onldg(Pn4_H)){goto mWwy9;}else{MzfY6=JCQ6Q;goto Z05Oy;}mWwy9:
if(lnU6U(N2UxV,E6Xnb,APBny,sabLl,sguyX)){goto SWoii;}else{goto TUP5D;}SWoii:
if(Wd_zo(N2UxV,E6Xnb,APBny,sabLl,sguyX)){goto vGR8D;}else{goto LUti5;}vGR8D:
if(Bp1jS(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){goto xmsgO;}else{
DF6gv(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,0.0,&gQP2R);*MqVoX=
BEy1V;goto Z05Oy;}xmsgO:


if(zs6Zz(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX,&ZCUBa,&SEtWr)){

if(SEtWr!=JCQ6Q){SEtWr+=MzfY6;}MzfY6+=ZCUBa;*MqVoX=BEy1V;goto Z05Oy;}else{MzfY6=
JCQ6Q;goto Z05Oy;}LUti5:
if(bdqe9(N2UxV,APBny,sabLl,BkjIW,YOMpW,sguyX)){goto RFkkG;}else{goto dLiWS;}
RFkkG:


if(QGHcW(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX,&ZCUBa,&SEtWr)){

if(SEtWr!=JCQ6Q){SEtWr+=MzfY6;}MzfY6+=ZCUBa;*MqVoX=BEy1V;goto Z05Oy;}else{goto 
Z4Qny;}dLiWS:
if(qq5iI(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
hX4Vl(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,0.0,&gQP2R);if(
gQP2R){
DF6gv(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,0.0,&gQP2R);*MqVoX=
BEy1V;}else{*MqVoX=OyKu3;}goto Z05Oy;}else{
DF6gv(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,0.0,&gQP2R);if(
gQP2R){
hX4Vl(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,0.0,&gQP2R);*MqVoX=
OyKu3;}else{*MqVoX=BEy1V;}goto Z05Oy;}TUP5D:
if(q65de(N2UxV,APBny,sabLl,BkjIW,YOMpW,sguyX)){goto Z4Qny;}else{
hX4Vl(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,0.0,&gQP2R);*MqVoX=
OyKu3;goto Z05Oy;}Z4Qny:


if(gS8__(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX,&ZCUBa,&SEtWr)){

if(SEtWr!=JCQ6Q){SEtWr+=MzfY6;}MzfY6+=ZCUBa;*MqVoX=OyKu3;goto Z05Oy;}else{MzfY6=
JCQ6Q;goto Z05Oy;}Z05Oy:*jxnfV=MzfY6;*Ldd82=SEtWr;return(gQP2R);}
