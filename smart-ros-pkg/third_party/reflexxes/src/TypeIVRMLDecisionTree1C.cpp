
































#include <TypeIVRMLDecisionTree1C.h>
#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLStep1Profiles.h>
#include <RMLPositionFlags.h>


bool diBqY::sfoTf(const double&Z45C3,const double&g93mR,const double&_Qa5G,const
 double&sguyX,const double&E6Xnb,const double&g85jV,const double&AUze_,const 
double&hmD2W,const fT_oe&Pn4_H,const fT_oe&NC87S,const int&hS63U,double*eXgqi){
bool gQP2R=false;double MzfY6=0.0,BkjIW=Z45C3,APBny=g93mR,N2UxV=_Qa5G,YOMpW=
AUze_,sabLl=hmD2W;
if(vwVqL(N2UxV)){goto mb9VU;}
else{
tvfxt(&BkjIW,&APBny,&N2UxV,&YOMpW,&sabLl);goto mb9VU;}mb9VU:
if(l7omV(N2UxV,E6Xnb)){goto LWUUW;}
else{
x96T7(&MzfY6,&BkjIW,&APBny,&N2UxV,E6Xnb,sguyX);goto LWUUW;}LWUUW:
if(QNJM_(N2UxV,APBny,g85jV,sguyX)){goto bDdBf;}
else{
AFNhq(&MzfY6,&BkjIW,&APBny,&N2UxV,sguyX);
tvfxt(&BkjIW,&APBny,&N2UxV,&YOMpW,&sabLl);goto vzHXO;}bDdBf:
if(FtH2C(APBny,g85jV)){goto f5c98;}
else{goto vzHXO;}vzHXO:if(hS63U==RMLPositionFlags::GET_INTO_BOUNDARIES_FAST){
if(mE5Hh(N2UxV,E6Xnb,APBny,g85jV,sguyX)){goto olFSn;}
else{goto ouL94;}}else
{
if(uZESL(N2UxV,E6Xnb,APBny,-g85jV,sguyX)){yb9FF(&MzfY6,&BkjIW,&APBny,&N2UxV,
g85jV,E6Xnb,sguyX);}
else{Fg7ou(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);}goto f5c98;}olFSn:
if(NdjPI(N2UxV,APBny,g85jV,sguyX)){
SOwi2(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);goto f5c98;}
else{
trCCb(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);goto f5c98;}ouL94:
if(eRkwV(N2UxV,E6Xnb,APBny,g85jV,sguyX)){goto o4b4r;}
else{
vRLUs(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,E6Xnb,sguyX);goto f5c98;}o4b4r:
if(Ua0IY(N2UxV,E6Xnb,APBny,g85jV,sguyX)){
trCCb(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);goto f5c98;}
else{
eWaLB(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,E6Xnb,sguyX);goto f5c98;}f5c98:
if(_lqQb(sabLl)){goto baMkR;}else{

tvfxt(&BkjIW,&APBny,&N2UxV,&YOMpW,&sabLl);goto baMkR;}baMkR:
if(L4uIC(N2UxV)){goto eqPlo;}else{
kGHQ1(&MzfY6,&BkjIW,&APBny,&N2UxV,sguyX);if(APBny<-g85jV){APBny=-g85jV;
}goto jCI_z;}eqPlo:
if(V66ya(Pn4_H,NC87S)){AFNhq(&MzfY6,&BkjIW,&APBny,&N2UxV,sguyX);if(APBny>g85jV){
APBny=g85jV;
}goto jCI_z;}else{goto jCI_z;}jCI_z:
if(UBpeN(N2UxV,E6Xnb,APBny,g85jV,sguyX)){goto CvzoW;}else{goto oeDij;}CvzoW:
if(oL4md(N2UxV,E6Xnb,APBny,g85jV,sabLl,sguyX)){goto QaIHW;}else{goto O8i1a;}
QaIHW:
if(a0Alv(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
l4KsC(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,E6Xnb,sguyX);
kVdy5(&MzfY6,BkjIW,YOMpW,APBny,sabLl,N2UxV,sguyX,&gQP2R);goto Z05Oy;}else{goto 
Id1LJ;}Id1LJ:
if(AUj_t(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){YP_Hn(&MzfY6,BkjIW,
YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,&gQP2R);goto Z05Oy;}else{z9bCK(&MzfY6,
BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,&gQP2R);goto Z05Oy;}O8i1a:
if(JidWA(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
l4KsC(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,E6Xnb,sguyX);
NnK2z(&MzfY6,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,&gQP2R);goto Z05Oy;}else{
goto DPpTB;}DPpTB:
if(HqfuH(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){Q5ekw(&MzfY6,BkjIW,
YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,&gQP2R);goto Z05Oy;}else{goto gtb44;}
gtb44:
if(gChmN(N2UxV,E6Xnb,APBny,sabLl,sguyX)){goto Id1LJ;}else{goto mItn3;}mItn3:
if(yLL7T(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){rNksa(&MzfY6,BkjIW,
YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,&gQP2R);if(gQP2R){z9bCK(&MzfY6,BkjIW,
YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,&gQP2R);}goto Z05Oy;}else{z9bCK(&MzfY6
,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,&gQP2R);if(gQP2R){rNksa(&MzfY6,
BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,&gQP2R);}goto Z05Oy;}oeDij:
if(ip9ju(N2UxV,E6Xnb,APBny,g85jV,sabLl,sguyX)){goto Hu7Fr;}else{goto q41vB;}
Hu7Fr:
if(awseK(N2UxV,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
JCYJB(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);
kVdy5(&MzfY6,BkjIW,YOMpW,APBny,sabLl,N2UxV,sguyX,&gQP2R);goto Z05Oy;}else{z9bCK(
&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,&gQP2R);goto Z05Oy;}q41vB
:
if(wdvzU(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
JCYJB(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);
NnK2z(&MzfY6,BkjIW,YOMpW,APBny,sabLl,N2UxV,E6Xnb,sguyX,&gQP2R);goto Z05Oy;}else{
goto mItn3;}Z05Oy:*eXgqi=MzfY6;return(gQP2R);}
