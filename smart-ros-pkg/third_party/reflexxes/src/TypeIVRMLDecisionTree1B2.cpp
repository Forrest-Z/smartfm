
































#include <TypeIVRMLDecisionTree1B2.h>
#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLStep1Profiles.h>
#include <RMLPositionFlags.h>


bool diBqY::Wcx0t(const double&Z45C3,const double&g93mR,const double&_Qa5G,const
 double&sguyX,const double&E6Xnb,const double&g85jV,const double&AUze_,const 
double&hmD2W,const double&_zBD2,const int&hS63U,fT_oe*MqVoX,double*jxnfV){bool 
gQP2R=false,JuQuQ=false;double MzfY6=0.0,BkjIW=Z45C3,APBny=g93mR,N2UxV=_Qa5G,
YOMpW=AUze_,sabLl=hmD2W,EEKof=0.0;
if(kELK9(N2UxV)){goto xmkE2;}
else{
tvfxt(&BkjIW,&APBny,&N2UxV,&YOMpW,&sabLl);goto xmkE2;}xmkE2:
if(J4hnf(N2UxV,E6Xnb)){goto M8BaQ;}
else{
x96T7(&MzfY6,&BkjIW,&APBny,&N2UxV,E6Xnb,sguyX);goto M8BaQ;}M8BaQ:
if(B2SR3(N2UxV,APBny,g85jV,sguyX)){goto ADe3O;}
else{
AFNhq(&MzfY6,&BkjIW,&APBny,&N2UxV,sguyX);
tvfxt(&BkjIW,&APBny,&N2UxV,&YOMpW,&sabLl);goto RZWxT;}ADe3O:
if(EkfQK(APBny,g85jV)){goto jcVDt;}
else{goto RZWxT;}RZWxT:if(hS63U==RMLPositionFlags::GET_INTO_BOUNDARIES_FAST){
if(RYtZY(N2UxV,E6Xnb,APBny,g85jV,sguyX)){goto wIEEI;}
else{goto ONLI5;}}else
{
if(lLDLn(N2UxV,E6Xnb,APBny,-g85jV,sguyX)){yb9FF(&MzfY6,&BkjIW,&APBny,&N2UxV,
g85jV,E6Xnb,sguyX);}
else{Fg7ou(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);}goto jcVDt;}wIEEI:
if(ha93u(N2UxV,APBny,g85jV,sguyX)){
SOwi2(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);goto jcVDt;}
else{
trCCb(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);goto jcVDt;}ONLI5:
if(DV5xt(N2UxV,E6Xnb,APBny,g85jV,sguyX)){goto TLeiY;}
else{
vRLUs(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,E6Xnb,sguyX);goto jcVDt;}TLeiY:
if(dLV3B(N2UxV,E6Xnb,APBny,g85jV,sguyX)){
trCCb(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,sguyX);goto jcVDt;}
else{
eWaLB(&MzfY6,&BkjIW,&APBny,&N2UxV,g85jV,E6Xnb,sguyX);goto jcVDt;}jcVDt:EEKof=
fabs(N2UxV/sguyX);if((WFRE7((APBny+0.5*N2UxV*EEKof),sabLl,pgyq8))&&(WFRE7((BkjIW
+APBny*EEKof+0.5*N2UxV*KNYa5(EEKof)-wModI(N2UxV)*D_rgb(EEKof)*sguyX/6.0),YOMpW,
pgyq8))){MzfY6+=EEKof;goto Z05Oy;}
if(bHIVJ(N2UxV,APBny,sguyX)){

tvfxt(&BkjIW,&APBny,&N2UxV,&YOMpW,&sabLl);}
if(NuTgm(N2UxV)){JuQuQ=false;goto kk9wX;}else{JuQuQ=true;goto t21xx;}kk9wX:
if(TVphj(N2UxV,APBny,sabLl,g85jV,sguyX)){goto dJmQ_;}else{
AFNhq(&MzfY6,&BkjIW,&APBny,&N2UxV,sguyX);if(APBny>g85jV){APBny=g85jV;
}goto mbM3D;}dJmQ_:
if(ABNtV(N2UxV,E6Xnb,APBny,sabLl,sguyX)){goto EiXQE;}else{goto NL5PM;}EiXQE:
if(OS6qO(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
AFNhq(&MzfY6,&BkjIW,&APBny,&N2UxV,sguyX);if(APBny>g85jV){APBny=g85jV;
}goto ngQBK;}else{
DF6gv(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=BEy1V;goto Z05Oy;}ngQBK:
if(xJQ_3(N2UxV,E6Xnb,APBny,sguyX)){goto xFtil;}else{goto fYYfG;}xFtil:
if(wfj20(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
z_T_D(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=nlydG;goto Z05Oy;}else{goto DUvmO;}DUvmO:
if(KDxXd(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
z_T_D(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=nlydG;goto Z05Oy;}else{goto Tol4F;}Tol4F:
if(dMvfh(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
iIiZM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=zfD15;}else{*MqVoX=XnpQc;}goto Z05Oy;}else{goto ZRxYW;}ZRxYW:
if(RTv7w(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
iIiZM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=zfD15;}else{*MqVoX=XnpQc;}goto Z05Oy;}else{MzfY6=JCQ6Q;goto Z05Oy;
}fYYfG:
if(KOi7e(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
z_T_D(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=nlydG;goto Z05Oy;}else{goto RpYbr;}RpYbr:
if(UPA1D(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
z_T_D(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=nlydG;goto Z05Oy;}else{MzfY6=JCQ6Q;goto Z05Oy;}NL5PM:
if(fx4dT(N2UxV,APBny,sabLl,BkjIW,YOMpW,sguyX)){
AFNhq(&MzfY6,&BkjIW,&APBny,&N2UxV,sguyX);if(APBny>g85jV){APBny=g85jV;
}goto AIHqe;}else{goto GcY17;}AIHqe:
if(JMEcZ(N2UxV,E6Xnb,APBny,sguyX)){goto JrWl1;}else{goto b28z6;}JrWl1:
if(fJ4Eo(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){goto gMPem;}else{goto Ouur2
;}gMPem:
if(WpD6G(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
aXdEM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=V1Tsd;goto Z05Oy;}else{
z_T_D(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=nlydG;goto Z05Oy;}Ouur2:
if(tdH3Q(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
aXdEM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=V1Tsd;goto Z05Oy;}else{goto DUvmO;}b28z6:
if(itEEY(E6Xnb,sabLl,sguyX)){goto tRhmx;}else{goto BjlF_;}tRhmx:
if(hhiih(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){goto gMPem;}else{goto DgxcR
;}DgxcR:
if(psyRE(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
aXdEM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=V1Tsd;goto Z05Oy;}else{goto x2XHi;}x2XHi:
if(I5MSf(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){goto gMPem;}else{
MzfY6=JCQ6Q;goto Z05Oy;}BjlF_:
if(AgQlv(N2UxV,APBny,sabLl,BkjIW,YOMpW,sguyX)){
aXdEM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=bC_iO;}else{*MqVoX=V1Tsd;}goto Z05Oy;}else{goto yVHQM;}yVHQM:
if(A68S7(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
aXdEM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=bC_iO;}else{*MqVoX=V1Tsd;}goto Z05Oy;}else{MzfY6=JCQ6Q;goto Z05Oy;
}GcY17:
if(qs90k(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
hX4Vl(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=OyKu3;if(gQP2R){
DF6gv(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=BEy1V;}goto Z05Oy;}else{
DF6gv(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=BEy1V;if(gQP2R){
hX4Vl(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=OyKu3;}goto Z05Oy;}mbM3D:
if(K3p0Y(N2UxV,E6Xnb,APBny,sabLl,sguyX)){goto ZVxMr;}else{goto oqnuj;}ZVxMr:
if(Ba8IA(E6Xnb,sabLl,sguyX)){goto bRTU4;}else{goto hFNFh;}bRTU4:
if(x9Zgn(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
tyUqM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=tmFJN;}else{*MqVoX=pPyJE;}goto Z05Oy;}else{goto AioA6;}AioA6:
if(l9t2A(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
tyUqM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=tmFJN;}else{*MqVoX=pPyJE;}goto Z05Oy;}else{goto Tol4F;}hFNFh:
if(VN7tH(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
tyUqM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=tmFJN;}else{*MqVoX=pPyJE;}goto Z05Oy;}else{goto dusgG;}dusgG:
if(w10As(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
tyUqM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=tmFJN;}else{*MqVoX=pPyJE;}goto Z05Oy;}else{MzfY6=JCQ6Q;goto Z05Oy;
}oqnuj:
if(vhEYc(N2UxV,E6Xnb,APBny,sguyX)){goto LlIg2;}else{goto BjlF_;}LlIg2:
if(p5v2y(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
aXdEM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=bC_iO;}else{*MqVoX=V1Tsd;}goto Z05Oy;}else{goto PWsau;}PWsau:
if(GruKb(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
aXdEM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=bC_iO;}else{*MqVoX=V1Tsd;}*MqVoX=V1Tsd;goto Z05Oy;}else{goto ZVxMr
;}t21xx:
if(Fmqc5(N2UxV,APBny,sabLl,sguyX)){goto qeBmd;}else{goto XnNHt;}qeBmd:
if(CnP0q(N2UxV,E6Xnb,APBny,sabLl,sguyX)){goto JoUAs;}else{goto U4HVm;}JoUAs:
if(RkfAv(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
ww9Hd(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=BEy1V;
goto Z05Oy;}else{goto ZVxMr;}U4HVm:
if(U5UeM(N2UxV,APBny,sabLl,BkjIW,YOMpW,sguyX)){goto ydl02;}else{goto mbM3D;}
ydl02:
if(mtrbZ(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
c6JAf(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=OyKu3;if(gQP2R){
ww9Hd(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=BEy1V;}goto Z05Oy;}else{
ww9Hd(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=BEy1V;if(gQP2R){
c6JAf(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);*
MqVoX=OyKu3;}goto Z05Oy;}XnNHt:
if(rpBOH(N2UxV,E6Xnb,APBny,sabLl,sguyX)){goto KsTqV;}else{goto irQaR;}KsTqV:
if(xtJ6k(N2UxV,E6Xnb,APBny,sabLl,sguyX)){goto oqnuj;}else{goto QoMQ4;}QoMQ4:
if(aOLsR(E6Xnb,sabLl,sguyX)){goto N3m6P;}else{goto BjlF_;}N3m6P:
if(liWft(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
aXdEM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=bC_iO;}else{*MqVoX=V1Tsd;}goto Z05Oy;}else{goto _1F1a;}_1F1a:
if(CaKTg(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
aXdEM(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=bC_iO;}else{*MqVoX=V1Tsd;}goto Z05Oy;}else{goto irQaR;}irQaR:
if(Zty4X(N2UxV,E6Xnb,APBny,sguyX)){goto X_iiw;}else{goto fYYfG;}X_iiw:
if(PQg1A(N2UxV,E6Xnb,APBny,sabLl,BkjIW,YOMpW,sguyX)){
z_T_D(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=X4fbV;}else{*MqVoX=nlydG;}goto Z05Oy;}else{goto cLGN8;}cLGN8:
if(ILzvk(N2UxV,E6Xnb,APBny,g85jV,sabLl,BkjIW,YOMpW,sguyX)){
z_T_D(&MzfY6,BkjIW,YOMpW,APBny,g85jV,sabLl,N2UxV,E6Xnb,sguyX,_zBD2,&gQP2R);if(
JuQuQ){*MqVoX=X4fbV;}else{*MqVoX=nlydG;}goto Z05Oy;}else{goto Tol4F;}Z05Oy:*
jxnfV=MzfY6;return(gQP2R);}
