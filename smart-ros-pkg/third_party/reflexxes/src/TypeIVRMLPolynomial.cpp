
































#include <TypeIVRMLPolynomial.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>


diBqY::hAXqB::hAXqB(){BQaBa=0.0;nFNxF=0.0;fjoX_=0.0;ntiBa=0.0;G0Jic=0.0;UwGEO=
(0x1301+3219-0x1f94);}

diBqY::hAXqB::~hAXqB(){}


void diBqY::hAXqB::iPzPj(const double&k4gVG,const double&fRHp8,const double&
gpXZ6,const double&i0yhN,const double&XTwYX){BQaBa=i0yhN;nFNxF=gpXZ6;fjoX_=fRHp8
;ntiBa=k4gVG;G0Jic=XTwYX;if(ntiBa!=0.0){UwGEO=(0x200c+1252-0x24ed);return;}if(
fjoX_!=0.0){UwGEO=(0x1d65+1036-0x216f);return;}if(nFNxF!=0.0){UwGEO=
(0x613+4798-0x18d0);return;}UwGEO=(0x54b+5933-0x1c78);return;}

void diBqY::hAXqB::r2HAp(double*k4gVG,double*fRHp8,double*gpXZ6,double*i0yhN,
double*XTwYX)const{*k4gVG=this->ntiBa;*fRHp8=this->fjoX_;*gpXZ6=this->nFNxF;*
i0yhN=this->BQaBa;*XTwYX=this->G0Jic;return;}


double diBqY::hAXqB::EHwVi(const double&oG7tP)const{return((UwGEO==
(0xedd+1539-0x14dd))?(ntiBa*(oG7tP-G0Jic)*(oG7tP-G0Jic)*(oG7tP-G0Jic)+fjoX_*(
oG7tP-G0Jic)*(oG7tP-G0Jic)+nFNxF*(oG7tP-G0Jic)+BQaBa):((UwGEO==
(0x1b3+1765-0x896))?(fjoX_*(oG7tP-G0Jic)*(oG7tP-G0Jic)+nFNxF*(oG7tP-G0Jic)+BQaBa
):((UwGEO==(0x573+887-0x8e9))?(nFNxF*(oG7tP-G0Jic)+BQaBa):(BQaBa))));}

void diBqY::hAXqB::oZfa4(unsigned int*gnnNs,double*fDo_w,double*Y6GNL,double*
au8gB)const{if(this->UwGEO==(0x18fa+1369-0x1e51)){double j3z4i=this->BQaBa/this
->fjoX_,YDn87=this->nFNxF/this->fjoX_,XDFjK=0.25*KNYa5(YDn87)-j3z4i;if(XDFjK<
ASnUB){
*fDo_w=0.0;*Y6GNL=0.0;*au8gB=0.0;*gnnNs=(0x120d+4584-0x23f5);}else{
XDFjK=diBqY::SUYGr(XDFjK);*fDo_w=-0.5*YDn87+XDFjK+this->G0Jic;*Y6GNL=-0.5*YDn87-
XDFjK+this->G0Jic;*au8gB=0.0;*gnnNs=(0x32+7682-0x1e32);}return;}if(this->UwGEO==
(0x101+390-0x286)){*fDo_w=-this->BQaBa/this->nFNxF+this->G0Jic;*Y6GNL=0.0;*au8gB
=0.0;*gnnNs=(0xc05+873-0xf6d);return;}if(this->UwGEO==(0x1849+1129-0x1caf)){
*fDo_w=0.0;*Y6GNL=0.0;*au8gB=0.0;*gnnNs=(0x3f7+6749-0x1e54);return;}if(this->
UwGEO==(0x960+1901-0x10cd)){*fDo_w=0.0;*Y6GNL=0.0;*au8gB=0.0;*gnnNs=
(0x12b1+4162-0x22f3);}return;}
