































#include <TypeIVRMLQuicksort.h>


void diBqY::momGP(const int&CUTd4,const int&lH0O8,double*jM1zJ){int PvrwW,OuKGy;
double qSXnr,jKA_P;qSXnr=jM1zJ[(CUTd4+lH0O8)/(0x13b3+2706-0x1e43)];PvrwW=CUTd4;
OuKGy=lH0O8;
while(PvrwW<=OuKGy){while(jM1zJ[PvrwW]<qSXnr){PvrwW=PvrwW+(0x125b+2296-0x1b52);}
while(jM1zJ[OuKGy]>qSXnr){OuKGy=OuKGy-(0x2a6+456-0x46d);}if(PvrwW<=OuKGy){
jKA_P=jM1zJ[PvrwW];jM1zJ[PvrwW]=jM1zJ[OuKGy];jM1zJ[OuKGy]=jKA_P;PvrwW=PvrwW+
(0x239+2043-0xa33);OuKGy=OuKGy-(0x60b+183-0x6c1);}}
if(CUTd4<OuKGy){momGP(CUTd4,OuKGy,jM1zJ);}if(PvrwW<lH0O8){momGP(PvrwW,lH0O8,
jM1zJ);}}
