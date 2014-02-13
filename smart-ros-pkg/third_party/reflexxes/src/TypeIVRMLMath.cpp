































#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <math.h>


void diBqY::PHbIW(double*Y8Kvo,double*NDlsh){double fDwCh=*Y8Kvo,cAN_m=*NDlsh,
dl7Hv=WvOhx*(cAN_m-fDwCh)+iEiah;*Y8Kvo+=dl7Hv;*NDlsh-=dl7Hv;if(*Y8Kvo<=*NDlsh){
return;}else{*Y8Kvo=fDwCh;*NDlsh=cAN_m;}return;}

void diBqY::w_roO(double*Y8Kvo,double*NDlsh){double fDwCh=*Y8Kvo,cAN_m=*NDlsh;*
Y8Kvo=(fDwCh-iEiah-WvOhx*(cAN_m+fDwCh))/(1.0-2.0*WvOhx);*NDlsh=(cAN_m+iEiah-
WvOhx*(cAN_m+fDwCh))/(1.0-2.0*WvOhx);if(*Y8Kvo>*NDlsh){*Y8Kvo=fDwCh;*NDlsh=cAN_m
;}if(i5gUI(*Y8Kvo)!=i5gUI(*NDlsh)){if(fabs(*NDlsh)>fabs(*Y8Kvo)){*Y8Kvo=0.0;}
else{*NDlsh=0.0;}}return;}

double diBqY::fEBjW(const double&_ksNf){unsigned int i=(0x376+126-0x3f4);double 
ReturnValue=1.0,Time=_ksNf;if(Time>50.0){Time*=0.1;for(i=(0x13dd+860-0x1739);i<
(0xcb0+2766-0x1772);i++){if(Time>10.0){Time*=0.1;ReturnValue*=4.0;}else{break;}}
return(ReturnValue);}else{return(0.0);}}
