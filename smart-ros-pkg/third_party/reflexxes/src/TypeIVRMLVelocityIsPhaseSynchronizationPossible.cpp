






























#include <TypeIVRMLVelocity.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLMath.h>
#include <RMLVelocityInputParameters.h>
#include <RMLVelocityOutputParameters.h>
#include <RMLVector.h>
#include <ReflexxesAPI.h>
#include <RMLVelocityFlags.h>
using namespace diBqY;

bool XdeRR::KSiWK(void){bool vhwls=true,BF1yT=false;unsigned int i=
(0x1bea+1041-0x1ffb);double _AHiK=0.0,okPrc=0.0,RnMR2=0.0,vnYJR=0.0;for(i=
(0x1652+2420-0x1fc6);i<this->NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]){(
this->KSW1w->VecData)[i]=(this->mYHVu->CurrentVelocityVector->VecData)[i];(this
->tpjD5->VecData)[i]=(this->mYHVu->CurrentAccelerationVector->VecData)[i];(this
->J49Ku->VecData)[i]=(this->mYHVu->TargetVelocityVector->VecData)[i];_AHiK+=
KNYa5((this->tpjD5->VecData)[i]);okPrc+=KNYa5((this->KSW1w->VecData)[i]);RnMR2+=
KNYa5((this->J49Ku->VecData)[i]);}else{(this->KSW1w->VecData)[i]=0.0;(this->
tpjD5->VecData)[i]=0.0;(this->J49Ku->VecData)[i]=0.0;}}_AHiK=SUYGr(_AHiK);okPrc=
SUYGr(okPrc);RnMR2=SUYGr(RnMR2);if((_AHiK!=DeB6Y)&&(_AHiK!=0.0)){for(i=
(0x16ad+963-0x1a70);i<this->NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]){(
this->tpjD5->VecData)[i]/=_AHiK;}}}else{for(i=(0x1774+3408-0x24c4);i<this->
NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]){(this->tpjD5->VecData)[i]=0.0;}}
}if((okPrc!=DeB6Y)&&(okPrc!=0.0)){for(i=(0xe1f+1231-0x12ee);i<this->NumberOfDOFs
;i++){if((this->zRNIW->VecData)[i]){(this->KSW1w->VecData)[i]/=okPrc;}}}else{for
(i=(0x1160+18-0x1172);i<this->NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]){(
this->KSW1w->VecData)[i]=0.0;}}}if((RnMR2!=DeB6Y)&&(RnMR2!=0.0)){for(i=
(0x2c+1832-0x754);i<this->NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]){(this
->J49Ku->VecData)[i]/=RnMR2;}}}else{for(i=(0x1d43+1799-0x244a);i<this->
NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]){(this->J49Ku->VecData)[i]=0.0;}}
}
vnYJR=dd74u;if((_AHiK>=okPrc)&&(_AHiK>=RnMR2)&&(_AHiK>=dd74u)){*(this->q6cTW)=*(
this->tpjD5);vnYJR=_AHiK;}if((okPrc>=_AHiK)&&(okPrc>=RnMR2)&&(okPrc>=dd74u)){*(
this->q6cTW)=*(this->KSW1w);vnYJR=okPrc;}if((RnMR2>=_AHiK)&&(RnMR2>=okPrc)&&(
RnMR2>=dd74u)){*(this->q6cTW)=*(this->J49Ku);vnYJR=RnMR2;}if(vnYJR>dd74u){
BF1yT=true;for(i=(0x217d+455-0x2344);i<this->NumberOfDOFs;i++){if((this->zRNIW->
VecData)[i]){if((i5gUI((this->tpjD5->VecData)[i])==i5gUI((this->q6cTW->VecData)[
i]))&&(fabs((this->tpjD5->VecData)[i])>dd74u)){BF1yT=false;break;}}}if(BF1yT){
for(i=(0x128c+206-0x135a);i<this->NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]
){(this->tpjD5->VecData)[i]=-(this->tpjD5->VecData)[i];}}}BF1yT=true;for(i=
(0xe54+4302-0x1f22);i<this->NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]){if((
i5gUI((this->KSW1w->VecData)[i])==i5gUI((this->q6cTW->VecData)[i]))&&(fabs((this
->KSW1w->VecData)[i])>dd74u)){BF1yT=false;break;}}}if(BF1yT){for(i=
(0xd67+4408-0x1e9f);i<this->NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]){(
this->KSW1w->VecData)[i]=-(this->KSW1w->VecData)[i];}}}BF1yT=true;for(i=
(0xa15+6982-0x255b);i<this->NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]){if((
i5gUI((this->J49Ku->VecData)[i])==i5gUI((this->q6cTW->VecData)[i]))&&(fabs((this
->J49Ku->VecData)[i])>dd74u)){BF1yT=false;break;}}}if(BF1yT){for(i=
(0x619+3269-0x12de);i<this->NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]){(
this->J49Ku->VecData)[i]=-(this->J49Ku->VecData)[i];}}}
for(i=(0xedb+2368-0x181b);i<this->NumberOfDOFs;i++){if((this->zRNIW->VecData)[i]
){if(((fabs((this->q6cTW->VecData)[i]-(this->tpjD5->VecData)[i])>dd74u)&&(fabs((
this->tpjD5->VecData)[i])>dd74u))||((fabs((this->q6cTW->VecData)[i]-(this->KSW1w
->VecData)[i])>dd74u)&&(fabs((this->KSW1w->VecData)[i])>dd74u))||((fabs((this->
q6cTW->VecData)[i]-(this->J49Ku->VecData)[i])>dd74u)&&(fabs((this->J49Ku->
VecData)[i])>dd74u))){vhwls=false;break;}}}}else{vhwls=false;}if(!vhwls){this->
q6cTW->Set(0.0);}return(vhwls);}
