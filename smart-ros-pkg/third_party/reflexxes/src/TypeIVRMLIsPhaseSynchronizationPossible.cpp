






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLDecisionTree1A.h>
#include <RMLPositionInputParameters.h>
using namespace diBqY;

bool L62Wd::KSiWK(RMLDoubleVector*Mk66j){bool vhwls=true,BF1yT=false;unsigned 
int i=(0x227+140-0x2b3);double _AHiK=0.0,T3_4Z=0.0,okPrc=0.0,RnMR2=0.0,vnYJR=0.0
;for(i=(0x18a9+1812-0x1fbd);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[
i]){(this->gCgoK->VecData)[i]=(this->mYHVu->TargetPositionVector->VecData)[i]-(
this->mYHVu->CurrentPositionVector->VecData)[i];(this->KSW1w->VecData)[i]=(this
->mYHVu->CurrentVelocityVector->VecData)[i];(this->tpjD5->VecData)[i]=(this->
mYHVu->CurrentAccelerationVector->VecData)[i];(this->J49Ku->VecData)[i]=(this->
mYHVu->TargetVelocityVector->VecData)[i];T3_4Z+=KNYa5((this->gCgoK->VecData)[i])
;_AHiK+=KNYa5((this->tpjD5->VecData)[i]);okPrc+=KNYa5((this->KSW1w->VecData)[i])
;RnMR2+=KNYa5((this->J49Ku->VecData)[i]);}else{(this->gCgoK->VecData)[i]=0.0;(
this->KSW1w->VecData)[i]=0.0;(this->tpjD5->VecData)[i]=0.0;(this->J49Ku->VecData
)[i]=0.0;}}T3_4Z=SUYGr(T3_4Z);_AHiK=SUYGr(_AHiK);okPrc=SUYGr(okPrc);RnMR2=SUYGr(
RnMR2);if((T3_4Z!=DeB6Y)&&(T3_4Z!=0.0)){for(i=(0x39b+5316-0x185f);i<this->
NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(this->gCgoK->VecData)[i]/=T3_4Z
;}}}else{for(i=(0x9c7+1615-0x1016);i<this->NumberOfDOFs;i++){if((this->Ijlxn->
VecData)[i]){(this->gCgoK->VecData)[i]=0.0;}}}if((_AHiK!=DeB6Y)&&(_AHiK!=0.0)){
for(i=(0x946+3382-0x167c);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]
){(this->tpjD5->VecData)[i]/=_AHiK;}}}else{for(i=(0x421+2542-0xe0f);i<this->
NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(this->tpjD5->VecData)[i]=0.0;}}
}if((okPrc!=DeB6Y)&&(okPrc!=0.0)){for(i=(0xa9f+4447-0x1bfe);i<this->NumberOfDOFs
;i++){if((this->Ijlxn->VecData)[i]){(this->KSW1w->VecData)[i]/=okPrc;}}}else{for
(i=(0x595+1751-0xc6c);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(
this->KSW1w->VecData)[i]=0.0;}}}if((RnMR2!=DeB6Y)&&(RnMR2!=0.0)){for(i=
(0x967+4938-0x1cb1);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(
this->J49Ku->VecData)[i]/=RnMR2;}}}else{for(i=(0x2d6+1374-0x834);i<this->
NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(this->J49Ku->VecData)[i]=0.0;}}
}
vnYJR=dd74u;if((T3_4Z>=_AHiK)&&(T3_4Z>=okPrc)&&(T3_4Z>=RnMR2)&&(T3_4Z>=dd74u)){*
(this->p0GMV)=*(this->gCgoK);vnYJR=T3_4Z;this->zeGXc=L62Wd::HBU__;}if((_AHiK>=
T3_4Z)&&(_AHiK>=okPrc)&&(_AHiK>=RnMR2)&&(_AHiK>=dd74u)){*(this->p0GMV)=*(this->
tpjD5);vnYJR=_AHiK;this->zeGXc=L62Wd::zH7cC;}if((okPrc>=T3_4Z)&&(okPrc>=_AHiK)&&
(okPrc>=RnMR2)&&(okPrc>=dd74u)){*(this->p0GMV)=*(this->KSW1w);vnYJR=okPrc;this->
zeGXc=L62Wd::y95V4;}if((RnMR2>=T3_4Z)&&(RnMR2>=okPrc)&&(RnMR2>=_AHiK)&&(RnMR2>=
dd74u)){*(this->p0GMV)=*(this->J49Ku);vnYJR=RnMR2;this->zeGXc=L62Wd::gY3Wk;}if(
vnYJR>dd74u){
BF1yT=true;for(i=(0x23f5+508-0x25f1);i<this->NumberOfDOFs;i++){if((this->Ijlxn->
VecData)[i]){if((i5gUI((this->gCgoK->VecData)[i])==i5gUI((this->p0GMV->VecData)[
i]))&&(fabs((this->gCgoK->VecData)[i])>dd74u)){BF1yT=false;break;}}}if(BF1yT){
for(i=(0xc72+28-0xc8e);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(
this->gCgoK->VecData)[i]=-(this->gCgoK->VecData)[i];}}}BF1yT=true;for(i=
(0xf5a+821-0x128f);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if((
i5gUI((this->tpjD5->VecData)[i])==i5gUI((this->p0GMV->VecData)[i]))&&(fabs((this
->tpjD5->VecData)[i])>dd74u)){BF1yT=false;break;}}}if(BF1yT){for(i=
(0x4f8+1759-0xbd7);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(this
->tpjD5->VecData)[i]=-(this->tpjD5->VecData)[i];}}}BF1yT=true;for(i=
(0x18ef+2930-0x2461);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if(
(i5gUI((this->KSW1w->VecData)[i])==i5gUI((this->p0GMV->VecData)[i]))&&(fabs((
this->KSW1w->VecData)[i])>dd74u)){BF1yT=false;break;}}}if(BF1yT){for(i=
(0x90a+5108-0x1cfe);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(
this->KSW1w->VecData)[i]=-(this->KSW1w->VecData)[i];}}}BF1yT=true;for(i=
(0xb63+2323-0x1476);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if((
i5gUI((this->J49Ku->VecData)[i])==i5gUI((this->p0GMV->VecData)[i]))&&(fabs((this
->J49Ku->VecData)[i])>dd74u)){BF1yT=false;break;}}}if(BF1yT){for(i=
(0xaed+4979-0x1e60);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(
this->J49Ku->VecData)[i]=-(this->J49Ku->VecData)[i];}}}
for(i=(0x258+3496-0x1000);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]
){if(((fabs((this->p0GMV->VecData)[i]-(this->gCgoK->VecData)[i])>(wDW1M*fabs((
this->p0GMV->VecData)[i])))&&(T3_4Z>=dd74u))||((fabs((this->p0GMV->VecData)[i]-(
this->tpjD5->VecData)[i])>(wDW1M*fabs((this->p0GMV->VecData)[i])))&&(_AHiK>=
dd74u))||((fabs((this->p0GMV->VecData)[i]-(this->KSW1w->VecData)[i])>(wDW1M*fabs
((this->p0GMV->VecData)[i])))&&(okPrc>=dd74u))||((fabs((this->p0GMV->VecData)[i]
-(this->J49Ku->VecData)[i])>(wDW1M*fabs((this->p0GMV->VecData)[i])))&&(RnMR2>=
dd74u))){vhwls=false;break;}}}}else{vhwls=false;}if(vhwls){*Mk66j=*(this->p0GMV)
;}else{Mk66j->Set(0.0);}this->XAnjF;if(this->AEz_Y(*(this->Ijlxn))==
(0x1918+1904-0x2088)){Mk66j->Set(0.0);vhwls=true;}return(vhwls);}
