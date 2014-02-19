






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLThreadControl.h>
#include <TypeIVRMLStep1IntermediateTimeProfiles.h>
#include <TypeIVRMLStep1Profiles.h>
#include <TypeIVRMLStep1RootFunctions.h>
#include <TypeIVRMLQuicksort.h>
#include <TypeIVRMLStep1Decisions.h>
#include <TypeIVRMLDecisionTree1A.h>
#include <TypeIVRMLDecisionTree1B1.h>
#include <TypeIVRMLDecisionTree1B2.h>
#include <TypeIVRMLDecisionTree1B3.h>
#include <TypeIVRMLDecisionTree1C.h>
#include <TypeIVRMLABK.h>
#include <RMLPositionInputParameters.h>
#include <ReflexxesAPI.h>
#ifdef sh5HF
#include <pthread.h>
#endif
using namespace diBqY;

bool L62Wd::bsPjs(void){bool VD465=true;double X2k34=0.0,qS4_b=0.0,uGfZX=0.0,
sSwC1=0.0,F623Q=0.0;unsigned int i=(0xa36+6250-0x22a0),xMSLD=(0x218+6407-0x1b1f)
,A5isX=(0x13cf+289-0x14f0),Ihots=(0x19fb+1641-0x2064);this->eATG1->NAzWd(this->
mYHVu->SelectionVector->VecData,HKmbW::FsClZ);


while(this->eATG1->u8XtS(&xMSLD)){XUKfp(this,xMSLD);}
this->eATG1->s0_L_();for(i=(0x10f6+5115-0x24f1);i<this->NumberOfDOFs;i++){if((
this->mYHVu->SelectionVector->VecData)[i]){if((this->kbWCu->VecData)[i]){return(
L62Wd::s8ZpY);}if((this->oMyUa->VecData)[i]>X2k34){X2k34=(this->oMyUa->VecData)[
i];this->n9GnY=i;}}}
this->k64nO();
if((this->dBmd_)||(this->AEz_Y(*(this->Ijlxn))==(0xf5a+1045-0x136f))){this->
SynchronizationTime=X2k34;return(L62Wd::s8mhe);}

if(this->GXBSW){
this->GXBSW=KSiWK(this->q6cTW);}if((this->GXBSW)&&(fabs((this->q6cTW->VecData)[
this->n9GnY])>dd74u)){qS4_b=(this->mYHVu->MaxJerkVector->VecData)[this->n9GnY]/
fabs((this->q6cTW->VecData)[this->n9GnY]);uGfZX=(this->mYHVu->
MaxAccelerationVector->VecData)[this->n9GnY]/fabs((this->q6cTW->VecData)[this->
n9GnY]);sSwC1=(this->mYHVu->MaxVelocityVector->VecData)[this->n9GnY]/fabs((this
->q6cTW->VecData)[this->n9GnY]);for(i=(0x104b+5645-0x2658);i<this->NumberOfDOFs;
i++){if((this->Ijlxn->VecData)[i]){(this->kGPsN->VecData)[i]=0.0;(this->nqgQB->
VecData)[i]=fabs(qS4_b*(this->q6cTW->VecData)[i]);(this->NJ168->VecData)[i]=fabs
(uGfZX*(this->q6cTW->VecData)[i]);(this->cFLRO->VecData)[i]=fabs(sSwC1*(this->
q6cTW->VecData)[i]);if(((this->nqgQB->VecData)[i]>((this->mYHVu->MaxJerkVector->
VecData)[i]*(1.0+DLNWm)+dd74u))||((this->NJ168->VecData)[i]>((this->mYHVu->
MaxAccelerationVector->VecData)[i]*(1.0+DLNWm)+dd74u))||((this->cFLRO->VecData)[
i]>((this->mYHVu->MaxVelocityVector->VecData)[i]*(1.0+DLNWm)+dd74u))){this->
GXBSW=false;break;}}}}else{this->GXBSW=false;}if(this->GXBSW){*(this->jJSbp)=*(
this->mYHVu->CurrentPositionVector);*(this->KSW1w)=*(this->mYHVu->
CurrentVelocityVector);*(this->tpjD5)=*(this->mYHVu->CurrentAccelerationVector);
*(this->RNX4p)=*(this->mYHVu->TargetPositionVector);*(this->J49Ku)=*(this->mYHVu
->TargetVelocityVector);
for(i=(0x417+4821-0x16ec);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]
){if((i5gUI((this->RNX4p->VecData)[i]-(this->jJSbp->VecData)[i])!=i5gUI((this->
RNX4p->VecData)[this->n9GnY]-(this->jJSbp->VecData)[this->n9GnY]))){tvfxt(&((
this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[
i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]));}}}
for(i=(0x1098+3741-0x1f35);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i
]){zxODs(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->
KSW1w->VecData)[i]),&((this->tpjD5->VecData)[i]),(this->nqgQB->VecData)[i],(this
->NJ168->VecData)[i],(this->cFLRO->VecData)[i],this->J8wsd.
BehaviorIfInitialStateBreachesConstraints,&((this->RNX4p->VecData)[i]),&((this->
J49Ku->VecData)[i]));}}switch((this->LOh9b->VecData)[this->n9GnY]){case zfD15:
case XnpQc:for(i=(0xb36+3799-0x1a0d);i<this->NumberOfDOFs;i++){if((this->Ijlxn->
VecData)[i]){if((this->tpjD5->VecData)[i]<0.0){tvfxt(&((this->jJSbp->VecData)[i]
),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[i]),&((this->RNX4p->
VecData)[i]),&((this->J49Ku->VecData)[i]));}if((this->LOh9b->VecData)[this->
n9GnY]==XnpQc){
AFNhq(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->
VecData)[i]),&((this->tpjD5->VecData)[i]),(this->nqgQB->VecData)[i]);tvfxt(&((
this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[
i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]));}
if(!n6URU(&eHpM4,&wEEQc,&FW1Ej,(this->tpjD5->VecData)[i],(this->NJ168->VecData)[
i],(this->KSW1w->VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i]
,(this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->nqgQB->VecData)[i]))
{this->GXBSW=false;break;}}}break;case tmFJN:case pPyJE:for(i=
(0xc40+1556-0x1254);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if((
this->tpjD5->VecData)[i]<0.0){tvfxt(&((this->jJSbp->VecData)[i]),&((this->KSW1w
->VecData)[i]),&((this->tpjD5->VecData)[i]),&((this->RNX4p->VecData)[i]),&((this
->J49Ku->VecData)[i]));}if((this->LOh9b->VecData)[this->n9GnY]==pPyJE){
AFNhq(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->
VecData)[i]),&((this->tpjD5->VecData)[i]),(this->nqgQB->VecData)[i]);tvfxt(&((
this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[
i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]));}
if(!n6URU(&WP2Bw,&O_tX1,&MWTWT,(this->tpjD5->VecData)[i],(this->NJ168->VecData)[
i],(this->KSW1w->VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i]
,(this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->nqgQB->VecData)[i]))
{this->GXBSW=false;break;}}}break;case X4fbV:case nlydG:for(i=
(0xb7f+2149-0x13e4);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if((
this->tpjD5->VecData)[i]<0.0){tvfxt(&((this->jJSbp->VecData)[i]),&((this->KSW1w
->VecData)[i]),&((this->tpjD5->VecData)[i]),&((this->RNX4p->VecData)[i]),&((this
->J49Ku->VecData)[i]));}if((this->LOh9b->VecData)[this->n9GnY]==nlydG){
AFNhq(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->
VecData)[i]),&((this->tpjD5->VecData)[i]),(this->nqgQB->VecData)[i]);tvfxt(&((
this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[
i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]));}
if(!n6URU(&_WRKy,&ni1W7,&tlbcQ,(this->tpjD5->VecData)[i],(this->NJ168->VecData)[
i],(this->KSW1w->VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i]
,(this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->nqgQB->VecData)[i]))
{this->GXBSW=false;break;}}}break;case bC_iO:case V1Tsd:for(i=
(0x1089+3740-0x1f25);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if(
(this->tpjD5->VecData)[i]<0.0){tvfxt(&((this->jJSbp->VecData)[i]),&((this->KSW1w
->VecData)[i]),&((this->tpjD5->VecData)[i]),&((this->RNX4p->VecData)[i]),&((this
->J49Ku->VecData)[i]));}if((this->LOh9b->VecData)[this->n9GnY]==V1Tsd){
AFNhq(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->
VecData)[i]),&((this->tpjD5->VecData)[i]),(this->nqgQB->VecData)[i]);tvfxt(&((
this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[
i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]));}
if(!n6URU(&nNN9F,&v2SZG,&AkYML,(this->tpjD5->VecData)[i],(this->NJ168->VecData)[
i],(this->KSW1w->VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i]
,(this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->nqgQB->VecData)[i]))
{this->GXBSW=false;break;}}}break;case j5_G9:case OAWPr:for(i=
(0x1733+3104-0x2353);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if(
(this->tpjD5->VecData)[i]<0.0){tvfxt(&((this->jJSbp->VecData)[i]),&((this->KSW1w
->VecData)[i]),&((this->tpjD5->VecData)[i]),&((this->RNX4p->VecData)[i]),&((this
->J49Ku->VecData)[i]));}if((this->LOh9b->VecData)[this->n9GnY]==OAWPr){
AFNhq(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->
VecData)[i]),&((this->tpjD5->VecData)[i]),(this->nqgQB->VecData)[i]);tvfxt(&((
this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[
i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]));}
if(!HUFN1((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->nqgQB->VecData)[i],false)){this->
GXBSW=false;break;}else{
if(NmEnT((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i],(this->nqgQB->
VecData)[i],true)){this->GXBSW=false;break;}else{
if(!AD6q3((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i],(this->jJSbp->
VecData)[i],(this->RNX4p->VecData)[i],(this->nqgQB->VecData)[i],false)){this->
GXBSW=false;break;}}}}}break;case YPfXT:case R_UAP:for(i=(0x11c5+2547-0x1bb8);i<
this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if((this->tpjD5->VecData)[
i]<0.0){tvfxt(&((this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this
->tpjD5->VecData)[i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]))
;}if((this->LOh9b->VecData)[this->n9GnY]==R_UAP){
AFNhq(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->
VecData)[i]),&((this->tpjD5->VecData)[i]),(this->nqgQB->VecData)[i]);tvfxt(&((
this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[
i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]));}
if(!HUFN1((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->nqgQB->VecData)[i],false)){this->
GXBSW=false;break;}else{
if(!NmEnT((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i],(this->nqgQB->
VecData)[i],false)){this->GXBSW=false;break;}else{
if(!ZkBn3((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i],(this->jJSbp->
VecData)[i],(this->RNX4p->VecData)[i],(this->nqgQB->VecData)[i],false)){this->
GXBSW=false;break;}}}}}break;case CytxK:case st497:for(i=(0x4d5+6149-0x1cda);i<
this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if((this->tpjD5->VecData)[
i]<0.0){tvfxt(&((this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this
->tpjD5->VecData)[i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]))
;}if((this->LOh9b->VecData)[this->n9GnY]==st497){
AFNhq(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->
VecData)[i]),&((this->tpjD5->VecData)[i]),(this->nqgQB->VecData)[i]);tvfxt(&((
this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[
i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]));}
if(HUFN1((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->nqgQB->VecData)[i],true)){this->
GXBSW=false;break;}else{
if(M3TyU((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i],(this->nqgQB->
VecData)[i],true)){this->GXBSW=false;break;}else{
if(!IuCbq((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i],(this->jJSbp->
VecData)[i],(this->RNX4p->VecData)[i],(this->nqgQB->VecData)[i],false)){this->
GXBSW=false;break;}}}}}break;case IflJS:case _bCK3:for(i=(0xba2+2259-0x1475);i<
this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if((this->tpjD5->VecData)[
i]<0.0){tvfxt(&((this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this
->tpjD5->VecData)[i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]))
;}if((this->LOh9b->VecData)[this->n9GnY]==_bCK3){
AFNhq(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->
VecData)[i]),&((this->tpjD5->VecData)[i]),(this->nqgQB->VecData)[i]);tvfxt(&((
this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[
i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]));}
if(HUFN1((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->nqgQB->VecData)[i],true)){this->
GXBSW=false;break;}else{
if(!M3TyU((this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i],(this->nqgQB->
VecData)[i],false)){this->GXBSW=false;break;}else{
if(!GKvMJ((this->tpjD5->VecData)[i],(this->KSW1w->VecData)[i],(this->cFLRO->
VecData)[i],(this->J49Ku->VecData)[i],(this->jJSbp->VecData)[i],(this->RNX4p->
VecData)[i],(this->nqgQB->VecData)[i],false)){this->GXBSW=false;break;}}}}}break
;case OyKu3:for(i=(0x129+8135-0x20f0);i<this->NumberOfDOFs;i++){if((this->Ijlxn
->VecData)[i]){if((this->tpjD5->VecData)[i]<0.0){tvfxt(&((this->jJSbp->VecData)[
i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[i]),&((this->RNX4p->
VecData)[i]),&((this->J49Ku->VecData)[i]));}
if(!n6URU(&tIQ8E,&pqZE_,&_aosC,(this->tpjD5->VecData)[i],(this->NJ168->VecData)[
i],(this->KSW1w->VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i]
,(this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->nqgQB->VecData)[i]))
{this->GXBSW=false;break;}}}break;case BEy1V:for(i=(0xe1d+1124-0x1281);i<this->
NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if((this->tpjD5->VecData)[i]<0.0
){tvfxt(&((this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5
->VecData)[i]),&((this->RNX4p->VecData)[i]),&((this->J49Ku->VecData)[i]));}
if(!n6URU(&WkhoZ,&P7bor,&Fgf1e,(this->tpjD5->VecData)[i],(this->NJ168->VecData)[
i],(this->KSW1w->VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i]
,(this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->nqgQB->VecData)[i]))
{this->GXBSW=false;break;}}}break;default:this->GXBSW=false;break;}}if(this->
GXBSW){this->LFcQx=(unsigned int)(this->LOh9b->VecData)[this->n9GnY];




switch(this->LFcQx){case zfD15:case XnpQc:for(i=(0x58+6604-0x1a24);i<this->
NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){FVGK6(&((this->kGPsN->VecData)[i
]),(this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->KSW1w->VecData)[i]
,(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i],(this->tpjD5->VecData)[i],(
this->NJ168->VecData)[i],(this->nqgQB->VecData)[i],&VD465);}}break;case tmFJN:
case pPyJE:for(i=(0xb4d+4551-0x1d14);i<this->NumberOfDOFs;i++){if((this->Ijlxn->
VecData)[i]){Hs4VR(&((this->kGPsN->VecData)[i]),(this->jJSbp->VecData)[i],(this
->RNX4p->VecData)[i],(this->KSW1w->VecData)[i],(this->cFLRO->VecData)[i],(this->
J49Ku->VecData)[i],(this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->
nqgQB->VecData)[i],&VD465);}}break;case X4fbV:case nlydG:for(i=(0xc8+1808-0x7d8)
;i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){Wvqqg(&((this->kGPsN->
VecData)[i]),(this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->KSW1w->
VecData)[i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i],(this->tpjD5->
VecData)[i],(this->NJ168->VecData)[i],(this->nqgQB->VecData)[i],&VD465);}}break;
case bC_iO:case V1Tsd:for(i=(0x1806+203-0x18d1);i<this->NumberOfDOFs;i++){if((
this->Ijlxn->VecData)[i]){STu9t(&((this->kGPsN->VecData)[i]),(this->jJSbp->
VecData)[i],(this->RNX4p->VecData)[i],(this->KSW1w->VecData)[i],(this->cFLRO->
VecData)[i],(this->J49Ku->VecData)[i],(this->tpjD5->VecData)[i],(this->NJ168->
VecData)[i],(this->nqgQB->VecData)[i],&VD465);}}break;case j5_G9:case OAWPr:for(
i=(0xf15+2970-0x1aaf);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){
l4KsC(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->
VecData)[i]),&((this->tpjD5->VecData)[i]),(this->cFLRO->VecData)[i],(this->NJ168
->VecData)[i],(this->nqgQB->VecData)[i]);NnK2z(&((this->kGPsN->VecData)[i]),(
this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->KSW1w->VecData)[i],(
this->J49Ku->VecData)[i],(this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(
this->nqgQB->VecData)[i],&VD465);}}break;case YPfXT:case R_UAP:for(i=
(0x7a+2559-0xa79);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){l4KsC(
&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->
VecData)[i]),&((this->tpjD5->VecData)[i]),(this->cFLRO->VecData)[i],(this->NJ168
->VecData)[i],(this->nqgQB->VecData)[i]);kVdy5(&((this->kGPsN->VecData)[i]),(
this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->KSW1w->VecData)[i],(
this->J49Ku->VecData)[i],(this->tpjD5->VecData)[i],(this->nqgQB->VecData)[i],&
VD465);}}break;case CytxK:case st497:for(i=(0xdd+5087-0x14bc);i<this->
NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){JCYJB(&((this->kGPsN->VecData)[i
]),&((this->jJSbp->VecData)[i]),&((this->KSW1w->VecData)[i]),&((this->tpjD5->
VecData)[i]),(this->cFLRO->VecData)[i],(this->nqgQB->VecData)[i]);NnK2z(&((this
->kGPsN->VecData)[i]),(this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this
->KSW1w->VecData)[i],(this->J49Ku->VecData)[i],(this->tpjD5->VecData)[i],(this->
NJ168->VecData)[i],(this->nqgQB->VecData)[i],&VD465);}}break;case IflJS:case 
_bCK3:for(i=(0x3a6+4321-0x1487);i<this->NumberOfDOFs;i++){if((this->Ijlxn->
VecData)[i]){JCYJB(&((this->kGPsN->VecData)[i]),&((this->jJSbp->VecData)[i]),&((
this->KSW1w->VecData)[i]),&((this->tpjD5->VecData)[i]),(this->cFLRO->VecData)[i]
,(this->nqgQB->VecData)[i]);kVdy5(&((this->kGPsN->VecData)[i]),(this->jJSbp->
VecData)[i],(this->RNX4p->VecData)[i],(this->KSW1w->VecData)[i],(this->J49Ku->
VecData)[i],(this->tpjD5->VecData)[i],(this->nqgQB->VecData)[i],&VD465);}}break;
case OyKu3:for(i=(0x83c+4751-0x1acb);i<this->NumberOfDOFs;i++){if((this->Ijlxn->
VecData)[i]){WvabL(&((this->kGPsN->VecData)[i]),(this->jJSbp->VecData)[i],(this
->RNX4p->VecData)[i],(this->KSW1w->VecData)[i],(this->cFLRO->VecData)[i],(this->
J49Ku->VecData)[i],(this->tpjD5->VecData)[i],(this->NJ168->VecData)[i],(this->
nqgQB->VecData)[i],&VD465);}}break;case BEy1V:for(i=(0x1097+3150-0x1ce5);i<this
->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){VqC7F(&((this->kGPsN->VecData)
[i]),(this->jJSbp->VecData)[i],(this->RNX4p->VecData)[i],(this->KSW1w->VecData)[
i],(this->cFLRO->VecData)[i],(this->J49Ku->VecData)[i],(this->tpjD5->VecData)[i]
,(this->NJ168->VecData)[i],(this->nqgQB->VecData)[i],&VD465);}}break;default:
this->GXBSW=false;break;}F623Q=0.0;Ihots=(0x12a1+843-0x15ec);for(i=
(0xf96+2728-0x1a3e);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){
F623Q+=(this->kGPsN->VecData)[i];Ihots++;}}if(Ihots==(0x200f+1741-0x26dc)){
return(L62Wd::s8mhe);}F623Q/=((double)Ihots);for(i=(0xfac+1403-0x1527);i<this->
NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if(fabs((this->kGPsN->VecData)[i
]-F623Q)>(dd74u+DLNWm*F623Q)){this->GXBSW=false;break;}}}}if(this->GXBSW){this->
SynchronizationTime=this->oMyUa->VecData[this->n9GnY];return(L62Wd::s8mhe);}

this->eATG1->NAzWd(this->Ijlxn->VecData,HKmbW::KZcT3);


while(this->eATG1->u8XtS(&xMSLD)){dj3ZS(this,xMSLD);}
this->eATG1->s0_L_();for(i=(0x13f6+1925-0x1b7b);i<this->NumberOfDOFs;i++){if((
this->Ijlxn->VecData)[i]){if((this->kbWCu->VecData)[i]){return(L62Wd::s8ZpY);}(
this->oMyUa->VecData)[i]+=El5fW;if((this->jWcz4->VecData)[i]<(this->oMyUa->
VecData)[i]){(this->jWcz4->VecData)[i]=(this->oMyUa->VecData)[i];}if((this->
iqugE->VecData)[i]<(this->jWcz4->VecData)[i]){(this->iqugE->VecData)[i]=(this->
jWcz4->VecData)[i]=((this->jWcz4->VecData)[i]+(this->iqugE->VecData)[i])*0.5;(
this->jWcz4->VecData)[i]-=El5fW;(this->iqugE->VecData)[i]+=El5fW;if((this->jWcz4
->VecData)[i]<(this->oMyUa->VecData)[i]){(this->oMyUa->VecData)[i]=(this->iqugE
->VecData)[i];(this->jWcz4->VecData)[i]=JCQ6Q;(this->iqugE->VecData)[i]=JCQ6Q;}}
if((this->iqugE->VecData)[i]<(this->oMyUa->VecData)[i]){(this->jWcz4->VecData)[i
]=JCQ6Q;(this->iqugE->VecData)[i]=JCQ6Q;}}}


for(i=(0x759+1800-0xe61);i<this->NumberOfDOFs;i++){if(!(this->Ijlxn->VecData)[i]
){(this->jWcz4->VecData)[i]=JCQ6Q;(this->iqugE->VecData)[i]=JCQ6Q;}(this->Jhv79
->VecData)[i]=(this->jWcz4->VecData)[i];(this->Jhv79->VecData)[i+this->
NumberOfDOFs]=(this->iqugE->VecData)[i];}


momGP((0xe3a+6330-0x26f4),((0xf4d+5131-0x2356)*this->NumberOfDOFs-
(0x13a+8870-0x23df)),&((this->Jhv79->VecData)[(0x5a9+2040-0xda1)]));
for(A5isX=(0x22c5+66-0x2307);A5isX<(0x5d3+5578-0x1b9b)*this->NumberOfDOFs;A5isX
++){if((this->Jhv79->VecData)[A5isX]>X2k34){break;}}this->SynchronizationTime=
X2k34;
while((sSxA4(this->SynchronizationTime,*(this->jWcz4),*(this->iqugE)))&&(A5isX<
(0xedb+3917-0x1e26)*this->NumberOfDOFs)){this->SynchronizationTime=(this->Jhv79
->VecData)[A5isX];A5isX++;}return(L62Wd::s8mhe);}

bool L62Wd::sSxA4(const double&GVV0z,const RMLDoubleVector&jxnfV,const 
RMLDoubleVector&eXgqi)const{unsigned int i;for(i=(0x13a2+1639-0x1a09);i<this->
NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){if(((jxnfV.VecData)[i]<GVV0z)&&(
GVV0z<(eXgqi.VecData)[i])){return(true);}}}return(false);}
