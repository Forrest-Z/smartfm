





























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLMath.h>
#include <TypeIVRMLDefinitions.h>
#include <TypeIVRMLThreadControl.h>
#include <TypeIVRMLDecisionTree2.h>
#include <TypeIVRMLStep2WithoutSynchronization.h>
#include <RMLPositionInputParameters.h>
#include <ReflexxesAPI.h>
#ifdef sh5HF
#include <pthread.h>
#endif
using namespace diBqY;

bool L62Wd::OL4RJ(void){unsigned int xMSLD=(0x6ea+3263-0x13a9),i=
(0x227d+989-0x265a);if((this->GXBSW)&&(!this->giFax)){

this->CsimU();}else{if((this->dBmd_)||(this->giFax)){for(i=(0xd51+5371-0x224c);i
<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){UWt0U((this->mYHVu->
CurrentPositionVector->VecData)[i],(this->mYHVu->CurrentVelocityVector->VecData)
[i],(this->mYHVu->CurrentAccelerationVector->VecData)[i],(this->mYHVu->
MaxJerkVector->VecData)[i],(this->mYHVu->MaxAccelerationVector->VecData)[i],(
this->mYHVu->MaxVelocityVector->VecData)[i],(this->mYHVu->TargetPositionVector->
VecData)[i],(this->mYHVu->TargetVelocityVector->VecData)[i],(this->sxkd9->
VecData)[i],(this->LOh9b->VecData)[i],this->J8wsd.
BehaviorIfInitialStateBreachesConstraints,&((this->Polynomials)[i]));}}}else{
this->eATG1->NAzWd(this->Ijlxn->VecData,HKmbW::Bbl2Z);


while(this->eATG1->u8XtS(&xMSLD)){CwpP6(this,xMSLD);}
this->eATG1->s0_L_();}}for(i=(0xbcc+5869-0x22b9);i<this->NumberOfDOFs;i++){if((
this->Ijlxn->VecData)[i]){if((this->kbWCu->VecData)[i]){return(L62Wd::s8ZpY);}}}
return(L62Wd::s8mhe);}
