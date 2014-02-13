






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLDefinitions.h>


void L62Wd::gRxRh(void){unsigned int i=(0x695+2687-0x1114);this->xNsXi=false;for
(i=(0xe7a+2361-0x17b3);i<this->NumberOfDOFs;i++){if((this->mYHVu->
SelectionVector->VecData)[i]){if((this->mYHVu->MaxJerkVector->VecData)[i]<MyU2U)
{(this->B8rSd->VecData)[i]=MyU2U/(this->mYHVu->MaxJerkVector->VecData)[i];this->
xNsXi=true;continue;}else{if((this->mYHVu->MaxAccelerationVector->VecData)[i]<
MyU2U){(this->B8rSd->VecData)[i]=MyU2U/(this->mYHVu->MaxAccelerationVector->
VecData)[i];this->xNsXi=true;continue;}else{if((this->mYHVu->MaxVelocityVector->
VecData)[i]<MyU2U){(this->B8rSd->VecData)[i]=MyU2U/(this->mYHVu->
MaxVelocityVector->VecData)[i];this->xNsXi=true;continue;}else{if((this->mYHVu->
MaxVelocityVector->VecData)[i]>wO433){(this->B8rSd->VecData)[i]=wO433/(this->
mYHVu->MaxVelocityVector->VecData)[i];this->xNsXi=true;}else{if((this->mYHVu->
MaxAccelerationVector->VecData)[i]>wO433){(this->B8rSd->VecData)[i]=wO433/(this
->mYHVu->MaxAccelerationVector->VecData)[i];this->xNsXi=true;}else{if((this->
mYHVu->MaxJerkVector->VecData)[i]>wO433){(this->B8rSd->VecData)[i]=wO433/(this->
mYHVu->MaxJerkVector->VecData)[i];this->xNsXi=true;}else{(this->B8rSd->VecData)[
i]=1.0;continue;}}}}}}if(((this->mYHVu->MaxJerkVector->VecData)[i]*(this->B8rSd
->VecData)[i])<MyU2U){(this->B8rSd->VecData)[i]*=MyU2U/((this->mYHVu->
MaxJerkVector->VecData)[i]*(this->B8rSd->VecData)[i]);continue;}if(((this->mYHVu
->MaxAccelerationVector->VecData)[i]*(this->B8rSd->VecData)[i])<MyU2U){(this->
B8rSd->VecData)[i]*=MyU2U/((this->mYHVu->MaxAccelerationVector->VecData)[i]*(
this->B8rSd->VecData)[i]);continue;}if(((this->mYHVu->MaxVelocityVector->VecData
)[i]*(this->B8rSd->VecData)[i])<MyU2U){(this->B8rSd->VecData)[i]*=MyU2U/((this->
mYHVu->MaxVelocityVector->VecData)[i]*(this->B8rSd->VecData)[i]);continue;}}else
{(this->B8rSd->VecData)[i]=1.0;}}}

void L62Wd::JbkXj(void){unsigned int i=(0x1e62+1252-0x2346);if(this->xNsXi){for(
i=(0x10b1+2647-0x1b08);i<this->NumberOfDOFs;i++){if((this->mYHVu->
SelectionVector->VecData)[i]){(this->mYHVu->CurrentPositionVector->VecData)[i]*=
(this->B8rSd->VecData)[i];(this->mYHVu->CurrentVelocityVector->VecData)[i]*=(
this->B8rSd->VecData)[i];(this->mYHVu->CurrentAccelerationVector->VecData)[i]*=(
this->B8rSd->VecData)[i];(this->mYHVu->MaxVelocityVector->VecData)[i]*=(this->
B8rSd->VecData)[i];(this->mYHVu->MaxAccelerationVector->VecData)[i]*=(this->
B8rSd->VecData)[i];(this->mYHVu->MaxJerkVector->VecData)[i]*=(this->B8rSd->
VecData)[i];(this->mYHVu->TargetPositionVector->VecData)[i]*=(this->B8rSd->
VecData)[i];(this->mYHVu->TargetVelocityVector->VecData)[i]*=(this->B8rSd->
VecData)[i];}}}}

void L62Wd::G_CFs(RMLPositionOutputParameters*YfM6p)const{unsigned int i=
(0xa01+6122-0x21eb),j=(0xe0b+3388-0x1b47);if(this->xNsXi){for(i=
(0x1cb9+1534-0x22b7);i<this->NumberOfDOFs;i++){if((this->Ijlxn->VecData)[i]){(
YfM6p->NewPositionVector->VecData)[i]/=(this->B8rSd->VecData)[i];(YfM6p->
NewVelocityVector->VecData)[i]/=(this->B8rSd->VecData)[i];(YfM6p->
NewAccelerationVector->VecData)[i]/=(this->B8rSd->VecData)[i];(YfM6p->
MinPosExtremaPositionVectorOnly->VecData)[i]/=(this->B8rSd->VecData)[i];(YfM6p->
MaxPosExtremaPositionVectorOnly->VecData)[i]/=(this->B8rSd->VecData)[i];for(j=
(0xcc0+1813-0x13d5);j<this->NumberOfDOFs;j++){(((YfM6p->
MinPosExtremaPositionVectorArray)[i])->VecData)[j]/=(this->B8rSd->VecData)[i];((
(YfM6p->MinPosExtremaVelocityVectorArray)[i])->VecData)[j]/=(this->B8rSd->
VecData)[i];(((YfM6p->MinPosExtremaAccelerationVectorArray)[i])->VecData)[j]/=(
this->B8rSd->VecData)[i];(((YfM6p->MaxPosExtremaPositionVectorArray)[i])->
VecData)[j]/=(this->B8rSd->VecData)[i];(((YfM6p->
MaxPosExtremaVelocityVectorArray)[i])->VecData)[j]/=(this->B8rSd->VecData)[i];((
(YfM6p->MaxPosExtremaAccelerationVectorArray)[i])->VecData)[j]/=(this->B8rSd->
VecData)[i];}}}}}
