






























#include <TypeIVRMLPosition.h>
#include <TypeIVRMLDecisionTree1A.h>
#include <TypeIVRMLDecisionTree1B1.h>
#include <TypeIVRMLDecisionTree1B2.h>
#include <TypeIVRMLDecisionTree1B3.h>
#include <TypeIVRMLDecisionTree1C.h>
#include <TypeIVRMLDecisionTree2.h>
#include <TypeIVRMLThreadControl.h>
#include <TypeIVRMLMath.h>
#ifdef sh5HF
#include <pthread.h>
#endif
using namespace diBqY;
#ifdef sh5HF


void*L62Wd::w4FuL(void*b7Qd_){unsigned int xMSLD=(0x16ab+1943-0x1e42),OCCGO=
HKmbW::m7fM6,eaQ4p=(0x112+54-0x148);L62Wd*QALiy=(L62Wd*)b7Qd_;eaQ4p=QALiy->ZgdIh
;
pthread_mutex_lock(&(QALiy->QV5he));QALiy->Y6Opw=true;pthread_mutex_unlock(&(
QALiy->QV5he));pthread_cond_signal(&(QALiy->PvW1U));for(;;){QALiy->eATG1->X9lF6(
eaQ4p);if(QALiy->eATG1->eqru7()){break;}QALiy->eATG1->W7W0i(eaQ4p,&xMSLD,&OCCGO)
;if(OCCGO==HKmbW::FsClZ){XUKfp(QALiy,xMSLD);continue;}if(OCCGO==HKmbW::KZcT3){
dj3ZS(QALiy,xMSLD);continue;}if(OCCGO==HKmbW::Bbl2Z){CwpP6(QALiy,xMSLD);continue
;}}pthread_exit(NULL);return((void*)NULL);
}
#endif


void L62Wd::XUKfp(L62Wd*QALiy,unsigned int&xMSLD){(QALiy->kbWCu->VecData)[xMSLD]
=MuFYW((QALiy->mYHVu->CurrentPositionVector->VecData)[xMSLD],(QALiy->mYHVu->
CurrentVelocityVector->VecData)[xMSLD],(QALiy->mYHVu->CurrentAccelerationVector
->VecData)[xMSLD],(QALiy->mYHVu->MaxJerkVector->VecData)[xMSLD],(QALiy->mYHVu->
MaxAccelerationVector->VecData)[xMSLD],(QALiy->mYHVu->MaxVelocityVector->VecData
)[xMSLD],(QALiy->mYHVu->TargetPositionVector->VecData)[xMSLD],(QALiy->mYHVu->
TargetVelocityVector->VecData)[xMSLD],QALiy->J8wsd.
BehaviorIfInitialStateBreachesConstraints,&((QALiy->LOh9b->VecData)[xMSLD]),&((
QALiy->oMyUa->VecData)[xMSLD]),&((QALiy->sxkd9->VecData)[xMSLD]));}

void L62Wd::dj3ZS(L62Wd*QALiy,unsigned int&xMSLD){wqYyk mvnhR;(QALiy->jWcz4->
VecData)[xMSLD]=JCQ6Q;(QALiy->iqugE->VecData)[xMSLD]=JCQ6Q;mvnhR=P1Dlk((QALiy->
mYHVu->CurrentPositionVector->VecData)[xMSLD],(QALiy->mYHVu->
CurrentVelocityVector->VecData)[xMSLD],(QALiy->mYHVu->CurrentAccelerationVector
->VecData)[xMSLD],(QALiy->mYHVu->MaxJerkVector->VecData)[xMSLD],(QALiy->mYHVu->
MaxAccelerationVector->VecData)[xMSLD],(QALiy->mYHVu->MaxVelocityVector->VecData
)[xMSLD],(QALiy->mYHVu->TargetPositionVector->VecData)[xMSLD],(QALiy->mYHVu->
TargetVelocityVector->VecData)[xMSLD],QALiy->J8wsd.
BehaviorIfInitialStateBreachesConstraints);if(mvnhR==ycTr8){(QALiy->kbWCu->
VecData)[xMSLD]=Wcx0t((QALiy->mYHVu->CurrentPositionVector->VecData)[xMSLD],(
QALiy->mYHVu->CurrentVelocityVector->VecData)[xMSLD],(QALiy->mYHVu->
CurrentAccelerationVector->VecData)[xMSLD],(QALiy->mYHVu->MaxJerkVector->VecData
)[xMSLD],(QALiy->mYHVu->MaxAccelerationVector->VecData)[xMSLD],(QALiy->mYHVu->
MaxVelocityVector->VecData)[xMSLD],(QALiy->mYHVu->TargetPositionVector->VecData)
[xMSLD],(QALiy->mYHVu->TargetVelocityVector->VecData)[xMSLD],(QALiy->oMyUa->
VecData)[xMSLD],QALiy->J8wsd.BehaviorIfInitialStateBreachesConstraints,&((QALiy
->mlPqe->VecData)[xMSLD]),&((QALiy->jWcz4->VecData)[xMSLD]));}if(mvnhR==d9KnM){(
QALiy->kbWCu->VecData)[xMSLD]=z8RFA((QALiy->mYHVu->CurrentPositionVector->
VecData)[xMSLD],(QALiy->mYHVu->CurrentVelocityVector->VecData)[xMSLD],(QALiy->
mYHVu->CurrentAccelerationVector->VecData)[xMSLD],(QALiy->mYHVu->MaxJerkVector->
VecData)[xMSLD],(QALiy->mYHVu->MaxAccelerationVector->VecData)[xMSLD],(QALiy->
mYHVu->MaxVelocityVector->VecData)[xMSLD],(QALiy->mYHVu->TargetPositionVector->
VecData)[xMSLD],(QALiy->mYHVu->TargetVelocityVector->VecData)[xMSLD],(QALiy->
LOh9b->VecData)[xMSLD],QALiy->J8wsd.BehaviorIfInitialStateBreachesConstraints,&(
(QALiy->mlPqe->VecData)[xMSLD]),&((QALiy->jWcz4->VecData)[xMSLD]),&((QALiy->
iqugE->VecData)[xMSLD]));}
if(!((QALiy->kbWCu->VecData)[xMSLD])&&((QALiy->jWcz4->VecData)[xMSLD]!=JCQ6Q)){
if((QALiy->iqugE->VecData)[xMSLD]==JCQ6Q){(QALiy->kbWCu->VecData)[xMSLD]=sfoTf((
QALiy->mYHVu->CurrentPositionVector->VecData)[xMSLD],(QALiy->mYHVu->
CurrentVelocityVector->VecData)[xMSLD],(QALiy->mYHVu->CurrentAccelerationVector
->VecData)[xMSLD],(QALiy->mYHVu->MaxJerkVector->VecData)[xMSLD],(QALiy->mYHVu->
MaxAccelerationVector->VecData)[xMSLD],(QALiy->mYHVu->MaxVelocityVector->VecData
)[xMSLD],(QALiy->mYHVu->TargetPositionVector->VecData)[xMSLD],(QALiy->mYHVu->
TargetVelocityVector->VecData)[xMSLD],(QALiy->LOh9b->VecData)[xMSLD],(QALiy->
mlPqe->VecData)[xMSLD],QALiy->J8wsd.BehaviorIfInitialStateBreachesConstraints,&(
(QALiy->iqugE->VecData)[xMSLD]));}if(!(QALiy->kbWCu->VecData)[xMSLD]){(QALiy->
iqugE->VecData)[xMSLD]+=El5fW;(QALiy->jWcz4->VecData)[xMSLD]-=El5fW;}}else{(
QALiy->iqugE->VecData)[xMSLD]=JCQ6Q;}}

void L62Wd::CwpP6(L62Wd*QALiy,unsigned int&xMSLD){(QALiy->kbWCu->VecData)[xMSLD]
=r40xy((QALiy->mYHVu->CurrentPositionVector->VecData)[xMSLD],(QALiy->mYHVu->
CurrentVelocityVector->VecData)[xMSLD],(QALiy->mYHVu->CurrentAccelerationVector
->VecData)[xMSLD],(QALiy->mYHVu->MaxJerkVector->VecData)[xMSLD],(QALiy->mYHVu->
MaxAccelerationVector->VecData)[xMSLD],(QALiy->mYHVu->MaxVelocityVector->VecData
)[xMSLD],(QALiy->mYHVu->TargetPositionVector->VecData)[xMSLD],(QALiy->mYHVu->
TargetVelocityVector->VecData)[xMSLD],QALiy->SynchronizationTime,QALiy->J8wsd.
BehaviorIfInitialStateBreachesConstraints,&((QALiy->Polynomials)[xMSLD]));}
