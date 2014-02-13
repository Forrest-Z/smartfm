





























#ifndef sTm2c
#define sTm2c
#include <string.h>
#include <math.h>
#ifdef sh5HF
#include <pthread.h>
#endif













































class HKmbW{public:




















HKmbW(const unsigned int&Ozc3Y,const unsigned int&Xmzp1){unsigned int i=
(0x1b1b+1721-0x21d4);this->EACYZ=Ozc3Y;this->NumberOfDOFs=Xmzp1;this->OCCGO=
HKmbW::m7fM6;this->JmQSP=(unsigned int)floor((float)this->NumberOfDOFs/(float)
this->EACYZ)+(0x1c1d+2198-0x24b2);this->k_2OJ=(0xe3a+2064-0x164a);this->begFk=
(0x1a87+2609-0x24b8);this->ph8BO=new unsigned int[this->EACYZ];this->qFKDw=new 
unsigned int*[this->EACYZ];for(i=(0x43f+544-0x65f);i<this->EACYZ;i++){(this->
qFKDw)[i]=new unsigned int[this->JmQSP];memset((this->qFKDw)[i],
(0x133+6488-0x1a8b),(this->JmQSP*sizeof(unsigned int)));(this->ph8BO)[i]=
(0x225c+355-0x23bf);}
#ifdef sh5HF
pthread_mutex_init(&(this->XtBZ5),NULL);pthread_cond_init(&(this->mBrOZ),NULL);
pthread_cond_init(&(this->ek03L),NULL);
#endif
}












HKmbW(const HKmbW&U5h61){unsigned int i=(0x1283+3437-0x1ff0);this->ph8BO=new 
unsigned int[U5h61.EACYZ];this->qFKDw=new unsigned int*[U5h61.EACYZ];for(i=
(0x468+5879-0x1b5f);i<this->EACYZ;i++){(this->qFKDw)[i]=new unsigned int[U5h61.
JmQSP];}*this=U5h61;}





~HKmbW(void){unsigned int i=(0x26a+7104-0x1e2a);for(i=(0x513+4753-0x17a4);i<this
->EACYZ;i++){delete[](this->qFKDw)[i];}delete[]this->qFKDw;delete[]this->ph8BO;
this->qFKDw=NULL;this->ph8BO=NULL;
#ifdef sh5HF		
pthread_mutex_destroy(&(this->XtBZ5));pthread_cond_destroy(&(this->mBrOZ));
pthread_cond_destroy(&(this->ek03L));
#endif
}








HKmbW&operator=(const HKmbW&U5h61){unsigned int i=(0x1533+4465-0x26a4);this->
EACYZ=U5h61.EACYZ;this->NumberOfDOFs=U5h61.NumberOfDOFs;this->OCCGO=U5h61.OCCGO;
this->JmQSP=U5h61.JmQSP;this->k_2OJ=U5h61.k_2OJ;this->begFk=U5h61.begFk;memcpy(
this->ph8BO,U5h61.ph8BO,(this->EACYZ*sizeof(unsigned int)));memcpy(this->qFKDw,
U5h61.qFKDw,(this->EACYZ*sizeof(unsigned int*)));for(i=(0x870+6410-0x217a);i<
this->EACYZ;i++){memcpy((this->qFKDw)[i],(U5h61.qFKDw)[i],(this->JmQSP*sizeof(
unsigned int)));}
#ifdef sh5HF
this->XtBZ5=U5h61.XtBZ5;this->mBrOZ=U5h61.mBrOZ;this->ek03L=U5h61.ek03L;
#endif
return(*this);}



















enum w2t9m{
m7fM6=(0x227+7702-0x203d),

FsClZ=(0x1e2+433-0x392),

KZcT3=(0x88+3772-0xf42),

Bbl2Z=(0x1182+2121-0x19c8),

ntGsr=(0x5e4+4372-0x16b9)};





























inline bool u8XtS(unsigned int*nt3jI){if((this->ph8BO)[(0x1142+2520-0x1b1a)]==
(0x728+7997-0x2665)){
#ifdef sh5HF		
pthread_mutex_lock(&(this->XtBZ5));
#endif
(this->k_2OJ)++;
#ifdef sh5HF			
pthread_mutex_unlock(&(this->XtBZ5));
#endif
return(false);}else{*nt3jI=((this->qFKDw)[(0x2d1+870-0x637)])[(this->ph8BO)[
(0x6cb+7013-0x2230)]-(0x9f6+464-0xbc5)];((this->ph8BO)[(0x9d2+4181-0x1a27)])--;
return(true);}}

































inline void W7W0i(const unsigned int&uwzvA,unsigned int*nt3jI,unsigned int*xSyqv
){*nt3jI=(this->qFKDw)[uwzvA][(this->ph8BO)[uwzvA]-(0x2ba+7114-0x1e83)];((this->
ph8BO)[uwzvA])--;*xSyqv=this->OCCGO;}
#ifdef sh5HF



















inline void X9lF6(const unsigned int&uwzvA){if((this->ph8BO)[uwzvA]==
(0x76d+2691-0x11f0)){pthread_mutex_lock(&(this->XtBZ5));(this->k_2OJ)++;
pthread_mutex_unlock(&(this->XtBZ5));pthread_cond_signal(&(this->mBrOZ));}
pthread_mutex_lock(&(this->XtBZ5));while(((this->ph8BO)[uwzvA]==
(0x105d+1746-0x172f))&&(this->OCCGO!=HKmbW::ntGsr)){pthread_cond_wait(&(this->
ek03L),&(this->XtBZ5));}pthread_mutex_unlock(&(this->XtBZ5));}
#endif











inline void s0_L_(void){
#ifdef sh5HF	
pthread_mutex_lock(&(this->XtBZ5));while(this->k_2OJ<this->begFk){
pthread_cond_wait(&(this->mBrOZ),&(this->XtBZ5));}pthread_mutex_unlock(&(this->
XtBZ5));
#endif
return;}









inline void taNV1(void){
#ifdef sh5HF	
pthread_mutex_lock(&(this->XtBZ5));this->OCCGO=HKmbW::ntGsr;pthread_mutex_unlock
(&(this->XtBZ5));pthread_cond_broadcast(&(this->ek03L));
#endif
return;}











inline bool eqru7(void){bool ZtHFL=false;
#ifdef sh5HF	
pthread_mutex_lock(&(this->XtBZ5));ZtHFL=(this->OCCGO==HKmbW::ntGsr);
pthread_mutex_unlock(&(this->XtBZ5));
#endif		
return(ZtHFL);}
































inline void NAzWd(const bool*SelectionVector,const unsigned int&xSyqv){unsigned 
int i=(0x6d2+6901-0x21c7),XAnjF=(0x689+3954-0x15fb),geNZM=(0x161f+427-0x17ca),
zD7o6=(0x79c+5123-0x1b9f),w2lxC=(0xfb7+1020-0x13b3),IINZg=(0x5a9+1517-0xb96);for
(i=(0x181d+1983-0x1fdc);i<this->NumberOfDOFs;i++){if(SelectionVector[i]){XAnjF++
;}}geNZM=(unsigned int)floor((float)XAnjF/(float)this->EACYZ);zD7o6=XAnjF-(geNZM
*this->EACYZ);
#ifdef sh5HF
pthread_mutex_lock(&(this->XtBZ5));
#endif
this->k_2OJ=(0x1da4+1127-0x220b);for(i=(0x1000+1756-0x16dc);i<this->EACYZ;i++){
if(zD7o6>(0x18e2+3404-0x262e)){(this->ph8BO)[i]=geNZM+(0x21fa+65-0x223a);zD7o6--
;}else{(this->ph8BO)[i]=geNZM;}}IINZg=(0x51a+939-0x8c5);w2lxC=
(0x1890+2686-0x230e);for(i=(0xea9+3290-0x1b83);i<this->NumberOfDOFs;i++){if(
SelectionVector[i]){(this->qFKDw)[IINZg][w2lxC]=i;w2lxC++;if(w2lxC==(this->ph8BO
)[IINZg]){w2lxC=(0x2b7+2718-0xd55);IINZg++;}}}this->OCCGO=xSyqv;if(XAnjF<this->
EACYZ){this->begFk=XAnjF;}else{this->begFk=this->EACYZ;}
#ifdef sh5HF		
pthread_mutex_unlock(&(this->XtBZ5));pthread_cond_broadcast(&(this->ek03L));
#endif
}









inline void PgiG8(void){
#ifdef sh5HF
pthread_mutex_lock(&(this->XtBZ5));while(this->k_2OJ<this->EACYZ-
(0x1efc+1411-0x247e)){pthread_cond_wait(&(this->mBrOZ),&(this->XtBZ5));}
pthread_mutex_unlock(&(this->XtBZ5));
#endif
return;}









unsigned int JmQSP;








unsigned int OCCGO;









unsigned int EACYZ;








unsigned int NumberOfDOFs;









unsigned int*ph8BO;









unsigned int**qFKDw;








unsigned int begFk;











unsigned int k_2OJ;
#ifdef sh5HF







pthread_mutex_t XtBZ5;






pthread_cond_t mBrOZ;






pthread_cond_t ek03L;
#endif
};
#endif

