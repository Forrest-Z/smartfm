#ifndef __THREADED__H__
#define __THREADED__H__

#include <pthread.h>

class Threaded
{
public:
    Threaded();
    void startThread();
    virtual void run() = 0;

private:
    pthread_t tid_;
    bool started_;
    static void * runner(void *a);
};


#endif
