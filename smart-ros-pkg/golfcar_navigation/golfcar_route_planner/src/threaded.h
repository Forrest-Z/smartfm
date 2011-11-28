#ifndef __THREADED__H__
#define __THREADED__H__

#include <pthread.h>
#include <ros/ros.h>

class Threaded
{
public:
    virtual void run() = 0;

    void startThread()
    {
        pthread_create(&tid_, 0, runner, (void *)this);
    }

private:
    pthread_t tid_;

    static void * runner(void *a)
    {
        Threaded & thread( *((Threaded *)a) );
        while( ros::ok() )
            thread.run();
        return (void *)0;
    }
};


#endif
