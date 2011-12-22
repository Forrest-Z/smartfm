#include <stdexcept>
#include <pthread.h>
#include "Threaded.h"


Threaded::Threaded() : started_(false)
{

}


void Threaded::startThread()
{
    if( ! started_ )
    {
        if( pthread_create(&tid_, 0, runner, (void *)this)==0 )
            started_ = true;
        else
            throw std::runtime_error("Could not start the thread");
    }
}


void * Threaded::runner(void *a)
{
    Threaded & thread( *((Threaded *)a) );

    while( true )
        thread.run();

    return (void *)0;
}
