#include <stdlib.h>
#include <iostream>
#include <limits>

#include <cstdio>

#include "PassengerComm.h"


void DummyPassengerComm::waitForPassengerInAtPickup()
{
    std::cout <<"At pickup station. Waiting 3 seconds for passenger." <<std::endl;
    ::sleep(3);
    std::cout <<"OK, let's go." <<std::endl;
}

void DummyPassengerComm::waitForPassengerOutAtDropoff()
{
    std::cout <<"At dropoff station. Waiting 3 seconds for passenger to exit." <<std::endl;
    ::sleep(3);
    std::cout <<"Please stand clear from the car, it may move." <<std::endl;
}




void SimplePassengerComm::waitForPassengerInAtPickup()
{
    std::cout <<"At pickup station. Waiting for passenger..." <<std::endl;
    std::cout <<"Press ENTER when you are ready.";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    std::cout <<"OK, let's go." <<std::endl;
}

void SimplePassengerComm::waitForPassengerOutAtDropoff()
{
    std::cout <<"At dropoff station. Waiting for passenger to exit..." <<std::endl;
    std::cout <<"Press ENTER when you are out of the car.";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
    std::cout <<"Please stand clear from the car, as it may move at any time now." <<std::endl;
    sleep(5);
}
