#include <TelloDrone.h>
#include <iostream>

int main()
{
    Tello::Drone drone;
    drone.wait_until_connected();
    std::cout << "Connected to the drone! disconnecting..." << std::endl;
}
