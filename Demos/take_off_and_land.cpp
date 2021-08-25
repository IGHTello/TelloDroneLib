#include <TelloDrone.h>
#include <iostream>

int main()
{
    Tello::Drone drone;
    std::cout << "Connecting to the drone..." << std::endl;
    drone.wait_until_connected();
    std::cout << "Connected to the drone! Taking off..." << std::endl;
    drone.take_off();
    std::cout << "Waiting 10 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::cout << "Landing..." << std::endl;
    drone.land();
    std::cout << "Disconnecting..." << std::endl;
}
