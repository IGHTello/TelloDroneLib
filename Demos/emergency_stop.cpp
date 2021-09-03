#include <TelloDrone.h>
#include <iostream>

int main()
{
    Tello::Drone drone;
    std::cout << "Connecting to the drone..." << std::endl;
    drone.wait_until_connected();
    std::cout << "Landing..." << std::endl;
    drone.land();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    drone.land();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    drone.land();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    std::cout << "Disconnecting..." << std::endl;
    return 0;
}
