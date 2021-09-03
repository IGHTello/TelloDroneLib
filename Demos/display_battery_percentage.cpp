#include <TelloDrone.h>
#include <iostream>

int main()
{
    Tello::Drone drone;
    drone.wait_until_connected();
    std::cout << "Connected to the drone! Waiting for 100ms..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "Current battery percentage: " << (int)drone.get_flight_data().battery_percentage << "%, Disconnecting..." << std::endl;
}
