#include <TelloDrone.h>
#include <iostream>

int main()
{
    Tello::Drone drone;
    drone.wait_until_connected();
    std::cout << "Connected to the drone! Current flight height limit: " << drone.get_flight_height_limit() << std::endl;
    std::cout << "Enter new flight height limit: ";
    u16 new_flight_height_limit = 10;
    std::cin >> new_flight_height_limit;
    drone.set_flight_height_limit(new_flight_height_limit);
    std::cout << "Flight height limit updated! disconnecting..." << std::endl;
}
