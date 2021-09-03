#include <TelloDrone.h>
#include <iostream>

int main()
{
    Tello::Drone drone;
    drone.wait_until_connected();
    std::cout << "Connected to the drone!" << std::endl;
    while(true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "Current height: " << ((float)drone.get_flight_data().height) / 10 << "m" << std::endl;
    }
}
