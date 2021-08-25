#include <TelloDrone.h>
#include <iostream>

int main()
{
    Tello::Drone drone;
    std::cout << "Connecting to the drone..." << std::endl;
    drone.wait_until_connected();
    std::cout << "Connected to the drone! Taking off..." << std::endl;
    if (!drone.take_off()) {
        std::cerr << "Failed taking off! Disconnecting..." << std::endl;
        return 1;
    }
    std::cout << "Waiting 10 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(10));
    std::cout << "Landing..." << std::endl;
    if (!drone.land()) {
        std::cerr << "Failed landing! Disconnecting..." << std::endl;
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5)); // Drone ACKs land packet immediately, sleep for a couple of seconds of landing video
    std::cout << "Disconnecting..." << std::endl;
    return 0;
}
