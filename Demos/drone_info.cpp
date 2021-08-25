#include <TelloDrone.h>
#include <iostream>

int main()
{
    Tello::Drone drone;
    drone.wait_until_connected();
    std::cout << "Connected to the drone! Drone Info:" << std::endl;
    std::cout << "Firmware Version: " << drone.get_firmware_version() << std::endl;
    std::cout << "Loader Version: " << drone.get_loader_version() << std::endl;
    std::cout << "Bitrate: " << (int)drone.get_bitrate() << std::endl;
    std::cout << "Flight Height Limit: " << drone.get_flight_height_limit() << std::endl;
    std::cout << "Low Battery Warning: " << drone.get_low_battery_warning() << std::endl;
    std::cout << "Attitude Angle: " << drone.get_attitude_angle() << std::endl;
    std::cout << "Country Code: " << drone.get_country_code() << std::endl;
    std::cout << "SSID: " << drone.get_ssid() << std::endl;
    std::cout << "Activation Status: " << drone.get_activation_status() << std::endl;
    std::cout << "Disconnecting..." << std::endl;
}
