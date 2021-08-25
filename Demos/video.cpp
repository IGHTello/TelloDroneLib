#include <TelloDrone.h>
#include <iostream>

int main() {
	std::cout << "Please start an ffplay/ffmpeg listener with: 'ffplay -probesize 32 -sync ext -framerate 35 udp://127.0.0.1:9999' and press any key to continue" << std::endl;
	std::cin.ignore();

	TelloDrone drone;
	std::cout << "Connecting to the drone..." << std::endl;
	drone.wait_until_connected();
	std::cout << "Connected to the drone! waiting for a minute..." << std::endl;
	std::this_thread::sleep_for(std::chrono::minutes(1));

	std::cout << "Disconnecting..." << std::endl;
}
