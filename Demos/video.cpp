#include <TelloDrone.h>
#include <iostream>

int main() {
	TelloDrone drone;
	drone.wait_until_connected();
	std::cout << "Connected to the drone! waiting for a minute..." << std::endl;
	std::this_thread::sleep_for(std::chrono::minutes(1));
	std::cout << "Disconnecting..." << std::endl;
}
