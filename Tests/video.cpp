#include <TelloDrone.h>
#include <iostream>

int main() {
	TelloDrone drone;
	drone.wait_until_connected();
	std::cout << "Connected to the drone! waiting..." << std::endl;
	while(true)
		std::this_thread::sleep_for(std::chrono::seconds(1));
}
