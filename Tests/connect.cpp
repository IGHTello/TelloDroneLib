#include <TelloDrone.h>
#include <iostream>

int main() {
	TelloDrone drone;
	drone.wait_until_connected();
	std::cout << "Connected to the drone! disconnecting..." << std::endl;
}
