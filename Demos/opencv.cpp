#include <TelloDrone.h>
#include <iostream>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

int main()
{
    Tello::Drone drone;
    std::cout << "Connecting to the drone..." << std::endl;
    drone.wait_until_connected();
    std::cout << "Connected to the drone!" << std::endl;

    cv::VideoCapture cap("udp://127.0.0.1:9999");
    if(!cap.isOpened()) {
        std::cerr << "Unable to open cv::VideoCapture, disconnecting..." << std::endl;
        return 1;
    }

    std::cout << "Displaying frames, press any key to terminate" << std::endl;

    cv::Mat frame;
    for(;;) {
        cap.read(frame);
        if(frame.empty()) {
            std::cerr << "Read blank frame, exiting" << std::endl;
            break;
        }

        cv::imshow("Live Feed", frame);
        if(cv::waitKey(0) >= 0)
            break;
    }

    std::cout << "Disconnecting..." << std::endl;
}
