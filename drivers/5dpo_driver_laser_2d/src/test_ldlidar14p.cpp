//
// Created by afonsotmm on 10/25/26.
//
#include "sdpo_driver_laser_2d/LDLIDAR14P.h"

#include <chrono>
#include <iostream>
#include <thread>

int main()
{
    sdpo_driver_laser_2d::LDLIDAR14P lidar;

    lidar.setSerialPortParam("/dev/ttyUSB0", 230400);

    lidar.setPubLaserData([&lidar]() {
        std::cout
            << "Processed scan: "
            << lidar.data_count
            << " points"
            << std::endl;

        if (lidar.data_count > 0) {
            std::cout
                << "First point: "
                << lidar.dist_data[0]
                << " m | "
                << lidar.ang_data[0]
                << " rad"
                << std::endl;
        }
    });

    if (!lidar.initialize()) {
        return 1;
    }

    lidar.start();

    std::this_thread::sleep_for(
        std::chrono::seconds(10));

    lidar.stop();

    return 0;
}