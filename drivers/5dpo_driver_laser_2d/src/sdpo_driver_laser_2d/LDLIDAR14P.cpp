//
// Created by afonsotmm on 10/24/26.
//

#include "sdpo_driver_laser_2d/LDLIDAR14P.h"

#include <iostream>
#include <ostream>

#include "sdpo_driver_laser_2d/utils.h"

// from https://github.com/ldrobotSensorTeam/ldlidar_ros/blob/master/src/main.cpp
namespace {
    uint64_t GetTimestamp(void) {
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp =
          std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
        auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
        return ((uint64_t)tmp.count());
    }
}

namespace sdpo_driver_laser_2d {

    LDLIDAR14P::~LDLIDAR14P() {
        stop();

        if (ldlidar_drv_ != nullptr) {

            ldlidar_drv_->Disconnect();

            ldlidar::LDLidarDriverLinuxInterface::Destory(
                ldlidar_drv_);

            ldlidar_drv_ = nullptr;
        }
    }

    bool LDLIDAR14P::initialize(const bool dbg) {
        ldlidar_drv_ = ldlidar::LDLidarDriverLinuxInterface::Create();
        if (ldlidar_drv_==nullptr) return false;

        ldlidar_drv_->RegisterGetTimestampFunctional(std::bind(&GetTimestamp));
        ldlidar_drv_->EnablePointCloudDataFilter(true);

        if (ldlidar_drv_->Connect(lidar_type_, serial_port_name_, baud_rate_)) {
            std::cout << "ldlidar serial connect is success" << std::endl;
        } else {
            std::cout << "ldlidar serial connect is fail" << std::endl;
            return false;
        }

        if (ldlidar_drv_->WaitLidarComm(3500)) {
            std::cout << "ldlidar communication is normal." << std::endl;
        } else {
            std::cout << "ldlidar communication is abnormal." << std::endl;
            return false;
        }

        return true;
    }

    void LDLIDAR14P::start() {
        if (running_) return;
        if (ldlidar_drv_ == nullptr) return;
        if (!ldlidar_drv_->Start()) {
            std::cout << "ldlidar start is fail" << std::endl;
            return;
        }

        std::cout << "ldlidar start is success" << std::endl;
        running_ = true;

        // start scans acquisition thread
        acquisition_thread_ = std::thread(&LDLIDAR14P::acquisitionLoop, this);
    }

    void LDLIDAR14P::stop() {
        if (!running_) return;
        running_ = false;


        if (acquisition_thread_.joinable()) acquisition_thread_.join();
        if (ldlidar_drv_!=nullptr) ldlidar_drv_->Stop();

        std::cout << "ldlidar stop is success" << std::endl;
    }

    void LDLIDAR14P::restart() {
        stop();
        start();
    }

    void LDLIDAR14P::acquisitionLoop()
    {
        ldlidar::Points2D laser_scan_points;

        while (running_) {

            switch (
                ldlidar_drv_->GetLaserScanData(
                    laser_scan_points, 1500))
            {
                case ldlidar::LidarStatus::NORMAL:

                    processScan(laser_scan_points);
                    break;

                case ldlidar::LidarStatus::DATA_TIME_OUT:

                    std::cerr
                        << "LDLiDAR data timeout."
                        << std::endl;
                    break;

                case ldlidar::LidarStatus::DATA_WAIT:
                    break;

                default:
                    break;
            }

            // Same acquisition rate used by the official ROS driver
            std::this_thread::sleep_for(
                std::chrono::milliseconds(100));
        }
    }

    void LDLIDAR14P::processScan(const ldlidar::Points2D& points)
    {
        if (dist_data.size() < points.size()) {
            dist_data.resize(points.size());
        }

        if (ang_data.size() < points.size()) {
            ang_data.resize(points.size());
        }

        data_count = 0;

        size_t zero_points = 0;
        size_t below_min = 0;
        size_t above_max = 0;

        for (const auto& point : points) {

            float distance =
                static_cast<float>(point.distance) / 1000.0f;

            float angle =
                normAngRad(
                    -point.angle * M_PIf32 / 180.0f
                );

            if (distance == 0.0f) {
                zero_points++;
            }

            if (dist_range_check_ && distance < dist_min_) {
                below_min++;
            }

            if (dist_range_check_ && distance > dist_max_) {
                above_max++;
            }

            bool is_sample_ok = true;

            if (dist_range_check_) {
                if (distance < dist_min_ ||
                    distance > dist_max_) {
                    is_sample_ok = false;
                    }
            }

            if (ang_range_check_) {
                if (angle < ang_min_ ||
                    angle > ang_max_) {
                    is_sample_ok = false;
                    }
            }

            if (is_sample_ok) {
                dist_data[data_count] = distance;
                ang_data[data_count] = angle;
                data_count++;
            }
        }

        std::cout
            << "Raw: " << points.size()
            << " | valid: " << data_count
            << " | zero: " << zero_points
            << " | below min: " << below_min
            << " | above max: " << above_max
            << std::endl;

        if (pubLaserData) {
            pubLaserData();
        }
    }

    // =======================
    //    not used methods
    // =======================
    void LDLIDAR14P::processSerialData(unsigned char& ch)
    {
        // serial communication is handled by the LDROBOT SDK.
    }

    void LDLIDAR14P::processLaserData()
    {
        // scan processing is handled by the LDROBOT SDK.
    }

    void LDLIDAR14P::printPkgDataInfo() const
    {
    }

    void LDLIDAR14P::printLaserData() const
    {
    }

}

