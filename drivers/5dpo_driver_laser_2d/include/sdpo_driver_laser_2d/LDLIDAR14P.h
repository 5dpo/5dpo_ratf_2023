//
// Created by afonsotmm on 10/24/26.
//

#ifndef INC_5DPO_RAFT_LDLIDAR14P_H
#define INC_5DPO_RAFT_LDLIDAR14P_H

#include "sdpo_driver_laser_2d/SdpoDriverLaser2D.h"
#include "ldlidar_driver/ldlidar_driver_linux.h"

#include <chrono>
#include <functional>
#include <iostream>
#include <thread>
#include <atomic>

namespace sdpo_driver_laser_2d {

    const std::string kSdpoDriverLaser2DLDLIDAR14PStr = "ldlidar14p";

    class LDLIDAR14P : public SdpoDriverLaser2D {
    private:
        ldlidar::LDLidarDriverLinuxInterface *ldlidar_drv_{nullptr};
        ldlidar::LDType lidar_type_ = ldlidar::LDType::LD_14P;

        std::atomic<bool> running_{false};
        std::thread acquisition_thread_;

        void acquisitionLoop();
        void processScan(const ldlidar::Points2D& points);

    public:
        LDLIDAR14P() = default;
        ~LDLIDAR14P();

        bool initialize(const bool dbg = false) override;
        void start()   override;
        void stop()    override;
        void restart() override;

        // not used methods
        void processSerialData(unsigned char& ch) override;
        void processLaserData() override;
        void printPkgDataInfo() const override;
        void printLaserData() const override;

    };
}

#endif //INC_5DPO_RAFT_LDLIDAR14P_H
