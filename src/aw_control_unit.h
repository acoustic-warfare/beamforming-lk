//
// Created by janne on 2024-07-04.
//

#ifndef BEAMFORMER_AW_CONTROL_UNIT_H
#define BEAMFORMER_AW_CONTROL_UNIT_H


#include <vector>
#include <libgpsmm.h>
#include <condition_variable>
#include <wara_ps_client.h>
#include "awpu.h"
#include "WaraPS/target_handler.h"
#include "rtmp.hpp"

class AWControlUnit {
private:
    std::vector<AWProcessingUnit> processingUnits;
    WaraPSClient client_;
    gps_data_t gpsData_{};
    std::thread data_thread_;

    std::vector<AWProcessingUnit> awpus_;

    TargetHandler targetHandler_;

    bool usingGps_ = false;
    bool usingWaraPS_ = false;

    std::condition_variable pausedCV_{};
    std::mutex pauseMutex_;

    bool paused_ = false;

    RtmpStreamer streamer;

    void publishData();
public:
    AWControlUnit();
    AWControlUnit(const AWControlUnit&) = delete;
    AWControlUnit operator=(const AWControlUnit&) = delete;


    /**
     * Spools up the AW Control unit and blocks the current thread until it finished.
     */
    void Start();

};


#endif //BEAMFORMER_AW_CONTROL_UNIT_H
