//
// Created by janne on 2024-07-17.
//

#ifndef BEAMFORMER_TARGET_H
#define BEAMFORMER_TARGET_H

#include <wara_ps_client.h>
#include <Eigen/Core>

namespace WaraPS {
    class Target {
    private:
        std::mutex rw_mutex;

        WaraPSClient client;
        Eigen::Vector2d position_;
        gps_data_t lk_position_;
        std::shared_ptr<bool> running = std::make_shared<bool>(false);
        std::thread data_thread_;

        nlohmann::json PositionToGPS() const;

    public:
        Target(Eigen::Vector2d position, const gps_data_t &lk_position, int index);
        void Start();
        void Destroy();

        void SetPosition(const Eigen::Vector2d &position, const gps_data_t &lk_position) {
            rw_mutex.lock();

            position_ = position;
            lk_position_ = lk_position;

            rw_mutex.unlock();
        }
    };

}
#endif //BEAMFORMER_TARGET_H
