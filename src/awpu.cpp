#include "awpu.h"

AWProcessingUnit::AWProcessingUnit(const std::string ip_address) : ip_address(ip_address) {
    this->pipeline = new Pipeline();
    //this->pipeline->connect(ip_address);
    this->running = false;
}

AWProcessingUnit::~AWProcessingUnit() {
    pipeline->disconnect();
    delete pipeline;
}

bool AWProcessingUnit::start(const worker_t worker) {
    Worker *job;
    switch (worker) {
        case PSO:
            job = new PSOWorker(SWARM_SIZE, SWARM_ITERATIONS);
            break;
        case MIMO:
            job = nullptr;
            break;
        case SOUND:
            job = nullptr;
            break;

        default:
            return false;
    }

    if (job) {
        workers.push_back(job);
        return true;
    } else {
        return false;
    }
}

bool AWProcessingUnit::stop(const worker_t worker) {
    for (auto& obj : workers) {
        if (obj->get_type() == worker) {
            delete obj;
            return true;
        }
    }

    return false;
}

void AWProcessingUnit::pause() {
    if (this->running) {
        this->running = false;
    }
}

void AWProcessingUnit::resume() {
    if (!this->running) {
        this->running = true;
    }
}


int main() {

    AWProcessingUnit awpu = AWProcessingUnit("127.0.0.1");

    awpu.start(PSO);
    awpu.resume();

    awpu.pause();

    awpu.stop(PSO);

    return 0;
}