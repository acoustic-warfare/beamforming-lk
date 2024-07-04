#include "pipeline.h"

struct Direction {
    double azimuth;
    double elevation;
};

enum worker_t {
    PSO,
    MIMO,
    SOUND
};

class Worker {
public:
    Worker() {};

    ~Worker() {};

    virtual worker_t get_type() {};

    

    Direction getDirection() {
        return Direction(0, 0);

    };
private:
    Direction direction;
    
};

class PSOWorker : public Worker {
public:
    PSOWorker(std::size_t swarm_size, std::size_t iterations) : swarm_size(swarm_size), iterations(iterations) {};
    worker_t get_type() {
        return worker_t::PSO;
    }
private:
    std::size_t swarm_size;
    std::size_t iterations;
};




class AWProcessingUnit {    
public:
    AWProcessingUnit(const std::string ip_address);
    ~AWProcessingUnit();

    bool start(const worker_t worker);
    bool stop(const worker_t worker);
    void pause();
    void resume();
    


private:
    std::string ip_address;
    std::vector<Worker*> workers;
    Pipeline *pipeline;
    bool running;
};