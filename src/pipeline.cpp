#include "pipeline.h"

Pipeline::Pipeline(const char *address, const int port, bool verbose) : address(address), port(port), verbose(verbose), synthetic(false) {
    streams = new Streams();
}

Pipeline::Pipeline(float frequency, Spherical direction, bool verbose) : verbose(verbose), synthetic(true), port(-1) {
    streams = new Streams();
}

Pipeline::~Pipeline() {
    delete streams;

    for (int sensor_index = 0; sensor_index < n_sensors; sensor_index++) {
        delete[] exposure_buffer[sensor_index];
    }

    delete[] exposure_buffer;
}

/**
 * Connect beamformer to antenna
 */
int Pipeline::connect() {

    if (connected) {
        std::cerr << "Beamformer is already connected" << std::endl;

        return -1;
    }

    if (synthetic) {
        return connect_synthetic();
    } else {
        return connect_real();
    }
}

int Pipeline::connect_real() {

    socket_desc = init_receiver(address, port);

    if (socket_desc == -1) {
        std::cerr << "Unable to establish a connection to antenna" << std::endl;
        return -1;
    } else {
        connected = 1;
    }

    if (receive_message(socket_desc, &msg) < 0) {

        std::cerr << "Unable to receive message" << std::endl;

        return -1;
    }

    // Populating configs
    n_sensors = msg.n_arrays * ELEMENTS;

    // Allocate memory for UDP packet data
    exposure_buffer = new float *[n_sensors];

    for (int sensor_index = 0; sensor_index < n_sensors; sensor_index++) {
        exposure_buffer[sensor_index] = new float[N_SAMPLES];
        this->streams->create_stream(sensor_index);

        if (verbose) {
            std::cout << "[Pipeline] Adding stream: " << sensor_index << std::endl;
        }
        
    }

    receiver_thread = std::thread(&Pipeline::producer, this);

    return 0;
}

int Pipeline::connect_synthetic() {
    n_sensors = ELEMENTS;

    // Allocate memory for UDP packet data
    exposure_buffer = new float *[n_sensors];

    for (int sensor_index = 0; sensor_index < n_sensors; sensor_index++) {
        exposure_buffer[sensor_index] = new float[N_SAMPLES];
        this->streams->create_stream(sensor_index);

        if (verbose) {
            std::cout << "[Pipeline] Adding Synthetic stream: " << sensor_index << std::endl;
        }
        }

    connected = 1;

    receiver_thread = std::thread(&Pipeline::synthetic_producer, this);

    return 0;
}

#define PHASE(delay, frequency) 2 * M_PI *frequency *delay

void Pipeline::synthetic_producer() {
    Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);
    Spherical angle(TO_RADIANS(0), TO_RADIANS(0));
    Eigen::VectorXf tmp_delays1 = steering_vector_spherical(antenna, angle.theta, angle.phi);
    Eigen::VectorXf tmp_delays2 = steering_vector_spherical(antenna, angle.theta + TO_RADIANS(10), angle.phi);
    std::cout << tmp_delays1 << std::endl;

    const std::chrono::milliseconds targetDuration(static_cast<int>(1000.0 * N_SAMPLES / SAMPLE_RATE));
    int p = 0;
    int p2 = 0;
    constexpr float carrier = 9e3;
    constexpr float carrier2 = 4e3;
    while (isRunning()) {

        auto start = std::chrono::steady_clock::now();

        float signals[ELEMENTS][N_SAMPLES];

        for (int s = 0; s < ELEMENTS; s++) {
            float delay1 = tmp_delays1(s);
            float delay2 = tmp_delays2(s);
            for (int i = 0; i < N_SAMPLES; i++) {
                signals[s][i] = 0.0;
                float t = static_cast<float>(p + i) / SAMPLE_RATE;// Time in seconds
                signals[s][i] += sin(2 * M_PI * carrier * t + PHASE(delay1, carrier));
                //signals[s][i] += sin(2 * M_PI * carrier2 * t + PHASE(delay2, carrier2));

                signals[s][i] *= 1e-2;
            }
            streams->write_stream(s, &signals[s][0]);
        }

        p += N_SAMPLES;
        p2 += N_SAMPLES;


        {
            std::unique_lock<std::mutex> lock(barrier_mutex);
            streams->forward();
        }
        // Measure time taken
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // Sleep if the elapsed time is less than 5 ms
        if (elapsed < targetDuration) {
            std::this_thread::sleep_for(targetDuration - elapsed);
            //std::cout << "Sleeping: " << 0 << " " << streams->get_signal(0, 0)[5] << std::endl;
        }

        release_barrier();
    }
}

/**
 * Disconnect beamformer from antenna
 */
int Pipeline::disconnect() {
    if (!connected) {
        std::cerr << "Beamformer is not connected" << std::endl;
        return -1;
    }

    {
        std::unique_lock<std::mutex> lock(pool_mutex);

        connected = 0;
    }

    receiver_thread.join();
    std::cout << "Disconnected connection" << std::endl;

    release_barrier();

    std::cout << "Barrier released for pipeline" << std::endl;

    if (port != -1) {
        close(socket_desc);
    }
    

    std::cout << "Destroyed pipeline" << std::endl;

    return 0;
}

/**
 * Check if the producer thread is generating data
 */
int Pipeline::isRunning() {
    std::unique_lock<std::mutex> lock(pool_mutex);
    return connected;
}

/**
 * check if no new data has been added
 */
int Pipeline::mostRecent() {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    return modified;
}

/**
 * @brief Wait until a release is dispatched
 */
void Pipeline::barrier() {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    barrier_count++;

    barrier_condition.wait(lock, [&] { return barrier_count == 0; });
}

Streams *Pipeline::getStreams() {
    return streams;
}

/**
 * Allow the worker threads to continue
 */
void Pipeline::release_barrier() {
    std::unique_lock<std::mutex> lock(barrier_mutex);
    barrier_count = 0;
    modified++;
    barrier_condition.notify_all();
}

/**
 * The main distributer of data to the threads (this is also a thread)
 */
void Pipeline::producer() {
    while (isRunning()) {

        receive_exposure();

        {
            std::unique_lock<std::mutex> lock(barrier_mutex);
            streams->forward();
        }

        release_barrier();
    }
}

/**
 * Receive a window to the streams buffers 
 */
void Pipeline::receive_exposure() {
    for (int i = 0; i < N_SAMPLES; i++) {
        for (int s = 0; s < 1; s++) {
            if (receive_message(socket_desc, &msg) < 0) {
                std::cerr << "Error exposure" << std::endl;
                break;
            }
        }
        

        // Flip columns
        int inverted = 0;

        // Actual microphone index
        unsigned index;

        for (unsigned sensor_index = 0; sensor_index < n_sensors; sensor_index++) {
            // Arrays are daisy-chained so flip on columns
            if (sensor_index % COLUMNS == 0) {
                inverted = !inverted;
            }

            if (inverted) {
                index = COLUMNS * (1 + sensor_index / COLUMNS) - 1 - sensor_index % COLUMNS;
            } else {
                index = sensor_index;
            }

            // Normalize mic data between -1.0 and 1.0
            exposure_buffer[sensor_index][i] = (float) msg.stream[index] / (float) MAX_VALUE_FLOAT;
        }
    }

    for (unsigned sensor_index = 0; sensor_index < n_sensors; sensor_index++) {
        streams->write_stream(sensor_index, &exposure_buffer[sensor_index][0]);
    }
}

// Debugging
int Pipeline::save_pipeline(std::string path) {
    std::ofstream outfile(path, std::ios::binary);

    if (!outfile.is_open()) {
        std::cerr << "Error opening file for writing" << std::endl;
        return 1;
    }

    //outfile.write(reinterpret_cast<char *>(rb.data),
    //              sizeof(float) * N_SENSORS * BUFFER_LENGTH);

    // Close the file
    outfile.close();

    return 0;
}