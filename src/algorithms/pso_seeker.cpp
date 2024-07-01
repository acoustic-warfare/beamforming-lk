#include "pso_seeker.h"

#define VALID_SENSOR(i) (64 <= i) && (i < 128)


float beamform(Streams *streams, int *offset_delays, float *fractional_delays, int n_sensors) {
  float out[N_SAMPLES] = {0.0f};
  unsigned n = 0;

  for (unsigned s = 0; s < n_sensors; s++) {
    if (VALID_SENSOR(s)) {
      float fraction = fractional_delays[s - 64];
      int offset = offset_delays[s - 64];

      float *signal = (float *)((char *)streams->buffers[s] +
                                streams->position + offset * sizeof(float));

      for (int i = 0; i < N_SAMPLES; i++) {
        out[i] += signal[i + 1] + fraction * (signal[i] - signal[i + 1]);
      }

      n++;
    }
  }

  float power = 0.0f;
  float norm = 1 /(float)n;

  for (int i = 0; i < N_SAMPLES; i++) {
    power += powf(out[i] * norm, 2);
  }

  return power / (float)N_SAMPLES;

}


Particle::Particle(Antenna &antenna, Streams *streams, int n_sensors) : antenna(antenna), streams(streams), n_sensors(n_sensors) {
    azimuth = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
    elevation = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
    velocity_azimuth = 0.0f;
    velocity_elevation = 0.0f;
    best_azimuth = azimuth;
    best_elevation = elevation;
    best_magnitude = compute(azimuth, elevation, n_sensors);
}


void Particle::random() {
  azimuth = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
  elevation = (static_cast<float>(rand()) / RAND_MAX - 0.5f) * 2.0 * ANGLE_LIMIT;
}

float Particle::compute(float azimuth, float elevation, int n_sensors) {
  Eigen::VectorXf tmp_delays = steering_vector_horizontal(antenna, azimuth, elevation);
  float fractional_delays[n_sensors]; 
  int offset_delays[n_sensors];
  int i = 0;
  for (float del : tmp_delays) {
    double _offset;
    float fraction;
    fraction = (float)modf((double)del, &_offset);
    int offset = N_SAMPLES - (int)_offset;
    // cout << del << endl;
    fractional_delays[i] = fraction;
    offset_delays[i] = offset;
    i++;
  }
  return beamform(streams, &offset_delays[0], &fractional_delays[0], n_sensors);
}

void Particle::update() {
  float magnitude = compute(azimuth, elevation, n_sensors);
  best_magnitude *= 0.99;
  if (magnitude > best_magnitude) {
    best_magnitude = magnitude;
    best_azimuth = azimuth;
    best_elevation = elevation;
  }
}




inline float clip(float n, float lower, float upper) {
  return std::max(lower, std::min(n, upper));
}

inline float wrapAngle( float angle )
{
    double twoPi = 2.0 * 3.141592865358979;
    return (float)(angle - twoPi * floor( angle / twoPi ));
}


PSO::PSO(int n_particles, Antenna &antenna, Streams *streams, int n_sensors) : antenna(antenna), streams(streams), kf(0.1) {
  this->n_particles = n_particles;
  this->n_sensors = n_sensors;
  global_best_magnitude = 0.0f;
  initialize_particles();
}

void PSO::initialize_particles() {
  particles.clear();
  for (int i = 0; i < n_particles; i++) {
    particles.emplace_back(antenna, streams, n_sensors);

    Particle &particle = particles.back();

    if (particle.best_magnitude > global_best_magnitude) {
        global_best_magnitude = particle.best_magnitude;
        global_best_azimuth = particle.best_azimuth;
        global_best_elevation = particle.best_elevation;
    }
  }
}

void PSO::optimize(int iterations) {
  float w = 0.5f, c1 = 2.0f, c2 = 2.0f;
  float delta = to_radians(FOV / iterations * 2.0);
  global_best_magnitude *= 0.9f;
  for (auto& particle : particles) {
    particle.random();
  }
  for (int i = 0; i < iterations; i++) {
    for (auto& particle : particles) {
      particle.velocity_azimuth = w * particle.velocity_azimuth \
      + c1 * static_cast<float>(rand()) / RAND_MAX * (particle.best_azimuth - particle.azimuth) * LOCAL_AREA_RATIO \
      + c2 * static_cast<float>(rand()) / RAND_MAX * (global_best_azimuth - particle.azimuth) * GLOBAL_AREA_RATIO;
      particle.velocity_elevation = w * particle.velocity_elevation \
      + c1 * static_cast<float>(rand()) / RAND_MAX * (particle.best_elevation - particle.elevation) * LOCAL_AREA_RATIO \
      + c2 * static_cast<float>(rand()) / RAND_MAX * (global_best_elevation - particle.elevation) * GLOBAL_AREA_RATIO;
      particle.azimuth += particle.velocity_azimuth * delta;
      particle.elevation += particle.velocity_elevation * delta;
      
      //particle.azimuth = wrapAngle(particle.azimuth);
      //particle.azimuth %= (float)(2.0*M_PI); //clip(particle.azimuth, -ANGLE_LIMIT, ANGLE_LIMIT);
      //particle.elevation = clip(particle.elevation, to_radians(5.0), to_radians(80.0)); // Should be 2pi
      particle.azimuth = clip(particle.azimuth, -ANGLE_LIMIT, ANGLE_LIMIT);
      particle.elevation = clip(particle.elevation, -ANGLE_LIMIT, ANGLE_LIMIT);

      particle.update();

      if (particle.best_magnitude > global_best_magnitude) {
        global_best_magnitude = particle.best_magnitude;
        global_best_azimuth = particle.best_azimuth;
        global_best_elevation = particle.best_elevation;
      }
    }
  }
}

Eigen::Vector3f PSO::sanitize() {
  Eigen::Vector3f sample(global_best_azimuth, global_best_elevation, 0.0);

#if USE_KALMAN_FILTER
  kf.update(sample);
  return kf.getState();
#else
  return sample;
#endif
}




void pso_finder(Pipeline *pipeline, int stream_id, int n_sensors_in) {
  Antenna antenna = create_antenna(Position(0, 0, 0), COLUMNS, ROWS, DISTANCE);
  Streams *streams = pipeline->getStreams(stream_id);
  //n_sensors = n_sensors_in;

  PSO pso(40, antenna, streams, n_sensors_in);

  int newData;

  int prevX = 0;
  int prevY = 0;

  while (pipeline->isRunning()) {

    // Wait for incoming data
    pipeline->barrier();

    // This loop may run until new data has been produced, meaning its up to
    // the machine to run as fast as possible
    newData = pipeline->mostRecent();

    pso.initialize_particles();

    pso.optimize(30);

    Eigen::Vector3f sample = pso.sanitize();

    float azimuth = sample(0);
    float elevation = sample(1);

    azimuth = clip(azimuth, -ANGLE_LIMIT, ANGLE_LIMIT);
    elevation = clip(elevation, -ANGLE_LIMIT, ANGLE_LIMIT);

    //float x = (float)(cos((double)theta) * sin((double)phi));
    //float y = (float)(sin((double)theta) * sin((double)phi));

    //float x = (float)(cos((double)pso.global_best_theta) * sin((double)pso.global_best_phi));
    //float y = (float)(sin((double)pso.global_best_theta) * sin((double)pso.global_best_phi));
    
    //int xi = (int)((x + 1.0) / 2.0 * X_RES);
    //int yi = (int)((y + 1.0) / 2.0 * Y_RES);

    pipeline->magnitudeHeatmap->setTo(cv::Scalar(0));

    for (auto& particle : pso.particles) {
      azimuth = particle.best_azimuth;
      elevation = particle.best_elevation;
      int xi = (int)((double)X_RES * (azimuth + ANGLE_LIMIT) / 2.0);
      int yi = (int)((double)Y_RES * (elevation + ANGLE_LIMIT) / 2.0);
      
      //magnitudeHeatmap.at<uchar>(prevY, prevX) = (uchar)(0);
      //magnitudeHeatmap.at<uchar>(yi, xi) = (uchar)(255);
      pipeline->magnitudeHeatmap->at<uchar>(Y_RES - 1 - yi, xi) = (uchar)(255);
    }

    //int xi = (int)((double)X_RES * ((theta) + to_radians(FOV / 2)) / 2.0);
    //int yi = (int)((double)Y_RES * ((phi) + to_radians(FOV / 2)) / 2.0);
    //
    ////magnitudeHeatmap.at<uchar>(prevY, prevX) = (uchar)(0);
    ////magnitudeHeatmap.at<uchar>(yi, xi) = (uchar)(255);
    //magnitudeHeatmap.at<uchar>(Y_RES - 1 - yi, xi) = (uchar)(255);
    //prevX = xi;
    //prevY = yi;
    pipeline->canPlot = 1;
    //std::cout << "(" << x << ", " << y << ")" << std::endl;
    std::cout << "Theta: " << pso.global_best_azimuth << " Phi: " << pso.global_best_elevation << std::endl;
  }
}